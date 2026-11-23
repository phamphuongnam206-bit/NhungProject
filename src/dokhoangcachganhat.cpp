#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

constexpr uint8_t SENSOR_COUNT = 4;
constexpr uint16_t FRONT_OFFSET_MM = 15;
constexpr uint8_t FRONT_LEFT = 1;
constexpr uint8_t FRONT_RIGHT = 2;
constexpr uint16_t SLOW_MM = 300;
constexpr uint16_t STOP_MM = 200;
constexpr uint16_t NO_TARGET_MM = 8190;
constexpr uint16_t MIN_SIGNAL_Q7 = 32;
constexpr uint8_t VALID_RANGE_STATUS = 11;

VL53L0X sensors[SENSOR_COUNT];

const uint32_t xshutPins[SENSOR_COUNT] = {
  PA2, PA3, PA4, PA5
};

const uint8_t addresses[SENSOR_COUNT] = {
  0x30, 0x31, 0x32, 0x33
};

const char *names[SENSOR_COUNT] = {
  "J8_TRAI",
  "J9_TRUOC_TRAI",
  "J10_TRUOC_PHAI",
  "J11_PHAI"
};

bool ready[SENSOR_COUNT] = {false};
struct Sample {
  const char *result = "OFF";
  int32_t raw = -1;
  int16_t deviceStatus = -1;
  int32_t signalQ7 = -1;
  uint16_t corrected = 0;
  uint16_t median = 0;
  uint16_t previous[2] = {};
  uint8_t count = 0;
};
Sample samples[SENSOR_COUNT];

uint32_t lastCycle = 0;

void disableSensor(uint8_t i) {
  digitalWrite(xshutPins[i], LOW);
  pinMode(xshutPins[i], OUTPUT);
  ready[i] = false;
}

bool probe(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

uint16_t correctDistance(uint8_t i, uint16_t rawMM) {
  if (i == FRONT_LEFT || i == FRONT_RIGHT) {
    if (rawMM <= FRONT_OFFSET_MM) {
      return 0;
    }
    return rawMM - FRONT_OFFSET_MM;
  }
  return rawMM;
}

bool readResultBlock(uint8_t address, uint8_t (&bytes)[12]) {
  Wire.beginTransmission(address);
  Wire.write(uint8_t(VL53L0X::RESULT_RANGE_STATUS));
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, uint8_t(sizeof(bytes))) != sizeof(bytes)) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t &byte : bytes) {
    const int value = Wire.read();
    if (value < 0) return false;
    byte = uint8_t(value);
  }
  return true;
}

uint16_t medianOfThree(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) {
    const uint16_t swap = a;
    a = b;
    b = swap;
  }
  if (c < a) return a;
  return c > b ? b : c;
}

void setup() {
  digitalWrite(PB0, LOW);
  pinMode(PB0, OUTPUT);

  Serial.setRx(PA10);
  Serial.setTx(PA9);
  Serial.begin(115200, SERIAL_8N1);

  delay(1000);

  Serial.println();
  Serial.println("BOOT: TEST 4 VL53L0X");
  Serial.println("FRONT OFFSET = 15 mm");
  Serial.flush();

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    disableSensor(i);
  }

  delay(200);

  Wire.setSCL(PB10);
  Wire.setSDA(PB11);
  Wire.begin();
  Wire.setClock(50000);

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print("INIT ");
    Serial.println(names[i]);
    Serial.flush();

    pinMode(xshutPins[i], INPUT);
    delay(100);

    if (!probe(0x29)) {
      Serial.println("FAIL: NO ACK");
      disableSensor(i);
      continue;
    }

    sensors[i].setBus(&Wire);
    sensors[i].setTimeout(250);

    if (!sensors[i].init() || sensors[i].last_status != 0) {
      Serial.println("FAIL: INIT");
      disableSensor(i);
      continue;
    }

    sensors[i].setAddress(addresses[i]);

    uint8_t addressStatus = sensors[i].last_status;
    bool addressOK = probe(addresses[i]);

    if (addressStatus != 0 || !addressOK) {
      Serial.println("FAIL: ADDRESS");
      disableSensor(i);
      continue;
    }

    if (!sensors[i].setMeasurementTimingBudget(33000) || sensors[i].last_status != 0) {
      Serial.println("FAIL: TIMING");
      disableSensor(i);
      continue;
    }

    ready[i] = true;

    Serial.print("READY: 0x");
    Serial.println(addresses[i], HEX);
  }

  Serial.println("START READING - MOTORS OFF");
}

const char *readSingle(uint8_t i, Sample &sample) {
  if (!ready[i]) return "OFF";
  const uint16_t reported = sensors[i].readRangeSingleMillimeters();
  if (sensors[i].timeoutOccurred()) return "TIMEOUT";
  if (sensors[i].last_status != 0) return "I2C";

  // Single-shot mode keeps this result available until the next shot.
  uint8_t bytes[12];
  if (!readResultBlock(addresses[i], bytes)) return "I2C";
  sample.raw = (uint16_t(bytes[10]) << 8) | bytes[11];
  sample.deviceStatus = (bytes[0] & 0x78) >> 3;
  sample.signalQ7 = (uint16_t(bytes[6]) << 8) | bytes[7];
  if (sample.raw != reported) return "FRAME_MISMATCH";
  if (sample.raw == 0 || sample.raw >= NO_TARGET_MM) return "INVALID_RANGE";
  if (sample.deviceStatus != VALID_RANGE_STATUS) return "INVALID_STATUS";
  if (sample.signalQ7 < MIN_SIGNAL_Q7) return "LOW_SIGNAL";
  return nullptr;
}

void readSensor(uint8_t i) {
  Sample &sample = samples[i];
  sample.raw = sample.deviceStatus = sample.signalQ7 = -1;
  sample.result = readSingle(i, sample);
  if (sample.result != nullptr) {
    sample.count = 0;
    return;
  }

  sample.corrected = correctDistance(i, uint16_t(sample.raw));
  if (sample.count >= 2) {
    sample.median = medianOfThree(sample.corrected, sample.previous[0], sample.previous[1]);
  }
  sample.previous[1] = sample.previous[0];
  sample.previous[0] = sample.corrected;
  if (sample.count < 3) ++sample.count;
}

void printDistance(uint8_t i) {
  const Sample &sample = samples[i];
  Serial.print(names[i]);
  Serial.print(" RAW="); Serial.print(sample.raw);
  Serial.print(" ST="); Serial.print(sample.deviceStatus);
  Serial.print(" SIG_Q7="); Serial.print(sample.signalQ7);
  if (sample.result != nullptr) {
    Serial.print(" "); Serial.print(sample.result);
  } else {
    Serial.print(" MM="); Serial.print(sample.corrected);
    Serial.print(" MED="); Serial.print(sample.count >= 3 ? int(sample.median) : -1);
  }
}

void loop() {
  if (millis() - lastCycle < 200) {
    return;
  }

  lastCycle = millis();

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    readSensor(i);
  }

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    printDistance(i);
    Serial.print(" | ");
  }

  if (samples[FRONT_LEFT].result || samples[FRONT_RIGHT].result) {
    Serial.println("FRONT=UNKNOWN | THIEU_DU_LIEU");
    return;
  }

  const uint16_t leftMM = samples[FRONT_LEFT].corrected;
  const uint16_t rightMM = samples[FRONT_RIGHT].corrected;
  const uint16_t frontMM = leftMM < rightMM ? leftMM : rightMM;

  Serial.print("FRONT_MIN=");
  Serial.print(frontMM / 10.0f, 1);
  Serial.print("cm | ");

  if (frontMM <= STOP_MM) {
    Serial.println("VAT_CAN_GAN");
  } else if (frontMM <= SLOW_MM) {
    Serial.println("CAN_GIAM_TOC");
  } else {
    Serial.println("NGOAI_NGUONG_30CM");
  }
}
