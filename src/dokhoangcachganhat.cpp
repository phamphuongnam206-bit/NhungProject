#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

constexpr uint8_t SENSOR_COUNT = 4;
constexpr uint16_t FRONT_OFFSET_MM = 15;
constexpr uint8_t FRONT_LEFT = 1;
constexpr uint8_t FRONT_RIGHT = 2;
constexpr uint16_t SLOW_MM = 300;
constexpr uint16_t STOP_MM = 200;

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
bool valid[SENSOR_COUNT] = {false};
uint16_t distanceMM[SENSOR_COUNT] = {0};

uint32_t lastCycle = 0;

void disableSensor(uint8_t i) {
  digitalWrite(xshutPins[i], LOW);
  pinMode(xshutPins[i], OUTPUT);
  ready[i] = false;
  valid[i] = false;
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

    if (!sensors[i].init()) {
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

    if (!sensors[i].setMeasurementTimingBudget(33000)) {
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

void readSensor(uint8_t i) {
  valid[i] = false;

  if (!ready[i]) {
    return;
  }

  uint16_t rawMM = sensors[i].readRangeSingleMillimeters();

  bool timedOut = sensors[i].timeoutOccurred();
  uint8_t busStatus = sensors[i].last_status;

  if (timedOut || busStatus != 0) {
    return;
  }

  uint8_t raw = sensors[i].readReg(VL53L0X::RESULT_RANGE_STATUS);
  uint8_t deviceStatus = (raw & 0x78) >> 3;

  valid[i] = sensors[i].last_status == 0 &&
             deviceStatus == 11 &&
             rawMM < 8190;

  if (valid[i]) {
    distanceMM[i] = correctDistance(i, rawMM);
  }
}

void printDistance(uint8_t i) {
  Serial.print(names[i]);
  Serial.print("=");

  if (!ready[i]) {
    Serial.print("OFF");
  } else if (!valid[i]) {
    Serial.print("INVALID");
  } else {
    Serial.print(distanceMM[i] / 10.0f, 1);
    Serial.print("cm");
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

  if (!valid[FRONT_LEFT] || !valid[FRONT_RIGHT]) {
    Serial.println("FRONT=UNKNOWN | THIEU_DU_LIEU");
    return;
  }

  uint16_t frontMM =
    distanceMM[FRONT_LEFT] < distanceMM[FRONT_RIGHT]
      ? distanceMM[FRONT_LEFT]
      : distanceMM[FRONT_RIGHT];

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