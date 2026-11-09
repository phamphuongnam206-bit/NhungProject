#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <math.h>

constexpr uint8_t SENSOR_COUNT = 3;

constexpr uint32_t LED_PIN = PC13;
constexpr uint32_t STBY_PIN = PB0;

constexpr uint32_t LEFT_IN1 = PB14;
constexpr uint32_t LEFT_IN2 = PB15;
constexpr uint32_t LEFT_PWM = PB9;

constexpr uint32_t RIGHT_IN1 = PB12;
constexpr uint32_t RIGHT_IN2 = PB13;
constexpr uint32_t RIGHT_PWM = PB8;

constexpr uint32_t LEFT_ENC_A = PA6;
constexpr uint32_t LEFT_ENC_B = PA7;
constexpr uint32_t RIGHT_ENC_A = PA0;
constexpr uint32_t RIGHT_ENC_B = PA1;

constexpr uint16_t FRONT_OFFSET_MM = 15;
constexpr uint16_t START_MM = 200;
constexpr uint16_t STOP_MM = 210;
constexpr uint16_t SIDE_DIFFERENCE_MM = 30;

constexpr int BASE_PWM = 140;
constexpr int TURN_PWM = 15;
constexpr int MAX_CORRECTION = 20;
constexpr float SYNC_GAIN = 0.04f;

constexpr int8_t STRAIGHT = 0;
constexpr int8_t HEAD_LEFT = -1;
constexpr int8_t HEAD_RIGHT = 1;

VL53L0X sensors[SENSOR_COUNT];

const uint32_t xshutPins[SENSOR_COUNT] = {
  PA2, PA3, PA4
};

const uint8_t addresses[SENSOR_COUNT] = {
  0x30, 0x31, 0x32
};

const char *names[SENSOR_COUNT] = {
  "J8", "J9", "J10"
};

// Khoi tao cap giua truoc, sau do bat them cam bien trai ngoai.
const uint8_t initOrder[SENSOR_COUNT] = {1, 2, 0};

bool ready[SENSOR_COUNT] = {false};
bool valid[SENSOR_COUNT] = {false};
uint16_t distanceMM[SENSOR_COUNT] = {0};

const int8_t quadratureTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

volatile int32_t leftCount = 0;
volatile int32_t rightCount = 0;
volatile uint8_t leftState = 0;
volatile uint8_t rightState = 0;

int32_t previousLeft = 0;
int32_t previousRight = 0;

uint32_t lastControl = 0;
uint32_t lastPrint = 0;
uint32_t lastLedToggle = 0;

bool reversing = false;
bool ledOn = false;
bool ledActive = false;

int8_t preferredTurn = STRAIGHT;
int8_t activeTurn = STRAIGHT;
int8_t previousTurn = STRAIGHT;

float correctionFiltered = 0.0f;

int leftPower = 0;
int rightPower = 0;

void leftEncoderISR() {
  uint8_t current =
    (digitalRead(LEFT_ENC_A) << 1) |
    digitalRead(LEFT_ENC_B);

  leftCount += quadratureTable[(leftState << 2) | current];
  leftState = current;
}

void rightEncoderISR() {
  uint8_t current =
    (digitalRead(RIGHT_ENC_A) << 1) |
    digitalRead(RIGHT_ENC_B);

  rightCount += quadratureTable[(rightState << 2) | current];
  rightState = current;
}

void holdLow(uint32_t pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

uint8_t probeAddress(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission();
}

void printBusLevels() {
  Serial.print("SCL=");
  Serial.print(digitalRead(PB10));
  Serial.print(" SDA=");
  Serial.println(digitalRead(PB11));
}

void disableSensor(uint8_t i) {
  holdLow(xshutPins[i]);
  ready[i] = false;
  valid[i] = false;
}

void stopMotors() {
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
  digitalWrite(STBY_PIN, LOW);

  leftPower = 0;
  rightPower = 0;
  correctionFiltered = 0.0f;
}

void driveReverse(int leftValue, int rightValue) {
  leftPower = constrain(leftValue, 0, 255);
  rightPower = constrain(rightValue, 0, 255);

  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);

  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, HIGH);

  analogWrite(LEFT_PWM, leftPower);
  analogWrite(RIGHT_PWM, rightPower);

  digitalWrite(STBY_PIN, HIGH);
}

void updateStatusLed(
  uint32_t now,
  bool measurementValid,
  uint16_t nearestMM
) {
  if (!measurementValid) {
    ledOn = false;
    ledActive = false;
    digitalWrite(LED_PIN, HIGH);
    lastLedToggle = now;
    return;
  }

  uint32_t cycleMs = 4000;

  if (nearestMM <= 210) {
    cycleMs = 1000;
  } else if (nearestMM < 250) {
    cycleMs = 4000UL -
      ((250UL - nearestMM) * 3000UL / 40UL);
  }

  if (!ledActive) {
    ledActive = true;
    ledOn = true;
    lastLedToggle = now;
    digitalWrite(LED_PIN, LOW);
    return;
  }

  if (now - lastLedToggle >= cycleMs / 2) {
    lastLedToggle = now;
    ledOn = !ledOn;
    digitalWrite(LED_PIN, ledOn ? LOW : HIGH);
  }
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

  uint8_t rawStatus =
    sensors[i].readReg(VL53L0X::RESULT_RANGE_STATUS);

  uint8_t deviceStatus = (rawStatus & 0x78) >> 3;

  if (sensors[i].last_status != 0 ||
      deviceStatus != 11 ||
      rawMM >= 8190) {
    return;
  }

  if (i == 1 || i == 2) {
    distanceMM[i] =
      rawMM > FRONT_OFFSET_MM ? rawMM - FRONT_OFFSET_MM : 0;
  } else {
    distanceMM[i] = rawMM;
  }

  valid[i] = true;
}

uint16_t sideDistance(uint8_t first, uint8_t second) {
  uint16_t result = 65535;

  if (valid[first]) {
    result = distanceMM[first];
  }

  if (valid[second] && distanceMM[second] < result) {
    result = distanceMM[second];
  }

  return result;
}

int8_t chooseTurn(uint16_t leftMM, uint16_t rightMM) {
  bool leftNear = leftMM < START_MM;
  bool rightNear = rightMM < START_MM;

  if (leftNear && rightNear) {
    return STRAIGHT;
  }

  if (leftNear) {
    if (rightMM == 65535 ||
        (int32_t)rightMM - leftMM >= SIDE_DIFFERENCE_MM) {
      return HEAD_RIGHT;
    }
  }

  if (rightNear) {
    if (leftMM == 65535 ||
        (int32_t)leftMM - rightMM >= SIDE_DIFFERENCE_MM) {
      return HEAD_LEFT;
    }
  }

  return STRAIGHT;
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

void setup() {
  digitalWrite(LED_PIN, HIGH);
  pinMode(LED_PIN, OUTPUT);

  holdLow(STBY_PIN);
  holdLow(LEFT_IN1);
  holdLow(LEFT_IN2);
  holdLow(RIGHT_IN1);
  holdLow(RIGHT_IN2);

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);

  analogWriteResolution(8);
  stopMotors();

  Serial.setRx(PA10);
  Serial.setTx(PA9);
  Serial.begin(115200, SERIAL_8N1);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  leftState =
    (digitalRead(LEFT_ENC_A) << 1) |
    digitalRead(LEFT_ENC_B);

  rightState =
    (digitalRead(RIGHT_ENC_A) << 1) |
    digitalRead(RIGHT_ENC_B);

  attachInterrupt(
    digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, CHANGE
  );
  attachInterrupt(
    digitalPinToInterrupt(LEFT_ENC_B), leftEncoderISR, CHANGE
  );
  attachInterrupt(
    digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, CHANGE
  );
  attachInterrupt(
    digitalPinToInterrupt(RIGHT_ENC_B), rightEncoderISR, CHANGE
  );

  holdLow(PA5);  // J11 ben phai ngoai giu tat trong bai thu 3 cam bien.
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    disableSensor(i);
  }

  delay(1000);

  Serial.println("\nBOOT: 3 SENSORS J9 J10 J8 - J11 DISABLED");
  Serial.flush();

  Wire.setSCL(PB10);
  Wire.setSDA(PB11);
  Wire.begin();
  Wire.setClock(50000);

  Serial.print("BUS BEFORE INIT ");
  printBusLevels();

  uint8_t readyCount = 0;

  for (uint8_t step = 0; step < SENSOR_COUNT; step++) {
    uint8_t i = initOrder[step];
    Serial.println();
    Serial.print("INIT ");
    Serial.println(names[i]);
    Serial.flush();

    pinMode(xshutPins[i], INPUT);
    delay(100);

    Serial.print("XSHUT=");
    Serial.print(digitalRead(xshutPins[i]));
    Serial.print(" | ");
    printBusLevels();

    uint8_t defaultProbe = probeAddress(0x29);

    Serial.print("PROBE_0x29=");
    Serial.println(defaultProbe);
    Serial.flush();

    if (defaultProbe != 0) {
      Serial.println("FAIL: NO ACK");
      disableSensor(i);
      delay(10);

      Serial.print("AFTER_SHUTDOWN ");
      printBusLevels();
      continue;
    }

    sensors[i].setBus(&Wire);
    sensors[i].setTimeout(100);

    bool initOK = sensors[i].init();
    uint8_t initStatus = sensors[i].last_status;

    Serial.print("INIT_OK=");
    Serial.print(initOK);
    Serial.print(" | INIT_LAST_STATUS=");
    Serial.println(initStatus);

    if (!initOK) {
      Serial.println("FAIL: INIT");
      disableSensor(i);
      delay(10);

      Serial.print("AFTER_SHUTDOWN ");
      printBusLevels();
      continue;
    }

    sensors[i].setAddress(addresses[i]);

    uint8_t addressStatus = sensors[i].last_status;

    delay(10);

    uint8_t newProbeStatus = probeAddress(addresses[i]);

    Serial.print("NEW_ADDRESS=0x");
    Serial.print(addresses[i], HEX);

    Serial.print(" | ADDRESS_WRITE=");
    Serial.print(addressStatus);

    Serial.print(" | PROBE_NEW=");
    Serial.print(newProbeStatus);

    Serial.print(" | XSHUT=");
    Serial.print(digitalRead(xshutPins[i]));

    Serial.print(" | ");
    printBusLevels();
    Serial.flush();

    if (addressStatus != 0 || newProbeStatus != 0) {
      Serial.println("FAIL: ADDRESS");
      disableSensor(i);
      delay(10);

      Serial.print("AFTER_SHUTDOWN ");
      printBusLevels();
      continue;
    }

    if (!sensors[i].setMeasurementTimingBudget(33000)) {
      Serial.println("FAIL: TIMING");
      disableSensor(i);
      continue;
    }

    ready[i] = true;
    readyCount++;

    Serial.print("READY ");
    Serial.print(names[i]);
    Serial.print(" AT 0x");
    Serial.println(addresses[i], HEX);
  }

  Serial.println();
  Serial.print("READY_COUNT=");
  Serial.println(readyCount);

  Serial.println("ARMING IN 5 SECONDS");
  Serial.flush();

  delay(5000);

  noInterrupts();
  previousLeft = leftCount;
  previousRight = rightCount;
  interrupts();

  lastControl = millis();

  Serial.println("ACTIVE");
}

void loop() {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    readSensor(i);
  }

  uint32_t now = millis();
  uint32_t elapsed = now - lastControl;
  lastControl = now;

  noInterrupts();
  int32_t left = leftCount;
  int32_t right = rightCount;
  interrupts();

  int32_t deltaLeft = left - previousLeft;
  int32_t deltaRight = right - previousRight;

  previousLeft = left;
  previousRight = right;

  uint16_t leftMM = sideDistance(0, 1);
  uint16_t rightMM = valid[2] ? distanceMM[2] : 65535;
  uint16_t nearestMM = leftMM < rightMM ? leftMM : rightMM;

  bool anyValid = nearestMM != 65535;
  bool wasReversing = reversing;

  updateStatusLed(now, anyValid, nearestMM);

  if (!anyValid) {
    reversing = false;
  } else if (!reversing && nearestMM < START_MM) {
    reversing = true;
    preferredTurn = chooseTurn(leftMM, rightMM);
  } else if (reversing && nearestMM >= STOP_MM) {
    reversing = false;
  }

  if (!reversing) {
    preferredTurn = STRAIGHT;
    activeTurn = STRAIGHT;
    stopMotors();
  } else {
    if (preferredTurn == STRAIGHT) {
      preferredTurn = chooseTurn(leftMM, rightMM);
    }

    activeTurn = preferredTurn;

    if (activeTurn == HEAD_RIGHT && rightMM < START_MM) {
      activeTurn = STRAIGHT;
    }

    if (activeTurn == HEAD_LEFT && leftMM < START_MM) {
      activeTurn = STRAIGHT;
    }

    int targetLeft = BASE_PWM;
    int targetRight = BASE_PWM;

    if (activeTurn == HEAD_RIGHT) {
      targetLeft -= TURN_PWM;
      targetRight += TURN_PWM;
    } else if (activeTurn == HEAD_LEFT) {
      targetLeft += TURN_PWM;
      targetRight -= TURN_PWM;
    }

    if (!wasReversing || activeTurn != previousTurn) {
      correctionFiltered = 0.0f;
    } else if (elapsed > 0) {
      float leftRate =
        fabsf((float)deltaLeft) * 1000.0f / elapsed;

      float rightRate =
        fabsf((float)deltaRight) * 1000.0f / elapsed;

      float ratioError =
        (leftRate / targetLeft - rightRate / targetRight)
        * BASE_PWM;

      float requestedCorrection = ratioError * SYNC_GAIN;

      if (requestedCorrection > MAX_CORRECTION) {
        requestedCorrection = MAX_CORRECTION;
      }

      if (requestedCorrection < -MAX_CORRECTION) {
        requestedCorrection = -MAX_CORRECTION;
      }

      correctionFiltered =
        0.7f * correctionFiltered + 0.3f * requestedCorrection;
    }

    int correction = (int)correctionFiltered;

    driveReverse(
      targetLeft - correction,
      targetRight + correction
    );
  }

  previousTurn = activeTurn;

  if (now - lastPrint >= 200) {
    lastPrint = now;

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      printDistance(i);
      Serial.print(" | ");
    }

    if (!anyValid) {
      Serial.print("DUNG_MAT_DU_LIEU");
    } else if (!reversing) {
      Serial.print("DUNG");
    } else if (activeTurn == HEAD_RIGHT) {
      Serial.print("LUI_DAU_PHAI");
    } else if (activeTurn == HEAD_LEFT) {
      Serial.print("LUI_DAU_TRAI");
    } else {
      Serial.print("LUI_THANG");
    }

    Serial.print(" | PWM_L=");
    Serial.print(leftPower);

    Serial.print(" PWM_R=");
    Serial.print(rightPower);

    Serial.print(" | ENC_L=");
    Serial.print(deltaLeft);

    Serial.print(" ENC_R=");
    Serial.print(deltaRight);

    Serial.print(" | dt=");
    Serial.print(elapsed);
    Serial.println("ms");
  }
}
