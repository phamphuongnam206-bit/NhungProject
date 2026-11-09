#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <math.h>

constexpr uint8_t SENSOR_COUNT = 4;

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
  PA2, PA3, PA4, PA5
};

const uint8_t addresses[SENSOR_COUNT] = {
  0x30, 0x31, 0x32, 0x33
};

const char *names[SENSOR_COUNT] = {
  "J8", "J9", "J10", "J11"
};

bool ready[SENSOR_COUNT] = {false};
bool valid[SENSOR_COUNT] = {false};
uint16_t distanceMM[SENSOR_COUNT] = {0};

uint16_t previousLeftRaw = 0;
uint16_t previousRightRaw = 0;

int32_t leftTotal = 0;
int32_t rightTotal = 0;
int32_t deltaLeft = 0;
int32_t deltaRight = 0;

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

void holdLow(uint32_t pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

void configureEncoderTimer(TIM_TypeDef *timer) {
  timer->CR1 = 0;
  timer->CR2 = 0;
  timer->SMCR = 0;
  timer->DIER = 0;
  timer->CCER = 0;
  timer->PSC = 0;
  timer->ARR = 65535;
  timer->CCMR1 =
    1U | (8U << 4) | (1U << 8) | (8U << 12);
  timer->CCMR2 = 0;
  timer->SMCR = 3U;
  timer->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
  timer->EGR = TIM_EGR_UG;
  timer->CNT = 0;
  timer->SR = 0;
  timer->CR1 = TIM_CR1_CEN;
}

void initHardwareEncoders() {
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();

  HAL_NVIC_DisableIRQ(TIM2_IRQn);
  HAL_NVIC_DisableIRQ(TIM3_IRQn);
  HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
  HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);

  __HAL_RCC_TIM2_FORCE_RESET();
  __HAL_RCC_TIM2_RELEASE_RESET();
  __HAL_RCC_TIM3_FORCE_RESET();
  __HAL_RCC_TIM3_RELEASE_RESET();

  uint32_t mapr = AFIO->MAPR;

  mapr &= ~(AFIO_MAPR_TIM2_REMAP |
            AFIO_MAPR_TIM3_REMAP |
            AFIO_MAPR_SWJ_CFG);

  mapr |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
  AFIO->MAPR = mapr;

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  configureEncoderTimer(TIM2);
  configureEncoderTimer(TIM3);

  previousLeftRaw = (uint16_t)TIM3->CNT;
  previousRightRaw = (uint16_t)TIM2->CNT;
}

int32_t counterDifference(uint16_t current, uint16_t previous) {
  uint16_t difference = (uint16_t)(current - previous);

  if (difference < 32768U) {
    return difference;
  }

  return (int32_t)difference - 65536L;
}

void readEncoders() {
  uint16_t leftRaw = (uint16_t)TIM3->CNT;
  uint16_t rightRaw = (uint16_t)TIM2->CNT;

  deltaLeft = counterDifference(leftRaw, previousLeftRaw);
  deltaRight = counterDifference(rightRaw, previousRightRaw);

  previousLeftRaw = leftRaw;
  previousRightRaw = rightRaw;

  leftTotal += deltaLeft;
  rightTotal += deltaRight;
}

uint8_t probeAddress(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission();
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

void initSensors() {
  Wire.setSCL(PB10);
  Wire.setSDA(PB11);
  Wire.begin();
  Wire.setClock(50000);

  uint8_t readyCount = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print("INIT ");
    Serial.println(names[i]);
    Serial.flush();

    pinMode(xshutPins[i], INPUT);
    delay(100);

    uint8_t initialProbe = probeAddress(0x29);

    if (initialProbe != 0) {
      Serial.print("FAIL: PROBE_0x29=");
      Serial.println(initialProbe);
      disableSensor(i);
      continue;
    }

    sensors[i].setBus(&Wire);
    sensors[i].setTimeout(100);

    bool initOK = sensors[i].init();
    uint8_t initStatus = sensors[i].last_status;

    Serial.print("INIT_OK=");
    Serial.print(initOK);
    Serial.print(" LAST_STATUS=");
    Serial.println(initStatus);

    if (!initOK || initStatus != 0) {
      Serial.println("FAIL: INIT");
      disableSensor(i);
      continue;
    }

    sensors[i].setAddress(addresses[i]);
    uint8_t addressStatus = sensors[i].last_status;

    delay(10);

    uint8_t newProbe = probeAddress(addresses[i]);

    if (addressStatus != 0 || newProbe != 0) {
      Serial.print("FAIL: ADDRESS WRITE=");
      Serial.print(addressStatus);
      Serial.print(" PROBE=");
      Serial.println(newProbe);
      disableSensor(i);
      continue;
    }

    bool timingOK =
      sensors[i].setMeasurementTimingBudget(33000);

    if (!timingOK || sensors[i].last_status != 0) {
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

  Serial.print("READY_COUNT=");
  Serial.println(readyCount);
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

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    disableSensor(i);
  }

  initHardwareEncoders();

  delay(1000);

  Serial.println("\nBOOT: HARDWARE ENCODER + 4 VL53L0X + LED");
  Serial.println("LEFT=TIM3 PA6/PA7 | RIGHT=TIM2 PA0/PA1");
  Serial.flush();

  initSensors();

  Serial.println("ARMING IN 5 SECONDS");
  Serial.flush();

  delay(5000);

  previousLeftRaw = (uint16_t)TIM3->CNT;
  previousRightRaw = (uint16_t)TIM2->CNT;

  leftTotal = 0;
  rightTotal = 0;
  deltaLeft = 0;
  deltaRight = 0;

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

  readEncoders();

  uint16_t leftMM = sideDistance(0, 1);
  uint16_t rightMM = sideDistance(2, 3);
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

    Serial.print(" | TOTAL_L=");
    Serial.print(leftTotal);
    Serial.print(" TOTAL_R=");
    Serial.print(rightTotal);

    Serial.print(" | dt=");
    Serial.print(elapsed);
    Serial.println("ms");
  }
}