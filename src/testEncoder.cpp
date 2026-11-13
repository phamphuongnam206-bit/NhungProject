#include <Arduino.h>
#include <math.h>

constexpr uint32_t STBY_PIN = PB0;

constexpr uint32_t LEFT_IN1 = PB14;
constexpr uint32_t LEFT_IN2 = PB15;
constexpr uint32_t LEFT_PWM = PB9;
constexpr uint32_t LEFT_ENC_A = PA6;
constexpr uint32_t LEFT_ENC_B = PA7;

constexpr uint32_t RIGHT_IN1 = PB12;
constexpr uint32_t RIGHT_IN2 = PB13;
constexpr uint32_t RIGHT_PWM = PB8;
constexpr uint32_t RIGHT_ENC_A = PA0;
constexpr uint32_t RIGHT_ENC_B = PA1;

constexpr uint8_t MOTOR_PWM = 160;
constexpr bool LEFT_REVERSE = true;
constexpr bool RIGHT_REVERSE = false;

constexpr uint32_t START_DELAY_MS = 5000;
constexpr uint32_t READ_INTERVAL_MS = 10;
constexpr uint32_t PRINT_INTERVAL_MS = 500;
constexpr uint32_t TEST_DURATION_MS = 10000;
constexpr uint32_t WARMUP_MS = 2000;
constexpr uint32_t COAST_TIMEOUT_MS = 3000;
constexpr uint32_t QUIET_INTERVAL_MS = 500;

constexpr uint32_t ENCODER_FILTER = 8;
constexpr uint32_t DIRECTION_MIN_COUNTS = 2;

constexpr uint8_t STATE_DONE = 0;
constexpr uint8_t STATE_RUNNING = 1;
constexpr uint8_t STATE_COAST = 2;

uint8_t testState = STATE_DONE;

uint16_t previousLeftCounter = 0;
uint16_t previousRightCounter = 0;

int32_t totalLeft = 0;
int32_t totalRight = 0;
int32_t previousPrintedLeft = 0;
int32_t previousPrintedRight = 0;

int32_t steadyStartLeft = 0;
int32_t steadyStartRight = 0;

int8_t referenceDirectionLeft = 0;
int8_t referenceDirectionRight = 0;

uint32_t oppositeSamplesLeft = 0;
uint32_t oppositeSamplesRight = 0;

uint32_t testStartMs = 0;
uint32_t lastReadMs = 0;
uint32_t lastPrintMs = 0;
uint32_t steadyStartMs = 0;
uint32_t maxReadGapMs = 0;

uint32_t coastStartMs = 0;
uint32_t lastMovementMs = 0;

bool steadyStarted = false;

uint32_t magnitude(int32_t value) {
  if (value < 0) {
    return (uint32_t)(-(int64_t)value);
  }
  return (uint32_t)value;
}

void outputLow(uint32_t pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

void stopMotors() {
  digitalWrite(STBY_PIN, LOW);

  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);

  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
}

void startMotors() {
  digitalWrite(LEFT_IN1, LEFT_REVERSE ? LOW : HIGH);
  digitalWrite(LEFT_IN2, LEFT_REVERSE ? HIGH : LOW);

  digitalWrite(RIGHT_IN1, RIGHT_REVERSE ? LOW : HIGH);
  digitalWrite(RIGHT_IN2, RIGHT_REVERSE ? HIGH : LOW);

  analogWrite(LEFT_PWM, MOTOR_PWM);
  analogWrite(RIGHT_PWM, MOTOR_PWM);

  digitalWrite(STBY_PIN, HIGH);
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
    1U |
    (ENCODER_FILTER << 4) |
    (1U << 8) |
    (ENCODER_FILTER << 12);

  timer->CCMR2 = 0;
  timer->SMCR = 3U;
  timer->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;

  timer->EGR = TIM_EGR_UG;
  timer->CNT = 0;
  timer->SR = 0;

  timer->CR1 = TIM_CR1_CEN;
}

bool timerConfigOK(TIM_TypeDef *timer) {
  uint32_t expectedCCMR1 =
    1U |
    (ENCODER_FILTER << 4) |
    (1U << 8) |
    (ENCODER_FILTER << 12);

  return
    (timer->CR1 & TIM_CR1_CEN) != 0 &&
    (timer->SMCR & 7U) == 3U &&
    timer->PSC == 0 &&
    timer->ARR == 65535 &&
    timer->DIER == 0 &&
    (timer->CCMR1 & 0xFFFFU) == expectedCCMR1 &&
    (timer->CCER & 0xFFFFU) ==
      (TIM_CCER_CC1E | TIM_CCER_CC2E);
}

bool initHardwareEncoders() {
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();

  HAL_NVIC_DisableIRQ(TIM2_IRQn);
  HAL_NVIC_DisableIRQ(TIM3_IRQn);

  __HAL_RCC_TIM2_FORCE_RESET();
  __HAL_RCC_TIM2_RELEASE_RESET();
  __HAL_RCC_TIM3_FORCE_RESET();
  __HAL_RCC_TIM3_RELEASE_RESET();

  HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
  HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);

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

  bool mappingOK =
    (AFIO->MAPR &
      (AFIO_MAPR_TIM2_REMAP | AFIO_MAPR_TIM3_REMAP)) == 0;

  return mappingOK &&
         timerConfigOK(TIM2) &&
         timerConfigOK(TIM3);
}

void resetMeasurements() {
  TIM2->CR1 &= ~TIM_CR1_CEN;
  TIM3->CR1 &= ~TIM_CR1_CEN;

  TIM2->CNT = 0;
  TIM3->CNT = 0;
  TIM2->SR = 0;
  TIM3->SR = 0;

  previousLeftCounter = 0;
  previousRightCounter = 0;

  totalLeft = 0;
  totalRight = 0;
  previousPrintedLeft = 0;
  previousPrintedRight = 0;

  steadyStartLeft = 0;
  steadyStartRight = 0;
  steadyStartMs = 0;
  steadyStarted = false;

  referenceDirectionLeft = 0;
  referenceDirectionRight = 0;
  oppositeSamplesLeft = 0;
  oppositeSamplesRight = 0;
  maxReadGapMs = 0;

  TIM2->CR1 |= TIM_CR1_CEN;
  TIM3->CR1 |= TIM_CR1_CEN;
}

int32_t counterDelta(uint16_t current, uint16_t previous) {
  uint16_t difference = (uint16_t)(current - previous);

  if (difference < 32768U) {
    return (int32_t)difference;
  }

  return (int32_t)difference - 65536L;
}

void checkDirection(
  int32_t delta,
  int8_t &referenceDirection,
  uint32_t &oppositeSamples
) {
  if (magnitude(delta) < DIRECTION_MIN_COUNTS) {
    return;
  }

  int8_t direction = delta > 0 ? 1 : -1;

  if (referenceDirection == 0) {
    referenceDirection = direction;
  } else if (direction != referenceDirection) {
    oppositeSamples++;
  }
}

void readRunningEncoders() {
  uint16_t leftCounter = (uint16_t)TIM3->CNT;
  uint16_t rightCounter = (uint16_t)TIM2->CNT;
  uint32_t now = millis();

  int32_t deltaLeft =
    counterDelta(leftCounter, previousLeftCounter);

  int32_t deltaRight =
    counterDelta(rightCounter, previousRightCounter);

  previousLeftCounter = leftCounter;
  previousRightCounter = rightCounter;

  uint32_t gap = now - lastReadMs;

  if (gap > maxReadGapMs) {
    maxReadGapMs = gap;
  }

  lastReadMs = now;

  totalLeft += deltaLeft;
  totalRight += deltaRight;

  if (!steadyStarted && now - testStartMs >= WARMUP_MS) {
    steadyStarted = true;
    steadyStartMs = now;
    steadyStartLeft = totalLeft;
    steadyStartRight = totalRight;
  } else if (steadyStarted) {
    checkDirection(
      deltaLeft,
      referenceDirectionLeft,
      oppositeSamplesLeft
    );

    checkDirection(
      deltaRight,
      referenceDirectionRight,
      oppositeSamplesRight
    );
  }
}

void printSample() {
  uint32_t dt = lastReadMs - lastPrintMs;

  int32_t windowLeft = totalLeft - previousPrintedLeft;
  int32_t windowRight = totalRight - previousPrintedRight;

  previousPrintedLeft = totalLeft;
  previousPrintedRight = totalRight;
  lastPrintMs = lastReadMs;

  Serial.print("LEFT_TRAI total=");
  Serial.print(magnitude(totalLeft));
  Serial.print(" delta=");
  Serial.print(magnitude(windowLeft));

  Serial.print(" | RIGHT_PHAI total=");
  Serial.print(magnitude(totalRight));
  Serial.print(" delta=");
  Serial.print(magnitude(windowRight));

  Serial.print(" | dt=");
  Serial.print(dt);
  Serial.println("ms");
}

void printRatesAndDifference(
  const char *leftLabel,
  const char *rightLabel,
  const char *differenceLabel,
  int32_t leftCount,
  int32_t rightCount,
  uint32_t durationMs
) {
  double left = magnitude(leftCount);
  double right = magnitude(rightCount);

  Serial.print(leftLabel);

  if (durationMs > 0) {
    Serial.println(left * 1000.0 / durationMs, 2);
  } else {
    Serial.println("N/A");
  }

  Serial.print(rightLabel);

  if (durationMs > 0) {
    Serial.println(right * 1000.0 / durationMs, 2);
  } else {
    Serial.println("N/A");
  }

  Serial.print(differenceLabel);

  double mean = (left + right) / 2.0;

  if (mean > 0 && durationMs > 0) {
    Serial.println(fabs(left - right) / mean * 100.0, 2);
  } else {
    Serial.println("N/A");
  }
}

void printResult() {
  uint32_t durationMs = lastReadMs - testStartMs;

  Serial.println();
  Serial.println("========== TEST RESULT ==========");

  Serial.print("PWM_L=");
  Serial.print(MOTOR_PWM);
  Serial.print(" PWM_R=");
  Serial.println(MOTOR_PWM);

  Serial.print("TOTAL_L=");
  Serial.println(magnitude(totalLeft));

  Serial.print("TOTAL_R=");
  Serial.println(magnitude(totalRight));

  Serial.print("DURATION_MS=");
  Serial.println(durationMs);

  printRatesAndDifference(
    "AVG_PULSE_S_L=",
    "AVG_PULSE_S_R=",
    "DIFFERENCE_PERCENT=",
    totalLeft,
    totalRight,
    durationMs
  );

  if (steadyStarted && lastReadMs > steadyStartMs) {
    uint32_t steadyDuration = lastReadMs - steadyStartMs;

    Serial.print("STEADY_DURATION_MS=");
    Serial.println(steadyDuration);

    printRatesAndDifference(
      "STEADY_PULSE_S_L=",
      "STEADY_PULSE_S_R=",
      "STEADY_DIFFERENCE_PERCENT=",
      totalLeft - steadyStartLeft,
      totalRight - steadyStartRight,
      steadyDuration
    );
  }

  Serial.print("OPPOSITE_SAMPLE_L=");
  Serial.println(oppositeSamplesLeft);

  Serial.print("OPPOSITE_SAMPLE_R=");
  Serial.println(oppositeSamplesRight);

  Serial.print("MAX_READ_GAP_MS=");
  Serial.println(maxReadGapMs);

  if (totalLeft == 0 || totalRight == 0) {
    Serial.println("CHECK: ONE OR BOTH ENCODERS HAVE ZERO NET COUNT");
  }

  if (oppositeSamplesLeft > 0 || oppositeSamplesRight > 0) {
    Serial.println("CHECK: OPPOSITE DIRECTION COUNTS DETECTED");
  }

  Serial.println("TEST COMPLETE");
  Serial.println("MOTOR DRIVE DISABLED");
  Serial.println("CHECKING COAST");
}

void monitorCoast() {
  uint32_t now = millis();

  if (now - lastReadMs >= READ_INTERVAL_MS) {
    uint16_t leftCounter = (uint16_t)TIM3->CNT;
    uint16_t rightCounter = (uint16_t)TIM2->CNT;

    int32_t deltaLeft =
      counterDelta(leftCounter, previousLeftCounter);

    int32_t deltaRight =
      counterDelta(rightCounter, previousRightCounter);

    previousLeftCounter = leftCounter;
    previousRightCounter = rightCounter;
    lastReadMs = millis();

    if (deltaLeft != 0 || deltaRight != 0) {
      lastMovementMs = lastReadMs;
    }
  }

  now = millis();

  if (now - lastMovementMs >= QUIET_INTERVAL_MS) {
    Serial.print("ENCODERS_QUIET_FOR_MS=");
    Serial.println(QUIET_INTERVAL_MS);
    testState = STATE_DONE;
  } else if (now - coastStartMs >= COAST_TIMEOUT_MS) {
    Serial.println("ENCODER_ACTIVITY_AFTER_COAST_TIMEOUT");
    Serial.println("MOTOR DRIVE REMAINS DISABLED");
    testState = STATE_DONE;
  }
}

void setup() {
  outputLow(STBY_PIN);
  outputLow(LEFT_IN1);
  outputLow(LEFT_IN2);
  outputLow(RIGHT_IN1);
  outputLow(RIGHT_IN2);
  outputLow(LEFT_PWM);
  outputLow(RIGHT_PWM);

  analogWriteResolution(8);
  stopMotors();

  Serial.setRx(PA10);
  Serial.setTx(PA9);
  Serial.begin(115200, SERIAL_8N1);

  bool encoderOK = initHardwareEncoders();

  Serial.println();
  Serial.println("BOOT: MOTOR + HARDWARE ENCODER TEST");
  Serial.println("LEFT: TIM3 PA6/PA7");
  Serial.println("RIGHT: TIM2 PA0/PA1");
  Serial.println("PWM: TIM4 LEFT=PB9 RIGHT=PB8");
  Serial.println("ENCODER MODE=3 COUNT=x4 ARR=65535");
  Serial.println("SWD ENABLED");

  Serial.print("FILTER=");
  Serial.println(ENCODER_FILTER);

  Serial.print("LEFT IN1=");
  Serial.print(LEFT_REVERSE ? "LOW" : "HIGH");
  Serial.print(" IN2=");
  Serial.println(LEFT_REVERSE ? "HIGH" : "LOW");

  Serial.print("RIGHT IN1=");
  Serial.print(RIGHT_REVERSE ? "LOW" : "HIGH");
  Serial.print(" IN2=");
  Serial.println(RIGHT_REVERSE ? "HIGH" : "LOW");

  Serial.print("PWM_L=");
  Serial.print(MOTOR_PWM);
  Serial.print(" PWM_R=");
  Serial.println(MOTOR_PWM);

  Serial.print("READ_INTERVAL_MS=");
  Serial.println(READ_INTERVAL_MS);

  Serial.print("PRINT_INTERVAL_MS=");
  Serial.println(PRINT_INTERVAL_MS);

  Serial.print("TEST_DURATION_MS=");
  Serial.println(TEST_DURATION_MS);

  if (!encoderOK) {
    stopMotors();
    Serial.println("ENCODER CONFIG ERROR");
    Serial.println("MOTOR DRIVE DISABLED");
    return;
  }

  Serial.print("START_AFTER_MS=");
  Serial.println(START_DELAY_MS);
  Serial.flush();

  delay(START_DELAY_MS);

  Serial.println("TEST START");
  Serial.flush();

  resetMeasurements();

  testStartMs = millis();
  lastReadMs = testStartMs;
  lastPrintMs = testStartMs;

  startMotors();
  testState = STATE_RUNNING;
}

void loop() {
  if (testState == STATE_DONE) {
    return;
  }

  if (testState == STATE_COAST) {
    monitorCoast();
    return;
  }

  uint32_t now = millis();

  if (now - testStartMs >= TEST_DURATION_MS) {
    stopMotors();
    readRunningEncoders();

    coastStartMs = lastReadMs;
    lastMovementMs = coastStartMs;
    testState = STATE_COAST;

    if (lastReadMs != lastPrintMs) {
      printSample();
    }

    printResult();
    return;
  }

  if (now - lastReadMs >= READ_INTERVAL_MS) {
    readRunningEncoders();

    if (lastReadMs - lastPrintMs >= PRINT_INTERVAL_MS) {
      printSample();
    }
  }
}