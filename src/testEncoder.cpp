#include <Arduino.h>

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
uint32_t lastPrint = 0;

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

void setup() {
  digitalWrite(STBY_PIN, LOW);
  pinMode(STBY_PIN, OUTPUT);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);

  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);

  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);

  analogWriteResolution(8);
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);

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

  Serial.println("TEST MOTOR + ENCODER");
  Serial.println("START AFTER 5 SECONDS");
  Serial.flush();

  delay(5000);

  noInterrupts();
  leftCount = 0;
  rightCount = 0;
  interrupts();

  digitalWrite(LEFT_IN1, LEFT_REVERSE ? LOW : HIGH);
  digitalWrite(LEFT_IN2, LEFT_REVERSE ? HIGH : LOW);

  digitalWrite(RIGHT_IN1, RIGHT_REVERSE ? LOW : HIGH);
  digitalWrite(RIGHT_IN2, RIGHT_REVERSE ? HIGH : LOW);

  analogWrite(LEFT_PWM, MOTOR_PWM);
  analogWrite(RIGHT_PWM, MOTOR_PWM);
  digitalWrite(STBY_PIN, HIGH);

  lastPrint = millis();
  Serial.println("MOTORS RUNNING");
}

void loop() {
  uint32_t now = millis();

  if (now - lastPrint < 500) {
    return;
  }

  uint32_t elapsed = now - lastPrint;
  lastPrint = now;

  noInterrupts();
  int32_t left = leftCount;
  int32_t right = rightCount;
  interrupts();

  int32_t deltaLeft = left - previousLeft;
  int32_t deltaRight = right - previousRight;

  previousLeft = left;
  previousRight = right;

  Serial.print("LEFT_TRAI total=");
  Serial.print(left);
  Serial.print(" delta=");
  Serial.print(deltaLeft);

  Serial.print(" | RIGHT_PHAI total=");
  Serial.print(right);
  Serial.print(" delta=");
  Serial.print(deltaRight);

  Serial.print(" | dt=");
  Serial.print(elapsed);
  Serial.println("ms");
}