
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <math.h>

// Da toi uu tu pid25.cpp:
// 1. Khong khoa cung toan bo xe neu 1 cam bien ben bi loi (chi can 2 cam bien truoc J9, J10 hoat dong)
// 2. Tang MAX_CPS len 2300 xung/s de xe chay nhanh, khong bi cham
// 3. Cho phep dung lenh CLEAR de xoa loi STALL / ENC_DIR ngay tren Serial

constexpr uint8_t COUNT = 4;
const uint32_t XSHUT_PINS[COUNT] = {PA2, PA3, PA4, PA5};
const uint8_t  VL53_ADDR[COUNT]  = {0x30, 0x31, 0x32, 0x33};
const char*    SENSOR_NAMES[COUNT] = {"J8", "J9", "J10", "J11"};
constexpr float SENSOR_OFFSETS_MM[COUNT] = {0.0f, -15.0f, -15.0f, 0.0f};

TwoWire SensorWire(PB11, PB10);
VL53L0X sensors[COUNT];

constexpr uint32_t LED_PIN = PC13;
constexpr uint32_t STBY_PIN = PB0;
constexpr uint32_t LEFT_IN1 = PB14, LEFT_IN2 = PB15, LEFT_PWM = PB9;
constexpr uint32_t RIGHT_IN1 = PB12, RIGHT_IN2 = PB13, RIGHT_PWM = PB8;
constexpr uint32_t LEFT_ENC_A = PA6, LEFT_ENC_B = PA7;
constexpr uint32_t RIGHT_ENC_A = PA0, RIGHT_ENC_B = PA1;

constexpr float MAX_CPS = 2300.0f; // Da tang tu 1200 -> 2300 xung/s
constexpr float MAX_TURN_CPS = 1100.0f;
constexpr float ACCEL_CPS2 = 4200.0f;
constexpr float DECEL_CPS2 = 6500.0f;
constexpr int   MAX_PWM = 140;
constexpr float RUN_MIN_PWM = 26.0f;

constexpr float LEFT_CPS_160 = 5690.5f, RIGHT_CPS_160 = 5617.0f;
constexpr float LEFT_START_PWM = 32.0f, RIGHT_START_PWM = 32.0f;
constexpr float WHEEL_KP = 0.011f, WHEEL_KI = 0.075f;

constexpr uint16_t TARGET_MM = 140;
constexpr uint16_t DEADBAND_MM = 12;
constexpr uint16_t MAX_DETECT_MM = 350; // Mo rong tam bam tu 200mm -> 350mm
constexpr float DIST_KP = 14.0f;
constexpr float STEER_KP = 5.5f;

bool sensorReady[COUNT] = {false};
uint16_t distMm[COUNT] = {8190, 8190, 8190, 8190};
bool distValid[COUNT] = {false};
uint8_t pollIdx = 0;

bool running = false;
String faultReason = "NONE";
uint16_t prevL = 0, prevR = 0;
int32_t totalL = 0, totalR = 0;
float velL = 0.0f, velR = 0.0f;
float intL = 0.0f, intR = 0.0f;
float smoothBody = 0.0f, smoothTurn = 0.0f;
int appliedL = 0, appliedR = 0;

uint32_t stallTimerL = 0, stallTimerR = 0;
int32_t stallBaseL = 0, stallBaseR = 0;
uint32_t lastCtrlMs = 0, lastPrintMs = 0;
String cmdBuf;

void holdLow(uint32_t p) { digitalWrite(p, LOW); pinMode(p, OUTPUT); }

void initEncoders() {
  __HAL_RCC_AFIO_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE(); __HAL_RCC_TIM3_CLK_ENABLE();
  uint32_t mapr = AFIO->MAPR;
  mapr &= ~(AFIO_MAPR_TIM2_REMAP | AFIO_MAPR_TIM3_REMAP | AFIO_MAPR_SWJ_CFG);
  AFIO->MAPR = mapr | AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

  pinMode(LEFT_ENC_A, INPUT_PULLUP); pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP); pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  for (TIM_TypeDef* t : {TIM2, TIM3}) {
    t->CR1 = 0; t->PSC = 0; t->ARR = 65535;
    t->CCMR1 = 1U | (8U << 4) | (1U << 8) | (8U << 12);
    t->SMCR = 3U; t->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    t->EGR = TIM_EGR_UG; t->CNT = 0; t->CR1 = TIM_CR1_CEN;
  }
  prevL = (uint16_t)TIM3->CNT; prevR = (uint16_t)TIM2->CNT;
}

int32_t diff16(uint16_t a, uint16_t b) {
  uint16_t d = (uint16_t)(a - b);
  return (d < 32768U) ? (int32_t)d : ((int32_t)d - 65536L);
}

void stopAll() {
  digitalWrite(STBY_PIN, LOW);
  analogWrite(LEFT_PWM, 0); analogWrite(RIGHT_PWM, 0);
  digitalWrite(LEFT_IN1, LOW); digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW); digitalWrite(RIGHT_IN2, LOW);
  appliedL = appliedR = 0; intL = intR = 0.0f;
  smoothBody = smoothTurn = 0.0f;
}

void setPwm(int l, int r) {
  l = constrain(l, -MAX_PWM, MAX_PWM); r = constrain(r, -MAX_PWM, MAX_PWM);
  appliedL = l; appliedR = r;
  if (l == 0 && r == 0) { stopAll(); return; }

  digitalWrite(LEFT_IN1, l > 0 ? LOW : HIGH); digitalWrite(LEFT_IN2, l > 0 ? HIGH : LOW);
  analogWrite(LEFT_PWM, abs(l));
  digitalWrite(RIGHT_IN1, r > 0 ? HIGH : LOW); digitalWrite(RIGHT_IN2, r > 0 ? LOW : HIGH);
  analogWrite(RIGHT_PWM, abs(r));
  digitalWrite(STBY_PIN, HIGH);
}

bool initSensorsGraceful() {
  for (uint8_t i = 0; i < COUNT; ++i) holdLow(XSHUT_PINS[i]);
  delay(60);
  SensorWire.begin(); SensorWire.setClock(100000);

  for (uint8_t i = 0; i < COUNT; ++i) {
    pinMode(XSHUT_PINS[i], INPUT);
    delay(60);
    SensorWire.beginTransmission(0x29);
    if (SensorWire.endTransmission() != 0) { holdLow(XSHUT_PINS[i]); continue; }
    sensors[i].setBus(&SensorWire);
    sensors[i].setTimeout(40);
    if (!sensors[i].init()) { holdLow(XSHUT_PINS[i]); continue; }
    sensors[i].setAddress(VL53_ADDR[i]);
    sensors[i].setMeasurementTimingBudget(20000);
    sensors[i].startContinuous();
    sensorReady[i] = true;
  }
  // Chi can it nhat 1 trong 2 mat truoc (J9 hoac J10) hoat dong la xe van chay duoc!
  return (sensorReady[1] || sensorReady[2]);
}

void pollSensor() {
  uint8_t i = pollIdx;
  pollIdx = (pollIdx + 1) % COUNT;
  if (!sensorReady[i]) return;

  SensorWire.beginTransmission(VL53_ADDR[i]);
  SensorWire.write((uint8_t)VL53L0X::RESULT_INTERRUPT_STATUS);
  if (SensorWire.endTransmission(false) != 0) return;
  if (SensorWire.requestFrom(VL53_ADDR[i], (uint8_t)1) != 1) return;
  if (!(SensorWire.read() & 0x07)) return;

  uint16_t raw = sensors[i].readRangeContinuousMillimeters();
  if (!sensors[i].timeoutOccurred() && raw > 10 && raw < 2000) {
    float mm = (float)raw + SENSOR_OFFSETS_MM[i];
    uint16_t val = mm > 0 ? (uint16_t)mm : 0;
    distMm[i] = distValid[i] ? (uint16_t)(0.65f * val + 0.35f * distMm[i]) : val;
    distValid[i] = true;
  } else {
    distMm[i] = 8190; distValid[i] = false;
  }
}

void processSerialLine(String line) {
  line.trim(); line.toUpperCase();
  if (line == "RUN") {
    if (faultReason != "NONE") { Serial.print("BLOCKED_BY_FAULT="); Serial.println(faultReason); return; }
    running = true; Serial.println("MODE=RUN");
  } else if (line == "STOP") {
    running = false; stopAll(); Serial.println("MODE=STOP");
  } else if (line == "CLEAR") {
    faultReason = "NONE"; stallTimerL = stallTimerR = 0;
    Serial.println("FAULT_CLEARED");
  } else if (line == "SENSOR") {
    for (int i = 0; i < COUNT; ++i) {
      Serial.print(SENSOR_NAMES[i]); Serial.print("_RDY="); Serial.print(sensorReady[i]);
      Serial.print("_MM="); Serial.print(distValid[i] ? distMm[i] : -1); Serial.print(" ");
    }
    Serial.println();
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);
  holdLow(STBY_PIN); holdLow(LEFT_IN1); holdLow(LEFT_IN2); holdLow(RIGHT_IN1); holdLow(RIGHT_IN2);
  pinMode(LEFT_PWM, OUTPUT); pinMode(RIGHT_PWM, OUTPUT);
  analogWriteResolution(8); stopAll();

  Serial.begin(115200);
  initEncoders();
  if (!initSensorsGraceful()) {
    faultReason = "NO_FRONT_SENSORS";
  }
  Serial.print("BOOT_DONE | FAULT="); Serial.println(faultReason);
  Serial.println("COMMANDS: RUN | STOP | CLEAR | SENSOR");
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') { if (cmdBuf.length() > 0) { processSerialLine(cmdBuf); cmdBuf = ""; } }
    else cmdBuf += c;
  }

  pollSensor();
  uint32_t now = millis();
  uint32_t dtMs = now - lastCtrlMs;
  if (dtMs < 10) return;
  lastCtrlMs = now;
  float dt = dtMs * 0.001f;

  uint16_t rawL = (uint16_t)TIM3->CNT, rawR = (uint16_t)TIM2->CNT;
  int32_t dL = -diff16(rawL, prevL), dR = diff16(rawR, prevR);
  prevL = rawL; prevR = rawR;
  totalL += dL; totalR += dR;
  velL += (dt / (0.045f + dt)) * ((dL / dt) - velL);
  velR += (dt / (0.045f + dt)) * ((dR / dt) - velR);

  if (!running || faultReason != "NONE") {
    stopAll();
    digitalWrite(LED_PIN, (faultReason != "NONE") ? ((now / 200) % 2) : HIGH);
    return;
  }

  // Tinh van toc bam muc tieu tu J9, J10
  float targetBody = 0.0f, targetTurn = 0.0f;
  bool hasL = distValid[1] && distMm[1] <= MAX_DETECT_MM;
  bool hasR = distValid[2] && distMm[2] <= MAX_DETECT_MM;

  if (hasL || hasR) {
    uint16_t frontMm = (hasL && hasR) ? min(distMm[1], distMm[2]) : (hasL ? distMm[1] : distMm[2]);
    int32_t errMm = (int32_t)frontMm - (int32_t)TARGET_MM;
    if (abs(errMm) > DEADBAND_MM) {
      targetBody = constrain(DIST_KP * (float)errMm, -MAX_CPS, MAX_CPS);
    }
    if (hasL && hasR) {
      float diffSide = (float)distMm[2] - (float)distMm[1];
      targetTurn = constrain(STEER_KP * diffSide, -MAX_TURN_CPS, MAX_TURN_CPS);
    }
  }

  auto slew = [](float cur, float tgt, float rate) {
    if (tgt > cur + rate) return cur + rate;
    if (tgt < cur - rate) return cur - rate;
    return tgt;
  };
  smoothBody = slew(smoothBody, targetBody, ((fabsf(targetBody) > fabsf(smoothBody)) ? ACCEL_CPS2 : DECEL_CPS2) * dt);
  smoothTurn = slew(smoothTurn, targetTurn, ACCEL_CPS2 * dt);

  float refL = smoothBody - smoothTurn;
  float refR = smoothBody + smoothTurn;

  auto wheelPi = [&](float ref, float meas, float &integ, float startPwm, float cps160) -> int {
    if (fabsf(ref) < 50.0f) { integ = 0.0f; return 0; }
    float dir = ref > 0 ? 1.0f : -1.0f;
    float aRef = fabsf(ref), aMeas = dir * meas, err = aRef - aMeas;
    float ff = RUN_MIN_PWM + (160.0f - RUN_MIN_PWM) * (aRef / cps160);
    if (aMeas < 80.0f && ff < startPwm) ff = startPwm;
    integ = constrain(integ + WHEEL_KI * err * dt, -25.0f, 25.0f);
    return (int)lroundf(dir * constrain(ff + WHEEL_KP * err + integ, RUN_MIN_PWM, (float)MAX_PWM));
  };

  setPwm(wheelPi(refL, velL, intL, LEFT_START_PWM, LEFT_CPS_160),
         wheelPi(refR, velR, intR, RIGHT_START_PWM, RIGHT_CPS_160));
  digitalWrite(LED_PIN, LOW);

  // Kiem tra ket banh (STALL) voi nguong hop ly 1200ms
  if (abs(appliedL) >= 48) {
    if (stallTimerL == 0) { stallTimerL = now; stallBaseL = totalL; }
    else if (abs(totalL - stallBaseL) >= 3) { stallTimerL = now; stallBaseL = totalL; }
    else if (now - stallTimerL > 1200) { faultReason = "STALL_L"; running = false; stopAll(); }
  } else stallTimerL = 0;

  if (abs(appliedR) >= 48) {
    if (stallTimerR == 0) { stallTimerR = now; stallBaseR = totalR; }
    else if (abs(totalR - stallBaseR) >= 3) { stallTimerR = now; stallBaseR = totalR; }
    else if (now - stallTimerR > 1200) { faultReason = "STALL_R"; running = false; stopAll(); }
  } else stallTimerR = 0;
}
