#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <math.h>

// The link-check environment must never energize the bridge, including braking.
#ifndef HYBRID_MOTOR_LOCK
#define HYBRID_MOTOR_LOCK 0
#endif
#if HYBRID_MOTOR_LOCK != 0 && HYBRID_MOTOR_LOCK != 1
#error "HYBRID_MOTOR_LOCK must be 0 or 1"
#endif
#ifndef HYBRID_PLANNER_INPUTS_VERIFIED
#define HYBRID_PLANNER_INPUTS_VERIFIED 0
#endif
#if HYBRID_PLANNER_ROUTE
#if !HYBRID_MOTOR_LOCK
#error "Planner route integration is diagnostic only and requires motor lock"
#endif
#include "planner_route_view.h"
#include "planner_runtime.h"
#include "planner_snapshot.h"
#include "map_grid.h"
PlannerRouteTarget plannerRouteTarget{};
#endif

// ============================================================================
// LUOT 2 V26 - TUYEN 180 x 120 CM - OM CUA TIEN
// ============================================================================

// --- CHAN PHAN CUNG ---
constexpr uint32_t LED_PIN = PC13, STBY_PIN = PB0;
constexpr uint32_t LEFT_IN1 = PB14, LEFT_IN2 = PB15, LEFT_PWM = PB9;
constexpr uint32_t RIGHT_IN1 = PB12, RIGHT_IN2 = PB13, RIGHT_PWM = PB8;
constexpr uint32_t LEFT_ENC_A = PA6, LEFT_ENC_B = PA7;
constexpr uint32_t RIGHT_ENC_A = PA0, RIGHT_ENC_B = PA1;

constexpr uint32_t BT_BAUD = 38400;
constexpr uint32_t BT_COMMAND_IDLE_MS = 60;
constexpr uint8_t SENSOR_COUNT = 4;
// [0]=Trai, [1]=Truoc-Trai, [2]=Truoc-Phai, [3]=Phai
const uint32_t XSHUT_PINS[SENSOR_COUNT] = {PA2, PA3, PA4, PA5};
const uint8_t  VL53_ADDR[SENSOR_COUNT]  = {0x30, 0x31, 0x32, 0x33};
TwoWire SensorWire(PB11, PB10);
VL53L0X sensors[SENSOR_COUNT];



// Sua theo quang duong do thuc: xung encoder cho moi cm lan banh.
#ifndef MAP_COUNTS_CM_L
#define MAP_COUNTS_CM_L 45.0f
#endif
#ifndef MAP_COUNTS_CM_R
#define MAP_COUNTS_CM_R 45.0f
#endif
#ifndef MAP_TRACK_CM
#define MAP_TRACK_CM 9.0f
#endif
static_assert(MAP_COUNTS_CM_L > 0 && MAP_COUNTS_CM_R > 0 && MAP_TRACK_CM > 0,
              "Encoder scales and wheel spacing must be positive");
constexpr int32_t LEFT_ENC_DIR = -1, RIGHT_ENC_DIR = 1;
constexpr float RAD = 0.01745329252f;
constexpr float FAST_CM_S = 17.5f, SLOW_CM_S = 11.5f;
constexpr float TURN_CM_S = 13.0f, MIN_WHEEL_CM_S = 5.5f;
constexpr float YAW_SIGN = -1.0f;
constexpr float BRAKE_DECEL_CM_S2 = 35.0f;
constexpr float REACTION_TIME_S = 0.18f;
constexpr float OBSTACLE_MARGIN_CM = 1.5f;
constexpr uint16_t FRONT_HARD_STOP_MM = 120;
constexpr float START_STRAIGHT_CM = 15.0f;
constexpr float MIN_LAUNCH_HEADING_ERROR_RAD = 1.5f * RAD;
constexpr float CURVE_CHANGE_PER_S = 0.70f;
constexpr float MAX_TURN_FRACTION = 0.72f;
constexpr float MAX_CURVE = 2.0f * MAX_TURN_FRACTION / MAP_TRACK_CM;
constexpr uint16_t DIAG_AVOID_MM = 90;
constexpr int MAX_PWM = 140, START_PWM = 78, RUN_PWM = 42;
constexpr uint32_t STALL_START_MS = 700, STALL_MOVING_MS = 400;
constexpr uint32_t WHEEL_FAULT_BRAKE_MS = 120;
constexpr uint32_t SENSOR_STALE_MS = 100;
constexpr uint32_t SENSOR_RECOVERY_LIMIT_MS = 500;
constexpr uint32_t SENSOR_REINIT_DELAY_MS = 250;
constexpr uint8_t SENSOR_REINIT_LIMIT = 2;
constexpr uint8_t SENSOR_RECOVERY_FRAMES = 2;
constexpr uint32_t SENSOR_SLOW_AGE_MS = 60;
constexpr uint32_t CLEAR_BEFORE_RESTART_MS = 100;
constexpr uint32_t WATCHDOG_TIMEOUT_US = 4000000UL;
constexpr uint32_t CONTROL_LOOP_TIMEOUT_MS = 150;
constexpr uint32_t WATCHDOG_RELOAD = static_cast<uint32_t>(
    (40000ULL * WATCHDOG_TIMEOUT_US / 1000000ULL / 64ULL) - 1ULL);
static_assert(WATCHDOG_RELOAD <= 0x0FFFU, "IWDG reload must fit STM32F1");
constexpr float FRONT_SLOW_RESERVE_CM = 8.0f;
constexpr float AVOID_EARLY_CM = 32.0f;
constexpr float MIN_ROLLING_CLEARANCE_CM = 11.0f;
constexpr uint32_t FRAME_SKEW_MS = 45;
constexpr uint16_t RANGE_OPEN_MM = 2000;

struct Point { float x, y; };
struct Pose { Point p; float yaw; };  // 0 = len map (+Y); duong = quay phai.
// Toa do sa hinh (cm), goc o goc trai duoi. Khung trong cua bang den:
constexpr float MAP_LEFT = 5.0f, MAP_RIGHT = 175.0f;
constexpr float MAP_BOTTOM = 5.0f, MAP_TOP = 115.0f;
constexpr float START_X_MIN = 5.0f, START_X_MAX = 55.0f;
constexpr float START_LINE_Y = 7.0f;
constexpr float FINISH_Y = 7.0f, FINISH_X_MIN = 125.0f, FINISH_X_MAX = 175.0f;
constexpr Point MAP_START{33.0f, 7.5f};
constexpr Point FINISH_APPROACH{152.5f, 7.5f};
constexpr float HALF_LENGTH = 6.8f, HALF_WIDTH = 5.8f;
// Diem ngam ao sau vach de giu huong. Xe phanh ngay khi ca than qua vach.
constexpr Point FINISH_EXTENSION{FINISH_APPROACH.x, -12.0f};

constexpr Point ROUTE[] = {
  MAP_START, {36.5f, 28.0f}, {39.5f, 54.0f}, {42.0f, 82.0f},
  {48.0f, 91.0f}, {88.0f, 89.0f}, {112.0f, 86.0f}, {116.0f, 68.0f},
  // Bat dau om vao huong dich som, khi than xe con cach bien duoi.
  {131.0f, 52.0f}, {144.0f, 26.0f}, {152.5f, 17.0f},
  FINISH_APPROACH, FINISH_EXTENSION,
};
constexpr uint8_t ROUTE_COUNT = sizeof(ROUTE) / sizeof(ROUTE[0]);

struct Box { float x, y, hx, hy; };
constexpr Box MAP_BOXES[] = {
  {90, 30, 30, 30},  // o cam di tat
  {10,20,5,5}, {55,30,5,5}, {25,50,5,5}, {55,50,5,5},
  {10,65,5,5}, {10,85,5,5}, {30,95,5,5}, {40,110,5,5},
  {65,75,5,5}, {75,70,5,5}, {90,100,5,5}, {95,70,5,5}, {115,110,5,5},
};

// J8, J9, J10, J11. Vi tri tinh tu tam xe, right > 0 ben phai.
// J9/J10 dat lui 1.5 cm so voi mep dau xe, nen goc do bat dau o forward=4.5.
struct Beam { float right, forward, angleDeg; };
constexpr Beam BEAMS[SENSOR_COUNT] = {
  {-2.5f, 4.5f, -35.43f}, {-2.5f, 4.5f, 0}, {2.5f, 4.5f, 0}, {2.5f, 4.5f, 35.43f},
};

enum class RunMode { WAIT_SENSORS, AUTO_RUN, LOCKED, FINISHED };
enum class Fault : uint8_t { NONE, MPU, LEFT_WHEEL, RIGHT_WHEEL, SENSOR, OBSTACLE, CONTROL };
RunMode mode = RunMode::WAIT_SENSORS;
Fault fault = Fault::NONE;
const char *status = "START";
Pose pose{MAP_START, 0};
bool leavingStartLine = true;

float pathS[ROUTE_COUNT] = {};
float progress = 0, lastCurve = 0, commandedSpeed = 0;
uint8_t segment = 0;
uint32_t autoStartMs = 0, lastControlMs = 0, lastPlanMs = 0;
uint32_t stoppedAt = 0;
uint32_t bootResetFlags = 0;

uint16_t prevRawL = 0, prevRawR = 0;
int32_t totalLeft = 0, totalRight = 0;
uint32_t lastEncoderSampleMs = 0;
constexpr uint8_t MPU_ADDR = 0x68;
bool gyroReady = false;
float gyroBias = 0, currentYawDeg = 0, yawRateDps = 0;
uint32_t lastGyroMs = 0, lastGyroGoodMs = 0;

bool vlReady[SENSOR_COUNT] = {};
struct Reading {
  uint16_t mm;
  uint32_t at;
  Pose origin;
};
Reading pendingRange[SENSOR_COUNT] = {}, frame[SENSOR_COUNT] = {};
constexpr uint8_t ALL_RANGES_MASK = (1U << SENSOR_COUNT) - 1U;
uint8_t pendingMask = 0, pollIndex = 0, receivedRangeMask = 0;
uint8_t rangeFaultMask = 0, rangeRestarts = 0;
uint16_t rangeReadErrors[SENSOR_COUNT] = {};
uint32_t lastRangePollUs = 0;
bool frameReady = false;
uint32_t frameAt = 0;

#if HYBRID_PLANNER_ROUTE
// The diagnostic build exercises the real sensor/timestamp path while keeping
// the final pose/map authorization explicit. A route is not armed from a
// guessed START pose or from the placeholder map.
PlannerSnapshotLimits plannerSnapshotLimits{100, SENSOR_STALE_MS, FRAME_SKEW_MS};
bool plannerMapVerified = false;
bool plannerPoseVerified = false;
bool plannerRuntimeReady = false;
bool plannerGoalReached = false;
uint32_t plannerLastPollMs = 0;
OneRunStatus plannerStatus = ONE_RUN_STOPPED;
#endif

void holdLow(uint32_t pin) { digitalWrite(pin, LOW); pinMode(pin, OUTPUT); }

// --- ENCODER ---
void configureEncoderTimer(TIM_TypeDef *t) {
  t->CR1 = 0; t->PSC = 0; t->ARR = 65535;
  t->CCMR1 = 1U | (8U << 4) | (1U << 8) | (8U << 12);
  t->SMCR = 3U; t->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
  t->EGR = TIM_EGR_UG; t->CNT = 0; t->CR1 = TIM_CR1_CEN;
}

void initHardwareEncoders() {
  __HAL_RCC_AFIO_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE(); __HAL_RCC_TIM3_CLK_ENABLE();
  HAL_NVIC_DisableIRQ(TIM2_IRQn); HAL_NVIC_DisableIRQ(TIM3_IRQn);
  __HAL_RCC_TIM2_FORCE_RESET(); __HAL_RCC_TIM2_RELEASE_RESET();
  __HAL_RCC_TIM3_FORCE_RESET(); __HAL_RCC_TIM3_RELEASE_RESET();
  uint32_t mapr = AFIO->MAPR;
  mapr &= ~(AFIO_MAPR_TIM2_REMAP | AFIO_MAPR_TIM3_REMAP | AFIO_MAPR_SWJ_CFG);
  AFIO->MAPR = mapr | AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
  pinMode(LEFT_ENC_A, INPUT_PULLUP); pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP); pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  configureEncoderTimer(TIM2); configureEncoderTimer(TIM3);
  prevRawL = (uint16_t)TIM3->CNT; prevRawR = (uint16_t)TIM2->CNT;
}

int32_t counterDiff(uint16_t cur, uint16_t prev) {
  uint16_t d = (uint16_t)(cur - prev);
  return (d < 32768U) ? (int32_t)d : ((int32_t)d - 65536L);
}

void updateEncoders(int32_t &dL, int32_t &dR) {
  uint16_t rawL = (uint16_t)TIM3->CNT, rawR = (uint16_t)TIM2->CNT;
  dL = LEFT_ENC_DIR * counterDiff(rawL, prevRawL);
  dR = RIGHT_ENC_DIR * counterDiff(rawR, prevRawR);
  prevRawL = rawL; prevRawR = rawR;
  totalLeft += dL; totalRight += dR;
}


struct Wheel {
  float cps = 0, iTerm = 0, pwm = 0;
  static constexpr uint8_t SPEED_SAMPLES = 4;
  int32_t speedPulses[SPEED_SAMPLES] = {};
  float speedSeconds[SPEED_SAMPLES] = {};
  uint8_t speedIndex = 0;
  uint32_t pulseAt = 0;
  bool watching = false, moving = false;
  int32_t wrongPulses = 0;

  void reset() { *this = Wheel{}; }

  bool observe(int32_t pulses, uint32_t now, float dt, float countsPerCm) {
    speedPulses[speedIndex] = pulses;
    speedSeconds[speedIndex] = dt;
    speedIndex = (speedIndex + 1) % SPEED_SAMPLES;
    int32_t sumPulses = 0;
    float sumSeconds = 0;
    for (uint8_t i = 0; i < SPEED_SAMPLES; ++i) {
      sumPulses += speedPulses[i];
      sumSeconds += speedSeconds[i];
    }
    cps = sumSeconds > 0 ? sumPulses / sumSeconds : 0;
    if (pwm < RUN_PWM - 0.5f) {
      watching = false;
      moving = false;
      wrongPulses = 0;
      return true;
    }
    if (!watching) { pulseAt = now; watching = true; }
    if (cps >= 0.35f * MIN_WHEEL_CM_S * countsPerCm) {
      pulseAt = now;
      moving = true;
      wrongPulses = 0;
    }
    if (pulses < 0) wrongPulses -= pulses;
    const uint32_t stallLimit = moving ? STALL_MOVING_MS : STALL_START_MS;
    return wrongPulses < 25 && now - pulseAt <= stallLimit;
  }

  int drive(float refCps, float cpsAt160, float dt) {
    const float error = refCps - cps;
    const float candidateI = constrain(iTerm + 0.033f * error * dt, -20.0f, 28.0f);
    const float base = RUN_PWM + (160 - RUN_PWM) * refCps / cpsAt160 + 0.010f * error;
    const int floor = moving ? RUN_PWM : START_PWM;
    const float request = base + candidateI;
    if ((request >= floor || error > 0) && (request <= MAX_PWM || error < 0)) iTerm = candidateI;
    const float target = constrain(base + iTerm, float(floor), float(MAX_PWM));
    pwm += constrain(target - pwm, -650.0f * dt, 480.0f * dt);
    return int(lroundf(pwm));
  }
};
Wheel wheelL, wheelR;

struct StopReport {
  const char *reason = nullptr;
  Point position{};
  float actualSpeed = 0;
  float targetSpeed = 0;
  int pwmL = 0, pwmR = 0;
  bool hasRanges = false;
  uint16_t ranges[SENSOR_COUNT] = {};
};
StopReport lastStop;

void initWatchdog() {
  // STM32F1 IWDG: /64, reload 2499 gives about 4 s at the nominal 40 kHz LSI.
  IWDG->KR = 0xCCCCU;
  IWDG->KR = 0x5555U;
  IWDG->PR = 4U;
  IWDG->RLR = WATCHDOG_RELOAD;
  while (IWDG->SR != 0U) {}
  IWDG->KR = 0xAAAAU;
}

void reloadWatchdog() {
  IWDG->KR = 0xAAAAU;
}

float measuredForwardSpeed() {
  return 0.5f * (wheelL.cps / MAP_COUNTS_CM_L + wheelR.cps / MAP_COUNTS_CM_R);
}

void rememberStop(const char *reason) {
  lastStop.reason = reason;
  lastStop.position = pose.p;
  lastStop.actualSpeed = measuredForwardSpeed();
  lastStop.targetSpeed = commandedSpeed;
  lastStop.pwmL = lroundf(wheelL.pwm);
  lastStop.pwmR = lroundf(wheelR.pwm);
  lastStop.hasRanges = frameReady;
  if (frameReady) {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) lastStop.ranges[i] = frame[i].mm;
  }
}

void stopMotors(bool brake = false) {
#if HYBRID_MOTOR_LOCK
  brake = false;
#endif
  if (!brake) digitalWrite(STBY_PIN, LOW);
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
  const PinStatus input = brake ? HIGH : LOW;
  digitalWrite(LEFT_IN1, input); digitalWrite(LEFT_IN2, input);
  digitalWrite(RIGHT_IN1, input); digitalWrite(RIGHT_IN2, input);
  if (brake) digitalWrite(STBY_PIN, HIGH);
  wheelL.reset();
  wheelR.reset();
  commandedSpeed = 0;
  lastCurve = 0;
}

void halt(const char *reason, Fault code) {
  rememberStop(reason);
  stopMotors(mode == RunMode::AUTO_RUN);
  status = reason;
  mode = code == Fault::NONE ? RunMode::FINISHED : RunMode::LOCKED;
  fault = code;
  stoppedAt = millis();
  Serial.println(reason);
}

void driveWheels(float leftCmS, float rightCmS, float dt) {
#if HYBRID_MOTOR_LOCK
  (void)leftCmS; (void)rightCmS; (void)dt;
  digitalWrite(STBY_PIN, LOW);
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
#else
  const int pwmL = wheelL.drive(leftCmS * MAP_COUNTS_CM_L, 5666.0f, dt);
  const int pwmR = wheelR.drive(rightCmS * MAP_COUNTS_CM_R, 5563.0f, dt);
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  analogWrite(LEFT_PWM, pwmL);
  analogWrite(RIGHT_PWM, pwmR);
  digitalWrite(STBY_PIN, HIGH);
#endif
}

void driveForward(float speed, float curve, float dt) {
  commandedSpeed += constrain(speed - commandedSpeed, -65.0f * dt, 62.0f * dt);
  const float turn = curve * MAP_TRACK_CM * 0.5f;
  driveWheels(commandedSpeed * (1.0f + turn), commandedSpeed * (1.0f - turn), dt);
}

bool wheelsSettled() {
  return fabsf(wheelL.cps) < 4.0f * MAP_COUNTS_CM_L &&
         fabsf(wheelR.cps) < 4.0f * MAP_COUNTS_CM_R;
}

// --- MPU6050 ---
bool writeMpuReg(uint8_t r, uint8_t v) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(r); Wire.write(v);
  return Wire.endTransmission() == 0;
}

bool readGyroRawZ(int16_t &rawZ) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x47);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)2) != 2) return false;
  const int high = Wire.read(), low = Wire.read();
  if (high < 0 || low < 0) return false;
  rawZ = int16_t((uint16_t(high) << 8) | low);
  return true;
}

bool initGyro() {
  Wire.setSDA(PB7); Wire.setSCL(PB6); Wire.begin(); Wire.setClock(100000);
  if (!writeMpuReg(0x6B, 0x01) || !writeMpuReg(0x1A, 0x03) ||
      !writeMpuReg(0x19, 0x09) || !writeMpuReg(0x1B, 0x08)) return false;
  delay(60);
  int32_t sum = 0;
  for (int i = 0; i < 50; ++i) {
    int16_t gz;
    if (!readGyroRawZ(gz)) return false;
    sum += gz;
    delay(6);
  }
  gyroBias = sum / 50.0f;
  currentYawDeg = 0.0f;
  lastGyroGoodMs = lastGyroMs = millis();
  return true;
}

void updateGyro(uint32_t now) {
  if (!gyroReady) return;
  uint32_t dtMs = now - lastGyroMs;
  if (dtMs < 5) return;
  int16_t rawZ;
  if (readGyroRawZ(rawZ)) {
    lastGyroGoodMs = now;
    yawRateDps = (rawZ - gyroBias) / 65.5f;
    if (fabsf(yawRateDps) < 0.30f) yawRateDps = 0.0f;
    currentYawDeg += yawRateDps * (dtMs * 0.001f);
  }
  lastGyroMs = now;
}

bool probeRangeAddress(uint8_t address) {
  SensorWire.beginTransmission(address);
  return SensorWire.endTransmission() == 0;
}

bool initArcSensors(bool restartBus = false) {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    holdLow(XSHUT_PINS[i]);
    vlReady[i] = false;
  }
  delay(40);
  if (restartBus) SensorWire.end();
  SensorWire.begin();
  SensorWire.setClock(100000);
  pendingMask = receivedRangeMask = pollIndex = 0;
  frameReady = false;
  frameAt = 0;

  uint8_t missingAtInit = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    pinMode(XSHUT_PINS[i], INPUT);
    delay(40);
    // XSHUT puts the chip back at 0x29. Also reset the driver's saved address.
    sensors[i] = VL53L0X{};
    sensors[i].setBus(&SensorWire);
    sensors[i].setTimeout(40);
    if (probeRangeAddress(0x29) && sensors[i].init()) {
      sensors[i].setAddress(VL53_ADDR[i]);
      vlReady[i] = sensors[i].last_status == 0 && probeRangeAddress(VL53_ADDR[i]) &&
                   sensors[i].setMeasurementTimingBudget(20000);
    }
    if (!vlReady[i]) {
      holdLow(XSHUT_PINS[i]);
      missingAtInit |= 1U << i;
    }
  }
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (!vlReady[i]) continue;
    sensors[i].startContinuous();
    if (sensors[i].last_status != 0 || !probeRangeAddress(VL53_ADDR[i])) {
      vlReady[i] = false;
      missingAtInit |= 1U << i;
    }
  }
  if (missingAtInit) rangeFaultMask = missingAtInit;
  return missingAtInit == 0;
}

uint8_t staleRangeMask(uint32_t now) {
  uint8_t mask = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (!(receivedRangeMask & (1U << i)) || now - pendingRange[i].at > SENSOR_STALE_MS) {
      mask |= 1U << i;
    }
  }
  return mask;
}

void noteRangeReadError(uint8_t i) {
  if (rangeReadErrors[i] < 65535U) ++rangeReadErrors[i];
}

bool readRangeRegisters(uint8_t i, uint8_t reg, uint8_t *data, uint8_t count) {
  SensorWire.beginTransmission(VL53_ADDR[i]);
  SensorWire.write(reg);
  if (SensorWire.endTransmission(false) != 0 ||
      SensorWire.requestFrom(VL53_ADDR[i], count) != count) return false;
  for (uint8_t j = 0; j < count; ++j) {
    const int value = SensorWire.read();
    if (value < 0) return false;
    data[j] = static_cast<uint8_t>(value);
  }
  return true;
}


// Khong cho goi ham cho du lieu cua thu vien trong vong dieu khien.
void pollRange() {
  const uint32_t nowUs = micros();
  if (nowUs - lastRangePollUs < 1000U) return;
  lastRangePollUs = nowUs;
  const uint8_t i = pollIndex;
  pollIndex = (pollIndex + 1) % SENSOR_COUNT;
  uint8_t ready = 0;
  if (!readRangeRegisters(i, VL53L0X::RESULT_INTERRUPT_STATUS, &ready, 1)) {
    noteRangeReadError(i);
    return;
  }
  if (!(ready & 7)) return;

  uint8_t raw[2] = {};
  if (!readRangeRegisters(i, VL53L0X::RESULT_RANGE_STATUS + 10, raw, 2)) {
    noteRangeReadError(i);
    return;
  }
  SensorWire.beginTransmission(VL53_ADDR[i]);
  SensorWire.write(uint8_t(VL53L0X::SYSTEM_INTERRUPT_CLEAR));
  SensorWire.write(uint8_t(1));
  const bool cleared = SensorWire.endTransmission() == 0;
  const uint16_t mm = (uint16_t(raw[0]) << 8) | raw[1];
  if (!cleared || mm == 65535) {
    noteRangeReadError(i);
    return;
  }

  const uint32_t now = millis();
  const uint16_t range = min(mm, RANGE_OPEN_MM);
  pendingRange[i] = {range, now, pose};
  receivedRangeMask |= 1U << i;
  pendingMask |= 1U << i;
  if (pendingMask != ALL_RANGES_MASK) return;
  for (uint8_t j = 0; j < SENSOR_COUNT; ++j) {
    if (now - pendingRange[j].at > FRAME_SKEW_MS) pendingMask &= ~(1U << j);
  }
  if (pendingMask != ALL_RANGES_MASK) return;
  for (uint8_t j = 0; j < SENSOR_COUNT; ++j) frame[j] = pendingRange[j];
  frameAt = now;
  frameReady = true;
  pendingMask = 0;
}

bool immediateFrontStop(uint32_t now) {
  for (uint8_t i = 1; i <= 2; ++i) {
    if ((receivedRangeMask & (1U << i)) && now - pendingRange[i].at <= SENSOR_STALE_MS &&
        pendingRange[i].mm <= FRONT_HARD_STOP_MM) return true;
  }
  return false;
}

float square(float x) { return x * x; }

void updatePose(int32_t l, int32_t r) {
  const float ds = (l / MAP_COUNTS_CM_L + r / MAP_COUNTS_CM_R) * 0.5f;
  const float yaw = YAW_SIGN * currentYawDeg * RAD;
  const float avg = (pose.yaw + yaw) * 0.5f;
  pose.p.x += ds * sinf(avg);
  pose.p.y += ds * cosf(avg);
  pose.yaw = yaw;
}

#if HYBRID_PLANNER_ROUTE
void initPlannerDiagnosticConfig() {
  plannerMapVerified = false;
  plannerPoseVerified = false;
#if HYBRID_PLANNER_INPUTS_VERIFIED
  plannerMapVerified = true;
  plannerPoseVerified = true;
#endif
  plannerRuntimeReady = false;
  plannerGoalReached = false;
  plannerStatus = ONE_RUN_STOPPED;
}

void fillPlannerConfig(OneRunConfig &config) {
  config = {};
  config.planner.search = {4U, 2U};
  config.planner.min_los_clearance_cells = 1U;
  config.planner.footprint = {HALF_LENGTH / 5.0f, HALF_WIDTH / 5.0f};
  config.navigation.follower = {5.0f, 20.0f, 0.35f, 5.0f, 20.0f,
                                3.0f, 8.0f, 20.0f, 2.0f};
  config.navigation.wheels = {10.0f, 30.0f, 40.0f, 80.0f};
  config.navigation.safety = {100U, SENSOR_STALE_MS, FRONT_HARD_STOP_MM};
  config.sensors.min_range_mm = 30U;
  config.sensors.max_range_mm = RANGE_OPEN_MM;
  config.sensors.confirmations_required = 2U;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    config.sensors.mounts[i] = {
      BEAMS[i].forward,
      -BEAMS[i].right,
      BEAMS[i].angleDeg * RAD,
    };
  }
  config.replan = {100U, 2U, 2U};
  // The finish cell is a fixture only until the official map/rule is supplied.
  config.goal = {1U, 30U};
  config.inflation_cells = 1U;
}

bool beginPlannerRuntime() {
  MapGrid baseMap{};
  OneRunConfig config{};
  (void)MapGrid_InitBase(&baseMap);
  fillPlannerConfig(config);
  if (!PlannerRuntime_Begin(&baseMap, &config, true, wheelsSettled())) return false;
  plannerLastPollMs = millis() - 30U;
  plannerRuntimeReady = true;
  return true;
}

void pollPlannerRuntime(uint32_t now, float dt, bool emergencyStop) {
  if (!plannerMapVerified || !plannerPoseVerified) return;
  if (!plannerRuntimeReady) {
    if (!beginPlannerRuntime()) {
      plannerStatus = ONE_RUN_STOPPED;
      return;
    }
  }
  if (!frameReady || now - plannerLastPollMs < 30U) return;

  PlannerSnapshotSource source{};
  source.x_cm = pose.p.x;
  source.y_cm = pose.p.y;
  source.hybrid_yaw_rad = pose.yaw;
  source.speed_cm_s = measuredForwardSpeed();
  source.dt_s = dt;
  source.now_ms = now;
  source.encoder_ms = lastEncoderSampleMs;
  source.gyro_ms = lastGyroGoodMs;
  source.pose_verified = plannerPoseVerified;
  source.encoder_valid = true;
  source.gyro_valid = gyroReady && now - lastGyroGoodMs <= 100U;
  source.emergency_stop = emergencyStop;
  source.ranges_have_origins = true;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    source.ranges[i] = {frame[i].mm, frame[i].at,
                        vlReady[i] && frame[i].mm != 0U,
                        frame[i].origin.p.x, frame[i].origin.p.y,
                        frame[i].origin.yaw};
  }
  OneRunInput snapshot{};
  plannerLastPollMs = now;
  if (!PlannerSnapshot_Build(&source, &plannerSnapshotLimits, &snapshot)) {
    PlannerRuntime_Invalidate();
    plannerRuntimeReady = false;
    plannerStatus = ONE_RUN_STOPPED;
    return;
  }
  plannerStatus = PlannerRuntime_Poll(&snapshot, plannerPoseVerified, wheelsSettled());
  if (plannerStatus == ONE_RUN_COMPLETE) plannerGoalReached = true;
  if (plannerStatus != ONE_RUN_OK && plannerStatus != ONE_RUN_COMPLETE) {
    PlannerRuntime_Invalidate();
    plannerRuntimeReady = false;
  }
}
#endif

void initRoute() {
  for (uint8_t i = 1; i < ROUTE_COUNT; ++i) {
    pathS[i] = pathS[i - 1] + hypotf(ROUTE[i].x - ROUTE[i - 1].x,
                                    ROUTE[i].y - ROUTE[i - 1].y);
  }
}

Point routeAt(float s) {
  for (uint8_t i = 0; i < ROUTE_COUNT - 1; ++i) {
    if (s <= pathS[i + 1]) {
      const float t = (s - pathS[i]) / (pathS[i + 1] - pathS[i]);
      return {ROUTE[i].x + t * (ROUTE[i+1].x - ROUTE[i].x),
              ROUTE[i].y + t * (ROUTE[i+1].y - ROUTE[i].y)};
    }
  }
  return ROUTE[ROUTE_COUNT - 1];
}

void updateProgress() {
  float bestD2 = 1e9f;
  float bestS = progress;
  const uint8_t end = min(int(segment) + 2, int(ROUTE_COUNT) - 2);
  for (uint8_t i = segment; i <= end; ++i) {
    const Point &a = ROUTE[i], &b = ROUTE[i+1];
    const float dx = b.x - a.x, dy = b.y - a.y;
    const float t = constrain(((pose.p.x-a.x)*dx + (pose.p.y-a.y)*dy) /
                              (dx*dx + dy*dy), 0.0f, 1.0f);
    const float s = pathS[i] + t * (pathS[i+1] - pathS[i]);
    const float d2 = square(pose.p.x-a.x-t*dx) + square(pose.p.y-a.y-t*dy);
    if (s >= progress - 1.0f && s <= progress + 24.0f && d2 < bestD2) {
      bestD2 = d2;
      bestS = s;
    }
  }
  progress = fmaxf(progress, bestS);
  while (segment < ROUTE_COUNT - 2 && progress >= pathS[segment + 1] - 0.05f) ++segment;
}

// Luu vai diem vat can gan nhat da do de xe khong quay vao vat khi mat do
// khong con chieu thang vao no. Vong doi ngan de theo kip vat moi.
struct Hit { Point p; uint32_t at; };
constexpr uint8_t MAX_HITS = 20;
constexpr uint32_t HIT_TTL_MS = 3500;
Hit hits[MAX_HITS] = {};
uint8_t nextHit = 0;
uint32_t usedFrameAt = 0;

bool fixedHit(Point p) {
  for (const Box &b : MAP_BOXES) {
    if (fabsf(p.x - b.x) <= b.hx + 1.2f && fabsf(p.y - b.y) <= b.hy + 1.2f) return true;
  }
  return false;
}

void recordFrame(uint32_t now) {
  if (frameAt == usedFrameAt) return;
  usedFrameAt = frameAt;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (frame[i].mm > 530) continue;
    const Pose &o = frame[i].origin;
    const Beam &beam = BEAMS[i];
    const float sinYaw = sinf(o.yaw), cosYaw = cosf(o.yaw);
    const float angle = o.yaw + beam.angleDeg * RAD;
    const float cm = frame[i].mm * 0.1f;
    const Point hit = {o.p.x + beam.right*cosYaw + beam.forward*sinYaw + cm*sinf(angle),
                       o.p.y - beam.right*sinYaw + beam.forward*cosYaw + cm*cosf(angle)};
    if (fixedHit(hit)) continue;
    bool merged = false;
    for (Hit &prev : hits) {
      if (prev.at && now - prev.at <= HIT_TTL_MS &&
          square(prev.p.x - hit.x) + square(prev.p.y - hit.y) < 4.0f) {
        prev = {hit, now};
        merged = true;
        break;
      }
    }
    if (!merged) {
      hits[nextHit] = {hit, now};
      nextHit = (nextHit + 1) % MAX_HITS;
    }
  }
}

bool footprintFree(float x, float y, float s, float c, uint32_t now, bool checkHits) {
  const float as = fabsf(s), ac = fabsf(c);
  const float ex = HALF_LENGTH*as + HALF_WIDTH*ac, ey = HALF_LENGTH*ac + HALF_WIDTH*as;
  const float left = x - ex, right = x + ex, bottom = y - ey;
  if (left < MAP_LEFT + OBSTACLE_MARGIN_CM || right > MAP_RIGHT - OBSTACLE_MARGIN_CM ||
      y + ey > MAP_TOP - OBSTACLE_MARGIN_CM) return false;
  if (bottom < MAP_BOTTOM) {
#if HYBRID_PLANNER_ROUTE
    // No inherited start/finish boundary exception from the fixed route.
    return false;
#else
    const bool leavingStart = segment == 0 && c > 0 && bottom >= 0.0f &&
                              left > START_X_MIN && right < START_X_MAX;
    const bool crossingFinish = segment >= ROUTE_COUNT - 3 &&
                                c < -0.65f && left > FINISH_X_MIN &&
                                right < FINISH_X_MAX && y >= FINISH_EXTENSION.y - HALF_LENGTH;
    if (!leavingStart && !crossingFinish) return false;
#endif
  }
  for (const Box &b : MAP_BOXES) {
    const float dx = b.x - x, dy = b.y - y;
    const float hx = b.hx + OBSTACLE_MARGIN_CM, hy = b.hy + OBSTACLE_MARGIN_CM;
    if (fabsf(dx) > hx + ex || fabsf(dy) > hy + ey) continue;
    if (fabsf(dx*s + dy*c) <= HALF_LENGTH + hx*as + hy*ac &&
        fabsf(dx*c - dy*s) <= HALF_WIDTH + hx*ac + hy*as) return false;
  }
  if (!checkHits) return true;
  constexpr float margin = 4.3f;
  for (const Hit &h : hits) {
    if (!h.at || now - h.at > HIT_TTL_MS) continue;
    const float dx = h.p.x - x, dy = h.p.y - y;
    if (fabsf(dx) > ex + margin || fabsf(dy) > ey + margin) continue;
    if (fabsf(dx*s + dy*c) <= HALF_LENGTH + margin &&
        fabsf(dx*c - dy*s) <= HALF_WIDTH + margin) return false;
  }
  return true;
}

bool finishPassed(float x, float y, float s, float c) {
#if HYBRID_PLANNER_ROUTE
  // Old ROUTE's final two points do not define the new route's finish gate.
  (void)x; (void)y; (void)s; (void)c;
  return false;
#else
  const float edgeX = HALF_WIDTH*fabsf(c) + HALF_LENGTH*fabsf(s);
  const float edgeY = HALF_WIDTH*fabsf(s) + HALF_LENGTH*fabsf(c);
  return progress >= pathS[ROUTE_COUNT-2] - 6.0f && c < -0.65f &&
         x - edgeX > FINISH_X_MIN && x + edgeX < FINISH_X_MAX && y + edgeY < FINISH_Y;
#endif
}

float arcFreeDistance(const Pose &start, float curve, float horizon, uint32_t now) {
  constexpr float ds = 0.5f;
  const float angle = curve * ds;
  const float sc = sinf(angle), cc = cosf(angle);
  const float hs = sinf(angle*0.5f), hc = cosf(angle*0.5f);
  float s = sinf(start.yaw), c = cosf(start.yaw), x = start.p.x, y = start.p.y;
  float safe = 0;
  for (float d = ds; d <= horizon; d += ds) {
    x += ds * (s*hc + c*hs);
    y += ds * (c*hc - s*hs);
    const float ns = s*cc + c*sc;
    c = c*cc - s*sc;
    s = ns;
    if (!footprintFree(x, y, s, c, now, true)) break;
    safe = d;
    if (finishPassed(x, y, s, c)) return horizon;
  }
  return safe;
}

float speedForClearance(float distanceCm) {
  const float reaction = BRAKE_DECEL_CM_S2 * REACTION_TIME_S;
  return sqrtf(square(reaction) + 2.0f * BRAKE_DECEL_CM_S2 * fmaxf(0, distanceCm)) - reaction;
}

float frontSpeedLimit(float clearanceCm) {
  const float noseBuffer = HALF_LENGTH - BEAMS[1].forward + 2.0f;
  // Giam toc som, de du khoang cach truoc khi cham nguong phanh gap.
  return speedForClearance(clearanceCm - noseBuffer - FRONT_SLOW_RESERVE_CM);
}

bool frontEmergency(float clearanceCm, float measuredSpeed) {
  const float noseBuffer = HALF_LENGTH - BEAMS[1].forward + 2.0f;
  return clearanceCm <= FRONT_HARD_STOP_MM * 0.1f ||
         measuredSpeed > speedForClearance(clearanceCm - noseBuffer) + 2.0f;
}

struct Plan { float speed, curve, clearCm; };
Plan plan{};

bool turnSideClear(float curve) {
  if (curve < -0.001f) return frame[0].mm >= DIAG_AVOID_MM;
  if (curve > 0.001f) return frame[3].mm >= DIAG_AVOID_MM;
  return true;
}

bool slewSafeCurve(Plan &target, float clearanceSpeed, float dt, uint32_t now) {
  if (target.speed <= 0 || !turnSideClear(target.curve)) return false;
  const float applied = lastCurve + constrain(target.curve - lastCurve,
                               -CURVE_CHANGE_PER_S * dt, CURVE_CHANGE_PER_S * dt);
  const float free = arcFreeDistance(pose, applied, 27.0f, now);
  const float safeSpeed = speedForClearance(free - 3.0f);
  if (turnSideClear(applied) && free >= 4.5f && safeSpeed + 2.0f >= clearanceSpeed) {
    target.curve = applied;
    target.speed = fminf(target.speed, safeSpeed);
    return true;
  }

  const float chosenFree = arcFreeDistance(pose, target.curve, 27.0f, now);
  const float chosenSpeed = speedForClearance(chosenFree - 3.0f);
  if (chosenFree < 4.5f || chosenSpeed + 2.0f < clearanceSpeed) return false;
  target.speed = fminf(target.speed, chosenSpeed);
  return true;
}

bool startLineCleared(float sinYaw, float cosYaw) {
  const float rearExtent = HALF_LENGTH * fabsf(cosYaw) + HALF_WIDTH * fabsf(sinYaw);
  return cosYaw > 0 && pose.p.y - rearExtent > START_LINE_Y &&
         pose.p.y >= MAP_START.y + START_STRAIGHT_CM;
}

Plan choosePlan(uint32_t now) {
#if HYBRID_PLANNER_ROUTE
  const float look = 11.0f + fminf(4.0f, commandedSpeed * 0.11f);
  if (!PlannerRoute_Target(OneRunFirmwareMission(), pose.p.x, pose.p.y,
                           look, &plannerRouteTarget)) return {};
  progress = plannerRouteTarget.progress_cm;
  segment = static_cast<uint8_t>(plannerRouteTarget.current_segment);
  const Point target{plannerRouteTarget.lookahead.x_cm, plannerRouteTarget.lookahead.y_cm};
#else
  if (leavingStartLine) {
    // Chi chinh lech nho de giu huong +Y cho den khi duoi xe qua vach xanh.
    constexpr float maxStartCurve = 0.025f;
    const float curve = fabsf(pose.yaw) <= MIN_LAUNCH_HEADING_ERROR_RAD ?
      0.0f : constrain(-0.14f * pose.yaw, -maxStartCurve, maxStartCurve);
    const float distance = arcFreeDistance(pose, curve, 15.0f, now);
    if (distance < 4.5f) return {};
    return {fminf(SLOW_CM_S, speedForClearance(distance - 3.0f)), curve, distance};
  }
  updateProgress();
  const float look = 11.0f + fminf(4.0f, commandedSpeed * 0.11f);
  const Point target = routeAt(progress + look);
#endif
  const float dx = target.x - pose.p.x, dy = target.y - pose.p.y;
  const float lateral = dx*cosf(pose.yaw) - dy*sinf(pose.yaw);
  const float forward = dx*sinf(pose.yaw) + dy*cosf(pose.yaw);
  const float preferred = forward > 2.0f ?
    constrain(2.0f * lateral / fmaxf(square(dx)+square(dy), 9.0f), -MAX_CURVE, MAX_CURVE) :
    copysignf(MAX_CURVE, lateral);

  const float front = min(frame[1].mm, frame[2].mm) * 0.1f;
  const float side = min(frame[0].mm, frame[3].mm) * 0.1f;
  const float horizon = 27.0f;
  const float clearanceNeeded = constrain(
    commandedSpeed * REACTION_TIME_S + square(commandedSpeed) / (2.0f * BRAKE_DECEL_CM_S2) + 3.0f,
    9.0f, horizon);
  const bool obstacleNearby = front <= AVOID_EARLY_CM;
  float bestScore = -1e9f, bestCurve = 0, bestFree = 0;
  bool bestHasRollingRoom = false;
  bool bestHasBrakeRoom = false;
  const float candidates[] = {preferred, lastCurve, 0, -MAX_CURVE, -0.78f*MAX_CURVE,
    -0.56f*MAX_CURVE, -0.37f*MAX_CURVE, -0.19f*MAX_CURVE,
     0.19f*MAX_CURVE, 0.37f*MAX_CURVE, 0.56f*MAX_CURVE, 0.78f*MAX_CURVE, MAX_CURVE};
  for (const float k : candidates) {
    if (!turnSideClear(k)) continue;
    const float free = arcFreeDistance(pose, k, horizon, now);
    if (free < 4.5f) continue;
    // Gan vat thi uu tien cung con du quang tien de tranh phai dung va xoay.
    const bool hasRollingRoom = obstacleNearby && free >= MIN_ROLLING_CLEARANCE_CM;
    const bool hasBrakeRoom = free >= clearanceNeeded;
    if (bestHasBrakeRoom && !hasBrakeRoom) continue;
    const float score = 1.5f * fminf(free, clearanceNeeded) -
                        145.0f * fabsf(k - preferred) - 22.0f * fabsf(k - lastCurve);
    const bool betterRolling = hasRollingRoom ? (!bestHasRollingRoom || score > bestScore)
                                              : (!bestHasRollingRoom && score > bestScore);
    if ((hasBrakeRoom && !bestHasBrakeRoom) || betterRolling) {
      bestHasBrakeRoom = hasBrakeRoom;
      bestHasRollingRoom = hasRollingRoom;
      bestScore = score;
      bestCurve = k;
      bestFree = free;
    }
  }
  if (bestFree < 4.5f) return {};

  const float proximity = fminf(constrain((front - 18.0f) /
                                  (AVOID_EARLY_CM + 16.0f - 18.0f), 0.0f, 1.0f),
                                constrain((side - 9.0f) / 24.0f, 0.0f, 1.0f));
  float speed = SLOW_CM_S + (FAST_CM_S - SLOW_CM_S) * proximity;
  speed = fminf(speed, speedForClearance(bestFree - 4.0f));
  speed = fminf(speed, 1.72f / fmaxf(fabsf(bestCurve), 0.02f));
  if (fabsf(bestCurve) > 0.045f) speed = fminf(speed, TURN_CM_S);
#if HYBRID_PLANNER_ROUTE
  const float beforeFinish = fmaxf(0, plannerRouteTarget.lookahead.length_cm - progress);
#else
  const float beforeFinish = fmaxf(0, pathS[ROUTE_COUNT-2] - progress);
#endif
  if (beforeFinish < 16.0f) speed = fminf(speed, SLOW_CM_S + beforeFinish);
  return {speed, bestCurve, bestFree};
}

void printStatus(uint32_t now) {
  Serial.print(status);
  Serial.print(" xy="); Serial.print(pose.p.x, 1); Serial.print(','); Serial.print(pose.p.y, 1);
  Serial.print(" yawR="); Serial.print(YAW_SIGN * currentYawDeg, 1);
  Serial.print(" s="); Serial.print(progress, 1);
  Serial.print(" arc_cm="); Serial.print(plan.clearCm, 1);
  Serial.print(" seg="); Serial.print(segment);
  if (!leavingStartLine) {
#if HYBRID_PLANNER_ROUTE
    PlannerRouteSample referenceSample{};
    const bool referenceValid = PlannerRoute_Sample(OneRunFirmwareMission(), progress, &referenceSample);
    const Point reference{referenceValid ? referenceSample.x_cm : NAN,
                          referenceValid ? referenceSample.y_cm : NAN};
#else
    const Point reference = routeAt(progress);
#endif
    Serial.print(" err_cm="); Serial.print(hypotf(pose.p.x - reference.x, pose.p.y - reference.y), 1);
  }
  Serial.print(" r=");
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (i) Serial.print(',');
    if (frameReady) Serial.print(frame[i].mm);
    else Serial.print('-');
  }
  Serial.print(" pwm="); Serial.print(lroundf(wheelL.pwm)); Serial.print(',');
  Serial.print(lroundf(wheelR.pwm));
  Serial.print(" frame_age_ms=");
  if (frameReady) {
    Serial.print(now - frameAt);
    uint32_t frameSkew = 0;
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) frameSkew = max(frameSkew, frameAt - frame[i].at);
    Serial.print(" frame_skew_ms="); Serial.print(frameSkew);
  } else {
    Serial.print('-');
  }
  Serial.print(" pending_mask=0x"); Serial.print(pendingMask, HEX);
  Serial.print(" tof_fault=0x"); Serial.print(rangeFaultMask, HEX);
  Serial.print(" tof_restart="); Serial.print(rangeRestarts);
  Serial.print(" tof_err=");
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (i) Serial.print(',');
    Serial.print(rangeReadErrors[i]);
  }
  Serial.print(" up_ms="); Serial.print(now);
  Serial.print(" reset_flags=0x"); Serial.print(bootResetFlags, HEX);
  if (lastStop.reason) {
    Serial.print(" last_stop="); Serial.print(lastStop.reason);
    Serial.print(" stop_xy="); Serial.print(lastStop.position.x, 1); Serial.print(',');
    Serial.print(lastStop.position.y, 1);
    Serial.print(" v_meas="); Serial.print(lastStop.actualSpeed, 1);
    Serial.print(" v_cmd="); Serial.print(lastStop.targetSpeed, 1);
    Serial.print(" stop_pwm="); Serial.print(lastStop.pwmL); Serial.print(','); Serial.print(lastStop.pwmR);
    if (fault == Fault::OBSTACLE) {
      Serial.print(" raw_front=");
      for (uint8_t i = 1; i <= 2; ++i) {
        if (i == 2) Serial.print(',');
        if (receivedRangeMask & (1U << i)) Serial.print(pendingRange[i].mm);
        else Serial.print('-');
      }
    }
    if (lastStop.hasRanges) {
      Serial.print(" stop_r=");
      for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        if (i) Serial.print(',');
        Serial.print(lastStop.ranges[i]);
      }
    }
  }
  Serial.println();
}

void serviceBluetooth() {
  static constexpr char command[] = "LOG";
  static uint8_t matched = 0;
  static bool invalid = false;
  static uint32_t lastByteAt = 0;

  for (uint8_t count = 0; count < 16 && Serial.available() > 0; ++count) {
    const int received = Serial.read();
    if (received < 0) break;
    lastByteAt = millis();
    if (received == '?') {
      printStatus(lastByteAt);
      matched = 0;
      invalid = false;
      continue;
    }
    if (received == '\r' || received == '\n') {
      if (!invalid && matched == sizeof(command) - 1) printStatus(lastByteAt);
      matched = 0;
      invalid = false;
      continue;
    }
    const int upper = (received >= 'a' && received <= 'z') ? received - 'a' + 'A' : received;
    if (matched < sizeof(command) - 1 && upper == command[matched]) ++matched;
    else invalid = true;
  }

  // Some Bluetooth terminals send text without a line ending.
  const uint32_t now = millis();
  if (!Serial.available() && (matched || invalid) && now - lastByteAt >= BT_COMMAND_IDLE_MS) {
    if (!invalid && matched == sizeof(command) - 1) printStatus(now);
    matched = 0;
    invalid = false;
  }
}

void setup() {
  bootResetFlags = RCC->CSR & (RCC_CSR_PINRSTF | RCC_CSR_PORRSTF | RCC_CSR_SFTRSTF |
                               RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF | RCC_CSR_LPWRRSTF);
  RCC->CSR |= RCC_CSR_RMVF;
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);
  holdLow(STBY_PIN); holdLow(LEFT_IN1); holdLow(LEFT_IN2);
  holdLow(RIGHT_IN1); holdLow(RIGHT_IN2);
  pinMode(LEFT_PWM, OUTPUT); pinMode(RIGHT_PWM, OUTPUT);
  analogWriteResolution(8);
  stopMotors();
  initWatchdog();
  Serial.setRx(PA10); Serial.setTx(PA9);
  Serial.begin(BT_BAUD, SERIAL_8N1);
#if HYBRID_MOTOR_LOCK
  Serial.println("HYBRID_LINK_CHECK_ONLY MOTOR_LOCK=1; NO_PLANNER_CONTROL");
#else
  Serial.println("AUTO_V26_FORWARD_BOOT");
#endif
#if !HYBRID_PLANNER_ROUTE
  initRoute();
#else
  initPlannerDiagnosticConfig();
#endif
  initHardwareEncoders();
  if (!initArcSensors()) { halt("VL53_INIT_FAIL", Fault::SENSOR); return; }
  for (int i = 0; i < 3 && !gyroReady; ++i) gyroReady = initGyro();
  if (!gyroReady) { halt("MPU_INIT_FAIL", Fault::MPU); return; }
  prevRawL = uint16_t(TIM3->CNT); prevRawR = uint16_t(TIM2->CNT);
  autoStartMs = lastControlMs = millis();
}

bool rangesPermitMotion(uint32_t now, bool fresh) {
  static bool paused = false, triedRestart = false;
  static uint32_t pauseAt = 0, checkedFrameAt = 0;
  static uint8_t freshFrames = 0;
  if (mode == RunMode::WAIT_SENSORS) {
    if (!fresh && now - autoStartMs >= 850) {
      rangeFaultMask = staleRangeMask(now);
      halt("VL53_FRAME_LOST", Fault::SENSOR);
    }
    return fresh;
  }
  if (!fresh) {
    freshFrames = 0;
    checkedFrameAt = frameAt;
    if (!paused) {
      paused = true;
      triedRestart = false;
      pauseAt = now;
      rangeFaultMask = staleRangeMask(now);
      status = "VL53_PAUSE";
      rememberStop(status);
      stopMotors(true);
      Serial.println(status);
      digitalWrite(LED_PIN, HIGH);
    }
  }
  if (!paused) return true;
  const uint32_t pausedFor = now - pauseAt;
  if (!fresh && !triedRestart && pausedFor >= SENSOR_REINIT_DELAY_MS &&
      pausedFor < SENSOR_RECOVERY_LIMIT_MS && rangeRestarts < SENSOR_REINIT_LIMIT && wheelsSettled()) {
    const uint8_t failed = staleRangeMask(now);
    if (failed) {
      rangeFaultMask = failed;
      triedRestart = true;
      ++rangeRestarts;
      status = "VL53_REINIT";
      Serial.println(status);
      if (!initArcSensors(true)) {
        halt("VL53_REINIT_FAIL", Fault::SENSOR);
      } else {
        pauseAt = millis();
        checkedFrameAt = frameAt;
        freshFrames = 0;
      }
      return false;
    }
  }
  if (pausedFor >= SENSOR_RECOVERY_LIMIT_MS) {
    halt("VL53_FRAME_LOST", Fault::SENSOR);
    return false;
  }
  if (fresh && frameAt != checkedFrameAt) {
    checkedFrameAt = frameAt;
    if (++freshFrames >= SENSOR_RECOVERY_FRAMES && wheelsSettled()) {
      paused = false;
      lastPlanMs = now - 30U;
      Serial.println("VL53_RESUME");
      return true;
    }
  }
  return false;
}

void loop() {
  reloadWatchdog();
  serviceBluetooth();
  if (mode == RunMode::LOCKED || mode == RunMode::FINISHED) {
    const uint32_t now = millis();
    if ((fault == Fault::LEFT_WHEEL || fault == Fault::RIGHT_WHEEL) &&
        now - stoppedAt >= WHEEL_FAULT_BRAKE_MS) {
      digitalWrite(STBY_PIN, LOW);
    }
    const uint32_t phase = (now - stoppedAt) % 2500;
    const uint8_t flashes = static_cast<uint8_t>(fault);
    const bool blink = phase < 250U * flashes && phase % 250U < 125U;
    digitalWrite(LED_PIN, blink ? LOW : HIGH);
    return;
  }

  pollRange();
  const uint32_t now = millis();
  if (immediateFrontStop(now)) {
    halt("FRONT_TOO_CLOSE", Fault::OBSTACLE);
    return;
  }
  const uint32_t controlGap = now - lastControlMs;
  if (controlGap > CONTROL_LOOP_TIMEOUT_MS) {
    lastControlMs = now;
    halt("CONTROL_LOOP_TIMEOUT", Fault::CONTROL);
    return;
  }
  if (controlGap < 10) return;
  const float dt = controlGap * 0.001f;
  lastControlMs = now;
  updateGyro(now);
  if (now - lastGyroGoodMs > 100) { halt("MPU_LOST", Fault::MPU); return; }
  int32_t dL, dR;
  updateEncoders(dL, dR);
  lastEncoderSampleMs = now;
  updatePose(dL, dR);
  if (!wheelL.observe(dL, now, dt, MAP_COUNTS_CM_L)) {
    halt("STALL_OR_DIR_L", Fault::LEFT_WHEEL); return;
  }
  if (!wheelR.observe(dR, now, dt, MAP_COUNTS_CM_R)) {
    halt("STALL_OR_DIR_R", Fault::RIGHT_WHEEL); return;
  }

  uint32_t oldestRangeAge = 0;
  if (frameReady) {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
      oldestRangeAge = max(oldestRangeAge, now - frame[i].at);
    }
  }
  if (!rangesPermitMotion(now, frameReady && oldestRangeAge <= SENSOR_STALE_MS)) return;
  mode = RunMode::AUTO_RUN;

  const float sinYaw = sinf(pose.yaw), cosYaw = cosf(pose.yaw);
  if (finishPassed(pose.p.x, pose.p.y, sinYaw, cosYaw)) {
    halt("FINISH_LINE_PASSED", Fault::NONE);
    return;
  }

  if (leavingStartLine && startLineCleared(sinYaw, cosYaw)) {
    leavingStartLine = false;
    lastPlanMs = now - 30U;
  }
  recordFrame(now);
  const float frontClearance = min(frame[1].mm, frame[2].mm) * 0.1f;
  const float motorDt = fminf(dt, 0.025f);
  const float forwardSpeed = fmaxf(0.0f, measuredForwardSpeed());
  const bool emergency = frontEmergency(frontClearance, forwardSpeed);
#if HYBRID_PLANNER_ROUTE
  pollPlannerRuntime(now, dt, emergency);
  if (plannerGoalReached) {
    plan = {};
    status = "PLANNER_GOAL_REACHED";
    stopMotors(true);
    mode = RunMode::FINISHED;
    stoppedAt = now;
    digitalWrite(LED_PIN, LOW);
    return;
  }
  if (plannerStatus != ONE_RUN_OK) plan = {};
#endif
  if (now - lastPlanMs >= 30) {
    lastPlanMs = now;
    plan = choosePlan(now);
  }
  const bool footprintBlocked = !footprintFree(pose.p.x, pose.p.y, sinYaw, cosYaw, now, false);
  Plan motion = plan;
  bool safeArc = slewSafeCurve(motion, forwardSpeed, motorDt, now);
  if (!safeArc && !emergency && !footprintBlocked && lastPlanMs != now) {
    // A new ToF frame may invalidate the old arc between planning ticks.
    lastPlanMs = now;
    plan = choosePlan(now);
    motion = plan;
    safeArc = slewSafeCurve(motion, forwardSpeed, motorDt, now);
  }
  // Con co the lua cung tranh o toc do bo khi front chi vua qua nguong khan cap.
  const float frontLimit = fmaxf(5.0f, frontSpeedLimit(frontClearance));
  float speed = fminf(motion.speed, frontLimit);
  if (oldestRangeAge > SENSOR_SLOW_AGE_MS) {
    const float freshFraction = float(SENSOR_STALE_MS - oldestRangeAge) /
                                float(SENSOR_STALE_MS - SENSOR_SLOW_AGE_MS);
    speed = fminf(speed, SLOW_CM_S + (FAST_CM_S - SLOW_CM_S) * freshFraction);
  }

  static bool braking = false;
  static uint32_t lastBlockedAt = 0;
  if (!safeArc || emergency || footprintBlocked) {
    lastBlockedAt = now;
    status = footprintBlocked ? "MAP_GUARD_STOP" :
             leavingStartLine ? "START_BLOCKED" :
             emergency ? "NEAR_STOP" : "NO_SAFE_FORWARD_ARC";
    if (!braking) rememberStop(status);
    stopMotors(true);
    if (!braking) Serial.println(status);
    braking = true;
  } else if (!braking || now - lastBlockedAt >= CLEAR_BEFORE_RESTART_MS) {
    braking = false;
    status = leavingStartLine ? "START_STRAIGHT" :
             frontClearance <= AVOID_EARLY_CM ? "ROLL_AVOID" : "AUTO_V26_RUN";
    driveForward(speed, motion.curve, motorDt);
    lastCurve = motion.curve;
  }
  digitalWrite(LED_PIN, braking ? HIGH : LOW);
}
