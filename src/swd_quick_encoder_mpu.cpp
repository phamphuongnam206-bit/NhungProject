#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "map_one_run_calibration.h"
#include "v1_mpu_profile.h"

#if !defined(SWD_QUICK_MOTOR_LOCK) || SWD_QUICK_MOTOR_LOCK != 1
#error "SWD quick diagnostic requires motor lock"
#endif

// Same wiring, timer polarity and MPU range as hybrid_2laps.cpp. No drive path.
namespace {
constexpr uint32_t STBY_PIN = PB0;
constexpr uint32_t MOTOR_PINS[] = {PB14, PB15, PB9, PB12, PB13, PB8};
constexpr uint8_t MPU_ADDR = 0x68;
constexpr uint32_t SAMPLE_MS = 10, QUIET_MS = 800, GYRO_TIMEOUT_MS = 100;
constexpr int32_t LEFT_DIR = MapOneRunCalibration::leftEncoderSign;
constexpr int32_t RIGHT_DIR = MapOneRunCalibration::rightEncoderSign;
constexpr uint8_t READY = 1, STRAIGHT = 2, TURN_READY = 3;
constexpr uint8_t TURNING = 4, DONE = 5, DIAG_ERROR = 6;
constexpr uint8_t ERR_MPU_INIT = 1, ERR_MPU_READ = 2, ERR_SAMPLE_GAP = 3;
uint16_t previousLeft = 0, previousRight = 0;
uint32_t lastSampleMs = 0, lastGyroGoodMs = 0, lastMovementMs = 0;
uint32_t lastTurnMs = 0;
float biasRaw = 0, yawRawDeg = 0, turnStartRawDeg = 0;
}

// Stable C symbol for SWD Watch. phase: 1 ready, 2 push, 3 ready to turn,
// 4 turning, 5 done, 6 error. command=1 resets the capture, never drives.
extern "C" {
struct SwdQuickResult {
  uint32_t magic, version, phase, error, command, sample_count;
  uint32_t straight_done, turn_done, last_sample_ms, gyro_age_ms;
  int32_t left_delta, right_delta, left_total, right_total;
  int32_t straight_left, straight_right, turn_left, turn_right;
  uint16_t tim3_raw, tim2_raw, mpu_who_am_i, mpu_gyro_config;
  float gyro_bias_raw, gyro_rate_dps, yaw_raw_deg, yaw_hybrid_deg;
  float straight_yaw_change_deg, turn_yaw_raw_deg, turn_yaw_hybrid_deg;
  float turn_90_error_deg;
  float measured_distance_cm, left_counts_per_cm, right_counts_per_cm;
};
volatile SwdQuickResult swd_quick = {};
}

static void forceMotorOff() {
  digitalWrite(STBY_PIN, LOW);
  for (uint32_t pin : MOTOR_PINS) digitalWrite(pin, LOW);
}

static void initMotorPins() {
  digitalWrite(STBY_PIN, LOW);
  pinMode(STBY_PIN, OUTPUT);
  for (uint32_t pin : MOTOR_PINS) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  forceMotorOff();
}

static void configureEncoderTimer(TIM_TypeDef *timer) {
  timer->CR1 = 0; timer->PSC = 0; timer->ARR = 65535;
  timer->CCMR1 = 1U | (8U << 4) | (1U << 8) | (8U << 12);
  timer->SMCR = 3U; timer->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
  timer->EGR = TIM_EGR_UG; timer->CNT = 0; timer->CR1 = TIM_CR1_CEN;
}

static void initEncoders() {
  __HAL_RCC_AFIO_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE(); __HAL_RCC_TIM3_CLK_ENABLE();
  HAL_NVIC_DisableIRQ(TIM2_IRQn); HAL_NVIC_DisableIRQ(TIM3_IRQn);
  __HAL_RCC_TIM2_FORCE_RESET(); __HAL_RCC_TIM2_RELEASE_RESET();
  __HAL_RCC_TIM3_FORCE_RESET(); __HAL_RCC_TIM3_RELEASE_RESET();
  uint32_t mapr = AFIO->MAPR;
  mapr &= ~(AFIO_MAPR_TIM2_REMAP | AFIO_MAPR_TIM3_REMAP | AFIO_MAPR_SWJ_CFG);
  AFIO->MAPR = mapr | AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
  pinMode(PA6, INPUT_PULLUP); pinMode(PA7, INPUT_PULLUP);
  pinMode(PA0, INPUT_PULLUP); pinMode(PA1, INPUT_PULLUP);
  configureEncoderTimer(TIM2); configureEncoderTimer(TIM3);
  previousLeft = uint16_t(TIM3->CNT);
  previousRight = uint16_t(TIM2->CNT);
}

static int32_t signedCounterDiff(uint16_t current, uint16_t previous) {
  const uint16_t delta = uint16_t(current - previous);
  return delta < 32768U ? int32_t(delta) : int32_t(delta) - 65536L;
}

static bool writeMpu(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool readMpu(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg);
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom(MPU_ADDR, uint8_t(1)) != 1) return false;
  const int data = Wire.read();
  if (data < 0) return false;
  value = uint8_t(data);
  return true;
}

static bool readGyroZ(int16_t &raw) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(uint8_t(0x47));
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom(MPU_ADDR, uint8_t(2)) != 2) return false;
  const int high = Wire.read(), low = Wire.read();
  if (high < 0 || low < 0) return false;
  raw = int16_t((uint16_t(high) << 8) | uint16_t(low));
  return true;
}

static bool initMpu() {
  Wire.setSDA(PB7); Wire.setSCL(PB6); Wire.begin(); Wire.setClock(100000);
  uint8_t who = 0, filter = 0, divider = 0, range = 0;
  if (!readMpu(0x75, who)) return false;
  swd_quick.mpu_who_am_i = who;
  if (who != 0x68 || !writeMpu(0x6B, 0x01) ||
      !writeMpu(0x1A, V1Mpu::filter) || !writeMpu(0x19, V1Mpu::sampleDivider) ||
      !writeMpu(0x1B, V1Mpu::gyroConfig)) return false;
  if (!readMpu(0x1A, filter) || filter != V1Mpu::filter ||
      !readMpu(0x19, divider) || divider != V1Mpu::sampleDivider ||
      !readMpu(0x1B, range) || range != V1Mpu::gyroConfig) return false;
  swd_quick.mpu_gyro_config = range;
  delay(60);
  V1Mpu::StationaryBias bias;
  for (unsigned i = 0; i < V1Mpu::biasSamples; ++i) {
    int16_t raw = 0;
    if (!readGyroZ(raw) || !bias.add(raw)) return false;
    delay(V1Mpu::samplePeriodMs);
  }
  biasRaw = bias.mean();
  swd_quick.gyro_bias_raw = biasRaw;
  lastGyroGoodMs = millis();
  return true;
}

static void resetCapture(uint32_t now) {
  swd_quick.command = 0;
  swd_quick.phase = READY;
  swd_quick.error = 0;
  swd_quick.straight_done = swd_quick.turn_done = 0;
  swd_quick.left_delta = swd_quick.right_delta = 0;
  swd_quick.left_total = swd_quick.right_total = 0;
  swd_quick.straight_left = swd_quick.straight_right = 0;
  swd_quick.turn_left = swd_quick.turn_right = 0;
  swd_quick.sample_count = 0;
  swd_quick.straight_yaw_change_deg = 0;
  swd_quick.turn_yaw_raw_deg = swd_quick.turn_yaw_hybrid_deg = 0;
  swd_quick.turn_90_error_deg = 0;
  swd_quick.measured_distance_cm = 0;
  swd_quick.left_counts_per_cm = swd_quick.right_counts_per_cm = 0;
  previousLeft = uint16_t(TIM3->CNT);
  previousRight = uint16_t(TIM2->CNT);
  yawRawDeg = 0;
  turnStartRawDeg = 0;
  lastSampleMs = lastGyroGoodMs = lastMovementMs = lastTurnMs = now;
}

static void updateMeasuredScale() {
  const float cm = swd_quick.measured_distance_cm;
  if (swd_quick.straight_done && isfinite(cm) && cm >= 10.0f && cm <= 30.0f) {
    swd_quick.left_counts_per_cm = fabsf(float(swd_quick.straight_left)) / cm;
    swd_quick.right_counts_per_cm = fabsf(float(swd_quick.straight_right)) / cm;
  } else {
    swd_quick.left_counts_per_cm = swd_quick.right_counts_per_cm = 0;
  }
}

void setup() {
  initMotorPins();
  swd_quick.magic = 0x53574431UL; // "SWD1"
  swd_quick.version = 1;
  swd_quick.phase = DIAG_ERROR;
  initEncoders();
  if (!initMpu()) {
    swd_quick.error = ERR_MPU_INIT;
    return;
  }
  resetCapture(millis());
}

void loop() {
  forceMotorOff();
  if (swd_quick.phase == DIAG_ERROR) return;
  const uint32_t now = millis();
  if (swd_quick.command == 1U) resetCapture(now);
  if (swd_quick.phase == DONE) {
    updateMeasuredScale();
    return; // preserve the latched result even if SWD halts the CPU
  }
  if (uint32_t(now - lastSampleMs) < SAMPLE_MS) return;
  const uint32_t elapsed = now - lastSampleMs;
  lastSampleMs = now;
  if (elapsed > GYRO_TIMEOUT_MS) {
    if (swd_quick.phase == READY || swd_quick.phase == TURN_READY) {
      // Watching the idle result may halt the core. Ignore only idle gaps;
      // never integrate motion across a missed interval.
      previousLeft = uint16_t(TIM3->CNT);
      previousRight = uint16_t(TIM2->CNT);
      return;
    }
    swd_quick.phase = DIAG_ERROR;
    swd_quick.error = ERR_SAMPLE_GAP;
    return;
  }
  int16_t rawGyro = 0;
  if (!readGyroZ(rawGyro)) {
    swd_quick.phase = DIAG_ERROR;
    swd_quick.error = ERR_MPU_READ;
    return;
  }
  lastGyroGoodMs = now;
  const float rate = (float(rawGyro) - biasRaw) / V1Mpu::gyroLsbPerDps;
  const float filteredRate = fabsf(rate) < 0.30f ? 0.0f : rate;
  yawRawDeg += filteredRate * (float(elapsed) * 0.001f);
  const uint16_t rawLeft = uint16_t(TIM3->CNT);
  const uint16_t rawRight = uint16_t(TIM2->CNT);
  const int32_t left = LEFT_DIR * signedCounterDiff(rawLeft, previousLeft);
  const int32_t right = RIGHT_DIR * signedCounterDiff(rawRight, previousRight);
  previousLeft = rawLeft; previousRight = rawRight;
  swd_quick.tim3_raw = rawLeft; swd_quick.tim2_raw = rawRight;
  swd_quick.left_delta = left; swd_quick.right_delta = right;
  swd_quick.gyro_rate_dps = filteredRate;
  swd_quick.yaw_raw_deg = yawRawDeg;
  swd_quick.yaw_hybrid_deg = MapOneRunCalibration::hybridYawSign * yawRawDeg;
  swd_quick.last_sample_ms = now;
  swd_quick.gyro_age_ms = now - lastGyroGoodMs;
  ++swd_quick.sample_count;

  const uint8_t phase = uint8_t(swd_quick.phase);
  if (phase == READY || phase == STRAIGHT) {
    if (left || right) {
      if (phase == READY) {
        swd_quick.phase = STRAIGHT;
        swd_quick.straight_yaw_change_deg = -yawRawDeg;
      }
      swd_quick.left_total += left;
      swd_quick.right_total += right;
      lastMovementMs = now;
    } else if (phase == STRAIGHT && uint32_t(now - lastMovementMs) >= QUIET_MS) {
      swd_quick.straight_left = swd_quick.left_total;
      swd_quick.straight_right = swd_quick.right_total;
      swd_quick.straight_yaw_change_deg += yawRawDeg;
      swd_quick.straight_done = 1;
      swd_quick.phase = TURN_READY;
    }
  } else if (phase == TURN_READY || phase == TURNING) {
    if (phase == TURN_READY && fabsf(filteredRate) >= 8.0f) {
      turnStartRawDeg = yawRawDeg;
      swd_quick.phase = TURNING;
      lastTurnMs = now;
    }
    if (swd_quick.phase == TURNING) {
      swd_quick.turn_left += left;
      swd_quick.turn_right += right;
      if (fabsf(filteredRate) >= 3.0f || left || right) lastTurnMs = now;
      if (uint32_t(now - lastTurnMs) >= QUIET_MS) {
        swd_quick.turn_yaw_raw_deg = yawRawDeg - turnStartRawDeg;
        swd_quick.turn_yaw_hybrid_deg = MapOneRunCalibration::hybridYawSign *
                                       swd_quick.turn_yaw_raw_deg;
        swd_quick.turn_90_error_deg = fabsf(fabsf(swd_quick.turn_yaw_raw_deg) - 90.0f);
        swd_quick.turn_done = 1;
        swd_quick.phase = DONE;
      }
    }
  }
  updateMeasuredScale();
}
