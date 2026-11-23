#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <EEPROM.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

#ifndef BT_BAUD
#define BT_BAUD 38400  // Baud UART HC-05 (PA9/PA10)
#endif

// ==================== THONG SO PWM & CHUNG ====================
constexpr int MIN_PWM = 80;
constexpr int MAX_PWM = 136;
constexpr int DEFAULT_PWM = 112;
constexpr uint32_t REVERSE_DEAD_MS = 25;

// ==================== HAM TU BLUETOOTH.CPP ====================
void bluetoothResetState();
void bluetoothFinishMotion();
bool bluetoothIsWheelTestActive();
void bluetoothProcessSerial();
void bluetoothUpdateManual(uint32_t now);

// ==================== PHAN CUNG & CHAN KET NOI ====================
constexpr uint32_t LED_PIN = PC13;
constexpr uint32_t STBY_PIN = PB0;

constexpr uint32_t LEFT_IN1 = PB14;
constexpr uint32_t LEFT_IN2 = PB15;
constexpr uint32_t LEFT_PWM = PB9;

constexpr uint32_t RIGHT_IN1 = PB12;
constexpr uint32_t RIGHT_IN2 = PB13;
constexpr uint32_t RIGHT_PWM = PB8;

constexpr uint32_t LEFT_ENC_A = PA6;   // TIM3_CH1
constexpr uint32_t LEFT_ENC_B = PA7;   // TIM3_CH2
constexpr uint32_t RIGHT_ENC_A = PA0;  // TIM2_CH1
constexpr uint32_t RIGHT_ENC_B = PA1;  // TIM2_CH2

constexpr uint8_t VL53_COUNT = 4;
constexpr uint32_t XSHUT_PINS[VL53_COUNT] = {PA2, PA3, PA4, PA5};
constexpr uint8_t VL53_ADDR[VL53_COUNT] = {0x30, 0x31, 0x32, 0x33};
constexpr const char *VL53_NAME[VL53_COUNT] = {"J8", "J9", "J10", "J11"};
constexpr float DEFAULT_OFFSET_MM[VL53_COUNT] = {0.0f, -15.0f, -15.0f, 0.0f};
enum SensorIdx : uint8_t { SIDE_L = 0, FRONT_A = 1, FRONT_B = 2, SIDE_R = 3 };

// ==================== THONG SO CO KHI & TU HANH (LAN 2) ====================
constexpr float PI_F = 3.14159265359f;
constexpr float DEG_TO_RAD_F = PI_F / 180.0f;
constexpr float RAD_TO_DEG_F = 180.0f / PI_F;

constexpr float TRACK_MM = 120.0f;
constexpr float COUNTS_PER_MM_L = 10.0f;  // x4 quadrature counts/mm
constexpr float COUNTS_PER_MM_R = 10.0f;
constexpr int LEFT_ENC_SIGN = -1;         // tien -> xung am
constexpr int RIGHT_ENC_SIGN = +1;        // tien -> xung duong
constexpr int GYRO_YAW_SIGN = 1;          // Z huong len, quay trai = duong
constexpr float GYRO_SCALE = 1.0f;

constexpr uint32_t CONTROL_MS = 10;
constexpr uint32_t AUTO_BOOT_DELAY_MS = 3000;   // Cho 3s sau khi hieu chuan gyro roi tu chay Lan 2
constexpr float RECORD_STEP_MM = 38.0f;          // Khoang cach ghi 1 diem lo trinh
constexpr float RECORD_ANGLE_RAD = 0.20f;        // Hoac doi huong ~11.5 do thi ghi diem
constexpr float LOOKAHEAD_MM = 72.0f;            // Diem nhin truoc ngan de om sat quy dao Lan 1
constexpr float WAYPOINT_REACH_MM = 32.0f;
constexpr float GOAL_REACH_MM = 25.0f;
constexpr float AUTO_MAX_SPEED_MM_S = 75.0f;
constexpr float AUTO_MIN_SPEED_MM_S = 24.0f;
constexpr float AUTO_MAX_OMEGA = 0.95f;
constexpr float AUTO_MIN_WHEEL_CPS = 180.0f;
constexpr float CTE_PULL_GAIN = 1.55f;           // Do manh luc keo xe quay lai tim duong Lan 1
constexpr float FRONT_SLOW_MM = 210.0f;          // Giam toc som tu 21cm
constexpr float FRONT_PIVOT_MM = 95.0f;          // Dung tien va xoay tai cho tai 9.5cm (khong lui)
constexpr float FRONT_RESUME_MM = 120.0f;        // Thoat huong can > 12cm moi chay tien tiep
constexpr float CORNER_GUARD_MM = 105.0f;        // Giu goc lai khong quet vao mep vat can khi dang luot qua

// ==================== LUU TRU LO TRINH TRONG FLASH/EEPROM ====================
constexpr int ROUTE_EEPROM_OFFSET = 64;  // Giu nguyen 64 byte dau cho hieu chuan VL53
constexpr uint16_t MAX_ROUTE_POINTS = 180;
constexpr uint32_t ROUTE_MAGIC = 0x3354524EUL;
constexpr uint16_t ROUTE_VERSION = 3;

struct TeachPoint {
  int16_t x;
  int16_t y;
};

struct RouteHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t count;
  uint32_t config;
  uint32_t crc;
};

struct StoredCalibration {
  uint32_t magic;
  float offsets[VL53_COUNT];
  float frontBiasMm[2];
  uint32_t frontBalMask;
  uint32_t checksum;
};

// ==================== BIEN TRANG THAI TOAN CUC ====================
enum class RunMode : uint8_t {
  Calibrating,
  Round1Teach,
  Round2Countdown,
  Round2Auto,
  Round2Finished,
  Paused
};

RunMode runMode = RunMode::Calibrating;
const char *statusText = "GYRO_CAL_KEEP_STILL";

bool userPaused = false;  // KHONG khoa xe khi bat nguon
bool encoderFault = false;

// Odometry & IMU (goc (0, 0), huong +Y = PI/2)
TwoWire imuBus(PB7, PB6);
uint8_t imuAddr = 0x68;
bool gyroReady = false;
uint16_t gyroBiasSamples = 0;
float gyroBiasSum = 0.0f;
float gyroBias = 0.0f;
float yawRateDps = 0.0f;
uint32_t lastImuMs = 0;

float poseX = 0.0f;
float poseY = 0.0f;
float poseH = PI_F * 0.5f;
float traveledMm = 0.0f;
float lastRecordH = PI_F * 0.5f;

// Lo trinh Lan 1 / Lan 2
TeachPoint taught[MAX_ROUTE_POINTS];
uint16_t taughtCount = 0;
bool routeSaved = false;
uint16_t autoTargetIdx = 1;
uint32_t countdownStartMs = 0;
bool pivotingObstacle = false;
int8_t pivotDir = 0;
uint32_t stallStartMs = 0;
uint32_t microReleaseUntilMs = 0;

// Dong co & Encoder
int currentLeft = 0;
int currentRight = 0;
int lastLeftBeforeStop = 0;
int lastRightBeforeStop = 0;
uint32_t motorsStoppedMs = 0;
int pendingLeft = 0;
int pendingRight = 0;
bool turnBraking = false;

uint16_t prevLeftRaw = 0;
uint16_t prevRightRaw = 0;
int32_t leftTotal = 0;
int32_t rightTotal = 0;
int32_t deltaLeft = 0;
int32_t deltaRight = 0;
int32_t odomDeltaLeft = 0;
int32_t odomDeltaRight = 0;
int32_t autoDeltaLeft = 0;
int32_t autoDeltaRight = 0;

// Vong toc do banh xe cho Lan 2
struct AutoWheel {
  float cps = 0.0f;
  float integral = 0.0f;
  float pwm = 0.0f;
  uint32_t lastPulse = 0;
  uint32_t stoppedAt = 0;
  int8_t dir = 0;

  void reset(uint32_t now) {
    if (pwm > 0.0f) stoppedAt = now;
    pwm = 0.0f;
    integral = 0.0f;
    lastPulse = now;
  }

  void observe(int32_t directedCounts, float dt, uint32_t now) {
    const float inst = float(directedCounts) / dt;
    cps += dt / (0.05f + dt) * (inst - cps);
    if (directedCounts != 0) lastPulse = now;
  }

  int compute(float targetCps, float dt, uint32_t now) {
    if (fabsf(targetCps) < 1.0f) {
      reset(now);
      return 0;
    }
    const int8_t sign = targetCps > 0.0f ? 1 : -1;
    if (pwm > 0.0f && sign != dir) reset(now);
    if (dir != 0 && sign != dir && now - stoppedAt < REVERSE_DEAD_MS) return 0;
    if (pwm <= 0.0f) {
      dir = sign;
      lastPulse = now;
    }
    const float ref = fabsf(targetCps);
    const float measured = sign * cps;
    const float err = ref - measured;
    const float stallBoost = 108.0f * constrain((float(now - lastPulse) - 40.0f) / 100.0f, 0.0f, 1.0f);
    const float floorPwm = fmaxf(72.0f, stallBoost);
    const float ff = 44.0f + (160.0f - 44.0f) * ref / 5600.0f;
    if ((err > 0.0f && ff + integral < MAX_PWM) || (err < 0.0f && ff + integral > floorPwm)) {
      integral = constrain(integral + 0.08f * err * dt, -25.0f, 25.0f);
    }
    const float wanted = constrain(ff + 0.012f * err + integral, floorPwm, float(MAX_PWM));
    const float slew = wanted < pwm ? 2000.0f : 900.0f;
    pwm += constrain(wanted - pwm, -slew * dt, slew * dt);
    return sign * int(lroundf(pwm));
  }
};

AutoWheel autoWheelL, autoWheelR;

// Cam bien khoang cach VL53L0X (I2C2 PB10/PB11)
VL53L0X vl53[VL53_COUNT];
float rangeOffsets[VL53_COUNT] = {0.0f, -15.0f, -15.0f, 0.0f};
bool vl53Ready[VL53_COUNT] = {};
bool allVl53Ready = false;
struct RangeReading {
  bool valid = false;
  uint16_t raw = 0;
  float mm = 8190.0f;
  uint32_t updatedAt = 0;
  bool fresh(uint32_t now) const { return valid && (now - updatedAt) < 160; }
} ranges[VL53_COUNT];

// ==================== TIEN ICH TOAN HOC & PHAN CUNG ====================
float wrapAngle(float a) {
  while (a > PI_F) a -= 2.0f * PI_F;
  while (a < -PI_F) a += 2.0f * PI_F;
  return a;
}

void holdLow(uint32_t pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

// ==================== MPU6050 (I2C1: PB6/PB7) ====================
bool writeImuReg(uint8_t reg, uint8_t val) {
  imuBus.beginTransmission(imuAddr);
  imuBus.write(reg);
  imuBus.write(val);
  return imuBus.endTransmission() == 0;
}

bool readImuReg(uint8_t reg, uint8_t *buf, uint8_t len) {
  imuBus.beginTransmission(imuAddr);
  imuBus.write(reg);
  if (imuBus.endTransmission(false) != 0) return false;
  if (imuBus.requestFrom(imuAddr, len) != len) return false;
  for (uint8_t i = 0; i < len; ++i) {
    int b = imuBus.read();
    if (b < 0) return false;
    buf[i] = uint8_t(b);
  }
  return true;
}

void initImu() {
  imuBus.begin();
  imuBus.setClock(100000);
  for (uint8_t addr = 0x68; addr <= 0x69; ++addr) {
    imuAddr = addr;
    uint8_t who = 0;
    if (readImuReg(0x75, &who, 1) && who == 0x68) {
      // FS_SEL = 1 (+/- 500 deg/s -> 65.5 LSB/(deg/s))
      if (writeImuReg(0x6B, 0x01) && writeImuReg(0x1A, 0x03) &&
          writeImuReg(0x19, 0x09) && writeImuReg(0x1B, 0x08)) {
        gyroReady = true;
        gyroBiasSamples = 0;
        gyroBiasSum = 0.0f;
        lastImuMs = millis();
        return;
      }
    }
  }
  gyroReady = false;
}

// ==================== ENCODER PHAN CUNG (TIM2 & TIM3) ====================
void configureEncoderTimer(TIM_TypeDef *timer) {
  timer->CR1 = 0;
  timer->CR2 = 0;
  timer->SMCR = 0;
  timer->DIER = 0;
  timer->CCER = 0;
  timer->PSC = 0;
  timer->ARR = 65535;
  timer->CCMR1 = 1U | (8U << 4) | (1U << 8) | (8U << 12);
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
  mapr &= ~(AFIO_MAPR_TIM2_REMAP | AFIO_MAPR_TIM3_REMAP | AFIO_MAPR_SWJ_CFG);
  mapr |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
  AFIO->MAPR = mapr;

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  configureEncoderTimer(TIM2);
  configureEncoderTimer(TIM3);

  prevLeftRaw = uint16_t(TIM3->CNT);
  prevRightRaw = uint16_t(TIM2->CNT);
}

int32_t counterDifference(uint16_t current, uint16_t previous) {
  uint16_t diff = uint16_t(current - previous);
  return (diff < 32768U) ? int32_t(diff) : int32_t(diff) - 65536L;
}

void readEncoders() {
  uint16_t lRaw = uint16_t(TIM3->CNT);
  uint16_t rRaw = uint16_t(TIM2->CNT);
  deltaLeft = counterDifference(lRaw, prevLeftRaw);
  deltaRight = counterDifference(rRaw, prevRightRaw);
  leftTotal += deltaLeft;
  rightTotal += deltaRight;
  odomDeltaLeft += deltaLeft;
  odomDeltaRight += deltaRight;
  autoDeltaLeft += deltaLeft;
  autoDeltaRight += deltaRight;
  prevLeftRaw = lRaw;
  prevRightRaw = rRaw;
}

// ==================== VL53L0X (I2C2: PB10/PB11) ====================
void loadVl53Offsets() {
  StoredCalibration cal = {};
  EEPROM.get(0, cal);
  if (cal.magic == 0x48464C33UL) {
    for (uint8_t i = 0; i < VL53_COUNT; ++i) {
      if (isfinite(cal.offsets[i]) && fabsf(cal.offsets[i]) <= 150.0f) {
        rangeOffsets[i] = cal.offsets[i];
      }
    }
  } else {
    for (uint8_t i = 0; i < VL53_COUNT; ++i) rangeOffsets[i] = DEFAULT_OFFSET_MM[i];
  }
}

bool initVl53Sensors() {
  Wire.setSCL(PB10);
  Wire.setSDA(PB11);
  Wire.begin();
  Wire.setClock(100000);

  uint8_t okCount = 0;
  for (uint8_t i = 0; i < VL53_COUNT; ++i) {
    pinMode(XSHUT_PINS[i], INPUT);
    delay(40);
    Wire.beginTransmission(0x29);
    if (Wire.endTransmission() != 0) {
      holdLow(XSHUT_PINS[i]);
      continue;
    }
    vl53[i].setBus(&Wire);
    vl53[i].setTimeout(80);
    if (!vl53[i].init() || vl53[i].last_status != 0) {
      holdLow(XSHUT_PINS[i]);
      continue;
    }
    vl53[i].setAddress(VL53_ADDR[i]);
    delay(5);
    vl53[i].setMeasurementTimingBudget(25000);
    vl53[i].setSignalRateLimit(0.25f);
    vl53[i].startContinuous();
    if (vl53[i].last_status == 0) {
      vl53Ready[i] = true;
      ++okCount;
    }
  }
  return okCount == VL53_COUNT;
}

void pollSingleVl53(uint8_t i) {
  if (!vl53Ready[i]) return;
  uint8_t addr = VL53_ADDR[i];
  Wire.beginTransmission(addr);
  Wire.write(uint8_t(VL53L0X::RESULT_INTERRUPT_STATUS));
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(addr, uint8_t(1)) != 1) return;
  uint8_t irq = Wire.read();
  if (!(irq & 0x07)) return;

  Wire.beginTransmission(addr);
  Wire.write(uint8_t(VL53L0X::RESULT_RANGE_STATUS));
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(addr, uint8_t(13)) != 13) return;
  uint8_t buf[13];
  for (uint8_t k = 0; k < 13; ++k) buf[k] = Wire.read();

  Wire.beginTransmission(addr);
  Wire.write(uint8_t(VL53L0X::SYSTEM_INTERRUPT_CLEAR));
  Wire.write(uint8_t(1));
  Wire.endTransmission();

  uint8_t status = (buf[0] & 0x78) >> 3;
  uint16_t sig = (uint16_t(buf[6]) << 8) | buf[7];
  uint16_t raw = (uint16_t(buf[10]) << 8) | buf[11];
  ranges[i].raw = raw;
  if (status == 11 && raw > 0 && raw < 8190 && sig >= 32) {
    ranges[i].mm = fmaxf(0.0f, float(raw) + rangeOffsets[i]);
    ranges[i].valid = true;
    ranges[i].updatedAt = millis();
  } else {
    ranges[i].valid = false;
  }
}

// ==================== DIEU KHIEN DONG CO TB6612 ====================
void stopMotors() {
  turnBraking = false;
  pendingLeft = pendingRight = 0;
  if (currentLeft || currentRight) {
    lastLeftBeforeStop = currentLeft;
    lastRightBeforeStop = currentRight;
    motorsStoppedMs = millis();
  }
  digitalWrite(STBY_PIN, LOW);
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
  currentLeft = 0;
  currentRight = 0;
}

void brakeTurn() {
  stopMotors();
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, HIGH);
  digitalWrite(STBY_PIN, HIGH);
  turnBraking = true;
}

void setWheel(bool left, int value) {
  value = constrain(value, -MAX_PWM, MAX_PWM);
  const uint32_t pwmPin = left ? LEFT_PWM : RIGHT_PWM;
  const uint32_t in1 = left ? LEFT_IN1 : RIGHT_IN1;
  const uint32_t in2 = left ? LEFT_IN2 : RIGHT_IN2;
  int &current = left ? currentLeft : currentRight;

  if (value == 0) {
    analogWrite(pwmPin, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  } else {
    bool forward = value > 0;
    if (left) {
      digitalWrite(in1, forward ? LOW : HIGH);
      digitalWrite(in2, forward ? HIGH : LOW);
    } else {
      digitalWrite(in1, forward ? HIGH : LOW);
      digitalWrite(in2, forward ? LOW : HIGH);
    }
    analogWrite(pwmPin, abs(value));
  }
  current = value;
}

void drive(int leftValue, int rightValue) {
  if (leftValue == 0 && rightValue == 0) {
    stopMotors();
    return;
  }
  if (currentLeft * leftValue < 0 || currentRight * rightValue < 0) stopMotors();
  if (!currentLeft && !currentRight && millis() - motorsStoppedMs < REVERSE_DEAD_MS &&
      (lastLeftBeforeStop * leftValue < 0 || lastRightBeforeStop * rightValue < 0)) {
    pendingLeft = leftValue;
    pendingRight = rightValue;
    return;
  }
  pendingLeft = pendingRight = 0;
  setWheel(true, leftValue);
  setWheel(false, rightValue);
  digitalWrite(STBY_PIN, HIGH);
}

// ==================== EEPROM LUU & DOC LO TRINH ====================
bool flashLayoutOK() {
#if defined(ARDUINO_ARCH_STM32) && defined(STM32F1xx)
  extern uint8_t flashDataLoad[] asm("_sidata");
  extern uint8_t ramDataBegin[] asm("_sdata");
  extern uint8_t ramDataEnd[] asm("_edata");
  uintptr_t loadEnd = reinterpret_cast<uintptr_t>(flashDataLoad) +
                      (reinterpret_cast<uintptr_t>(ramDataEnd) - reinterpret_cast<uintptr_t>(ramDataBegin));
  return loadEnd <= 0x0800FC00UL;
#else
  return true;
#endif
}

uint32_t computeConfigHash() {
  const float vals[] = {TRACK_MM, COUNTS_PER_MM_L, COUNTS_PER_MM_R, float(LEFT_ENC_SIGN), float(RIGHT_ENC_SIGN), float(GYRO_YAW_SIGN)};
  uint32_t h = 2166136261UL;
  const uint8_t *p = reinterpret_cast<const uint8_t *>(vals);
  for (size_t i = 0; i < sizeof(vals); ++i) h = (h ^ p[i]) * 16777619UL;
  return h;
}

uint32_t computeRouteCRC(const RouteHeader &hdr) {
  uint32_t crc = 2166136261UL;
  const uint8_t *a = reinterpret_cast<const uint8_t *>(&hdr);
  for (size_t i = 0; i < 12; ++i) crc = (crc ^ a[i]) * 16777619UL;
  a = reinterpret_cast<const uint8_t *>(taught);
  for (size_t i = 0; i < hdr.count * sizeof(TeachPoint); ++i) crc = (crc ^ a[i]) * 16777619UL;
  return crc;
}

bool storageCapacityOK() {
  return flashLayoutOK() &&
         EEPROM.length() >= ROUTE_EEPROM_OFFSET + int(sizeof(RouteHeader) + sizeof(taught));
}

bool loadRouteFromFlash() {
  routeSaved = false;
  taughtCount = 0;
  if (!storageCapacityOK()) return false;

  RouteHeader hdr = {};
  EEPROM.get(ROUTE_EEPROM_OFFSET, hdr);
  if (hdr.magic != ROUTE_MAGIC || hdr.version != ROUTE_VERSION ||
      hdr.count < 2 || hdr.count > MAX_ROUTE_POINTS ||
      hdr.config != computeConfigHash()) {
    return false;
  }
  for (uint16_t i = 0; i < hdr.count; ++i) {
    EEPROM.get(ROUTE_EEPROM_OFFSET + sizeof(RouteHeader) + i * sizeof(TeachPoint), taught[i]);
  }
  if (hdr.crc != computeRouteCRC(hdr)) return false;

  taughtCount = hdr.count;
  routeSaved = true;
  return true;
}

bool invalidateRouteInFlash() {
  stopMotors();
  routeSaved = false;
  taughtCount = 0;
  if (!storageCapacityOK()) return false;
  eeprom_buffer_fill();
  for (int i = 0; i < 4; ++i) eeprom_buffered_write_byte(ROUTE_EEPROM_OFFSET + i, 0);
  eeprom_buffer_flush();
  return true;
}

bool saveRouteToFlash() {
  stopMotors();
  if (!storageCapacityOK() || taughtCount < 2) return false;
  RouteHeader hdr = {ROUTE_MAGIC, ROUTE_VERSION, taughtCount, computeConfigHash(), 0};
  hdr.crc = computeRouteCRC(hdr);

  eeprom_buffer_fill();
  const uint8_t *hdrBytes = reinterpret_cast<const uint8_t *>(&hdr);
  for (size_t i = 0; i < sizeof(hdr); ++i) {
    eeprom_buffered_write_byte(ROUTE_EEPROM_OFFSET + i, hdrBytes[i]);
  }
  const uint8_t *ptsBytes = reinterpret_cast<const uint8_t *>(taught);
  for (size_t i = 0; i < taughtCount * sizeof(TeachPoint); ++i) {
    eeprom_buffered_write_byte(ROUTE_EEPROM_OFFSET + sizeof(hdr) + i, ptsBytes[i]);
  }
  eeprom_buffer_flush();

  RouteHeader verify = {};
  EEPROM.get(ROUTE_EEPROM_OFFSET, verify);
  if (verify.magic != hdr.magic || verify.count != hdr.count || verify.crc != hdr.crc) {
    return false;
  }
  routeSaved = true;
  return true;
}

// ==================== ODOMETRY & GHI LO TRINH (LAN 1) ====================
void resetPoseToOrigin() {
  poseX = 0.0f;
  poseY = 0.0f;
  poseH = PI_F * 0.5f;  // Huong thang phia truoc la +Y
  traveledMm = 0.0f;
  lastRecordH = poseH;
  odomDeltaLeft = 0;
  odomDeltaRight = 0;
}

void recordWaypoint(bool force) {
  if (runMode != RunMode::Round1Teach) return;
  if (!force && taughtCount > 0) {
    const float dx = poseX - taught[taughtCount - 1].x;
    const float dy = poseY - taught[taughtCount - 1].y;
    const float dist = hypotf(dx, dy);
    const float dAngle = fabsf(wrapAngle(poseH - lastRecordH));
    if (dist < RECORD_STEP_MM && dAngle < RECORD_ANGLE_RAD) return;
  }
  if (taughtCount >= MAX_ROUTE_POINTS) {
    Serial.println("WARN: ROUTE_BUFFER_FULL_PRESS_S_TO_SAVE");
    return;
  }
  taught[taughtCount++] = {int16_t(lroundf(poseX)), int16_t(lroundf(poseY))};
  lastRecordH = poseH;
}

void beginRound1Fresh(bool eraseFlash) {
  stopMotors();
  bluetoothResetState();
  encoderFault = false;
  userPaused = false;
  if (eraseFlash) invalidateRouteInFlash();
  resetPoseToOrigin();
  taughtCount = 0;
  runMode = RunMode::Round1Teach;
  recordWaypoint(true);  // Luu diem xuat phat (0, 0)
  statusText = "ROUND1_BT_READY";
  Serial.println("ROUND1_BT_READY: Lai xe bang Bluetooth, bam S de luu duong.");
}

void updateOdometryAndGyro(uint32_t now) {
  const uint32_t dtMs = now - lastImuMs;
  if (dtMs < CONTROL_MS) return;
  lastImuMs = now;
  const float dt = dtMs * 0.001f;

  const int32_t dL = odomDeltaLeft;
  const int32_t dR = odomDeltaRight;
  odomDeltaLeft = 0;
  odomDeltaRight = 0;

  // 1. Cap nhat quang duong tu encoder hai banh
  const float dl = (dL * LEFT_ENC_SIGN) / COUNTS_PER_MM_L;
  const float dr = (dR * RIGHT_ENC_SIGN) / COUNTS_PER_MM_R;
  const float ds = 0.5f * (dl + dr);
  float dh = (dr - dl) / TRACK_MM;

  // 2. Doc Gyro Z neu MPU6050 san sang
  if (gyroReady) {
    uint8_t buf[2] = {};
    if (readImuReg(0x47, buf, 2)) {
      int16_t rawZ = int16_t((uint16_t(buf[0]) << 8) | buf[1]);
      float rateRawDps = GYRO_YAW_SIGN * (rawZ / 65.5f);

      if (gyroBiasSamples < 250) {
        if (dL != 0 || dR != 0 || fabsf(rateRawDps) > 6.0f) {
          gyroBiasSamples = 0;
          gyroBiasSum = 0.0f;
        } else {
          gyroBiasSum += rateRawDps;
          if (++gyroBiasSamples == 250) {
            gyroBias = gyroBiasSum / 250.0f;
            // HOAN TAT HIEU CHUAN KHI BAT NGUON -> MO KHOA NGAY!
            if (routeSaved) {
              runMode = RunMode::Round2Countdown;
              countdownStartMs = now;
              statusText = "ROUND2_AUTO_IN_3S";
              Serial.println("ROUND2_AUTO_IN_3S: Giu xe yen, tu chay sau 3 giay...");
            } else {
              beginRound1Fresh(false);
            }
          }
        }
        return;
      }

      yawRateDps = GYRO_SCALE * (rateRawDps - gyroBias);
      if (currentLeft == 0 && currentRight == 0 && dL == 0 && dR == 0 && fabsf(yawRateDps) < 0.4f) {
        gyroBias += 0.005f * (rateRawDps - gyroBias);
        yawRateDps = 0.0f;
      }
      dh = yawRateDps * DEG_TO_RAD_F * dt;
    }
  } else if (runMode == RunMode::Calibrating) {
    // Khong co MPU6050 van mo khoa ngay bang encoder de khong bao gio khoa xe
    if (routeSaved) {
      runMode = RunMode::Round2Countdown;
      countdownStartMs = now;
      statusText = "ROUND2_AUTO_IN_3S_NO_MPU";
    } else {
      beginRound1Fresh(false);
    }
  }

  // 3. Tich phan toa do (x, y, h)
  poseX += ds * cosf(poseH + 0.5f * dh);
  poseY += ds * sinf(poseH + 0.5f * dh);
  poseH = wrapAngle(poseH + dh);
  traveledMm += fabsf(ds);

  if (runMode == RunMode::Round1Teach) {
    recordWaypoint(false);
  }
}

// ==================== BAM QUY DAO & KEO XE VE DUONG LAN 1 (LAN 2) ====================
void startRound2Auto(uint32_t now) {
  if (!routeSaved || taughtCount < 2) {
    beginRound1Fresh(false);
    return;
  }
  stopMotors();
  bluetoothResetState();
  userPaused = false;
  encoderFault = false;
  pivotingObstacle = false;
  pivotDir = 0;
  stallStartMs = now;
  microReleaseUntilMs = 0;
  resetPoseToOrigin();
  autoDeltaLeft = 0;
  autoDeltaRight = 0;
  autoTargetIdx = 1;
  autoWheelL.reset(now);
  autoWheelR.reset(now);
  runMode = RunMode::Round2Auto;
  statusText = "ROUND2_AUTO_RUNNING";
  Serial.print("ROUND2_AUTO_RUNNING points=");
  Serial.println(taughtCount);
}

void updateRound2Auto(uint32_t now, float dt) {
  const int32_t dL = autoDeltaLeft;
  const int32_t dR = autoDeltaRight;
  autoDeltaLeft = 0;
  autoDeltaRight = 0;

  autoWheelL.observe(dL * LEFT_ENC_SIGN, dt, now);
  autoWheelR.observe(dR * RIGHT_ENC_SIGN, dt, now);

  // 1. Doc cam bien VL53L0X
  float frontA = 8190.0f, frontB = 8190.0f;
  float sideL = 8190.0f, sideR = 8190.0f;
  float frontMm = 8190.0f;

  if (allVl53Ready) {
    if (ranges[FRONT_A].fresh(now)) frontA = ranges[FRONT_A].mm;
    if (ranges[FRONT_B].fresh(now)) frontB = ranges[FRONT_B].mm;
    if (ranges[SIDE_L].fresh(now))  sideL  = ranges[SIDE_L].mm;
    if (ranges[SIDE_R].fresh(now))  sideR  = ranges[SIDE_R].mm;
    frontMm = fminf(frontA, frontB);
  }

  // 2. Tim diem chieu gan nhat tren doan duong Lan 1 de keo xe ve tim duong
  uint16_t scanStart = (autoTargetIdx > 1) ? (autoTargetIdx - 1) : 1;
  uint16_t scanEnd = (autoTargetIdx + 5 < taughtCount) ? (autoTargetIdx + 5) : (taughtCount - 1);
  uint16_t bestSegIdx = autoTargetIdx;
  float bestDistSq = 1e18f;
  float bestProjX = taught[autoTargetIdx].x;
  float bestProjY = taught[autoTargetIdx].y;
  float bestCte = 0.0f;

  for (uint16_t i = scanStart; i <= scanEnd; ++i) {
    const float x1 = taught[i - 1].x;
    const float y1 = taught[i - 1].y;
    const float x2 = taught[i].x;
    const float y2 = taught[i].y;
    const float dx = x2 - x1;
    const float dy = y2 - y1;
    const float lenSq = dx * dx + dy * dy;

    float t = 0.0f;
    if (lenSq > 1e-4f) {
      t = constrain(((poseX - x1) * dx + (poseY - y1) * dy) / lenSq, 0.0f, 1.0f);
    }
    const float px = x1 + t * dx;
    const float py = y1 + t * dy;
    const float ex = poseX - px;
    const float ey = poseY - py;
    const float dSq = ex * ex + ey * ey;

    if (dSq < bestDistSq) {
      bestDistSq = dSq;
      bestSegIdx = i;
      bestProjX = px;
      bestProjY = py;
      const float len = sqrtf(lenSq);
      if (len > 1e-3f) {
        // Sai so lech ngang co dau: > 0 la xe dang lech ben TRAI duong Lan 1 -> can keo sang PHAI
        bestCte = (ex * (-dy) + ey * dx) / len;
      } else {
        bestCte = 0.0f;
      }
    }
  }

  // Neo chi so muc tieu vao dung doan duong Lan 1 gan xe nhat (khong bao gio bo sot doan duong)
  if (bestSegIdx > autoTargetIdx) {
    autoTargetIdx = bestSegIdx;
  }
  while (autoTargetIdx < taughtCount - 1) {
    const float tx = taught[autoTargetIdx].x;
    const float ty = taught[autoTargetIdx].y;
    const float dx = tx - poseX;
    const float dy = ty - poseY;
    const float dist = hypotf(dx, dy);

    const float sx = tx - taught[autoTargetIdx - 1].x;
    const float sy = ty - taught[autoTargetIdx - 1].y;
    const bool passedSegment = (dx * sx + dy * sy) <= 0.0f;

    if (dist <= WAYPOINT_REACH_MM || passedSegment) {
      ++autoTargetIdx;
    } else {
      break;
    }
  }

  // 3. Kiem tra ve dich
  const float endX = taught[taughtCount - 1].x;
  const float endY = taught[taughtCount - 1].y;
  const float distToEnd = hypotf(endX - poseX, endY - poseY);
  if (autoTargetIdx >= taughtCount - 1 && distToEnd <= GOAL_REACH_MM) {
    stopMotors();
    runMode = RunMode::Round2Finished;
    statusText = "ROUND2_FINISHED";
    Serial.println("ROUND2_FINISHED: Da hoan thanh lo trinh da luu!");
    return;
  }

  // Kiem tra doan hien tai la di tien hay di lui (neu Lan 1 co chu dong lui xe)
  const float segDx = taught[autoTargetIdx].x - taught[autoTargetIdx - 1].x;
  const float segDy = taught[autoTargetIdx].y - taught[autoTargetIdx - 1].y;
  const float segAngle = atan2f(segDy, segDx);
  const bool reverseSegment = (hypotf(segDx, segDy) > 20.0f) &&
                              (fabsf(wrapAngle(segAngle - poseH)) > 2.25f);

  // 4. Xu ly vat can phia truoc bang XOAY TAI CHO (khong lui, giu nguyen moc duong Lan 1)
  if (allVl53Ready && !reverseSegment) {
    // Chi nhich lui dung 180ms (~6mm) neu banh bi ti sat vat can khong xoay duoc qua 500ms
    if (now < microReleaseUntilMs) {
      const float nudgeCps = -35.0f * COUNTS_PER_MM_L;
      drive(autoWheelL.compute(nudgeCps, dt, now), autoWheelR.compute(nudgeCps, dt, now));
      statusText = "ROUND2_MICRO_UNJAM";
      return;
    }

    if (!pivotingObstacle && frontMm < FRONT_PIVOT_MM) {
      pivotingObstacle = true;
      stallStartMs = now;
      autoWheelL.reset(now);
      autoWheelR.reset(now);

      // Quet huong sap re cua lo trinh Lan 1 de xoay dung chieu, KHONG ghi de autoTargetIdx
      float bestTurnAngle = 0.0f;
      uint16_t maxScan = (autoTargetIdx + 7 < taughtCount) ? (autoTargetIdx + 7) : (taughtCount - 1);
      for (uint16_t k = autoTargetIdx; k <= maxScan; ++k) {
        float ang = atan2f(taught[k].y - poseY, taught[k].x - poseX);
        float err = wrapAngle(ang - poseH);
        if (fabsf(err) > fabsf(bestTurnAngle)) {
          bestTurnAngle = err;
        }
      }

      if (fabsf(bestTurnAngle) > 0.30f) {
        pivotDir = (bestTurnAngle > 0.0f) ? 1 : -1;
      } else {
        float leftSpace = fminf(sideL, frontA);
        float rightSpace = fminf(sideR, frontB);
        pivotDir = (leftSpace >= rightSpace) ? 1 : -1;
      }
    }

    if (pivotingObstacle) {
      if (frontMm >= FRONT_RESUME_MM) {
        pivotingObstacle = false;
        pivotDir = 0;
        autoWheelL.reset(now);
        autoWheelR.reset(now);
      } else {
        if (dL != 0 || dR != 0) {
          stallStartMs = now;
        } else if (now - stallStartMs > 500) {
          microReleaseUntilMs = now + 180;
          stallStartMs = now + 180;
          autoWheelL.reset(now);
          autoWheelR.reset(now);
          return;
        }

        const float pivotOmega = pivotDir * 0.65f;
        float targetCpsL = (-pivotOmega * (TRACK_MM * 0.5f)) * COUNTS_PER_MM_L;
        float targetCpsR = (pivotOmega * (TRACK_MM * 0.5f)) * COUNTS_PER_MM_R;
        if (fabsf(targetCpsL) < AUTO_MIN_WHEEL_CPS) targetCpsL = (targetCpsL >= 0.0f) ? AUTO_MIN_WHEEL_CPS : -AUTO_MIN_WHEEL_CPS;
        if (fabsf(targetCpsR) < AUTO_MIN_WHEEL_CPS) targetCpsR = (targetCpsR >= 0.0f) ? AUTO_MIN_WHEEL_CPS : -AUTO_MIN_WHEEL_CPS;
        drive(autoWheelL.compute(targetCpsL, dt, now), autoWheelR.compute(targetCpsR, dt, now));
        statusText = "ROUND2_PIVOT_OBS";
        return;
      }
    }
  }

  // 5. Tinh diem nhin truoc tu diem chieu (bestProjX, bestProjY) doc theo quy dao Lan 1
  float lookX = taught[bestSegIdx].x;
  float lookY = taught[bestSegIdx].y;
  float accum = hypotf(lookX - bestProjX, lookY - bestProjY);
  for (uint16_t i = bestSegIdx; i + 1 < taughtCount && accum < LOOKAHEAD_MM; ++i) {
    float segLen = hypotf(taught[i + 1].x - taught[i].x, taught[i + 1].y - taught[i].y);
    accum += segLen;
    lookX = taught[i + 1].x;
    lookY = taught[i + 1].y;
  }

  const float targetAngle = reverseSegment
                                ? wrapAngle(atan2f(lookY - poseY, lookX - poseX) + PI_F)
                                : atan2f(lookY - poseY, lookX - poseX);
  float headingErr = wrapAngle(targetAngle - poseH);

  // Cong them goc keo ngang (Cross-Track Pull-Back) de keo xe nhap lai dung duong Lan 1 khi bi lech
  if (!reverseSegment) {
    const float pullAngle = -atan2f(CTE_PULL_GAIN * bestCte, 65.0f);
    headingErr = wrapAngle(headingErr + 0.65f * pullAngle);
  }

  // Neu goc lech > 34 do (0.60 rad): xoay tai cho de om cua gon trong bien
  const bool turnInPlace = fabsf(headingErr) > 0.60f;

  float vMmS = 0.0f;
  float omega = constrain(headingErr * 1.75f, -AUTO_MAX_OMEGA, AUTO_MAX_OMEGA);

  // Khi dang luot qua mep vat can, khoa goc be nguoc vao vat can cho toi khi mui xe thoat han
  if (allVl53Ready && !reverseSegment && !turnInPlace) {
    if (frontB < CORNER_GUARD_MM && omega < -0.10f) {
      omega = -0.10f;  // Vat can con o goc truoc-phai -> chua be gap sang phai
    }
    if (frontA < CORNER_GUARD_MM && omega > 0.10f) {
      omega = 0.10f;   // Vat can con o goc truoc-trai -> chua be gap sang trai
    }
  }

  if (turnInPlace) {
    vMmS = 0.0f;
    if (fabsf(omega) < 0.35f) omega = (headingErr >= 0.0f) ? 0.35f : -0.35f;
    statusText = "ROUND2_TURN";
  } else {
    float speedScale = constrain(1.0f - 1.05f * fabsf(headingErr), 0.22f, 1.0f);
    if (distToEnd < 120.0f) {
      speedScale = fminf(speedScale, constrain(distToEnd / 120.0f, 0.25f, 1.0f));
    }
    if (allVl53Ready && frontMm < FRONT_SLOW_MM) {
      float obsScale = constrain((frontMm - FRONT_PIVOT_MM) / (FRONT_SLOW_MM - FRONT_PIVOT_MM), 0.20f, 1.0f);
      speedScale = fminf(speedScale, obsScale);
    }
    const float magSpeed = AUTO_MIN_SPEED_MM_S + (AUTO_MAX_SPEED_MM_S - AUTO_MIN_SPEED_MM_S) * speedScale;
    vMmS = reverseSegment ? -magSpeed : magSpeed;
    statusText = (fabsf(bestCte) > 25.0f) ? "ROUND2_PULLBACK" : "ROUND2_FOLLOW";
  }

  // Doi sang van toc goc banh trai/phai (counts/s)
  const float vLeftMmS = vMmS - omega * (TRACK_MM * 0.5f);
  const float vRightMmS = vMmS + omega * (TRACK_MM * 0.5f);

  float targetCpsL = vLeftMmS * COUNTS_PER_MM_L;
  float targetCpsR = vRightMmS * COUNTS_PER_MM_R;
  if (fabsf(targetCpsL) > 1.0f && fabsf(targetCpsL) < AUTO_MIN_WHEEL_CPS) {
    targetCpsL = (targetCpsL > 0.0f) ? AUTO_MIN_WHEEL_CPS : -AUTO_MIN_WHEEL_CPS;
  }
  if (fabsf(targetCpsR) > 1.0f && fabsf(targetCpsR) < AUTO_MIN_WHEEL_CPS) {
    targetCpsR = (targetCpsR > 0.0f) ? AUTO_MIN_WHEEL_CPS : -AUTO_MIN_WHEEL_CPS;
  }

  const int pwmL = autoWheelL.compute(targetCpsL, dt, now);
  const int pwmR = autoWheelR.compute(targetCpsR, dt, now);
  drive(pwmL, pwmR);
}

void printStatus() {
  Serial.print("ST mode=");
  Serial.print(statusText);
  Serial.print(" paused=");
  Serial.print(userPaused ? 1 : 0);
  Serial.print(" pts=");
  Serial.print(taughtCount);
  Serial.print(" idx=");
  Serial.print(autoTargetIdx);
  Serial.print(" X=");
  Serial.print(lroundf(poseX));
  Serial.print(" Y=");
  Serial.print(lroundf(poseY));
  Serial.print(" H=");
  Serial.print(poseH * RAD_TO_DEG_F, 1);
  Serial.print(" PWM=");
  Serial.print(currentLeft);
  Serial.print(",");
  Serial.println(currentRight);
}

// ==================== SETUP & LOOP CHINH ====================
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

  for (uint8_t i = 0; i < VL53_COUNT; ++i) {
    holdLow(XSHUT_PINS[i]);
  }

  Serial.setRx(PA10);
  Serial.setTx(PA9);
  Serial.begin(BT_BAUD, SERIAL_8N1);

  initHardwareEncoders();
  loadVl53Offsets();
  allVl53Ready = initVl53Sensors();
  initImu();

  // Kiem tra Flash xem da co duong luu tu Lan 1 chua
  routeSaved = loadRouteFromFlash();
  userPaused = false;  // KHONG KHOA XE KHI BAT NGUON

  if (!gyroReady) {
    if (routeSaved) {
      runMode = RunMode::Round2Countdown;
      countdownStartMs = millis();
      statusText = "ROUND2_AUTO_IN_3S_NO_MPU";
    } else {
      beginRound1Fresh(false);
    }
  } else {
    runMode = RunMode::Calibrating;
    statusText = routeSaved ? "CAL_GYRO_THEN_AUTO" : "CAL_GYRO_THEN_BT";
    Serial.println(routeSaved ? "BOOT: Co duong luu -> Chuan bi tu chay Lan 2..."
                              : "BOOT: Chua co duong -> Chuan bi mo khoa lai Bluetooth Lan 1...");
  }
}

void loop() {
  const uint32_t now = millis();
  readEncoders();
  bluetoothProcessSerial();
  updateOdometryAndGyro(now);

  // Doc luan phien 4 cam bien VL53L0X
  static uint8_t nextSensor = 0;
  static uint32_t lastSensorPollMs = 0;
  if (now - lastSensorPollMs >= 3) {
    lastSensorPollMs = now;
    pollSingleVl53(nextSensor);
    nextSensor = (nextSensor + 1) % VL53_COUNT;
  }

  // Xu ly dem nguoc tu chay Lan 2 sau khi bat nguon
  if (runMode == RunMode::Round2Countdown && !userPaused) {
    if (now - countdownStartMs >= AUTO_BOOT_DELAY_MS) {
      startRound2Auto(now);
    }
  }

  // Vong dieu khien Lan 2 (tu dong bam quy dao da luu)
  static uint32_t lastAutoControlMs = 0;
  if (runMode == RunMode::Round2Auto && !userPaused) {
    if (now - lastAutoControlMs >= CONTROL_MS) {
      float dt = (now - lastAutoControlMs) * 0.001f;
      if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;
      lastAutoControlMs = now;
      updateRound2Auto(now, dt);
    }
  }

  // Vong dieu khien Lan 1 (lai tay qua Bluetooth)
  if (runMode == RunMode::Round1Teach || bluetoothIsWheelTestActive()) {
    bluetoothUpdateManual(now);
  }

  // Hien thi trang thai bang LED PC13 (active LOW)
  if (statusText && !strcmp(statusText, "ROUTE_SAVED_POWER_OFF_ALLOWED")) {
    digitalWrite(LED_PIN, ((now / 700) & 1U) ? LOW : HIGH);  // Nhay cham: da luu xong, duoc phep tat nguon
  } else if (runMode == RunMode::Round2Countdown) {
    digitalWrite(LED_PIN, ((now / 150) & 1U) ? LOW : HIGH);  // Nhay nhanh 3s: sap tu chay Lan 2
  } else {
    digitalWrite(LED_PIN, (currentLeft != 0 || currentRight != 0) ? LOW : HIGH);
  }
}