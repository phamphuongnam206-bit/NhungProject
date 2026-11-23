#include <Arduino.h>
#include <stdint.h>
#include <math.h>

// ==================== THONG SO PWM & CHUNG ====================
constexpr int MIN_PWM = 80;
constexpr int MAX_PWM = 136;
constexpr int DEFAULT_PWM = 112;
constexpr uint32_t REVERSE_DEAD_MS = 25;

// ==================== BANG MA LENH BLUETOOTH ====================
constexpr char CMD_FORWARD    = 'F';
constexpr char CMD_BACKWARD   = 'B';
constexpr char CMD_LEFT       = 'L';
constexpr char CMD_RIGHT      = 'R';
constexpr char CMD_FWD_LEFT   = 'G';
constexpr char CMD_FWD_RIGHT  = 'I';
constexpr char CMD_BACK_LEFT  = 'H';
constexpr char CMD_BACK_RIGHT = 'J';
constexpr char CMD_STOP       = 'S';  // Trong Lan 1: Luu duong; khi dung/Lan 2: Dung
constexpr char CMD_STOP_ALT   = 'X';
constexpr char CMD_RELEASE    = '0';
constexpr char CMD_MAXSPEED   = 'Q';
constexpr char CMD_STATUS     = '?';
constexpr char CMD_SPEED_UP   = 'U';
constexpr char CMD_SPEED_DOWN = 'D';
constexpr char CMD_SPEED_MID  = 'M';
constexpr char CMD_START      = 'A';  // Mo khoa / bat dau ngay
constexpr char CMD_PAUSE      = 'P';  // Tam dung & khoa chu dong
constexpr char CMD_CLEAR_REC  = 'C';  // Xoa duong cu & bat dau ghi Lan 1 moi
constexpr char CMD_TEST_LEFT  = 'T';
constexpr char CMD_TEST_RIGHT = 'Y';

// ==================== TRANG THAI & HAM TU XE_FULL.CPP ====================
enum class RunMode : uint8_t {
  Calibrating,
  Round1Teach,
  Round2Countdown,
  Round2Auto,
  Round2Finished,
  Paused
};

extern RunMode runMode;
extern const char *statusText;
extern bool userPaused;
extern bool encoderFault;

extern bool gyroReady;
extern float yawRateDps;

extern int currentLeft;
extern int currentRight;
extern int lastLeftBeforeStop;
extern int lastRightBeforeStop;
extern uint32_t motorsStoppedMs;
extern int pendingLeft;
extern int pendingRight;
extern bool turnBraking;

extern int32_t leftTotal;
extern int32_t rightTotal;

extern uint16_t taughtCount;
extern bool routeSaved;
extern float traveledMm;

int32_t counterDifference(uint16_t current, uint16_t previous);
void stopMotors();
void brakeTurn();
void drive(int leftValue, int rightValue);
void recordWaypoint(bool force = false);
bool saveRouteToFlash();
void beginRound1Fresh(bool eraseFlash);
void startRound2Auto(uint32_t now);
void printStatus();

void bluetoothFinishMotion();

// ==================== THONG SO & XU LY LAI TAY (LAN 1) ====================
namespace {
constexpr int SPEED_STEP = 6;
constexpr int PIVOT_PWM = 90;
constexpr int TURN_DIFFERENCE = 28;
constexpr float PIVOT_TARGET_DEG = 20.0f;
constexpr float ARC_TARGET_DEG = 35.0f;
constexpr float TURN_APPROACH_DEG = 8.0f;
constexpr float TURN_COAST_SECONDS = 0.020f;
constexpr float TURN_MAX_LEAD_DEG = 0.8f;
constexpr uint32_t TURN_TIMEOUT_MS = 1500;
constexpr uint32_t TURN_BRAKE_MIN_MS = 140;
constexpr uint32_t DRIVE_COMMAND_TIMEOUT_MS = 667;
constexpr uint32_t WHEEL_TEST_MS = 400;
constexpr int WHEEL_TEST_PWM = 112;

constexpr uint32_t LAUNCH_WINDOW_MS = 200;
constexpr uint32_t LAUNCH_SAMPLE_MS = 25;
constexpr uint32_t FORWARD_LAUNCH_SAMPLE_MS = 15;
constexpr float SYNC_PWM_KP = 24.0f;
constexpr float SYNC_PWM_KI = 180.0f;
constexpr float HEADING_SYNC_MAX_RATIO = 0.20f;
constexpr float STRAIGHT_TRIM_LIMIT_PWM = 44.0f;
constexpr float HEADING_PWM_PER_DEG = 5.0f;
constexpr float HEADING_PWM_PER_DPS = 0.10f;
constexpr float HEADING_PWM_I_GAIN = 3.0f;
constexpr float HEADING_PWM_I_LIMIT = 12.0f;
constexpr float HEADING_TOLERANCE_DEG = 0.20f;
constexpr uint32_t HEADING_COAST_MS = 180;
constexpr uint32_t BALANCE_MS = 25;
constexpr uint32_t STALL_MS = 500;

int speedPwm = DEFAULT_PWM;
char activeCmd = CMD_RELEASE;

uint32_t sampleAt = 0;
uint32_t lastLeftPulse = 0;
uint32_t lastRightPulse = 0;
uint16_t sampleLeft = 0;
uint16_t sampleRight = 0;

float straightTrim = 0.0f;
float syncIntegralPwm = 0.0f;
uint32_t runStartMs = 0;
int32_t runStartLeft = 0;
int32_t runStartRight = 0;
uint32_t lastDriveCommandMs = 0;

char settlingCmd = 0;
uint32_t turnStoppedMs = 0;
uint32_t lastRotationMs = 0;
float motionYawDeg = 0.0f;
uint32_t gyroSampleMs = 0;

bool straightHeadingValid = false;
float straightHeadingErrorDeg = 0.0f;
float headingIntegralPwm = 0.0f;
uint32_t straightStoppedMs = 0;

bool isPivotCommand(char c)    { return c == CMD_LEFT || c == CMD_RIGHT; }
bool isForwardArc(char c)      { return c == CMD_FWD_LEFT || c == CMD_FWD_RIGHT; }
bool isReverseArc(char c)      { return c == CMD_BACK_LEFT || c == CMD_BACK_RIGHT; }
bool isArcCommand(char c)      { return isForwardArc(c) || isReverseArc(c); }
bool turnsBodyLeft(char c)     { return c == CMD_LEFT || c == CMD_FWD_LEFT || c == CMD_BACK_RIGHT; }
bool isStraightCommand(char c) { return c == CMD_FORWARD || c == CMD_BACKWARD; }
bool isMeasuredTurn(char c)    { return isPivotCommand(c) || isArcCommand(c); }
bool isDriveCommand(char c)    { return isStraightCommand(c); }
bool isWheelTest(char c)       { return c == CMD_TEST_LEFT || c == CMD_TEST_RIGHT; }
bool isMotionCommand(char c)   { return isDriveCommand(c) || isMeasuredTurn(c); }

float headingTrimPwm(char cmd) {
  const float steerSign = (cmd == CMD_FORWARD) ? -1.0f : 1.0f;
  const float correction = HEADING_PWM_PER_DEG * straightHeadingErrorDeg +
                           HEADING_PWM_PER_DPS * yawRateDps + headingIntegralPwm;
  return constrain(steerSign * correction, -STRAIGHT_TRIM_LIMIT_PWM, STRAIGHT_TRIM_LIMIT_PWM);
}

void applyManualMotion(char c) {
  const int full = speedPwm;
  const int inner = constrain(full - TURN_DIFFERENCE, MIN_PWM, MAX_PWM);

  if (isStraightCommand(c)) {
    const int sign = (c == CMD_FORWARD) ? 1 : -1;
    const int trim = int(straightTrim + (straightTrim >= 0 ? 0.5f : -0.5f));
    const int leftPwm = constrain(full - trim, 0, MAX_PWM);
    const int rightPwm = constrain(full + trim, 0, MAX_PWM);
    drive(sign * leftPwm, sign * rightPwm);
  } else if (isMeasuredTurn(c)) {
    const bool left = turnsBodyLeft(c);
    const bool pivot = isPivotCommand(c);
    const float progress = left ? motionYawDeg : -motionYawDeg;
    const float remaining = (pivot ? PIVOT_TARGET_DEG : ARC_TARGET_DEG) - progress;
    const float scale = gyroReady ? constrain(remaining / TURN_APPROACH_DEG, 0.0f, 1.0f) : 1.0f;
    if (pivot) {
      const int pwm = MIN_PWM + int((PIVOT_PWM - MIN_PWM) * scale + 0.5f);
      drive(left ? -pwm : pwm, left ? pwm : -pwm);
    } else {
      const int outer = full < MIN_PWM + 12 ? MIN_PWM + 12 : full;
      const int gap = outer - inner;
      const int slowGap = (gap + 1) / 2;
      const int outerPwm = inner + slowGap + int((gap - slowGap) * scale + 0.5f);
      if (isReverseArc(c)) {
        drive(c == CMD_BACK_LEFT ? -inner : -outerPwm,
              c == CMD_BACK_LEFT ? -outerPwm : -inner);
      } else {
        drive(left ? inner : outerPwm, left ? outerPwm : inner);
      }
    }
  } else {
    stopMotors();
  }
}

void beginManualSample(uint32_t now) {
  if (turnBraking) stopMotors();
  settlingCmd = 0;
  runStartMs = now;
  gyroSampleMs = now;
  motionYawDeg = 0.0f;
  sampleAt = lastLeftPulse = lastRightPulse = now;
  sampleLeft = uint16_t(TIM3->CNT);
  sampleRight = uint16_t(TIM2->CNT);
  runStartLeft = leftTotal;
  runStartRight = rightTotal;
  straightTrim = 0.0f;
  syncIntegralPwm = 0.0f;
  headingIntegralPwm = 0.0f;
}

void updateManualGyro(uint32_t now) {
  const bool arc = isArcCommand(activeCmd);
  const bool turn = isMeasuredTurn(activeCmd);
  const bool straight = isStraightCommand(activeCmd);
  const bool coasting = (activeCmd == CMD_RELEASE) && straightHeadingValid &&
                        (now - straightStoppedMs < HEADING_COAST_MS);
  if (!gyroReady || (!turn && !straight && !coasting && !settlingCmd)) return;

  const uint32_t elapsed = now - gyroSampleMs;
  if (elapsed < 10) return;
  gyroSampleMs = now;

  const float deltaYawDeg = yawRateDps * (elapsed * 0.001f);
  if (straight || turn || settlingCmd) motionYawDeg += deltaYawDeg;
  if (straight || coasting) straightHeadingErrorDeg += deltaYawDeg;

  if (settlingCmd) {
    if (fabsf(yawRateDps) > 2.0f) lastRotationMs = now;
    if ((now - lastRotationMs >= 70) || (now - turnStoppedMs >= 350)) settlingCmd = 0;
    return;
  }
  if (straight) {
    if (fabsf(straightHeadingErrorDeg) > HEADING_TOLERANCE_DEG) {
      const float step = straightHeadingErrorDeg * HEADING_PWM_I_GAIN * (elapsed * 0.001f);
      headingIntegralPwm = constrain(headingIntegralPwm + step, -HEADING_PWM_I_LIMIT, HEADING_PWM_I_LIMIT);
    }
    return;
  }
  if (!turn) return;

  const char cmd = activeCmd;
  const float turnProgress = turnsBodyLeft(cmd) ? motionYawDeg : -motionYawDeg;
  const float targetDeg = arc ? ARC_TARGET_DEG : PIVOT_TARGET_DEG;
  const float rateToward = turnsBodyLeft(cmd) ? yawRateDps : -yawRateDps;
  const float lead = constrain(rateToward * TURN_COAST_SECONDS, 0.0f, TURN_MAX_LEAD_DEG);
  if (turnProgress + lead >= targetDeg || now - runStartMs >= TURN_TIMEOUT_MS) {
    bluetoothFinishMotion();
  } else if (targetDeg - turnProgress <= TURN_APPROACH_DEG) {
    applyManualMotion(cmd);
  }
}

void updateManualWheelFeedback(uint32_t now) {
  const bool straight = isStraightCommand(activeCmd);
  const bool pivot = isPivotCommand(activeCmd);
  const bool launching = straight && (now - runStartMs < LAUNCH_WINDOW_MS);
  const uint32_t intervalMs = launching
                                  ? (activeCmd == CMD_FORWARD ? FORWARD_LAUNCH_SAMPLE_MS : LAUNCH_SAMPLE_MS)
                                  : BALANCE_MS;
  const uint32_t sampleElapsed = now - sampleAt;
  if ((!straight && !pivot) || sampleElapsed < intervalMs) return;

  const uint16_t left = uint16_t(TIM3->CNT);
  const uint16_t right = uint16_t(TIM2->CNT);
  const int leftDelta = counterDifference(left, sampleLeft);
  const int rightDelta = counterDifference(right, sampleRight);
  sampleLeft = left;
  sampleRight = right;
  sampleAt = now;

  if (leftDelta != 0) lastLeftPulse = now;
  if (rightDelta != 0) lastRightPulse = now;
  if (now - lastLeftPulse >= STALL_MS || now - lastRightPulse >= STALL_MS) {
    bluetoothFinishMotion();
    encoderFault = true;
    Serial.println("ENCODER_STALL");
    return;
  }
  if (!straight) return;

  const bool forward = (activeCmd == CMD_FORWARD);
  const int directedLeft = forward ? -leftDelta : leftDelta;
  const int directedRight = forward ? rightDelta : -rightDelta;
  const int leftSpeedTicks = directedLeft > 0 ? directedLeft : 0;
  const int rightSpeedTicks = directedRight > 0 ? directedRight : 0;
  const int totalTicks = leftSpeedTicks + rightSpeedTicks;

  if (totalTicks >= 3) {
    const float mismatch = 2.0f * (leftSpeedTicks - rightSpeedTicks) / float(totalTicks);
    float targetMismatch = 0.0f;
    if (gyroReady && fabsf(straightHeadingErrorDeg) > HEADING_TOLERANCE_DEG) {
      targetMismatch = constrain(-headingTrimPwm(activeCmd) / float(speedPwm),
                                 -HEADING_SYNC_MAX_RATIO, HEADING_SYNC_MAX_RATIO);
    }
    const float error = mismatch - targetMismatch;
    const float limit = float(speedPwm);
    const float prop = SYNC_PWM_KP * error;
    const float candI = constrain(syncIntegralPwm + SYNC_PWM_KI * error * (sampleElapsed * 0.001f), -limit, limit);
    const float candOut = prop + candI;
    if ((candOut >= -limit && candOut <= limit) || (candOut > limit && error < 0) || (candOut < -limit && error > 0)) {
      syncIntegralPwm = candI;
    }
    straightTrim = constrain(prop + syncIntegralPwm, -limit, limit);
  }
  applyManualMotion(activeCmd);
}

void handleSingleCommand(char c) {
  if (c == '\r' || c == '\n' || c == ' ') return;

  // P: Khoa / tam dung chu dong
  if (c == CMD_PAUSE) {
    userPaused = true;
    runMode = RunMode::Paused;
    statusText = "PAUSED_BY_USER";
    bluetoothFinishMotion();
    Serial.println("PAUSED: Bam A de mo khoa.");
    return;
  }

  // A: Mo khoa (hoac chay ngay Lan 2 neu dang dem nguoc / da luu)
  if (c == CMD_START) {
    userPaused = false;
    encoderFault = false;
    if (runMode == RunMode::Round2Countdown || runMode == RunMode::Round2Finished) {
      startRound2Auto(millis());
    } else if (routeSaved && runMode == RunMode::Paused) {
      startRound2Auto(millis());
    } else if (runMode == RunMode::Paused || runMode == RunMode::Calibrating) {
      if (taughtCount >= 1) {
        runMode = RunMode::Round1Teach;
        statusText = "ROUND1_BT_READY";
        Serial.println("UNLOCKED: Tiep tuc ghi Lan 1.");
      } else {
        beginRound1Fresh(false);
      }
    }
    return;
  }

  // C: Xoa duong cu & bat dau ghi lai Lan 1 moi tu (0,0)
  if (c == CMD_CLEAR_REC) {
    beginRound1Fresh(true);
    return;
  }

  // S: Trong Lan 1 -> Luu duong vao Flash; o che do khac -> Dung xe
  if (c == CMD_STOP) {
    if (runMode == RunMode::Round1Teach) {
      bluetoothFinishMotion();
      recordWaypoint(true);
      if (taughtCount < 2 || traveledMm < 40.0f) {
        Serial.println("SAVE_FAIL: Xe chua di chuyen du doan duong!");
        return;
      }
      if (saveRouteToFlash()) {
        runMode = RunMode::Paused;
        statusText = "ROUTE_SAVED_POWER_OFF_ALLOWED";
        Serial.print("ROUTE_SAVED_POWER_OFF_ALLOWED points=");
        Serial.println(taughtCount);
      } else {
        Serial.println("FLASH_SAVE_ERROR");
      }
    } else {
      bluetoothFinishMotion();
      if (runMode == RunMode::Round2Auto || runMode == RunMode::Round2Countdown) {
        runMode = RunMode::Paused;
        statusText = "STOPPED";
      }
    }
    return;
  }

  // Thu rieng tung banh khi da bam P
  if (isWheelTest(c)) {
    if (!userPaused) {
      Serial.println("WHEEL_TEST: Bam P truoc khi thu banh.");
      return;
    }
    beginManualSample(millis());
    activeCmd = c;
    drive(c == CMD_TEST_LEFT ? WHEEL_TEST_PWM : 0,
          c == CMD_TEST_RIGHT ? WHEEL_TEST_PWM : 0);
    return;
  }

  // Dieu chinh toc do tay lai
  if (c == CMD_SPEED_UP) {
    speedPwm = constrain(speedPwm + SPEED_STEP, MIN_PWM, MAX_PWM);
    if (isMotionCommand(activeCmd)) applyManualMotion(activeCmd);
    return;
  }
  if (c == CMD_SPEED_DOWN) {
    speedPwm = constrain(speedPwm - SPEED_STEP, MIN_PWM, MAX_PWM);
    if (isMotionCommand(activeCmd)) applyManualMotion(activeCmd);
    return;
  }
  if (c == CMD_SPEED_MID) {
    speedPwm = DEFAULT_PWM;
    if (isMotionCommand(activeCmd)) applyManualMotion(activeCmd);
    return;
  }
  if (c == CMD_MAXSPEED) {
    speedPwm = MAX_PWM;
    if (isMotionCommand(activeCmd)) applyManualMotion(activeCmd);
    return;
  }
  if (c >= '1' && c <= '9') {
    speedPwm = constrain(MIN_PWM + (c - '0') * SPEED_STEP, MIN_PWM, MAX_PWM);
    if (isMotionCommand(activeCmd)) applyManualMotion(activeCmd);
    return;
  }

  // Lenh lai Bluetooth (F, B, L, R, G, I, H, J)
  if (isMotionCommand(c)) {
    if (userPaused) {
      Serial.println("LOCKED_BY_P: Bam A de mo khoa.");
      return;
    }
    if (runMode == RunMode::Calibrating) {
      Serial.println("WAIT_GYRO_2S: Dang on dinh cam bien goc...");
      return;
    }
    // Neu dang cho Lan 2 ma nguoi dung chu dong lai tay, chuyen sang ghi Lan 1 moi
    if (runMode == RunMode::Round2Countdown || runMode == RunMode::Round2Finished) {
      beginRound1Fresh(true);
    }
    if (runMode != RunMode::Round1Teach) return;

    if (isDriveCommand(c)) lastDriveCommandMs = millis();
    if (activeCmd == c) return;

    beginManualSample(millis());
    if (!isStraightCommand(c)) {
      straightHeadingValid = false;
    } else if (gyroReady && !straightHeadingValid) {
      straightHeadingErrorDeg = 0.0f;
      straightHeadingValid = true;
    }
    activeCmd = c;
    applyManualMotion(c);
    return;
  }

  // Nha phim dieu huong ('0' hoac 'X')
  if (c == CMD_RELEASE || c == CMD_STOP_ALT) {
    if (runMode == RunMode::Round1Teach || isWheelTest(activeCmd)) {
      bluetoothFinishMotion();
    } else if (c == CMD_STOP_ALT && (runMode == RunMode::Round2Auto || runMode == RunMode::Round2Countdown)) {
      stopMotors();
      runMode = RunMode::Paused;
      statusText = "STOPPED";
    }
    return;
  }

  if (c == CMD_STATUS) {
    printStatus();
    return;
  }
}
}  // namespace

// ==================== HAM CONG KHAI CHO XE_FULL.CPP ====================
void bluetoothResetState() {
  activeCmd = CMD_RELEASE;
  settlingCmd = 0;
  straightHeadingValid = false;
  straightHeadingErrorDeg = 0.0f;
  headingIntegralPwm = 0.0f;
  straightTrim = 0.0f;
  syncIntegralPwm = 0.0f;
}

void bluetoothFinishMotion() {
  const char endedCmd = activeCmd;
  if (endedCmd == CMD_RELEASE && turnBraking) return;
  if (isStraightCommand(endedCmd)) straightStoppedMs = millis();
  if (gyroReady && isMeasuredTurn(endedCmd)) {
    settlingCmd = endedCmd;
    turnStoppedMs = lastRotationMs = millis();
  }
  activeCmd = CMD_RELEASE;
  if (isMeasuredTurn(endedCmd)) brakeTurn();
  else stopMotors();
  if (runMode == RunMode::Round1Teach) recordWaypoint(true);
}

bool bluetoothIsWheelTestActive() {
  return isWheelTest(activeCmd);
}

void bluetoothProcessSerial() {
  static bool pendingS = false;
  static uint32_t sReceivedMs = 0;

  while (Serial.available() > 0) {
    char c = char(Serial.read());
    if (pendingS) {
      pendingS = false;
      if (c == 'T') {
        printStatus();
        continue;
      }
      handleSingleCommand(CMD_STOP);
      if (c == '\r' || c == '\n') continue;
    }
    if (c == CMD_STOP) {
      pendingS = true;
      sReceivedMs = millis();
    } else {
      handleSingleCommand(c);
    }
  }

  if (pendingS && (millis() - sReceivedMs >= 80)) {
    pendingS = false;
    handleSingleCommand(CMD_STOP);
  }
}

void bluetoothUpdateManual(uint32_t now) {
  if (isWheelTest(activeCmd) && (now - runStartMs >= WHEEL_TEST_MS)) {
    bluetoothFinishMotion();
  }
  if (isDriveCommand(activeCmd) && (now - lastDriveCommandMs >= DRIVE_COMMAND_TIMEOUT_MS)) {
    bluetoothFinishMotion();
  }
  if (pendingLeft || pendingRight) {
    if (now - motorsStoppedMs >= REVERSE_DEAD_MS) {
      drive(pendingLeft, pendingRight);
      beginManualSample(now);
    }
  }
  updateManualGyro(now);
  updateManualWheelFeedback(now);
  if (turnBraking &&
      (now - turnStoppedMs >= 350 || (now - turnStoppedMs >= TURN_BRAKE_MIN_MS && !settlingCmd))) {
    stopMotors();
  }
}