/* LEAN MEMORY FIX: oriented planner costs uint8_t (safe saturated), compact logging.
Removed legacy hand-follow execution and MOTOR/SWEEP/BAL/CAL commands from map firmware.
Retains wheel PI, direction interlock, STALL/ENC_DIR, freshness, SysTick guard,
VL53 init/read parsing, saved calibration loading, teaching/Flash/autoboot,
#GYROZERO/#CALBEGIN/#CALREAD/#CALEND and map status.
Compile on target and inspect RAM/Flash summary; host tests are not target size verification.
*/
/* TWO ROUNDS / WHOLE-CAR POWER SWITCH / BLUETOOTH9600
CURRENT INSTRUCTIONS (override historical comments below):
FIRST BOOT WITHOUT SAVED ROUTE: keep still for MPU bias; START A teaches round1.
Arrow F/B/L/R; release0 stops. U/D/M speeds. Z ignored. P pauses; A resumes.
At end round1: SQUARE S STOPS and SAVES route to FLASH, read-back verified.
Wait at least2 seconds; saved status is ROUTE_SAVED_POWER_OFF_ALLOWED,
PC13 blinks slowly (700ms half period). Only then switch whole-car power OFF.
Place axle centre at x325,y125mm, heading +Y, then switch power ON.
NEXT BOOT WITH VALID SAVED ROUTE: motors locked during MPU calibration,
then wait5 seconds with fresh sensors; AUTO starts WITHOUT PHONE COMMANDS.
KEEP VEHICLE STILL THROUGH CALIBRATION AND COUNTDOWN.
Every later power-on repeats AUTO until a new teaching run is requested.
To erase and teach again, send C during boot/countdown; this invalidates old route.
Bluetooth is not needed for round2. P still cancels countdown/stops as an optional override.
Slow scan <450mm <=18mm/s; stop translation <220mm; >=6 fresh frames and350ms.
No line sensing, no rear sensor; geometry/counts-per-mm remain uncalibrated.
Flash layout: reserve last1KiB of physical64KiB F103C8 for EEPROM emulation.
Add board_upload.maximum_size = 64512 in [env:map_auto]. Runtime also checks
firmware load end before touching EEPROM; layout error disables save and auto.
Preserve existing calibration at EEPROM0; route starts at64, CRC+config validated.
Do not cut power during save. No EEPROM writes while driving.
Calibration diagnostic (terminal BT/UART9600, newline):
#CALBEGIN locks motors and zeros signed counters + raw gyro integral.
Move vehicle BY HAND; stop motion, then #CALEND prints measurement.
#CALREAD intermediate read is for stationary vehicle only.
Calibration never writes constants automatically. Edit counts/mm, TRACK,
GYRO_SIGN/GYRO_SCALE from measured data then rebuild/reteach.
Only host tests performed; STM32 build/flash and physical behavior NOT verified.
*/
/* MAP AUTO - single translation unit, based on supplied pid25.
Build: STM32duino BluePill F103C8 + Pololu VL53L0X 1.3.1.
Only compile THIS cpp, not other files with setup/loop.
PlatformIO: save as src/xe_map_auto.cpp. Add to existing platformio.ini:
[env:map_auto]
build_src_filter = -<*> +<xe_map_auto.cpp>
Build/upload with: pio run -e map_auto -t upload
Uses existing base env: ststm32, bluepill_f103c8, arduino, stlink,
pololu/VL53L0X@^1.3.1. Hardware build has NOT been run here.

HUONG DAN:
1. Giu xe dung yen khi bat nguon de lay bias MPU (~3 giay).
2. Dat TAM TRUC HAI BANH tai X=325mm, Y=125mm, dau huong len map.
   Goc (0,0) la goc duoi-trai map. Khong reset vi tri giua duong.
3. UART115200, ket thuc Newline: MAPRESET, sau do AUTO.
4. STOP dung; AUTO tiep tuc tu pose hien tai. MAP xem pose/trang thai.
5. Neu FAULT: sua nguyen nhan va reset board. Khong tu xoa fault.
6. Counts/mm=10 la UOC LUONG. Do va sua COUNTS_MM_L/R truoc khi chay san.
7. Hinh hoc hop, vi tri cam bien va drift budget la UOC LUONG tu anh.
   Khong duoc coi day la code da kiem chung tren xe.
8. Khong co nut chuyen mode gan vao mot chan tu doan; dung UART.
9. Khong co cam bien nhin vach den; MAP_MARGIN_STOP dua tren odometry.
10. Hit moi duoc luu cho den MAPRESET; xe khong tu lui khi bi chan.
Map/pose are ESTIMATES; no tape sensor, no guaranteed boundary detection.
Boot PAUSED. UART PA9/PA10 115200, newline commands:
MAPRESET (physically at x325 y125 mm heading up), AUTO, STOP, MAP.
Do not MAPRESET elsewhere. MPU Z must face up; keep still for boot bias.
Default counts/mm=10 NOT calibrated; calibrate before floor operation.
Box centres and sensor origins inferred from diagram, must measure.
Planner checks an oriented rectangular footprint, including the full swept
footprint during turns. This is NOT a verified race-ready firmware.
No blind reverse. No path / stale sensor / uncertainty => STOP, never force through.
Motor/SWEEP diagnostic commands retained for wheels lifted only.
No physical mode-button assigned because pin is unknown.
*/




  /*
  * Blue Pill STM32F103C8T6 / Pololu VL53L0X 1.3.1 / TB6612FNG.
  * Xe bam tay bon huong: <100 mm LUI | 100..110 mm GIU | >110..200 mm TIEN | >200 mm DUNG.
  * J9/J10 giu PID khoang cach. J8/J11 va J9/J10 cung chi huong xoay.
  * Khi CHI co mat ben nhin thay tay: chi XOAY tai cho, khong tien/lui.
  * Co hoc: J8 lech trai 35.43 deg, J11 lech phai 35.43 deg, lui 15 mm so voi mep truoc,
  * khoang cach tam hai banh 120 mm, co banh tu lua.
  * Tai cac mat do khoang cach, khong nhan dang duoc ban tay vs vat can khac.
  *
  * UART1 115200, ket thuc lenh bang Newline:
  *   STOP | RUN | SENSOR | MOTOR L 80 | MOTOR R -80 | MOTOR B 80 1500 | SWEEP F | SWEEP R
  *   CAL J9 100 | SAVE | SHOW | DEFAULT | LOG 0 | LOG 1 | HELP
  *
  * SUA LOI GOC (ban truoc xe khong chay): byte cuoi cua lan doc I2C nhieu byte bi lap lai
  * byte dung truoc no, nen khoang cach luon co dang 0x0000/0x0101/0x0202/0x1F1F va moi vat
  * trong 0..255 mm doc ra 0. Ban nay doc 2 byte khoang cach bang cac lan doc 1 byte.
  * Lenh SENSOR in song song 3 cach doc (BYTES / BLOCK / LIB16) de kiem chung tren xe.
  */
  // V2.1: controlled upgrade of the supplied V2; all new features default OFF.
  // =================== A/B FEATURE SWITCHES ============================
  // 0 = original behavior; 1 = enable exactly that experiment.
  // Do not flip several unverified switches at once on the real robot.
  #ifndef ENABLE_A1_FAST_SENSOR_BUS
  #define ENABLE_A1_FAST_SENSOR_BUS 1
  #endif
  #ifndef ENABLE_A2_BLOCK13
  #define ENABLE_A2_BLOCK13 0
  #endif
  #ifndef ENABLE_A3_ALPHA_BETA
  #define ENABLE_A3_ALPHA_BETA 1
  #endif
  #ifndef ENABLE_B_CONTINUOUS_DISTANCE
  #define ENABLE_B_CONTINUOUS_DISTANCE 0
  #endif
  #ifndef ENABLE_B3_SPEED_SLEW
  #define ENABLE_B3_SPEED_SLEW 1
  #endif
  #ifndef ENABLE_B4_FAST_ZONE_CONFIRM
  #define ENABLE_B4_FAST_ZONE_CONFIRM 1
  #endif
  #ifndef ENABLE_C1_PWM_TABLE
  #define ENABLE_C1_PWM_TABLE 0
  #endif
  #ifndef ENABLE_C2_LOW_SPEED_COMP
  #define ENABLE_C2_LOW_SPEED_COMP 0
  #endif
  #ifndef ENABLE_C3_WHEEL_WINDOW
  #define ENABLE_C3_WHEEL_WINDOW 0
  #endif
  #ifndef ENABLE_C4_PWM_20KHZ
  #define ENABLE_C4_PWM_20KHZ 0
  #endif
  #ifndef ENABLE_C5_COAST_OVERSPEED
  #define ENABLE_C5_COAST_OVERSPEED 0
  #endif
  #ifndef ENABLE_D1_STEER_DYNAMIC
  #define ENABLE_D1_STEER_DYNAMIC 0
  #endif
  #ifndef ENABLE_D2_DIRECT_STEER_MIX
  #define ENABLE_D2_DIRECT_STEER_MIX 0
  #endif
  #ifndef ENABLE_D4_GYRO_YAW
  #define ENABLE_D4_GYRO_YAW 0
  #endif
  #ifndef ENABLE_E1_RECOVERABLE_FAULTS
  #define ENABLE_E1_RECOVERABLE_FAULTS 0
  #endif
  #ifndef ENABLE_E2_EXTENDED_LOG
  #define ENABLE_E2_EXTENDED_LOG 1
  #endif
  // =====================================================================

  // V2.1 -- Optional improvements (change one flag at a time, then rebuild).
  #ifndef ENABLE_SYNC_REQUEST_DIFF
  #define ENABLE_SYNC_REQUEST_DIFF 0
  #endif
  #ifndef ENABLE_FAST_STEER_FILTER
  #define ENABLE_FAST_STEER_FILTER 0
  #endif
  #ifndef ENABLE_JUMP_PENDING
  #define ENABLE_JUMP_PENDING 0
  #endif
  #ifndef ENABLE_FRONT_HANDOFF
  #define ENABLE_FRONT_HANDOFF 0
  #endif

  #ifndef FOLLOW_I2C_HZ
  #define FOLLOW_I2C_HZ (ENABLE_A1_FAST_SENSOR_BUS ? 100000U : 50000U)
  #endif
  #ifndef FOLLOW_BUDGET_US
  #define FOLLOW_BUDGET_US (ENABLE_A1_FAST_SENSOR_BUS ? 20000U : 33000U)
  #endif
  #include <stdint.h>
  #include <math.h>
  #include <string.h>
  #include <stdio.h>
  #include <stdlib.h>

  namespace Follow {
  float frontPairBiasMm[2] = {0.0f, 0.0f};
  uint8_t frontBalMask = 0;
  constexpr uint8_t COUNT = 4;
  constexpr uint8_t FRONT_A = 1, FRONT_B = 2;
  constexpr float LOW_MM = 100.0f, HIGH_MM = 110.0f, MAX_MM = 200.0f;
  constexpr float MID_MM = (LOW_MM + HIGH_MM) * 0.5f;
  constexpr uint32_t BUDGET_US = FOLLOW_BUDGET_US; // us; 20ms trial, 33ms baseline
  constexpr uint32_t I2C_HZ = FOLLOW_I2C_HZ; // set 400000 only after checking bus
  // Bounds deliberately kept conservative pending AGE / I2C_ERROR measurements on real HW.
  // Measurements may take budget + polling + bus latency; never assume age=CONTROL_MS.
  constexpr uint32_t CONTROL_MS = 10, FRESH_MS = 100; // ms: retain 100 until real age measurements
  constexpr uint32_t WATCHDOG_MS = 100, GUARD_SAMPLE_MS = 125;
  constexpr uint32_t MAX_POLL_GAP_MS = 70;  // 4 sensors on 50 kHz bus, validate DT_MAX + age in logs
  constexpr uint32_t CONFIRM_WINDOW_MS = 120;   // 2 mau hop le cung vung trong cua so nay = xac nhan
  constexpr uint32_t PAIR_WINDOW_MS = 24, REVERSE_PAUSE_MS = 25;
  constexpr uint32_t STALL_MS = 850;
  constexpr float PAIR_TOL_MM = 18.0f;
  constexpr float DEFAULT_OFFSET_MM[COUNT] = {0.0f, -15.0f, -15.0f, 0.0f};
  constexpr bool LEFT_REVERSE = true;
  constexpr bool RIGHT_REVERSE = false;

  // ---------------------------------------------------------------------------
  // THONG SO CAN DO TREN XE (xem lenh MOTOR / SWEEP). Don vi toc do: xung encoder x4 / giay.
  // cpsAt160 lay tu phep do truoc cua ban (PWM 160: trai 5666, phai 5563 xung/s).
  // startFwd/startRev/runPwm van la uoc luong -> do bang SWEEP roi sua lai.
  // ---------------------------------------------------------------------------
  struct WheelTune {
    float startFwd, startRev;  // PWM nho nhat lam banh BAT DAU quay tu dung yen
    float runPwm;              // PWM nho nhat banh VAN TIEP TUC quay deu
    float cpsAt160;            // xung/s on dinh tai PWM 160
    int encoderForwardSign;    // 0 = chua do; +1/-1 = dau encoder khi banh quay TIEN
  };
  constexpr WheelTune LEFT_TUNE  = {78, 78, 42, 5666, -1};
  constexpr WheelTune RIGHT_TUNE = {78, 78, 42, 5563, +1};
  constexpr float RIGHT_COUNTS_PER_LEFT_COUNT = 1.0f;  // xung phai / xung trai tren cung quang duong thang

  // Vong khoang cach (PD) -> toc do tham chieu (xung/s).
  constexpr float DIST_KP = 22.0f;            // xung/s cho moi mm sai lech
  constexpr float DIST_KD = 4.0f;             // xung/s cho moi mm/s
  constexpr float MIN_CPS = 300.0f, MAX_CPS = 1600.0f;
  // B: setpoint generation and jerk-limited tuning (encoder counts/s and counts/s^2).
  constexpr float DIST_TARGET_MM = 105.0f, DIST_SOFT_ZONE_MM = 1.5f;
  constexpr float DIST_FEEDFORWARD_RATE = 0.18f; // relative-gap velocity != true hand velocity
  constexpr float DIST_DECEL_CPS2 = 3500.0f;    // counts/s^2, approximate wheel decel
  constexpr float BODY_ACCEL_CPS2 = 9000.0f, BODY_DECEL_CPS2 = 24000.0f;
  // A3 alpha-beta estimator; position mm, velocity mm/s.
  constexpr float AB_ALPHA = 0.70f, AB_BETA = 0.13f;
  constexpr float AB_V_MAX_MM_S = 400.0f, AB_RESET_JUMP_MM = 25.0f;
  // C: low-speed sensor/encoder/power tuning.
  constexpr float WHEEL_OBS_TAU_S = ENABLE_C3_WHEEL_WINDOW ? 0.038f : 0.060f;
  constexpr float BREAKAWAY_DELAY_MS = ENABLE_C2_LOW_SPEED_COMP ? 20.0f : 45.0f;
  constexpr float BREAKAWAY_RAMP_MS = ENABLE_C2_LOW_SPEED_COMP ? 60.0f : 110.0f;  // restore V3 tracking cap; PWM max unchanged
  constexpr float LOOKAHEAD_S = 0.08f;               // do tre do + phanh (giay)
  constexpr float EDGE_CPS = 250.0f, EDGE_CPS_PER_MM = 25.0f;  // giam toc som truoc vung 100..110
  // Vong toc do tung banh (feedforward + PI) -> PWM.
  constexpr float SPEED_KP = 0.010f, SPEED_KI = 0.080f, I_LIMIT_PWM = 25.0f;
  constexpr float MAX_PWM = 125.0f, ACCEL_PWM_S = 850.0f, DECEL_PWM_S = 2400.0f;
  constexpr float SYNC_KP = 0.18f, SYNC_MAX_CPS = 200.0f;
  constexpr int32_t WRONG_DIR_COUNTS = 40;

  // Hinh hoc va bo re: tinh theo do chenh toc do hai banh.
  // COUNTS_PER_MM_EST CHUA DUOC DO => he so goc chi la xap xi de thu.
  constexpr float WHEEL_TRACK_MM = 120.0f;
  constexpr float SIDE_LEFT_DEG = 35.43f, SIDE_RIGHT_DEG = 35.43f; // same nominal footprint angle, per user correction
  constexpr float SIDE_SETBACK_MM = 15.0f;
  constexpr float COUNTS_PER_MM_EST = 10.0f;
  constexpr float TURN_RAD_S_MAX = 1.25f; // raises side-only wheel reference to the proven V3 ceiling
  constexpr float TURN_CPS_MIN = 300.0f, TURN_CPS_MAX = 600.0f; // restore V3 maximum yaw wheel speed
  constexpr float TURN_DEADBAND = 0.12f;  // restore earlier steering responsiveness
  constexpr float FRONT_DIFF_DEADBAND_MM = 18.0f;
  constexpr float FRONT_DIFF_NORMALIZE_FACTOR = 0.90f;
  // BAL 100 and BAL 150 are deliberately optional: without both, keep stable front logic.
  extern float frontPairBiasMm[2];
  extern uint8_t frontBalMask;
  constexpr float STEER_TAU_BASE_S = 0.060f;
  constexpr float STEER_TAU_FAST_S = 0.040f;
  #if ENABLE_FAST_STEER_FILTER
  constexpr float STEER_TAU_S = STEER_TAU_FAST_S;
  #else
  constexpr float STEER_TAU_S = STEER_TAU_BASE_S;
  #endif
  #if ENABLE_JUMP_PENDING
  constexpr float JUMP_TRIGGER_MM = 35.0f, JUMP_MATCH_MM = 18.0f;
  #endif
  #if ENABLE_FRONT_HANDOFF
  constexpr uint32_t HANDOFF_BLEND_MS = 80;
  constexpr float HANDOFF_MAX_OFFSET_MM = 25.0f;
  #endif
  #if ENABLE_SYNC_REQUEST_DIFF
  // Diagnostic only; zeroed every control cycle even when motion is inhibited.
  float syncErrorCps = 0.0f;
  bool syncRequestActive = false;
  #endif
  constexpr float STEER_ON_DEG = 7.0f, STEER_OFF_DEG = 3.0f;
  constexpr float STEER_SLEW_DEG_S = 420.0f, STEER_LOST_GRACE_MS = 100.0f;
  // E1: limit automatic recovery attempts; permanent latch after exhausting this cap.
  constexpr uint32_t RECOVER_STABLE_MS = 300, MAX_AUTO_RECOVERIES = 2;
  constexpr float SIDE_ASSOC_TOL_MM = 85.0f;  // chi la cong dieu huong, khong phai nhan dang tay
  // 1.5s hard limit for a side-only rotation episode; rearm only after confirmed target is lost.
  constexpr uint32_t SIDE_TURN_MAX_MS = 1500;
  constexpr uint32_t SIDE_REARM_ABSENT_MS = 200;
  constexpr uint8_t SIDE_L = 0, SIDE_R = 3;


  static_assert(RIGHT_COUNTS_PER_LEFT_COUNT > 0 && LEFT_TUNE.cpsAt160 > 0 && RIGHT_TUNE.cpsAt160 > 0,
                "Wheel calibration scales must be positive");
  static_assert(LEFT_TUNE.startFwd <= MAX_PWM && LEFT_TUNE.startRev <= MAX_PWM &&
                RIGHT_TUNE.startFwd <= MAX_PWM && RIGHT_TUNE.startRev <= MAX_PWM,
                "Breakaway PWM exceeds the output limit");

  float clip(float x, float lo, float hi) { return fmaxf(lo, fminf(hi, x)); }
  int sign(float x) { return (x > 0) - (x < 0); }
  float towards(float x, float goal, float step) { return x + clip(goal - x, -step, step); }

  float calibratedFrontSteer(float j9mm, float j10mm, float distanceMm) {
    if (frontBalMask != 3U) return 0.0f;
    const float t = clip((distanceMm - 100.0f) / 50.0f, 0.0f, 1.0f);
    const float bias = frontPairBiasMm[0] + t * (frontPairBiasMm[1] - frontPairBiasMm[0]);
    const float diff = (j9mm - j10mm) - bias;
    if (fabsf(diff) <= FRONT_DIFF_DEADBAND_MM) return 0.0f;
    const float effective = diff > 0 ? diff - FRONT_DIFF_DEADBAND_MM : diff + FRONT_DIFF_DEADBAND_MM;
    // A fixed /65 factor causes false turns: scale with current target range.
    const float denom = fmaxf(85.0f, distanceMm * FRONT_DIFF_NORMALIZE_FACTOR);
    return clip(effective / denom, -0.42f, 0.42f);
  }


  // Filter measurement updates only on NEW accepted frames; predict from measurement timestamp.
  struct AlphaBeta {
    bool ready = false;
    float x = 0.0f, v = 0.0f;
    uint32_t at = 0, sequence = 0;
    void reset() { ready = false; x = v = 0; at = sequence = 0; }
    void accept(float measured, uint32_t received, uint32_t key) {
      if (ready && key == sequence) return;
      const float dt = ready ? (float)((uint32_t)(received - at)) * 0.001f : 0.0f;
      if (!ready || dt <= 0.0f || dt > 0.25f) {
        x = measured; v = 0; ready = true;
      } else {
        const float prediction = x + v * dt;
        const float residual = measured - prediction;
        if (fabsf(residual) >= AB_RESET_JUMP_MM) { x = measured; v = 0; }
        else {
          x = prediction + AB_ALPHA * residual;
          v = clip(v + AB_BETA * residual / dt, -AB_V_MAX_MM_S, AB_V_MAX_MM_S);
        }
      }
      at = received; sequence = key;
    }
    float predict(uint32_t now, uint32_t conservativeSampleAt) const {
      if (!ready) return 0;
      // Bounded sample age, conservative bound from original sensor parser.
      const float age = clip(float((uint32_t)(now - conservativeSampleAt)) * 0.001f, 0, 0.12f);
      return x + v * age;
    }
  };
  // Slew a signed setpoint; deceleration includes approaching zero and reverse crossing.
  struct SpeedSlew {
    float value = 0;
    void reset() { value = 0; }
    float update(float goal, float dt) {
      const float rate = (fabsf(goal) > fabsf(value) &&
          (sign(goal) == sign(value) || sign(value) == 0)) ?
                        BODY_ACCEL_CPS2 : BODY_DECEL_CPS2;
      value = towards(value, goal, rate * dt);
      return value;
    }
  };
  float continuousBodySpeed(float predictedGap, float relativeGapRate,
                            float countsPerMm, float kp, float maxCps) {
    const float e = predictedGap - DIST_TARGET_MM;
    if (fabsf(e) <= DIST_SOFT_ZONE_MM && fabsf(relativeGapRate) < 12.0f) return 0.0f;
    const float effective = (fabsf(e) > DIST_SOFT_ZONE_MM) ?
        sign(e) * (fabsf(e) - DIST_SOFT_ZONE_MM) : 0.0f;
    const float desired = kp * effective + DIST_FEEDFORWARD_RATE * relativeGapRate * countsPerMm;
    // Counts/s = sqrt(2 * (counts/s^2) * |error_mm| * (counts/mm)).
    // Also allow movement when hand is moving while currently within 100..110mm.
    const float movingError = fabsf(e + relativeGapRate * 0.040f);
    const float maxStop = sqrtf(2.0f * DIST_DECEL_CPS2 *
                              fmaxf(fabsf(e),movingError) * countsPerMm);
    return clip(desired, -fminf(maxCps, maxStop), fminf(maxCps, maxStop));
  }

  AlphaBeta alphaBetaDistance;
  uint32_t alphaBetaNow = 0;
  enum class Zone { Near, Band, Track, Far };
  struct ZoneConfirmMemory {
    bool confirmed = false;
    Zone zone = Zone::Far;
    uint32_t at = 0;
    void reset() { confirmed = false; at = 0; zone = Zone::Far; }
  };
  ZoneConfirmMemory zoneConfirm;
  Zone zoneOf(float mm) {
    if (mm < LOW_MM) return Zone::Near;
    if (mm <= HIGH_MM) return Zone::Band;
    if (mm <= MAX_MM) return Zone::Track;
    return Zone::Far;
  }

  // Mot cam bien. Ba loai su kien duoc tach rieng:
  //   accept()     - phep do MOI hop le
  //   miss()       - phep do moi nhung khong dang tin (khong co muc tieu, tin hieu yeu...)
  //   invalidate() - loi I2C hoac reset: bo ngay moi du lieu
  struct Reading {
    bool valid = false, seenFrame = false;
    uint8_t streak = 0, misses = 0, deviceStatus = 255;
    uint16_t raw = 0, rawBytes = 0, rawBlock = 0, rawBlock13 = 0, rawLib = 0, signalQ7 = 0;
    float mm = 0, filtered = 0, rate = 0;
    uint32_t receivedAt = 0, sampleAt = 0, frameAt = 0;

  #if ENABLE_JUMP_PENDING
    bool jumpPending = false;
    uint16_t jumpRaw = 0;
    float jumpMm = 0, jumpOriginMm = 0;
    uint32_t jumpReceivedAt = 0, jumpSampleAt = 0;
  #endif
    void invalidate() {
      valid = false; streak = 0; misses = 0; rate = 0;
  #if ENABLE_JUMP_PENDING
      jumpPending = false;
  #endif
    }
    // Mot khung xau don le khong xoa xac nhan; mau cu van tu het han theo FRESH_MS.
    void miss() {
  #if ENABLE_JUMP_PENDING
      jumpPending = false; // a bad frame cannot corroborate a pending jump
  #endif
      if (valid && ++misses >= 2) invalidate();
    }
    bool fresh(uint32_t now) const { return valid && now - sampleAt < FRESH_MS; }

    // Chi duoc goi khi pollSensor() vua lay mot khung MOI, nen moi lan tang streak
    // va moi lan tinh dao ham deu ung voi mot phep do moi.
    void accept(uint16_t rawMm, float offset, uint32_t received, uint32_t oldestStart) {
      const float next = fmaxf(0.0f, float(rawMm) + offset);
      const uint32_t elapsed = received - receivedAt;
      const bool adjacent = valid && elapsed > 0 && elapsed <= CONFIRM_WINDOW_MS;
      const bool sameZone = adjacent && zoneOf(next) == zoneOf(mm);
      streak = sameZone ? (streak < 3 ? streak + 1 : 3) : 1;
      if (!adjacent || fabsf(next - mm) >= 25.0f) {
        filtered = next;
        rate = 0;  // bat lai muc tieu / nhay bac khong phai la phep do van toc
      } else {
        const float dt = elapsed * 0.001f;
        const float tau = fabsf(next - filtered) > 6.0f ? 0.012f : 0.032f;
        const float previous = filtered;
        filtered += dt / (tau + dt) * (next - filtered);
        const float slope = clip((filtered - previous) / dt, -400, 400);
        rate += dt / (0.060f + dt) * (slope - rate);
      }
      raw = rawMm; mm = next; receivedAt = received; sampleAt = oldestStart;
      valid = true; misses = 0;
    }
  };

  #if ENABLE_JUMP_PENDING
  // Retain the last ACCEPTED frame (and its original age) while one outlier waits.
  // raw/frameAt still show the latest hardware return for SENSOR diagnostics.
  void commitJump(Reading &s) {
    const uint16_t raw = s.jumpRaw;
    const float offset = s.jumpMm - float(raw);
    const uint32_t received = s.jumpReceivedAt, sample = s.jumpSampleAt;
    s.jumpPending = false;
    s.accept(raw, offset, received, sample);
  }
  bool pendingFresh(const Reading &s, uint32_t now) {
    return s.jumpPending && (uint32_t)(now - s.jumpSampleAt) < FRESH_MS &&
           (uint32_t)(now - s.jumpReceivedAt) < CONFIRM_WINDOW_MS;
  }
  void acceptWithJumpGate(Reading (&r)[COUNT], uint8_t i, uint16_t raw,
                          float offset, uint32_t received, uint32_t oldestStart) {
    Reading &s = r[i];
    if ((uint32_t)(received - oldestStart) >= FRESH_MS) {
      s.jumpPending = false; return; // never renew age with a stale candidate
    }
    const float next = fmaxf(0.0f, float(raw) + offset);
    if (!s.fresh(received) || fabsf(next - s.mm) < JUMP_TRIGGER_MM) {
      s.jumpPending = false;
      s.accept(raw, offset, received, oldestStart);
      return; // normal motion uses the unmodified V2 path immediately
    }
    if (pendingFresh(s, received) && received != s.jumpReceivedAt &&
        zoneOf(next) == zoneOf(s.jumpMm) && fabsf(next - s.jumpMm) <= JUMP_MATCH_MM &&
        sign(next - s.mm) == sign(s.jumpMm - s.mm)) {
      // Two real samples: replay the withheld one before the current one so the
      // original streak/zone logic confirms them, without inventing a measurement.
      commitJump(s);
      s.accept(raw, offset, received, oldestStart);
      return;
    }
    s.jumpPending = true; s.jumpRaw = raw; s.jumpMm = next; s.jumpOriginMm = s.mm;
    s.jumpReceivedAt = received; s.jumpSampleAt = oldestStart;
    if (i != FRONT_A && i != FRONT_B) return;
    Reading &peer = r[i == FRONT_A ? FRONT_B : FRONT_A];
    // Early acceptance needs TWO NEW, independent front candidates, agreeing in
    // range, zone and direction. An old peer hit cannot validate a new jump.
    if (peer.fresh(received) && pendingFresh(peer, received) &&
        (uint32_t)(received - peer.jumpReceivedAt) <= PAIR_WINDOW_MS &&
        zoneOf(next) == zoneOf(peer.jumpMm) && fabsf(next - peer.jumpMm) <= PAIR_TOL_MM &&
        sign(next - s.jumpOriginMm) == sign(peer.jumpMm - peer.jumpOriginMm)) {
      commitJump(peer);
      commitJump(s);
    }
  }
  uint8_t jumpPendingMask(const Reading (&r)[COUNT], uint32_t now) {
    uint8_t mask = 0;
    for (uint8_t i = 0; i < COUNT; ++i)
      if (pendingFresh(r[i], now)) mask |= uint8_t(1U << i);
    return mask;
  }
  #endif

  enum class State { Lost, Far, Confirm, Hold, Forward, Reverse, TurnLeft, TurnRight, SideTimeout };
  const char *stateName(State state) {
    switch (state) {
      case State::Far: return "TREN_20CM";
      case State::Confirm: return "CHO_XAC_NHAN";
      case State::Hold: return "GIU_10_11";
      case State::Forward: return "TIEN";
      case State::Reverse: return "LUI";
      case State::TurnLeft: return "RE_TRAI";
      case State::TurnRight: return "RE_PHAI";
      case State::SideTimeout: return "SIDE_TURN_LIMIT";
      default: return "MAT_DU_LIEU";
    }
  }
  struct Target {
    int index = -1;
    float gap = 0, filtered = 0, rate = 0;
    float steer = 0; // am = trai, duong = phai; tinh theo mat dang thay tay
    uint8_t mask = 0;
    bool hasFront = false;
    uint32_t sampleAt = 0;
    State state = State::Lost;
  };
  Target locateFront(const Reading (&r)[COUNT], uint32_t now) {
    Target t;
    const bool a = r[FRONT_A].fresh(now), b = r[FRONT_B].fresh(now);
    if (!a && !b) return t;
    int i = (!a || (b && r[FRONT_B].mm < r[FRONT_A].mm)) ? FRONT_B : FRONT_A;  // mat gan hon
    int j = i == FRONT_A ? FRONT_B : FRONT_A;
    const bool both = a && b;
    const uint32_t ageI = now - r[i].receivedAt, ageJ = now - r[j].receivedAt;
    const uint32_t skew = ageI >= ageJ ? ageI - ageJ : ageJ - ageI;
    const bool sameZone = both && zoneOf(r[j].mm) == zoneOf(r[i].mm);
    const bool pair = sameZone && skew <= PAIR_WINDOW_MS && fabsf(r[i].mm - r[j].mm) <= PAIR_TOL_MM;
    bool pairOnly = false;
    if (r[i].streak < 2) {
      if (pair) pairOnly = true;
      else if (sameZone && r[j].streak >= 2) { const int k = i; i = j; j = k; }  // dung mat da tu xac nhan
    }
    const Zone z = zoneOf(r[i].mm);  // luon tinh tu dung cam bien cung cap t.gap
    bool confirmed = r[i].streak >= 2 || pairOnly;
  #if ENABLE_B4_FAST_ZONE_CONFIRM
    // Two independent current sensor returns in same zone are enough even when
    // older startup streaks have just reset. Never shortcut re-acquisition from Lost.
    // A single adjacent-zone transition is allowed only from previously confirmed
    // front data whose age is within the 120 ms window, moving in the same direction.
    const Zone zNow = zoneOf(r[i].mm);
    if (!confirmed && zoneConfirm.confirmed && (uint32_t)(now - zoneConfirm.at) < CONFIRM_WINDOW_MS) {
      const bool adjacent = (zoneConfirm.zone == Zone::Near && zNow == Zone::Band) ||
        (zoneConfirm.zone == Zone::Band && (zNow == Zone::Near || zNow == Zone::Track)) ||
        (zoneConfirm.zone == Zone::Track && zNow == Zone::Band);
      const float change = r[i].mm - DIST_TARGET_MM;
      const bool movingConsistent = (change > 0 && r[i].rate > 20.0f) ||
                                    (change < 0 && r[i].rate < -20.0f);
      if (adjacent && movingConsistent) confirmed = true;
    }
    if (confirmed) { zoneConfirm.confirmed = true; zoneConfirm.at = now; zoneConfirm.zone = zNow; }
    else if ((uint32_t)(now - zoneConfirm.at) >= FRESH_MS) zoneConfirm.confirmed = false;
  #endif
    t.index = i; t.gap = r[i].mm; t.filtered = r[i].filtered; t.rate = r[i].rate;
    t.sampleAt = r[i].sampleAt;
    if (z == Zone::Band) t.state = State::Hold;
    else if (z == Zone::Far) t.state = State::Far;
    else if (!confirmed) t.state = State::Confirm;
    else t.state = z == Zone::Near ? State::Reverse : State::Forward;
    // Neu hai khung dau tien la bang chung duy nhat thi het han theo khung cu hon.
    if (pairOnly && now - r[j].sampleAt > now - t.sampleAt) t.sampleAt = r[j].sampleAt;
    return t;
  }

  // Co hai mau hop le trong <=120ms moi su dung mat ben de re.
  // Uu tien mat truoc de giu khoang cach. Mat ben chi cung cap huong xoay.
  // Khong dung hieu J9-J10 de suy goc, vi da co sai lech 20..70mm.
  bool sideConfirmed(const Reading &r, uint32_t now) {
    return r.fresh(now) && r.streak >= 2 && r.mm >= 40.0f &&
          r.mm <= MAX_MM;  // strict side detection radius 20cm
  }

  void mergeSampleAge(Target &t, const Reading &r, uint32_t now) {
    if ((uint32_t)(now - r.sampleAt) > (uint32_t)(now - t.sampleAt)) t.sampleAt = r.sampleAt;
  }

  Target locate(const Reading (&r)[COUNT], uint32_t now) {
    Target t = locateFront(r, now);
    // Chi giu khoang cach khi mat truoc da xac nhan nhin thay muc tieu trong 20cm.
    t.hasFront = t.index >= 0 && (t.state == State::Forward ||
        t.state == State::Reverse || t.state == State::Hold);
    const bool fl = r[FRONT_A].fresh(now) && r[FRONT_A].streak >= 2 && r[FRONT_A].mm <= MAX_MM;
    const bool fr = r[FRONT_B].fresh(now) && r[FRONT_B].streak >= 2 && r[FRONT_B].mm <= MAX_MM;
    const bool sl0 = sideConfirmed(r[SIDE_L], now);
    const bool sr0 = sideConfirmed(r[SIDE_R], now);
    bool sl = sl0, sr = sr0;

    if (t.hasFront) {
      // Neu co vat the o ben nhung xa qua mat truoc, khong dung no de re.
      // Project along optical axis and account for sensor's 15mm setback.
      if (sl) {
        const float x = r[SIDE_L].mm * cosf(SIDE_LEFT_DEG * 0.01745329252f) - SIDE_SETBACK_MM;
        if (fabsf(x - t.gap) > SIDE_ASSOC_TOL_MM) sl = false;
      }
      if (sr) {
        const float x = r[SIDE_R].mm * cosf(SIDE_RIGHT_DEG * 0.01745329252f) - SIDE_SETBACK_MM;
        if (fabsf(x - t.gap) > SIDE_ASSOC_TOL_MM) sr = false;
      }
    }
    // Neu ca hai mat ben nhin thay vat the, tranh tu xoay khi huong mo ho.
    if (sl && sr) {
      if (r[SIDE_L].mm + 40.0f < r[SIDE_R].mm) sr = false;
      else if (r[SIDE_R].mm + 40.0f < r[SIDE_L].mm) sl = false;
      else { sl = false; sr = false; }
    }

    if (t.hasFront) {
      t.mask |= (uint8_t)(1U << t.index);
      if (fl) { t.mask |= (uint8_t)(1U << FRONT_A); mergeSampleAge(t, r[FRONT_A], now); }
      if (fr) { t.mask |= (uint8_t)(1U << FRONT_B); mergeSampleAge(t, r[FRONT_B], now); }
      // J9 left only = small left correction, J10 right only = small right correction.
      // Only when BAL 100+150 have been calibrated, allow a small front-pair correction.
      // Otherwise retain the previously working steering decision (zero for two front hits).
      if (fl && fr) t.steer = calibratedFrontSteer(r[FRONT_A].mm, r[FRONT_B].mm, t.gap);
      else if (fl && !fr) t.steer = -0.38f;
      else if (fr && !fl) t.steer = +0.38f;
      if (sl && !sr) {
        t.steer = -SIDE_LEFT_DEG / SIDE_RIGHT_DEG;
        t.mask |= (uint8_t)(1U << SIDE_L); mergeSampleAge(t, r[SIDE_L], now);
      } else if (sr && !sl) {
        t.steer = +1.0f;
        t.mask |= (uint8_t)(1U << SIDE_R); mergeSampleAge(t, r[SIDE_R], now);
      }
      return t;
    }

    // Front not confirmed/too far: if a side sensor sees a CONFIRMED nearby target,
    // rotate in place only. No forward/reverse based on side ray distance.
    if (sl || sr) {
      const bool left = sl && !sr;
      t.index = left ? SIDE_L : SIDE_R;
      t.gap = r[t.index].mm; t.filtered = r[t.index].filtered;
      t.rate = r[t.index].rate; t.sampleAt = r[t.index].sampleAt;
      t.steer = left ? -SIDE_LEFT_DEG / SIDE_RIGHT_DEG : +1.0f;
      t.mask = (uint8_t)(1U << t.index);
      t.state = left ? State::TurnLeft : State::TurnRight;
    } else if (t.index >= 0 && t.state == State::Confirm) {
      // Do not drive a target lacking confirmation.
      t.steer = 0;
    }
    return t;
  }

  #if ENABLE_FAST_STEER_FILTER
  // Same one-pole equation/deadband as V2; only the time constant changes.
  float fastSteerStep(float previous, float desired, float dt) {
    return previous + dt / (STEER_TAU_S + dt) * (desired - previous);
  }
  #endif

  #if ENABLE_FRONT_HANDOFF
  struct FrontHandoff {
    int source = -1;
    bool active = false, starting = false;
    float lastOutput = 0, offset = 0;
    Zone lastZone = Zone::Far;
    uint32_t anchorSampleAt = 0, lastSampleAt = 0, beganAt = 0;
    void reset() { *this = FrontHandoff(); }
    bool prepare(const Target &t, const Reading (&r)[COUNT], uint32_t now) {
      starting = false;
      const bool usable = t.hasFront && (t.index == FRONT_A || t.index == FRONT_B) &&
          (t.state == State::Hold || t.state == State::Forward || t.state == State::Reverse) &&
          (uint32_t)(now - t.sampleAt) < FRESH_MS;
      if (!usable) { reset(); return false; }
      const Reading &a = r[FRONT_A], &b = r[FRONT_B];
      const uint32_t ageA = now - a.receivedAt, ageB = now - b.receivedAt;
      const uint32_t skew = ageA > ageB ? ageA - ageB : ageB - ageA;
      const bool agree = a.fresh(now) && b.fresh(now) && skew <= PAIR_WINDOW_MS &&
          zoneOf(a.mm) == zoneOf(b.mm) && fabsf(a.mm - b.mm) <= PAIR_TOL_MM;
      const bool changed = source >= 0 && source != t.index;
      const bool sameZone = lastZone == zoneOf(t.gap);
      if (agree || !sameZone || (active &&
          ((uint32_t)(now - anchorSampleAt) >= FRESH_MS ||
           (uint32_t)(now - beganAt) >= HANDOFF_BLEND_MS))) active = false;
      if (changed && !agree && sameZone && r[t.index].streak >= 2 &&
          (uint32_t)(now - lastSampleAt) < FRESH_MS) {
        if (!active) { beganAt = now; anchorSampleAt = lastSampleAt; }
        // Repeated switches do not extend the original 80ms/age deadline.
        active = true; starting = true;
      }
      source = t.index;
      // A source discontinuity is not hand velocity. With agreeing fronts the
      // estimator remains untouched, so normal updates gain no extra delay.
      return changed && !agree;
    }
    void apply(Target &t, uint32_t now) {
      if (source < 0) return;
      if (active && ((uint32_t)(now - anchorSampleAt) >= FRESH_MS ||
          (uint32_t)(now - beganAt) >= HANDOFF_BLEND_MS)) active = false;
      if (active) {
        const float remaining = 1.0f - float((uint32_t)(now - beganAt)) / HANDOFF_BLEND_MS;
        if (starting) offset = clip((lastOutput - t.filtered) / fmaxf(remaining, 0.01f),
                                   -HANDOFF_MAX_OFFSET_MM, HANDOFF_MAX_OFFSET_MM);
        t.filtered += offset * remaining;
        // Raw zone/gap/state remain authoritative: never invent a reverse or
        // mask a confirmed <100mm / >200mm event with the smoothed distance.
        if (t.state == State::Forward) t.filtered = fmaxf(HIGH_MM + 0.5f, t.filtered);
        if (t.state == State::Reverse) t.filtered = fminf(LOW_MM - 0.5f, t.filtered);
        if ((uint32_t)(now - anchorSampleAt) > (uint32_t)(now - t.sampleAt))
          t.sampleAt = anchorSampleAt;
      }
      starting = false;
      lastOutput = t.filtered; lastSampleAt = t.sampleAt; lastZone = zoneOf(t.gap);
    }
  } frontHandoff;
  #endif

  struct Command { float cps = 0, edgeMm = 0, turnCps = 0; };
  Command decide(const Target &t) {
    Command c;
    // Steering independent of distance PD. Turn sign: +right, -left.
    if (t.state == State::Hold || t.state == State::Forward ||
        t.state == State::Reverse || t.state == State::TurnLeft ||
        t.state == State::TurnRight) {
      const float absSteer = fabsf(t.steer);
      if (absSteer > TURN_DEADBAND) {
        const float cpsFromGeometry = absSteer * WHEEL_TRACK_MM * 0.5f *
                                    COUNTS_PER_MM_EST * TURN_RAD_S_MAX;
        const float magnitude = clip(cpsFromGeometry, TURN_CPS_MIN, TURN_CPS_MAX);
        c.turnCps = sign(t.steer) * magnitude;
        c.edgeMm = 20.0f; // allow breakaway PWM while turning in place
      }
    }
  #if ENABLE_B_CONTINUOUS_DISTANCE
    // Continuous distance command also applies *inside* HOLD if hand moves.
    if (t.state != State::Forward && t.state != State::Reverse &&
        t.state != State::Hold) return c;
    // Conservative fallback when alpha-beta not ready uses existing filtered value.
    const float predicted = (ENABLE_A3_ALPHA_BETA && alphaBetaDistance.ready) ?
        alphaBetaDistance.predict(alphaBetaNow, t.sampleAt) : t.filtered;
    const float velocity = (ENABLE_A3_ALPHA_BETA && alphaBetaDistance.ready) ?
        alphaBetaDistance.v : t.rate;
    c.edgeMm = fmaxf(c.edgeMm, fabsf(t.gap - DIST_TARGET_MM));
    c.cps = continuousBodySpeed(predicted, velocity, COUNTS_PER_MM_EST, DIST_KP, MAX_CPS);
    // Do not drive toward a target beyond the legal 200mm detection range.
    if ((t.state == State::Reverse && c.cps > 0) ||
        (t.state == State::Forward && c.cps < 0)) c.cps = 0;
    return c;
  #else
    if (t.state != State::Forward && t.state != State::Reverse) return c;
  #endif
    const int d = t.state == State::Forward ? 1 : -1;
    c.edgeMm = fmaxf(c.edgeMm, d > 0 ? t.gap - HIGH_MM : LOW_MM - t.gap);
    // rate > 0 = khoang ho dang mo ra: tang luc khi tien, giam luc khi lui.
    const float e = clip(t.filtered - MID_MM, t.gap - MID_MM - 4.0f, t.gap - MID_MM + 4.0f);
    const float pd = d * (DIST_KP * e + DIST_KD * t.rate);
    // Bu do tre cua cam bien: neu theo da hien tai xe sap vao vung giu thi phanh som.
    const float ahead = t.filtered + t.rate * LOOKAHEAD_S;
    if (d > 0 ? ahead <= HIGH_MM : ahead >= LOW_MM) return c;
    const float cap = fminf(MAX_CPS, EDGE_CPS + EDGE_CPS_PER_MM * c.edgeMm);
    c.cps = d * clip(pd, ENABLE_C2_LOW_SPEED_COMP ? 0.0f : MIN_CPS,
                    fmaxf(MIN_CPS, cap));
    return c;
  }

  // SWEEP measured points; fill cps from real measurements before enabling C1.
  struct PwmCpsPoint { uint16_t pwm; float cps; };
  // Positive values; for reverse optionally use separate tables when calibrated.
  constexpr PwmCpsPoint LEFT_SWEEP[] = {
    {40,0},{50,0},{60,0},{70,0},{80,0},{90,0},{100,0},{110,0},{120,0},{130,0},{160,0}
  };
  constexpr PwmCpsPoint RIGHT_SWEEP[] = {
    {40,0},{50,0},{60,0},{70,0},{80,0},{90,0},{100,0},{110,0},{120,0},{130,0},{160,0}
  };
  float tableFeedforward(float ref, const WheelTune &p) {
    const PwmCpsPoint *table = (&p == &LEFT_TUNE) ? LEFT_SWEEP : RIGHT_SWEEP;
    const unsigned n = sizeof(LEFT_SWEEP) / sizeof(LEFT_SWEEP[0]);
    const float legacy = p.runPwm + (160.0f - p.runPwm) * ref / p.cpsAt160;
    if (!ENABLE_C1_PWM_TABLE || table[n - 1].cps < 100.0f) return legacy;
    // Reject incomplete / unsorted SWEEP tables instead of unsafe interpolation.
    for (unsigned k=1;k<n;++k)
      if (table[k].cps <= table[k-1].cps || table[k].pwm <= table[k-1].pwm) return legacy;
    if (ref <= table[0].cps) return (float)table[0].pwm;
    for (unsigned i = 1; i < n; ++i) {
      if (ref <= table[i].cps) {
        const float span = table[i].cps - table[i-1].cps;
        if (span < 1) return (float)table[i].pwm;
        const float t = clip((ref-table[i-1].cps)/span,0,1);
        return table[i-1].pwm + t * (table[i].pwm-table[i-1].pwm);
      }
    }
    return (float)table[n - 1].pwm;
  }

  struct Wheel {
    float cps = 0, integral = 0, applied = 0;
    int request = 0, lastDrive = 0, watched = 0;
    uint32_t zeroAt = 0, driveSince = 0, lastPulse = 0;
    int32_t wrongCounts = 0;
    int wrongMs = 0;
    bool interlock = false, coastRequested = false;
    int32_t history[4] = {}; float historyDt[4] = {};
    uint8_t historyPtr = 0, historyUsed = 0;

  #if ENABLE_SYNC_REQUEST_DIFF
    bool syncSampleValid = false;
    uint32_t syncSampleAt = 0;
    float syncInstantCps = 0;
    void observeSync(int32_t delta, float dt, uint32_t now) {
      syncSampleAt = now;
      syncSampleValid = isfinite(dt) && dt >= 0.001f &&
          dt < WATCHDOG_MS * 0.001f && delta > -32768 && delta < 32768;
      syncInstantCps = syncSampleValid ? float(delta) / dt : 0;
      syncSampleValid = syncSampleValid && isfinite(cps) && isfinite(syncInstantCps);
    }
  #endif

    void zero(uint32_t now) {
      if (applied != 0) { lastDrive = sign(applied); zeroAt = now; }
      applied = 0; integral = 0; request = 0; watched = 0; interlock = false;
  #if ENABLE_SYNC_REQUEST_DIFF
      syncSampleValid = false;
  #endif
      wrongCounts = 0; wrongMs = 0; coastRequested = false;
      historyPtr = historyUsed = 0;
      for (uint8_t k=0; k<4; ++k) { history[k]=0; historyDt[k]=0; }
    }
    void observe(int32_t delta, float dt) {
      float measured = float(delta)/dt;
  #if ENABLE_C3_WHEEL_WINDOW
      history[historyPtr] = delta; historyDt[historyPtr] = dt;
      historyPtr = (uint8_t)((historyPtr + 1U) % 4U);
      if (historyUsed < 4) ++historyUsed;
      int32_t sum = 0; float seconds = 0;
      for (uint8_t i=0; i<historyUsed; ++i) { sum += history[i]; seconds += historyDt[i]; }
      if (seconds > 0.001f) measured = float(sum)/seconds;
  #endif
      cps += dt / (WHEEL_OBS_TAU_S + dt) * (measured - cps);
    }
    // Chan doan khoang thoi gian vua qua theo chieu PWM thuc da cap.
    const char *motionFault(int32_t delta, uint32_t now, uint32_t dtMs, const WheelTune &p,
                            const char *stallName, const char *wrongName) {
      const int d = sign(applied);
      if (!d) { watched = 0; return nullptr; }
      if (d != watched) {
        watched = d; driveSince = lastPulse = now; wrongCounts = 0; wrongMs = 0;
      }
      if (delta != 0) lastPulse = now;
      if (now - lastPulse >= STALL_MS) return stallName;
      if (p.encoderForwardSign && now - driveSince > 120) {
        const int32_t directed = delta * p.encoderForwardSign * d;
        if (directed < 0) wrongCounts += -directed;
        if (directed > 0) { wrongCounts = 0; wrongMs = 0; }
        else if (wrongCounts) wrongMs += int(dtMs);
        if (wrongCounts >= WRONG_DIR_COUNTS && wrongMs >= 150) return wrongName;
      }
      return nullptr;
    }
    int update(const Command &c, const WheelTune &p, uint32_t now, float dt) {
      const int d = sign(c.cps);
      if (!d) { zero(now); return 0; }
      coastRequested = false;
      if (sign(applied) && sign(applied) != d) zero(now);  // bo mo-men cu ngay trong tick nay
      const bool waiting = lastDrive && d != lastDrive && now - zeroAt < REVERSE_PAUSE_MS;
      interlock = waiting;
      if (waiting) { request = 0; return 0; }  // PWM = 0 va trang thai noi bo cung = 0
      const float speed = p.encoderForwardSign ? d * p.encoderForwardSign * cps : fabsf(cps);
      const float ref = fabsf(c.cps), error = ref - speed;
      // Banh dang nhanh hon han toc do yeu cau: phanh ngan mach thay vi giu PWM san.
  #if ENABLE_C5_COAST_OVERSPEED
      // Coasting during modest overspeed replaces the old on/off short-brake chatter.
      if (speed > ref + 120.0f) {
        coastRequested = true;  // zero command but IN1=IN2=LOW -> coast
        applied = 0; request = 0; return 0;
      }
  #else
      if (speed > 1.4f * ref + 150.0f) {
        applied = 0; request = 0; return 0;
      }
  #endif
      const float start = d > 0 ? p.startFwd : p.startRev;
      const float limit = fminf(MAX_PWM, start + fminf(47.0f, 1.15f * c.edgeMm));
      // Table feedforward (when SWEEP-calibrated) or legacy linear approximation.
      // C2 uses this as the continuous low-speed base instead of a hard 300 cps demand.
      const float ff = tableFeedforward(ref, p);
      // Chi tang dan mo-men khoi dong khi encoder khong co xung.
      const uint32_t quiet = watched == d ? now - lastPulse : 0;
      const float boost = start * clip((float(quiet) - BREAKAWAY_DELAY_MS) / BREAKAWAY_RAMP_MS, 0, 1);
      // Khong bao gio de PWM tut duoi muc banh con quay duoc (tranh chu ky dung-giat-dung).
      const float minOut = fminf(limit, fmaxf(p.runPwm, boost));
      const float base = ff + SPEED_KP * error;
      const float oldRequest = base + integral;
      // Tich phan co dieu kien: khong tich khi dang cham tran, cham san hoac bi gioi han do doc.
      const bool up = error > 0 && oldRequest < limit && fabsf(applied) + 2 >= oldRequest;
      const bool down = error < 0 && oldRequest > minOut && fabsf(applied) <= oldRequest + 2;
      if (up || down) integral = clip(integral + SPEED_KI * error * dt, -I_LIMIT_PWM, I_LIMIT_PWM);
      const float wanted = clip(base + integral, minOut, limit);
      request = d * int(lroundf(wanted));
      const float rate = wanted < fabsf(applied) ? DECEL_PWM_S : ACCEL_PWM_S;
      applied = d * towards(fabsf(applied), wanted, rate * dt);
      lastDrive = d;
      return int(lroundf(applied));
    }
  };

  void wheelTargets(const Command &body, const Wheel &l, const Wheel &r, uint32_t now,
                    Command &lc, Command &rc, float &trim) {
    lc = rc = body; trim = 0;
    // +turn = quay phai: banh trai TIEN, banh phai LUI. -turn = quay trai.
    // Trong vung GIU: body.cps=0, hai banh quay nguoc chieu de xoay tai cho.
    lc.cps = body.cps + body.turnCps;
    rc.cps = (body.cps - body.turnCps) * RIGHT_COUNTS_PER_LEFT_COUNT;
    // A tiny mixed steering demand is below the known controllable speed range;
    // brake that wheel instead of repeatedly stalling/jerking at low PWM.
    const float minWheelCps = (ENABLE_B_CONTINUOUS_DISTANCE || ENABLE_C2_LOW_SPEED_COMP)
                                ? 20.0f : MIN_CPS * 0.50f;
    if (fabsf(lc.cps) < minWheelCps) lc.cps = 0;
    if (fabsf(rc.cps) < minWheelCps) rc.cps = 0;
  #if ENABLE_SYNC_REQUEST_DIFF
    syncErrorCps = 0; syncRequestActive = false;
    const int dl = sign(lc.cps), dr = sign(rc.cps);
    // Keep a stopped wheel stopped, and leave breakaway/low speed to the V2 PI.
    if (!dl || !dr || fabsf(lc.cps) < MIN_CPS || fabsf(rc.cps) < MIN_CPS) return;
    if (abs(LEFT_TUNE.encoderForwardSign) != 1 || abs(RIGHT_TUNE.encoderForwardSign) != 1 ||
        !l.syncSampleValid || !r.syncSampleValid ||
        (uint32_t)(now - l.syncSampleAt) >= CONTROL_MS * 2U ||
        (uint32_t)(now - r.syncSampleAt) >= CONTROL_MS * 2U ||
        l.interlock || r.interlock || l.wrongCounts || r.wrongCounts ||
        l.watched != dl || r.watched != dr || sign(l.applied) != dl || sign(r.applied) != dr ||
        now - l.driveSince <= 75 || now - r.driveSince <= 75) return;
    const float ratio = RIGHT_COUNTS_PER_LEFT_COUNT;
    const float ls = LEFT_TUNE.encoderForwardSign * l.cps;
    const float rs = RIGHT_TUNE.encoderForwardSign * r.cps / ratio;
    if (!isfinite(ls) || !isfinite(rs) || !isfinite(lc.cps) || !isfinite(rc.cps) ||
        dl * ls <= 0 || dr * rs <= 0 ||
        dl * LEFT_TUNE.encoderForwardSign * l.syncInstantCps <= 0 ||
        dr * RIGHT_TUNE.encoderForwardSign * r.syncInstantCps <= 0) return;
    const float requestDiff = lc.cps - rc.cps / ratio;
    syncErrorCps = (ls - rs) - requestDiff;
    const float minRef = fminf(fabsf(lc.cps), fabsf(rc.cps) / ratio);
    // A correction may not push a running wheel into the mixed-demand stop zone.
    const float floorRef = fmaxf(minWheelCps, minWheelCps / ratio);
    const float bound = fmaxf(0.0f, fminf(SYNC_MAX_CPS,
                           fminf(0.25f * minRef, minRef - floorRef)));
    trim = clip(SYNC_KP * syncErrorCps, -bound, bound);
    lc.cps -= trim;
    rc.cps += trim * ratio;
    syncRequestActive = true;
  #else
    const int d = sign(lc.cps);
    if (!d || sign(rc.cps) != d) return; // quay tai cho: khong ep hai banh bang nhau
    if (fabsf(lc.cps) < MIN_CPS || fabsf(rc.cps) < MIN_CPS) return;
    if (l.watched != d || r.watched != d || now - l.driveSince <= 75 || now - r.driveSince <= 75) return;
    const float ls = (float)d * LEFT_TUNE.encoderForwardSign * l.cps;
    const float rs = (float)d * RIGHT_TUNE.encoderForwardSign * r.cps;
    const float lcAbs = fabsf(lc.cps), rcAbs = fabsf(rc.cps);
    const float mismatch = (ls * rcAbs - rs * lcAbs) / fmaxf(lcAbs, rcAbs);
    const float bound = fminf(SYNC_MAX_CPS, 0.25f * fminf(lcAbs, rcAbs));
    trim = clip(SYNC_KP * mismatch, -bound, bound);
    lc.cps -= d * trim;
    rc.cps += d * trim;
  #endif
  }

  bool calibrationOffset(uint16_t *values, uint8_t count, float knownMm,
                        float &offset, float &spread) {
    if (count < 20) return false;
    for (uint8_t i = 1; i < count; ++i) {
      uint16_t v = values[i]; int j = i - 1;
      while (j >= 0 && values[j] > v) { values[j + 1] = values[j]; --j; }
      values[j + 1] = v;
    }
    const uint8_t trim = count / 10;
    float sum = 0.0f;
    for (uint8_t i = trim; i < count - trim; ++i) sum += values[i];
    const float mean = sum / (count - 2 * trim);
    spread = values[count - trim - 1] - values[trim];
    offset = knownMm - mean;
    return spread <= 12.0f && fabsf(offset) <= 150.0f;
  }
  } // namespace Follow

  #ifndef HAND_FOLLOW_NATIVE_TEST
  #include <Arduino.h>
  #include <Wire.h>
  #include <VL53L0X.h>
  #include <EEPROM.h>

  using namespace Follow;
  constexpr uint32_t LED_PIN = PC13, STBY_PIN = PB0;
  constexpr uint32_t LEFT_IN1 = PB14, LEFT_IN2 = PB15, LEFT_PWM = PB9;
  constexpr uint32_t RIGHT_IN1 = PB12, RIGHT_IN2 = PB13, RIGHT_PWM = PB8;
  constexpr uint32_t XSHUT[COUNT] = {PA2, PA3, PA4, PA5};
  constexpr uint8_t ADDRESS[COUNT] = {0x30, 0x31, 0x32, 0x33};
  constexpr const char *NAME[COUNT] = {"J8", "J9", "J10", "J11"};
  constexpr uint32_t ARM_DELAY_MS = 3000, LOG_PERIOD_MS = 200; // reduce UART load for 4-sensor logs
  constexpr int MOTOR_TEST_MAX_PWM = 160;
  constexpr uint32_t SWEEP_STEP_MS = 800;

  enum class Mode { Follow, SensorTest, MotorTest };
  Mode mode = Mode::Follow;

  VL53L0X sensor[COUNT];
  Reading reading[COUNT];
  float offsets[COUNT];
  bool ready[COUNT] = {}, allReady = false, paused = true, logging = false, arming = true;
  const char *fault = nullptr;
  Target target;
  Command command, leftCommand, rightCommand;
  Wheel wheelLeft, wheelRight;
  float syncCps = 0;
  float steerSmooth = 0;
  SpeedSlew bodySlew;
  bool steerEnabled = false;
  uint32_t steerLostAt = 0;
  float turnSlewCps = 0, rawBodyCps = 0, slewBodyCps = 0;
  float predictedGapMm = 0, estimatedGapRate = 0;
  uint32_t maxPollMs[COUNT] = {}, recoveryCount = 0, recoverStableAt = 0;
  const char *previousRecoverFault = nullptr;
  struct SideTurnLimit {
    bool active = false, blocked = false;
    uint32_t beganAt = 0, absentAt = 0;
    void reset() { active = blocked = false; beganAt = absentAt = 0; }
    void apply(Target &t, uint32_t now) {
      const bool sideOnly = t.state == State::TurnLeft || t.state == State::TurnRight;
      if (sideOnly) {
        absentAt = 0;
        if (blocked) { t.state = State::SideTimeout; t.steer = 0; return; }
        if (!active) { beganAt = now; active = true; }
        if ((uint32_t)(now - beganAt) >= SIDE_TURN_MAX_MS) {
          blocked = true; active = false; t.state = State::SideTimeout; t.steer = 0;
        }
      } else {
        active = false;
        // New confirmed front observation disarms side-only timeout immediately.
        if (t.hasFront && (t.state == State::Forward || t.state == State::Reverse || t.state == State::Hold)) {
          reset();
        } else if (blocked) {
          // Continued side returns are NOT a new acquisition. Require 200ms without
          // confirmed side observations, then a fresh two-frame confirmation.
          if (absentAt == 0) absentAt = now;
          if ((uint32_t)(now - absentAt) >= SIDE_REARM_ABSENT_MS) reset();
        }
      }
    }
  } sideTurnLimit;
  uint32_t armAt = 0, lastControl = 0, controlDt = 0, maxControlDt = 0, lastLog = 0;
  int32_t encWindowLeft = 0, encWindowRight = 0;   // co dau: giu duoc chieu quay
  uint16_t previousLeft = 0, previousRight = 0;
  int32_t deltaLeft = 0, deltaRight = 0;
  uint32_t lastProbe[COUNT] = {}, i2cErrors[COUNT] = {};
  bool probeKnown[COUNT] = {};
  volatile bool tickSeen = false, guardArmed = false;
  volatile uint8_t guardExpired = 0;
  volatile uint32_t guardControlAt = 0, guardSampleAt = 0;

  // Lop bao ve doc lap voi loop(): keo STBY xuong neu vong dieu khien tre hoac du lieu qua cu.
  // Chi duoc "len day" khi dong co dang duoc cap PWM; xe dung binh thuong khong kich hoat no.
  extern "C" void HAL_SYSTICK_Callback(void) {
    tickSeen = true;
    if (!guardArmed) return;
    const uint32_t now = HAL_GetTick();
    const bool late = now - guardControlAt >= WATCHDOG_MS;
    if (late || now - guardSampleAt >= GUARD_SAMPLE_MS) {
      GPIOB->BRR = 1U;  // PB0/STBY xuong LOW ke ca khi loop() ket trong I2C
      guardArmed = false;
      guardExpired = late ? 1 : 2;
    }
  }
  void holdLow(uint32_t pin) { digitalWrite(pin, LOW); pinMode(pin, OUTPUT); }
  void stopOutputs() {
    const uint32_t saved = __get_PRIMASK(); __disable_irq();
    digitalWrite(STBY_PIN, LOW); guardArmed = false;
    __set_PRIMASK(saved);
    analogWrite(LEFT_PWM, 0); analogWrite(RIGHT_PWM, 0);
    const uint32_t now = millis();
    wheelLeft.zero(now); wheelRight.zero(now);
  }

  // ---------------------------------------------------------------- Encoder (TIM3 trai, TIM2 phai)
  void configureEncoder(TIM_TypeDef *timer) {
    timer->CR1 = 0; timer->CR2 = 0; timer->SMCR = 0; timer->DIER = 0;
    timer->CCER = 0; timer->PSC = 0; timer->ARR = 65535;
    timer->CCMR1 = 1U | (8U << 4) | (1U << 8) | (8U << 12);
    timer->CCMR2 = 0; timer->SMCR = 3U;  // encoder mode 3 = x4
    timer->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    timer->EGR = TIM_EGR_UG; timer->CNT = 0; timer->SR = 0;
    timer->CR1 = TIM_CR1_CEN;
  }
  void initEncoders() {
    __HAL_RCC_AFIO_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE(); __HAL_RCC_TIM3_CLK_ENABLE();
    HAL_NVIC_DisableIRQ(TIM2_IRQn); HAL_NVIC_DisableIRQ(TIM3_IRQn);
    HAL_NVIC_ClearPendingIRQ(TIM2_IRQn); HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);
    __HAL_RCC_TIM2_FORCE_RESET(); __HAL_RCC_TIM2_RELEASE_RESET();
    __HAL_RCC_TIM3_FORCE_RESET(); __HAL_RCC_TIM3_RELEASE_RESET();
    uint32_t mapr = AFIO->MAPR;
    mapr &= ~(AFIO_MAPR_TIM2_REMAP | AFIO_MAPR_TIM3_REMAP | AFIO_MAPR_SWJ_CFG);
    AFIO->MAPR = mapr | AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
    pinMode(PA6, INPUT_PULLUP); pinMode(PA7, INPUT_PULLUP);
    pinMode(PA0, INPUT_PULLUP); pinMode(PA1, INPUT_PULLUP);
    configureEncoder(TIM2); configureEncoder(TIM3);
    previousLeft = (uint16_t)TIM3->CNT; previousRight = (uint16_t)TIM2->CNT;
  }
  int32_t counterDifference(uint16_t now, uint16_t previous) {
    const uint16_t difference = (uint16_t)(now - previous);
    return difference < 32768U ? difference : (int32_t)difference - 65536L;
  }
  void readEncoders() {
    const uint16_t l = (uint16_t)TIM3->CNT, r = (uint16_t)TIM2->CNT;
    deltaLeft = counterDifference(l, previousLeft);
    deltaRight = counterDifference(r, previousRight);
    previousLeft = l; previousRight = r;
  }

  // ---------------------------------------------------------------- VL53L0X
  bool readRegisters(uint8_t i, uint8_t reg, uint8_t *buffer, uint8_t count) {
    Wire.beginTransmission(ADDRESS[i]); Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    const uint8_t got = Wire.requestFrom(ADDRESS[i], count);
    if (got != count) { while (Wire.available()) Wire.read(); return false; }
    for (uint8_t k = 0; k < count; ++k) {
      if (!Wire.available()) return false;
      buffer[k] = Wire.read();
    }
    return true;
  }
  bool clearReady(uint8_t i) {
    Wire.beginTransmission(ADDRESS[i]);
    Wire.write((uint8_t)VL53L0X::SYSTEM_INTERRUPT_CLEAR); Wire.write((uint8_t)1);
    return Wire.endTransmission() == 0;
  }

  struct CalibrationJob {
    bool active = false;
    uint8_t index = 0, count = 0, discard = 3;
    uint16_t values[40] = {};
    float knownMm = 100.0f;
    uint32_t beganAt = 0;
  } calibration;
  struct BalJob {
    bool active = false;
    uint8_t index = 0, count = 0;
    uint32_t beganAt = 0, lastA = 0, lastB = 0;
    float diffs[24] = {};
  } balJob;

  void resetMeasurements() {
    for (uint8_t i = 0; i < COUNT; ++i) { reading[i].invalidate(); probeKnown[i] = false; }
    target = Target(); command = leftCommand = rightCommand = Command(); syncCps = 0;
    steerSmooth = 0; sideTurnLimit.reset();
    alphaBetaDistance.reset(); zoneConfirm.reset(); bodySlew.reset();
    steerEnabled = false; steerLostAt = 0; turnSlewCps = 0;
  #if ENABLE_FRONT_HANDOFF
    frontHandoff.reset();
  #endif
  #if ENABLE_SYNC_REQUEST_DIFF
    syncErrorCps = 0; syncRequestActive = false;
  #endif
  }
  void sensorBusError(uint8_t i) {
    ++i2cErrors[i]; probeKnown[i] = false;
    reading[i].deviceStatus = 255; reading[i].invalidate();
  }

  void pollSensor(uint8_t i) {
    if (!ready[i]) return;
    const uint32_t probeAt = millis();
    const uint32_t precedingProbe = lastProbe[i];
    const bool bounded = probeKnown[i] && probeAt - precedingProbe <= MAX_POLL_GAP_MS;
    uint8_t interrupt = 0;
    if (!readRegisters(i, VL53L0X::RESULT_INTERRUPT_STATUS, &interrupt, 1)) { sensorBusError(i); return; }
    if (!(interrupt & 0x07)) { lastProbe[i] = probeAt; probeKnown[i] = true; return; }

    // A2: 13-byte block experiments (status & signal & range at [10..11]).
    // KNOWN hardware caveat: last byte of STM32F1 multi-byte RX was observed repeating.
    // A2 OFF: 12-byte diagnostic block and hi-lo-hi independent 1-byte control reads.
    // A2 ON: 13-byte control block; byte[12] DISCARDED (user must validate in SENSOR).
    const uint8_t length = ENABLE_A2_BLOCK13 ? 13U : 12U;
    uint8_t block[13] = {};
    const bool blockOK = readRegisters(i, VL53L0X::RESULT_RANGE_STATUS, block, length);
    const uint8_t rangeReg = (uint8_t)(VL53L0X::RESULT_RANGE_STATUS + 10);
    uint8_t hi = 0, lo = 0, hiAgain = 0;
    const bool needBytes = !ENABLE_A2_BLOCK13 || mode == Mode::SensorTest;
    bool bytesOK = true;
    if (needBytes) bytesOK = readRegisters(i, rangeReg, &hi, 1) &&
        readRegisters(i, (uint8_t)(rangeReg + 1), &lo, 1) &&
        readRegisters(i, rangeReg, &hiAgain, 1);
    uint8_t otherBlock[13] = {};
    bool otherOK = true;
    if (mode == Mode::SensorTest) {
      // Compare 12-byte and 13-byte reads against the same ready result, BEFORE clear.
      otherOK = readRegisters(i, VL53L0X::RESULT_RANGE_STATUS, otherBlock,
                              ENABLE_A2_BLOCK13 ? 12U : 13U);
    }
    uint16_t lib16 = 0;
    if (mode == Mode::SensorTest) lib16 = sensor[i].readReg16Bit(rangeReg);
    const bool clearOK = clearReady(i);
    if (!blockOK || !bytesOK || !otherOK || !clearOK) { sensorBusError(i); return; }
    const uint16_t rawBytes = ((uint16_t)hi << 8) | lo;
    const uint16_t rawBlock13 = ENABLE_A2_BLOCK13 ?
        (((uint16_t)block[10] << 8) | block[11]) :
        (((uint16_t)otherBlock[10] << 8) | otherBlock[11]);
    const uint16_t rawBlock12 = ENABLE_A2_BLOCK13 ?
        (((uint16_t)otherBlock[10] << 8) | otherBlock[11]) :
        (((uint16_t)block[10] << 8) | block[11]);
    const uint16_t raw = ENABLE_A2_BLOCK13 ? rawBlock13 : rawBytes;
    const uint8_t status = (block[0] & 0x78) >> 3;
    const uint16_t signal = ((uint16_t)block[6] << 8) | block[7];
    Reading &s = reading[i];
    s.deviceStatus = status; s.signalQ7 = signal; s.seenFrame = true;
    s.raw = raw; s.rawBytes=rawBytes; s.rawBlock = rawBlock12; s.rawBlock13 = rawBlock13; s.rawLib = lib16;
    s.frameAt = millis();
    lastProbe[i] = probeAt; probeKnown[i] = true;

    if (needBytes && hi != hiAgain) { s.miss(); return; }  // no I2C bus error  // different frames: drop once, not a bus error
    if (!bounded) return;        // khong biet khung nay cu bao lau: bo, cho khung ke tiep
    // Status 11 = phep do duoc cam bien chap nhan. 0 va >=8190 la gia tri "khong co muc tieu".
    if (status != 11 || raw == 0 || raw >= 8190 || signal < 32) { s.miss(); return; }
  #if ENABLE_JUMP_PENDING
    acceptWithJumpGate(reading, i, raw, offsets[i], millis(), precedingProbe - BUDGET_US / 1000U);
  #else
    s.accept(raw, offsets[i], millis(), precedingProbe - BUDGET_US / 1000U);
  #endif
    if (calibration.active && calibration.index == i) {
      if (calibration.discard) --calibration.discard;
      else if (calibration.count < 40) calibration.values[calibration.count++] = raw;
    }
  }

  bool initSensors() {
    Wire.setSCL(PB10); Wire.setSDA(PB11); Wire.begin(); Wire.setClock(I2C_HZ);
    uint8_t count = 0;
    for (uint8_t i = 0; i < COUNT; ++i) {
      pinMode(XSHUT[i], INPUT); delay(100);
      Wire.beginTransmission(0x29);
      if (Wire.endTransmission() != 0) {
        holdLow(XSHUT[i]); Serial.print(NAME[i]); Serial.println(" INIT_PROBE_FAIL"); continue;
      }
      sensor[i].setBus(&Wire); sensor[i].setTimeout(100);
      if (!sensor[i].init() || sensor[i].last_status != 0) {
        holdLow(XSHUT[i]); Serial.print(NAME[i]); Serial.println(" INIT_FAIL"); continue;
      }
      sensor[i].setAddress(ADDRESS[i]); delay(5);
      Wire.beginTransmission(ADDRESS[i]);
      if (Wire.endTransmission() != 0 ||
          !sensor[i].setMeasurementTimingBudget(BUDGET_US) ||
          !sensor[i].setSignalRateLimit(0.25f) || sensor[i].last_status != 0) {
        holdLow(XSHUT[i]); Serial.print(NAME[i]); Serial.println(" CONFIG_FAIL"); continue;
      }
      ready[i] = true; ++count;
      Serial.print(NAME[i]); Serial.println(" READY");
    }
    if (count != COUNT) return false;
    for (uint8_t i = 0; i < COUNT; ++i) {
      sensor[i].startContinuous();
      if (sensor[i].last_status != 0) return false;
    }
    return true;
  }

  // ---------------------------------------------------------------- Hieu chuan offset
  struct LegacyCalibration {
    uint32_t magic;
    float offsets[COUNT];
    uint32_t checksum;
  };
  struct StoredCalibration {
    uint32_t magic;
    float offsets[COUNT];
    float frontBiasMm[2];        // BAL 100, BAL 150: signed J9 - J10 offset in mm
    uint32_t frontBalMask;      // bit 0/1: validated BAL at 100/150mm
    uint32_t checksum;
  };
  template <typename T> uint32_t checksumCalibration(const T &s) {
    uint32_t h = 2166136261UL;
    const uint8_t *p = (const uint8_t *)&s;
    for (size_t i = 0; i < sizeof(s) - sizeof(s.checksum); ++i) h = (h ^ p[i]) * 16777619UL;
    return h;
  }
  void loadCalibration() {
    StoredCalibration s = {};
    EEPROM.get(0, s);
    bool ok = s.magic == 0x48464C33UL && s.checksum == checksumCalibration(s) && !(s.frontBalMask & ~3UL);
    for (uint8_t i = 0; i < COUNT; ++i) ok = ok && isfinite(s.offsets[i]) && fabsf(s.offsets[i]) <= 150.0f;
    for (uint8_t i = 0; i < 2; ++i) ok = ok && isfinite(s.frontBiasMm[i]) && fabsf(s.frontBiasMm[i]) <= 150.0f;
    if (ok) {
      for (uint8_t i = 0; i < COUNT; ++i) offsets[i] = s.offsets[i];
      for (uint8_t i = 0; i < 2; ++i) frontPairBiasMm[i] = s.frontBiasMm[i];
      frontBalMask = (uint8_t)s.frontBalMask;
      Serial.println("CALIBRATION_V3_LOADED");
    } else {
      // Compatible with the known working 883/1029-line code's EEPROM format.
      LegacyCalibration old = {};
      EEPROM.get(0, old);
      bool legacyOK = old.magic == 0x48464C32UL && old.checksum == checksumCalibration(old);
      for (uint8_t i = 0; i < COUNT; ++i) legacyOK = legacyOK && isfinite(old.offsets[i]) && fabsf(old.offsets[i]) <= 150.0f;
      for (uint8_t i = 0; i < COUNT; ++i) offsets[i] = legacyOK ? old.offsets[i] : DEFAULT_OFFSET_MM[i];
      frontPairBiasMm[0] = frontPairBiasMm[1] = 0.0f;
      frontBalMask = 0;
      Serial.println(legacyOK ? "LEGACY_OFFSETS_LOADED_BAL_REQUIRED" : "CALIBRATION_DEFAULT_NEEDS_CHECK_WITH_RULER");
    }
  }
  void showCalibration() {
    for (uint8_t i = 0; i < COUNT; ++i) {
      Serial.print(NAME[i]); Serial.print(" OFFSET_MM="); Serial.println(offsets[i], 2);
    }
    Serial.print("BAL100_J9_MINUS_J10="); Serial.print(frontPairBiasMm[0], 2);
    Serial.print(" BAL150_J9_MINUS_J10="); Serial.print(frontPairBiasMm[1], 2);
    Serial.print(" BAL_MASK="); Serial.println(frontBalMask);
  }
  void saveCalibration() {
    paused = true; stopOutputs();
    StoredCalibration s = {};
    s.magic = 0x48464C33UL;
    for (uint8_t i = 0; i < COUNT; ++i) s.offsets[i] = offsets[i];
    for (uint8_t i = 0; i < 2; ++i) s.frontBiasMm[i] = frontPairBiasMm[i];
    s.frontBalMask = frontBalMask;
    s.checksum = checksumCalibration(s);
    EEPROM.put(0, s);
    StoredCalibration verify = {}; EEPROM.get(0, verify);
    Serial.println(memcmp(&s, &verify, sizeof(s)) == 0 ? "SAVED. RUN de chay." : "SAVE_FAILED");
    resetMeasurements();
  }
  void updateCalibration() {
    if (!calibration.active) return;
    if (calibration.count < 40 && (uint32_t)(millis() - calibration.beganAt) < 6000) return;
    float offset = 0.0f, spread = 0.0f;
    if (calibration.count == 40 && calibrationOffset(calibration.values, 40, calibration.knownMm, offset, spread)) {
      offsets[calibration.index] = offset;
      Serial.print("CAL_OK "); Serial.print(NAME[calibration.index]);
      Serial.print(" OFFSET_MM="); Serial.print(offset, 2);
      Serial.print(" SPREAD_MM="); Serial.println(spread, 1);
      Serial.println("Giu xe dung. CAL mat tiep theo; SAVE de luu; RUN de chay.");
    } else {
      Serial.print("CAL_FAIL: giu bia phang dung yen, che kin tam nhin. MAU=");
      Serial.print(calibration.count); Serial.print(" SPREAD_MM="); Serial.println(spread, 1);
    }
    calibration.active = false;
    resetMeasurements();
  }

  void updateBalCalibration() {
    if (!balJob.active) return;
    const uint32_t now = millis();
    Reading &a = reading[FRONT_A], &b = reading[FRONT_B];
    if (a.fresh(now) && b.fresh(now) && a.streak >= 2 && b.streak >= 2 &&
        a.receivedAt != balJob.lastA && b.receivedAt != balJob.lastB &&
        abs((int32_t)(a.receivedAt - b.receivedAt)) <= 75 &&
        fabsf(a.mm - (balJob.index ? 150.0f : 100.0f)) <= 85.0f &&
        fabsf(b.mm - (balJob.index ? 150.0f : 100.0f)) <= 85.0f) {
      balJob.lastA = a.receivedAt; balJob.lastB = b.receivedAt;
      balJob.diffs[balJob.count++] = a.mm - b.mm;
    }
    if (balJob.count < 24 && (uint32_t)(now - balJob.beganAt) < 8000) return;
    if (balJob.count == 24) {
      float samples[24];
      for (uint8_t i = 0; i < 24; ++i) samples[i] = balJob.diffs[i];
      for (uint8_t i = 1; i < 24; ++i) {
        float x = samples[i]; int j = (int)i - 1;
        while (j >= 0 && samples[j] > x) { samples[j + 1] = samples[j]; --j; }
        samples[j + 1] = x;
      }
      float sum = 0;
      for (uint8_t i = 3; i < 21; ++i) sum += samples[i];
      const float bias = sum / 18.0f;
      const float spread = samples[20] - samples[3];
      if (spread <= 30.0f && fabsf(bias) <= 150.0f) {
        frontPairBiasMm[balJob.index] = bias;
        frontBalMask |= (uint8_t)(1U << balJob.index);
        Serial.print("BAL_OK "); Serial.print(balJob.index ? "150" : "100");
        Serial.print(" J9_MINUS_J10="); Serial.print(bias, 2);
        Serial.print(" SPREAD="); Serial.println(spread, 1);
        Serial.println("BAL each point then SAVE to EEPROM; RUN to resume.");
      } else {
        Serial.print("BAL_FAIL_UNSTABLE SPREAD="); Serial.println(spread, 1);
      }
    } else {
      Serial.print("BAL_FAIL_TIMEOUT SAMPLES="); Serial.println(balJob.count);
    }
    balJob.active = false;
    resetMeasurements();
  }

  // ---------------------------------------------------------------- Dong co
  void latchFault(const char *reason) {
    stopOutputs();
    if (!fault) fault = reason;
    recoverStableAt = 0;
  }
  // E1 (disabled by default). Auto resume only after 300ms of confirmed fresh data,
  // zero encoder movement at rest, and at most 2 tries. Persistent faults remain latched.
  // ENC_DIR auto recovery can only be re-evaluated once motors move again: use cautiously.
  void tryRecoverFault(uint32_t now) {
  #if !ENABLE_E1_RECOVERABLE_FAULTS
    (void)now;
  #else
    if (!fault || recoveryCount >= MAX_AUTO_RECOVERIES || paused || arming ||
        mode != Mode::Follow || !allReady || calibration.active || balJob.active) return;
    const bool known = !strcmp(fault,"STALL_L") || !strcmp(fault,"STALL_R") ||
        !strcmp(fault,"ENC_DIR_L") || !strcmp(fault,"ENC_DIR_R") ||
        !strcmp(fault,"WATCHDOG_CONTROL") || !strcmp(fault,"WATCHDOG_SENSOR");
    if (!known) return; // boot/config errors are never recoverable
    const Target t = locate(reading,now);
    const bool confirmed = t.hasFront && (t.state == State::Hold ||
        t.state == State::Forward || t.state == State::Reverse) &&
        (uint32_t)(now-t.sampleAt)<FRESH_MS;
    if (!confirmed || deltaLeft != 0 || deltaRight != 0) { recoverStableAt=0; return; }
    if (!recoverStableAt) recoverStableAt=now;
    if ((uint32_t)(now-recoverStableAt)<RECOVER_STABLE_MS) return;
    previousRecoverFault=fault;
    const uint32_t saved=__get_PRIMASK(); __disable_irq();
    guardExpired=0; guardArmed=false; __set_PRIMASK(saved);
    fault=nullptr; ++recoveryCount; recoverStableAt=0;
    stopOutputs(); resetMeasurements(); // reacquire fresh confirmed samples
  #endif
  }
  bool encoderConfigOK(TIM_TypeDef *t) {
    return (t->CR1 & TIM_CR1_CEN) && (t->SMCR & 7U) == 3 && t->ARR == 65535 &&
          t->PSC == 0 && t->DIER == 0 && (t->CCMR1 & 0xffffU) == 0x8181U &&
          (t->CCER & 0xffffU) == (TIM_CCER_CC1E | TIM_CCER_CC2E);
  }
  // pwm > 0 = TIEN. Trai tien: IN1 LOW, IN2 HIGH. Phai tien: IN1 HIGH, IN2 LOW.
  void setWheelPins(int pwm, bool left, bool coast=false) {
    const uint32_t a = left ? LEFT_IN1 : RIGHT_IN1, b = left ? LEFT_IN2 : RIGHT_IN2;
    const uint32_t out = left ? LEFT_PWM : RIGHT_PWM;
    const bool reversed = left ? LEFT_REVERSE : RIGHT_REVERSE;
    if (!pwm) {
      analogWrite(out, 0);
      digitalWrite(a, coast ? LOW : HIGH); digitalWrite(b, coast ? LOW : HIGH);
      // both LOW = coast / both HIGH = short brake
      return;
    }
    const bool in1High = (pwm > 0) != reversed;
    digitalWrite(a, in1High ? HIGH : LOW); digitalWrite(b, in1High ? LOW : HIGH);
    analogWrite(out, abs(pwm));
  }
  // PWM thuc te doc lai tu thanh ghi TIM4 va chan IN/STBY, khong suy tu PWM yeu cau.
  int pwmReadback(bool left) {
    const uint32_t odr = GPIOB->ODR;
    const bool in1 = odr & (1U << (left ? 14 : 12)), in2 = odr & (1U << (left ? 15 : 13));
    if (!(odr & 1U) || in1 == in2 || !(TIM4->CR1 & TIM_CR1_CEN) ||
        !(TIM4->CCER & (left ? TIM_CCER_CC4E : TIM_CCER_CC3E))) return 0;
    const uint32_t period = TIM4->ARR + 1U;
    const uint32_t ccr = left ? TIM4->CCR4 : TIM4->CCR3;
    const int duty = int((ccr * 255U + period / 2U) / period);
    const bool forward = in1 != (left ? LEFT_REVERSE : RIGHT_REVERSE);
    return forward ? duty : -duty;
  }
  void applyOutputs(int l, int r, uint32_t sampleAt, bool coastL=false, bool coastR=false) {
    const bool moving = l != 0 || r != 0;
    if (moving && (guardExpired || millis() - sampleAt >= FRESH_MS)) { stopOutputs(); return; }
    setWheelPins(l, true, coastL); setWheelPins(r, false, coastR);
    const uint32_t saved = __get_PRIMASK(); __disable_irq();
    const uint32_t now = millis();
    const bool allowed = !guardExpired && (!moving || now - sampleAt < FRESH_MS);
    guardArmed = allowed && moving;
    if (guardArmed) { guardSampleAt = sampleAt; guardControlAt = now; }
    digitalWrite(STBY_PIN, allowed ? HIGH : LOW);
    __set_PRIMASK(saved);
    if (!allowed) stopOutputs();
  }

  // D4 optional MPU6050 yaw-rate feedback; no new libraries, disabled by default.
  // Assumes MPU Z is vertical; gyro mounting and zero-bias have NOT been checked.
  // If the sensor is missing or I2C read fails, automatically retain encoder-only steering.
  float gyroZDegS=0; bool gyroOK=false;
  void gyroInit() {
  #if ENABLE_D4_GYRO_YAW
    Wire.beginTransmission(0x68); Wire.write((uint8_t)0x6B); Wire.write((uint8_t)0);
    if (Wire.endTransmission()!=0) return;
    Wire.beginTransmission(0x68); Wire.write((uint8_t)0x1B); Wire.write((uint8_t)0);
    gyroOK=Wire.endTransmission()==0; // +/- 250 deg/s
  #endif
  }
  void gyroUpdate() {
  #if ENABLE_D4_GYRO_YAW
    if (!gyroOK) return;
    uint8_t buf[2]={};
    Wire.beginTransmission(0x68); Wire.write((uint8_t)0x47); // GYRO_ZOUT_H
    if (Wire.endTransmission()!=0 || Wire.requestFrom((uint8_t)0x68,(uint8_t)2)!=2) {
      gyroOK=false; return;
    }
    buf[0]=(uint8_t)Wire.read(); buf[1]=(uint8_t)Wire.read();
    gyroZDegS=(int16_t)((uint16_t(buf[0])<<8)|buf[1]) / 131.0f;
  #endif
  }
  // ---------------------------------------------------------------- MOTOR_TEST
  // Chay PWM co dinh trong thoi gian gioi han, khong dung cam bien. Watchdog vong dieu khien van hoat dong.
  struct MotorJob {
    bool active = false, sweep = false;
    int pwmL = 0, pwmR = 0, dir = 1;
    uint8_t step = 0;
    uint32_t stepAt = 0, durationMs = 1500, steadyMs = 0;
    int32_t countsL = 0, countsR = 0, steadyL = 0, steadyR = 0;
  } motorJob;
  constexpr uint8_t SWEEP_STEPS = 10;  // PWM 40, 50, ... 130

  void beginMotorStep(uint32_t now) {
    if (motorJob.sweep) motorJob.pwmL = motorJob.pwmR = motorJob.dir * (40 + 10 * motorJob.step);
    motorJob.stepAt = now; motorJob.steadyMs = 0;
    motorJob.countsL = motorJob.countsR = motorJob.steadyL = motorJob.steadyR = 0;
  }
  void printSigned(const char *label, long v) { Serial.print(label); Serial.print(v); }
  void finishMotorStep(uint32_t now) {
    stopOutputs();
    const float seconds = fmaxf(0.001f, motorJob.steadyMs * 0.001f);
    const long cpsL = lroundf(motorJob.steadyL / seconds), cpsR = lroundf(motorJob.steadyR / seconds);
    if (motorJob.sweep) {
      printSigned("SWEEP PWM=", motorJob.pwmL);
      printSigned(" CPS_L=", cpsL); printSigned(" CPS_R=", cpsR); Serial.println();
      if (++motorJob.step < SWEEP_STEPS) { beginMotorStep(now); return; }
      Serial.println("SWEEP_DONE. startFwd/startRev = PWM dau tien co CPS on dinh; runPwm thap hon mot chut.");
    } else {
      printSigned("MOTOR_DONE PWM_L=", motorJob.pwmL); printSigned(" PWM_R=", motorJob.pwmR);
      printSigned(" COUNTS_L=", motorJob.countsL); printSigned(" COUNTS_R=", motorJob.countsR);
      printSigned(" STEADY_CPS_L=", cpsL); printSigned(" STEADY_CPS_R=", cpsR); Serial.println();
      if (motorJob.pwmL && labs(motorJob.countsL) > 20)
        Serial.println((motorJob.countsL > 0) == (motorJob.pwmL > 0) ? "LEFT encoderForwardSign = +1" : "LEFT encoderForwardSign = -1");
      if (motorJob.pwmR && labs(motorJob.countsR) > 20)
        Serial.println((motorJob.countsR > 0) == (motorJob.pwmR > 0) ? "RIGHT encoderForwardSign = +1" : "RIGHT encoderForwardSign = -1");
      if ((motorJob.pwmL && labs(motorJob.countsL) <= 20) || (motorJob.pwmR && labs(motorJob.countsR) <= 20))
        Serial.println("KHONG_CO_XUNG: banh khong quay (nguon VM? PWM qua thap?) hoac day encoder loi.");
    }
    motorJob.active = false; mode = Mode::Follow; paused = true;
    Serial.println("PAUSED. RUN de bam vat can.");
  }
  void runMotorJob(uint32_t now) {
    if (!motorJob.active) { stopOutputs(); return; }
    const uint32_t elapsed = now - motorJob.stepAt;
    motorJob.countsL += deltaLeft; motorJob.countsR += deltaRight;
    if (elapsed >= motorJob.durationMs / 2) {
      motorJob.steadyL += deltaLeft; motorJob.steadyR += deltaRight; motorJob.steadyMs += controlDt;
    }
    if (elapsed >= motorJob.durationMs) { finishMotorStep(now); return; }
    applyOutputs(motorJob.pwmL, motorJob.pwmR, now);
    if (guardExpired) motorJob.active = false;
  }


// ================= MAP AUTO V1: CONFIGURATION IN MILLIMETRES =================
// Origin: bottom-left of map. +X right, +Y up. Heading CCW from +X.
// Tape is NOT sensed. Coordinates below are estimates from the supplied picture.
// Measure these before competition. Never claim collision-free operation from odometry.
namespace MapAuto {
// Round 1: Bluetooth route recording; Square saves into buffered emulated EEPROM.
// Round 2: oriented map planner prefers that taught corridor, replans for obstacles.
constexpr uint32_t BLUETOOTH_BAUD=9600; // match actual HC05/HC06 data baud
constexpr float SLOW_SCAN_MM=450, SCAN_STOP_MM=220, SCAN_RELEASE_MM=300;
constexpr uint8_t SCAN_FRAMES=6;
struct TeachPoint {int16_t x,y;};
TeachPoint taught[128]; uint8_t taughtCount=0;
bool teaching=false, taughtReady=false, manualEnabled=false, awaitingAuto=false;
char manualCommand='0'; uint32_t manualAt=0;
float manualSpeed=40, lastTeachH=0;
uint32_t scanAt=0, scanFrameAt[4]={}; uint8_t scanFrames[4]={};
bool scanActive=false;
bool autoBootPending=false, routeSaved=false;
uint32_t bootReadyAt=0;
constexpr uint32_t AUTO_BOOT_DELAY_MS=5000;
constexpr int ROUTE_EEPROM_OFFSET=64; // calibration occupies bytes 0..35; preserve it
constexpr uint32_t ROUTE_MAGIC=0x3254524E;
struct RouteHeader {uint32_t magic;uint16_t version,count;uint32_t config,crc;};
static_assert(sizeof(RouteHeader)==16,"route header layout");


constexpr float MAP_PI=3.14159265359f;
constexpr float START_X=325, START_Y=125, START_HEADING=MAP_PI/2;
constexpr float GOAL_X=1525, GOAL_Y=-175;
constexpr float BODY_WIDTH=100, BODY_LENGTH=120;
constexpr float COUNTS_MM_L=10, COUNTS_MM_R=10; // UNCALIBRATED x4 counts/mm
constexpr float GYRO_SCALE=1.0f; // measured actual angle / raw gyro angle
constexpr float TRACK=120, GYRO_SIGN=1; // Z up; change sign only after rotation test
constexpr float CLEARANCE=12; // geometric margin, PLUS accumulated position uncertainty
constexpr float RADIUS=79+ CLEARANCE; // circumscribed radius of 100x120 body
constexpr float MAX_SPEED=45, MAX_OMEGA=0.48f;
constexpr int NX=37, NY=33, N=NX*NY, STEP=50, YMIN=-400;
struct Box {float x,y;};
// T1..T13 only. Each box is 100x100. T14..17 are detected online.
const Box boxes[13]={{60,150},{550,250},{310,350},{550,450},{60,550},
 {60,750},{360,900},{460,1150},{660,750},{760,650},{860,1000},
 {960,650},{1160,1150}};
struct Hit {float x,y;};
Hit hits[24]; uint8_t hitCount=0;
constexpr int STATES=N*4;
uint8_t costs[STATES]; uint8_t flags[STATES]; // 8.6KB: quantized bounded cost, no heap
float x=START_X,y=START_Y,h=START_HEADING,travel=0,uncertainty=0;
int16_t startNode=-1, goalNode=-1, nextNode=-1;
bool active=false, planning=false, pathReady=false, done=false, positioned=false;
uint32_t runAt=0, lastProgress=0, sampleAt=0, turnAt=0, blindAt=0;
float progressX=0,progressY=0;
const char *status="BOOT_PAUSED";
TwoWire imu(PB7,PB6);
uint8_t imuAddress=0x68; bool imuReady=false; uint16_t biasCount=0;
float bias=0,biasSum=0,rate=0; uint32_t imuAt=0;
struct MeasureJob {
 bool active=false;int32_t rawL=0,rawR=0;float gyroDegrees=0;
 uint32_t beganAt=0,badReads=0,gapCount=0,maxDtMs=0;
} measure;
float wrap(float a){while(a>MAP_PI)a-=2*MAP_PI;while(a<-MAP_PI)a+=2*MAP_PI;return a;}
bool byteRead(uint8_t reg,uint8_t &v){
 imu.beginTransmission(imuAddress);imu.write(reg);
 if(imu.endTransmission()!=0 || imu.requestFrom(imuAddress,(uint8_t)1)!=1)return false;
 v=imu.read();return true;
}
bool writeReg(uint8_t r,uint8_t v){imu.beginTransmission(imuAddress);imu.write(r);imu.write(v);return imu.endTransmission()==0;}
void init(){
 imu.begin();imu.setClock(100000);
 uint8_t who=0;
 for(uint8_t a=0x68;a<=0x69;++a){imuAddress=a;if(byteRead(0x75,who)&&who==0x68){
 imuReady=writeReg(0x6B,1)&&writeReg(0x1B,0)&&writeReg(0x1A,3)&&writeReg(0x19,9);break;}}
 status=imuReady?"GYRO_CAL_KEEP_STILL":"MPU_INIT_FAILED";
}
bool updatePose(uint32_t now,float dt){
 if(measure.active){measure.rawL+=deltaLeft;measure.rawR+=deltaRight;
 uint32_t ms=(uint32_t)lroundf(dt*1000);if(ms>measure.maxDtMs)measure.maxDtMs=ms;
 if(ms>30)++measure.gapCount;}
 float dl=deltaLeft*LEFT_TUNE.encoderForwardSign/COUNTS_MM_L;
 float dr=deltaRight*RIGHT_TUNE.encoderForwardSign/COUNTS_MM_R;
 float ds=(dl+dr)*0.5f, de=(dr-dl)/TRACK;
 if(!imuReady){status="MPU_INIT_FAILED";return false;}
 uint8_t a,b,c;
 if(!byteRead(0x47,a)||!byteRead(0x48,b)||!byteRead(0x47,c)||a!=c){if(measure.active)++measure.badReads;status="MPU_READ_FAILED";return false;}
 float z=(int16_t)((uint16_t(a)<<8)|b)/131.0f;
 imuAt=now;
 if(biasCount<250){
  if(fabsf(dl)+fabsf(dr)>0.2f||fabsf(z)>5){biasCount=0;biasSum=0;status="GYRO_CAL_KEEP_STILL";return false;}
  biasSum+=z;if(++biasCount==250){bias=biasSum/250;status="READY_AUTO";}
  return false;
 }
 if(measure.active)measure.gyroDegrees+=GYRO_SIGN*(z-bias)*dt; // raw signed, unwrapped, BEFORE scale/fusion
 rate=GYRO_SCALE*GYRO_SIGN*(z-bias)*MAP_PI/180;
 float dh=0.9f*rate*dt+0.1f*de;
 if(active && fabsf(rate*dt-de)>0.16f){status="SLIP_OR_GYRO_SIGN";return false;}
 x+=ds*cosf(h+dh*0.5f);y+=ds*sinf(h+dh*0.5f);h=wrap(h+dh);
 travel+=fabsf(ds);
 // Conservative budget, NOT a bound proved for this vehicle.
 uncertainty=0.003f*travel; // assumed drift budget, MUST measure on floor
 return true;
}

float nodeX(int n){return ((n/4)%NX)*STEP+25;}
float nodeY(int n){return ((n/4)/NX)*STEP+YMIN+25;}
float nodeH(int n){return (n%4)*MAP_PI/2;}
int node(float px,float py){int ix=lroundf((px-25)/STEP),iy=lroundf((py-YMIN-25)/STEP);int ih=(int(lroundf(h/(MAP_PI/2)))%4+4)%4;return ix<0||ix>=NX||iy<0||iy>=NY?-1:(iy*NX+ix)*4+ih;}
bool overlaps(float px,float py,float angle,float bx,float by,float halfx,float halfy){
 // SAT: axes of the world box and the rotated robot rectangle.
 float u=cosf(angle),v=sinf(angle),dx=bx-px,dy=by-py;
 float hl=BODY_LENGTH/2+CLEARANCE+uncertainty,hw=BODY_WIDTH/2+CLEARANCE+uncertainty;
 float au=fabsf(u),av=fabsf(v);
 if(fabsf(dx)>halfx+hl*au+hw*av)return false;
 if(fabsf(dy)>halfy+hl*av+hw*au)return false;
 if(fabsf(dx*u+dy*v)>hl+halfx*au+halfy*av)return false;
 if(fabsf(-dx*v+dy*u)>hw+halfx*av+halfy*au)return false;
 return true;
}
bool poseFree(float px,float py,float angle){
 float hl=BODY_LENGTH/2+CLEARANCE+uncertainty,hw=BODY_WIDTH/2+CLEARANCE+uncertainty;
 float u=cosf(angle),v=sinf(angle),ex=hl*fabsf(u)+hw*fabsf(v),ey=hl*fabsf(v)+hw*fabsf(u);
 if(px-ex<0||px+ex>1800||py+ey>1200||py-ey<YMIN)return false;
 // Lower tape may be crossed ONLY across finish aperture [1200,1800].
 if(py-ey<0 && !(px-ex>=1200 && px+ex<=1800))return false;
 if(overlaps(px,py,angle,900,300,300,300))return false;
 for(const auto &b:boxes)if(overlaps(px,py,angle,b.x,b.y,50,50))return false;
 for(uint8_t i=0;i<hitCount;++i)if(overlaps(px,py,angle,hits[i].x,hits[i].y,120,120))return false;
 return true;
}
bool pointFree(float px,float py){return poseFree(px,py,h);}
bool segmentAt(float ax,float ay,float bx,float by,float angle){
 int steps=int(ceilf(hypotf(bx-ax,by-ay)/10))+1;
 for(int i=0;i<=steps;++i){float f=float(i)/steps;if(!poseFree(ax+(bx-ax)*f,ay+(by-ay)*f,angle))return false;}return true;
}
bool segmentFree(float ax,float ay,float bx,float by){return segmentAt(ax,ay,bx,by,h);}
bool rotationFree(float px,float py,float from,float to){
 float diff=wrap(to-from);
 for(int i=0;i<=18;++i)if(!poseFree(px,py,from+diff*i/18))return false;
 return true;
}
int previous(int n){
 switch(flags[n]&7){
 case 1:{const int hd=n%4;int cell=n/4;cell-=hd==0?1:hd==1?NX:hd==2?-1:-NX;return cell*4+hd;}
 case 2:return (n/4)*4+((n%4+3)%4);
 case 3:return (n/4)*4+((n%4+1)%4);
 default:return -1;}
}
void halt(const char *why){active=false;planning=false;pathReady=false;status=why;paused=true;stopOutputs();}
void plan(){
 stopOutputs();planning=true;pathReady=false;nextNode=-1;
 startNode=node(x,y);goalNode=node(GOAL_X,GOAL_Y);if(goalNode>=0)goalNode=(goalNode/4)*4+3;
 if(startNode<0||goalNode<0||!rotationFree(x,y,h,nodeH(startNode))||!segmentAt(x,y,nodeX(startNode),nodeY(startNode),nodeH(startNode))){halt("START_OUTSIDE_SAFE_MAP");return;}
 for(int i=0;i<STATES;++i){costs[i]=255;flags[i]=0;}
 costs[startNode]=0;flags[startNode]=64;status="PLANNING";
}
void relax(int from,int to,unsigned step,uint8_t action){
 if(to<0||to>=STATES||(flags[to]&128))return;
 // Soft preference for recorded corridor. Obstacles/boundaries remain hard constraints.
 unsigned penalty=0;
 if(taughtReady && taughtCount){float nearest=1e9f;
 for(uint8_t k=0;k<taughtCount;++k)nearest=fminf(nearest,hypotf(nodeX(to)-taught[k].x,nodeY(to)-taught[k].y));
 penalty=nearest>150?1:0;}
 unsigned g=costs[from]+(step+49)/50+penalty; // units50mm; 255=unreachable
 if(g>=255)return; // bounded planner: fail safely rather than wrap
 if(g<costs[to]){costs[to]=g;flags[to]=64|action;}
}
void planStep(){
 // Two expansions/tick keep the sensor/control/watchdog service running.
 for(int budget=0;budget<2 && planning;++budget){
 int best=-1;float score=1e9f;
 for(int i=0;i<STATES;++i)if((flags[i]&192)==64){float v=costs[i]+hypotf(nodeX(i)-nodeX(goalNode),nodeY(i)-nodeY(goalNode))/50.0f;if(v<score){score=v;best=i;}}
 if(best<0){halt("NO_SAFE_PATH");return;}
 if(best==goalNode){planning=false;pathReady=true;nextNode=goalNode;
  while(previous(nextNode)!=startNode && previous(nextNode)>=0)nextNode=previous(nextNode);
  status="TRACK_PATH";return;}
 flags[best]|=128;
 int hd=best%4,cell=best/4,ix=cell%NX,iy=cell/NX;
 int dx=hd==0?1:hd==2?-1:0,dy=hd==1?1:hd==3?-1:0;
 int fx=ix+dx,fy=iy+dy;
 if(fx>=0&&fx<NX&&fy>=0&&fy<NY){int dest=(fy*NX+fx)*4+hd;
 if(segmentAt(nodeX(best),nodeY(best),nodeX(dest),nodeY(dest),nodeH(best)))relax(best,dest,STEP,1);}
 for(int sign=-1;sign<=1;sign+=2){int dest=cell*4+(hd+sign+4)%4;
 if(rotationFree(nodeX(best),nodeY(best),nodeH(best),nodeH(dest)))relax(best,dest,75,sign==1?2:3);}
 }
}
bool addHit(float px,float py){
 // Known obstacle surfaces are already in the exact box costmap; do not double-inflate.
 for(const auto &b:boxes)if(fabsf(px-b.x)<=75 && fabsf(py-b.y)<=75)return false;
 if(px>=575 && px<=1225 && py>=-25 && py<=625)return false;

 for(uint8_t i=0;i<hitCount;++i)if(hypotf(px-hits[i].x,py-hits[i].y)<65)return false;
 if(hitCount>=24){halt("OBSTACLE_MEMORY_FULL");return false;}
 hits[hitCount++]={px,py};return true;
}
void recordPoint(bool force=false){
 if(!teaching)return;
 if(!force && taughtCount && hypotf(x-taught[taughtCount-1].x,y-taught[taughtCount-1].y)<40 && fabsf(wrap(h-lastTeachH))<0.25f)return;
 if(taughtCount>=128){teaching=false;taughtReady=false;manualEnabled=false;halt("TEACH_MEMORY_FULL_REDO");return;}
 taught[taughtCount++]={(int16_t)lroundf(x),(int16_t)lroundf(y)};lastTeachH=h;
}
void reset(){
 halt("RESET_POSE_KEEP_AT_START");x=START_X;y=START_Y;h=START_HEADING;
 travel=uncertainty=0;hitCount=0;done=false;positioned=true;
}
void start(){
 if(fault||!allReady||arming||!imuReady||biasCount<250||!positioned||calibration.active||balJob.active){status="NOT_READY_USE_MAPRESET";return;}
 if(done){status="DONE_USE_MAPRESET";return;}
 if(!taughtReady){status="TEACH_FIRST_C_THEN_E";return;}
 teaching=false;manualEnabled=false;scanActive=false;
 mode=Mode::Follow;paused=false;active=true;blindAt=turnAt=0;runAt=lastProgress=millis();progressX=x;progressY=y;plan();
}


// STM32duino buffered EEPROM emulation. Single stopped-state flush per save.
// Firmware MUST fit below 0x0800FC00 on the 64KiB F103C8 (reserve last 1KiB).
bool flashLayoutOK(){
#if defined(ARDUINO_ARCH_STM32) && defined(STM32F1xx)
 extern uint8_t flashDataLoad[] asm("_sidata");
 extern uint8_t ramDataBegin[] asm("_sdata");
 extern uint8_t ramDataEnd[] asm("_edata");
 uintptr_t loadEnd=reinterpret_cast<uintptr_t>(flashDataLoad)+
 (reinterpret_cast<uintptr_t>(ramDataEnd)-reinterpret_cast<uintptr_t>(ramDataBegin));
 return loadEnd<=0x0800FC00UL;
#else
 return false; // no unverified storage backend on a different target
#endif
}
uint32_t routeConfig(){
 // Change fingerprint when mechanical or odometry calibration changes.
 const float values[]={START_X,START_Y,START_HEADING,BODY_WIDTH,BODY_LENGTH,
 COUNTS_MM_L,COUNTS_MM_R,TRACK,GYRO_SIGN,GYRO_SCALE,CLEARANCE};
 uint32_t v=2166136261UL;const uint8_t *p=(const uint8_t*)values;
 for(size_t i=0;i<sizeof(values);++i)v=(v^p[i])*16777619UL;
 const uint8_t *b=(const uint8_t*)boxes;
 for(size_t i=0;i<sizeof(boxes);++i)v=(v^b[i])*16777619UL;
 return v;
}
uint32_t routeCRC(const RouteHeader &header){
 uint32_t crc=2166136261UL;
 const uint8_t *a=(const uint8_t*)&header;
 for(size_t i=0;i<12;++i)crc=(crc^a[i])*16777619UL;
 a=(const uint8_t*)taught;
 for(size_t i=0;i<header.count*sizeof(TeachPoint);++i)crc=(crc^a[i])*16777619UL;
 return crc;
}
bool storageOK(){return flashLayoutOK() && EEPROM.length()>=ROUTE_EEPROM_OFFSET+int(sizeof(RouteHeader)+sizeof(taught));}
void loadRoute(){
 taughtReady=routeSaved=autoBootPending=false;
 if(!storageOK()){status="FLASH_LAYOUT_OR_CAPACITY_ERROR";return;}
 RouteHeader header={};EEPROM.get(ROUTE_EEPROM_OFFSET,header);
 if(header.magic!=ROUTE_MAGIC || header.version!=1 || header.count<2 || header.count>128 || header.config!=routeConfig())return;
 for(uint16_t i=0;i<header.count;++i)EEPROM.get(ROUTE_EEPROM_OFFSET+sizeof(header)+i*sizeof(TeachPoint),taught[i]);
 if(header.crc!=routeCRC(header)){status="ROUTE_CRC_BAD_MANUAL_ONLY";return;}
 for(uint16_t i=0;i<header.count;++i)if(taught[i].x<0||taught[i].x>1800||taught[i].y<-250||taught[i].y>1200){status="ROUTE_BOUNDS_BAD";return;}
 if(hypotf(taught[0].x-START_X,taught[0].y-START_Y)>30){status="ROUTE_START_BAD";return;}
 taughtCount=header.count;taughtReady=routeSaved=autoBootPending=true;status="SAVED_ROUTE_BOOT_AUTO_PENDING";
}
bool invalidateRoute(){
 stopOutputs();autoBootPending=false;routeSaved=false;
 if(!storageOK()){status="FLASH_LAYOUT_OR_CAPACITY_ERROR";return false;}
 eeprom_buffer_fill();
 for(int i=0;i<4;++i)eeprom_buffered_write_byte(ROUTE_EEPROM_OFFSET+i,0);
 eeprom_buffer_flush();
 uint32_t magic=ROUTE_MAGIC;EEPROM.get(ROUTE_EEPROM_OFFSET,magic);
 if(magic==ROUTE_MAGIC){status="FLASH_INVALIDATE_FAILED";return false;}return true;
}
bool saveRoute(){
 stopOutputs();
 if(!storageOK()||!taughtReady||taughtCount<2){status="FLASH_SAVE_NOT_READY";return false;}
 RouteHeader header={ROUTE_MAGIC,1,taughtCount,routeConfig(),0};header.crc=routeCRC(header);
 eeprom_buffer_fill(); // preserve calibration and all unused bytes
 const uint8_t *data=(const uint8_t*)&header;
 for(size_t i=0;i<sizeof(header);++i)eeprom_buffered_write_byte(ROUTE_EEPROM_OFFSET+i,data[i]);
 data=(const uint8_t*)taught;
 for(size_t i=0;i<taughtCount*sizeof(TeachPoint);++i)eeprom_buffered_write_byte(ROUTE_EEPROM_OFFSET+sizeof(header)+i,data[i]);
 eeprom_buffer_flush();
 RouteHeader verify={};EEPROM.get(ROUTE_EEPROM_OFFSET,verify);
 if(verify.magic!=header.magic||verify.version!=header.version||verify.count!=header.count||verify.config!=header.config||verify.crc!=header.crc){status="FLASH_VERIFY_FAILED";return false;}
 for(size_t i=0;i<taughtCount*sizeof(TeachPoint);++i)if(EEPROM.read(ROUTE_EEPROM_OFFSET+sizeof(header)+i)!=data[i]){status="FLASH_VERIFY_FAILED";return false;}
 routeSaved=true;status="ROUTE_SAVED_POWER_OFF_ALLOWED";return true;
}
void bootAutoService(uint32_t now){
 if(!autoBootPending)return;
 if(fault){autoBootPending=false;status="BOOT_FAULT_NO_AUTO";return;}
 if(arming||!allReady||!imuReady||biasCount<250){bootReadyAt=0;return;}
 for(uint8_t i=0;i<COUNT;++i)if(!reading[i].fresh(now)||reading[i].streak<2){bootReadyAt=0;return;}
 if(!bootReadyAt){bootReadyAt=now;status="BOOT_AUTO_IN_5_SECONDS";return;}
 if(now-bootReadyAt<AUTO_BOOT_DELAY_MS)return;
 autoBootPending=false;reset();start();
}
void rawCommand(char c){
 if(measure.active){manualCommand='0';manualEnabled=false;stopOutputs();return;}
 if(c=='Z'||c=='\r'||c=='\n'||c==' ')return; // release of non-motion buttons
 if(c=='S' && teaching){rawCommand('E');return;} // square: stop and seal round 1
 if(c=='P'||c=='S'||c=='X'){
 autoBootPending=false;bootReadyAt=0;
 manualCommand='0';manualEnabled=false;halt("PAUSED");return;}
 if(c=='0'){manualCommand='0';if(teaching)stopOutputs();return;}
 if(c=='C'){
 if(fault||arming||!allReady||!imuReady||biasCount<250){status="NOT_READY";return;}
 if(!invalidateRoute()){halt(status);return;}
 reset();awaitingAuto=false;taughtCount=0;taughtReady=false;teaching=true;manualEnabled=true;
 mode=Mode::Follow;paused=false;manualCommand='0';recordPoint(true);status="ROUND1_BT_RECORD";return;}
 if(c=='E'){
 manualCommand='0';manualEnabled=false;recordPoint(true);teaching=false;taughtReady=taughtCount>=2;
 awaitingAuto=false;
 halt("ROUND1_STOP_SAVE");
 if(taughtReady){if(!saveRoute()){taughtReady=false;halt(status);}}
 else status="TEACH_TOO_SHORT";
 return;}
 if(c=='Y'){
 if(!taughtReady){status="NO_TAUGHT_ROUTE";return;}
 // User physically replaces car at ORIGINAL start and original heading before Y.
 reset();start();if(active)awaitingAuto=false;return;}
 if(c=='A'){
 if(routeSaved && !active && !teaching && !positioned){return;}
 if(routeSaved && !teaching && !active && !autoBootPending && status && !strcmp(status,"ROUTE_SAVED_POWER_OFF_ALLOWED")){return;}
 if(awaitingAuto){rawCommand('Y');return;}
 if(!teaching && !taughtReady){rawCommand('C');return;}
 if(teaching){manualEnabled=true;paused=false;mode=Mode::Follow;manualCommand='0';status="ROUND1_BT_RECORD";}
 else {start();}
 return;}
 if(c=='U'){manualSpeed=fminf(60,manualSpeed+5);return;}
 if(c=='D'){manualSpeed=fmaxf(25,manualSpeed-5);return;}
 if(c=='M'){manualSpeed=40;return;}
 if(c=='F'||c=='B'||c=='L'||c=='R'||c=='G'||c=='I'||c=='H'||c=='J'){
 if(!teaching||!manualEnabled||fault||arming)return;
 manualCommand=c;manualAt=millis();return;}
}
bool manualTick(uint32_t now){
 if(!teaching||!manualEnabled||paused)return false;
 recordPoint();if(!teaching)return false;
 if(now-manualAt>1000){manualCommand='0';status="BT_COMMAND_TIMEOUT";return false;}
 sampleAt=now;
 for(uint8_t i=0;i<4;++i){if(!reading[i].fresh(now)||reading[i].streak<2){status="BT_WAIT_SENSORS";return false;}
 if(now-reading[i].sampleAt>now-sampleAt)sampleAt=reading[i].sampleAt;}
 float v=0,w=0;
 switch(manualCommand){case 'F':v=manualSpeed;break;case 'B':v=-manualSpeed;break;
 case 'L':w=MAX_OMEGA;break;case 'R':w=-MAX_OMEGA;break;
 case 'G':v=manualSpeed;w=0.30f;break;case 'I':v=manualSpeed;w=-0.30f;break;
 case 'H':v=-manualSpeed;w=0.30f;break;case 'J':v=-manualSpeed;w=-0.30f;break;
 default:return false;}
 float front=fminf(reading[1].mm,reading[2].mm);
 if(v>0 && front<SCAN_STOP_MM){status="BT_FRONT_STOP";return false;}
 if(v>0 && front<SLOW_SCAN_MM)v=fminf(v,20);
 if(w!=0 && fminf(reading[0].mm,reading[3].mm)<85){status="BT_SIDE_STOP";return false;}
 // Reverse is operator-supervised: no rear range sensor exists.
 float px=x+v*0.35f*cosf(h),py=y+v*0.35f*sinf(h);
 if(!segmentFree(x,y,px,py)||!rotationFree(px,py,h,h+w*0.35f)){status="BT_MAP_MARGIN_STOP";return false;}
 command=Command();command.cps=v*10;command.turnCps=-w*TRACK*5;
 status="ROUND1_BT_RECORD";return true;
}
float scanScale(uint32_t now,bool &mayMap){
 mayMap=false;float nearest=8190;
 for(uint8_t i=0;i<4;++i)nearest=fminf(nearest,reading[i].mm);
 if(!scanActive && nearest<SLOW_SCAN_MM){
 scanActive=true;scanAt=now;
 for(uint8_t i=0;i<4;++i){scanFrames[i]=0;scanFrameAt[i]=reading[i].receivedAt;}}
 if(!scanActive)return 1;
 bool enough=true;
 for(uint8_t i=0;i<4;++i){if(scanFrameAt[i]!=reading[i].receivedAt){scanFrameAt[i]=reading[i].receivedAt;if(scanFrames[i]<SCAN_FRAMES)++scanFrames[i];}
 if(scanFrames[i]<SCAN_FRAMES)enough=false;}
 if(enough && now-scanAt>=350){
 mayMap=true;
 if(nearest>SCAN_RELEASE_MM){scanActive=false;status="SCAN_CLEAR";}
 else {scanAt=now;for(uint8_t i=0;i<4;++i)scanFrames[i]=0;}
 }
 status="SLOW_SCAN_NEW_FRAMES";
 return 0.40f; // <=18 mm/s at normal 45mm/s, no blind sweep rotation
}
bool tick(uint32_t now,float dt){
 bool poseOK=(mode==Mode::MotorTest) ? false : updatePose(now,dt);
 bootAutoService(now);
 if(!active){if(!poseOK)return false;return manualTick(now);}
 if(!poseOK){halt(status);return false;}
 if(now-runAt>180000){halt("MISSION_TIMEOUT");return false;}
 if(uncertainty>20){halt("POSITION_UNCERTAINTY_STOP");return false;}
 if(!pointFree(x,y)){halt("MAP_MARGIN_STOP");return false;}
 if(hypotf(x-progressX,y-progressY)>30){lastProgress=now;progressX=x;progressY=y;}
 if(!planning && now-lastProgress>30000){halt("NO_PROGRESS");return false;}
 sampleAt=now;
 for(uint8_t i=0;i<COUNT;++i){
 if(!reading[i].fresh(now)||reading[i].streak<2){status="WAIT_FRESH_4_SENSORS";if(!blindAt)blindAt=now;if(now-blindAt>1500)halt("SENSOR_DATA_TIMEOUT");return false;}
 if(now-reading[i].sampleAt>now-sampleAt)sampleAt=reading[i].sampleAt;
 }
 blindAt=0;
 // Emergency uses unfiltered accepted range: never allow smoothing to hide proximity.
 bool mapAfterScan=false;
 float scanSpeedScale=scanScale(now,mapAfterScan);
 bool changed=false, danger=false;
 const float angles[4]={35.43f,0,0,-35.43f};
 for(uint8_t i=0;i<COUNT;++i){float d=reading[i].mm;
 if(d<85)danger=true;
 if(mapAfterScan && d<260){float a=h+angles[i]*MAP_PI/180;
 // Approximate sensor origin: front +/- lateral offsets, configurable with actual mounting.
 float sx=x+45*cosf(h)-(i<2?25:-25)*sinf(h);
 float sy=y+45*sinf(h)+(i<2?25:-25)*cosf(h);
 changed=addHit(sx+d*cosf(a),sy+d*sinf(a))||changed;
 }}
 if(!active)return false;
 if(danger){halt("OBSTACLE_TOO_CLOSE_NO_BLIND_REVERSE");return false;}
 if(changed){plan();return false;}
 if(planning){planStep();lastProgress=now;return false;}
 if(!pathReady||nextNode<0)return false;
 if(y<-BODY_LENGTH*0.5f-30 && fabsf(wrap(h+MAP_PI/2))<0.25f){done=true;halt("FINISH_REAR_CLEAR");return false;}
 
float tx=nodeX(nextNode),ty=nodeY(nextNode),dist=hypotf(tx-x,ty-y);
 bool rotating=(dist<18 && fabsf(wrap(nodeH(nextNode)-h))>0.10f);
 if(dist<18 && !rotating){
 if(nextNode==goalNode){done=true;halt("FINISH_REAR_CLEAR");return false;}
 int n=goalNode;while(previous(n)!=nextNode&&previous(n)>=0)n=previous(n);
 if(previous(n)!=nextNode){halt("PATH_CHAIN_ERROR");return false;}nextNode=n;return false;}
 if(rotating){if(!rotationFree(x,y,h,nodeH(nextNode))){plan();return false;}}
 else if(!segmentAt(x,y,tx,ty,nodeH(nextNode))){plan();return false;}
 float error=wrap((rotating?nodeH(nextNode):atan2f(ty-y,tx-x))-h);
 float omega=clip(error*1.8f,-MAX_OMEGA,MAX_OMEGA);
 if(rotating && fabsf(omega)<0.28f)omega=error>0?0.28f:-0.28f;
 float front=fminf(reading[1].mm,reading[2].mm);
 float v=fminf(MAX_SPEED,dist*1.5f)*clip((front-110)/170,0,1);
 v*=scanSpeedScale;
 if(front<SCAN_STOP_MM)v=0; // scan stopped; rotation only if footprint test passed above
 if(scanActive && !mapAfterScan && scanAt && now-scanAt<350 && front<SCAN_STOP_MM){status="STOP_SCAN";return false;}
 if(rotating || fabsf(error)>0.20f){v=0;if(!turnAt)turnAt=now;
 if(now-turnAt>12000){halt("TURN_TIMEOUT");return false;}}
 else turnAt=0;
 // Positive turnCps in inherited mixer means clockwise: negative for CCW omega.
 command=Command();command.cps=v*(COUNTS_MM_L+COUNTS_MM_R)*0.5f;
 command.turnCps=-omega*TRACK*0.5f*(COUNTS_MM_L+COUNTS_MM_R)*0.5f;
 status=v>0?(scanActive?"SLOW_SCAN":"FORWARD_SLOW"):(front<SCAN_STOP_MM && !rotating?"WAIT_OBSTACLE":"TURN_IN_PLACE");
 return true;
}
} // namespace MapAuto

  // ---------------------------------------------------------------- Vong dieu khien 10 ms
  void updateControl(uint32_t now) {
    controlDt=now-lastControl;lastControl=now;
    if(controlDt>maxControlDt)maxControlDt=controlDt;
    readEncoders();
    const float dt=fmaxf(0.001f,controlDt*0.001f);
    wheelLeft.observe(deltaLeft,dt);wheelRight.observe(deltaRight,dt);
    if(arming && (int32_t)(now-armAt)>=0)arming=false;
    if(guardExpired && !fault)latchFault(guardExpired==1?"WATCHDOG_CONTROL":"WATCHDOG_SENSOR");
    if(!allReady||fault||arming||calibration.active||balJob.active){stopOutputs();return;}
    const bool navDrive=MapAuto::tick(now,dt);
    if(paused||mode!=Mode::Follow||!navDrive){stopOutputs();return;}
    const char *lf=wheelLeft.motionFault(deltaLeft,now,controlDt,LEFT_TUNE,"STALL_L","ENC_DIR_L");
    const char *rf=wheelRight.motionFault(deltaRight,now,controlDt,RIGHT_TUNE,"STALL_R","ENC_DIR_R");
    if(lf||rf){latchFault(lf?lf:rf);return;}
    wheelTargets(command,wheelLeft,wheelRight,now,leftCommand,rightCommand,syncCps);
    const int l=wheelLeft.update(leftCommand,LEFT_TUNE,now,dt);
    const int r=wheelRight.update(rightCommand,RIGHT_TUNE,now,dt);
    applyOutputs(l,r,MapAuto::sampleAt);
  }

  // ---------------------------------------------------------------- Log
  char logBuffer[384]; // room for legacy fields + appended E2 diagnostics
  size_t logSize = 0, logPosition = 0;
  void clearLog() { logPosition = logSize = 0; }
  void pumpLog() {
    int freeBytes = Serial.availableForWrite();
    if (freeBytes > 32) freeBytes = 32;
    while (freeBytes-- > 0 && logPosition < logSize) Serial.write((uint8_t)logBuffer[logPosition++]);
    if (logPosition == logSize) logPosition = logSize = 0;
  }
  struct MmText {
    char text[16] = "NA";
    explicit MmText(float mm, bool valid) {
      if (!valid) return;
      long v = lroundf(100 * mm), mag = labs(v);
      snprintf(text, sizeof(text), "%s%ld.%02ld", v < 0 ? "-" : "", mag / 100, mag % 100);
    }
  };
  const char *zoneName(const Reading &s, uint32_t now) {
    if (!s.fresh(now)) return "NA";
    switch (zoneOf(s.mm)) {
      case Zone::Near: return "<100"; case Zone::Band: return "100-110";
      case Zone::Track: return "110-200"; default: return ">200";
    }
  }
  void makeLog(uint32_t now) {
    if(!logging||logSize||now-lastLog<500)return;
    lastLog=now;
    int n=snprintf(logBuffer,sizeof(logBuffer),
      "MAP X=%ld Y=%ld H=%ld ST=%s FAULT=%s RANGE=%ld,%ld,%ld,%ld\r\n",
      (long)lroundf(MapAuto::x),(long)lroundf(MapAuto::y),
      (long)lroundf(MapAuto::h*180/MapAuto::MAP_PI),MapAuto::status,fault?fault:"NONE",
      reading[0].fresh(now)?(long)reading[0].mm:-1L,
      reading[1].fresh(now)?(long)reading[1].mm:-1L,
      reading[2].fresh(now)?(long)reading[2].mm:-1L,
      reading[3].fresh(now)?(long)reading[3].mm:-1L);
    if(n>0&&size_t(n)<sizeof(logBuffer))logSize=n;
  }

  // ---------------------------------------------------------------- Lenh UART
  void printHelp(){Serial.println("BT9600: A teach/resume; S save; P stop. Next power ON auto. #MAP #SENSOR #GYROZERO #CALBEGIN #CALEND");}
  void enterPause() {
    MapAuto::active=false; MapAuto::planning=false;MapAuto::manualEnabled=false;MapAuto::manualCommand='0';
    paused = true; mode = Mode::Follow; motorJob.active = false; balJob.active = false; stopOutputs(); clearLog();
  }
  bool testAllowed() {
    if (fault) { Serial.println("FAULT: reset board sau khi sua nguyen nhan."); return false; }
    if (calibration.active || balJob.active) { Serial.println("CAL_OR_BAL_BUSY"); return false; }
    if (arming) { Serial.println("ARMING: cho 3 giay sau khi bat nguon."); return false; }
    return true;
  }
  void executeLine(char *line) {
    MapAuto::autoBootPending=false;
    if(!strcmp(line,"GYROZERO")){
      enterPause();MapAuto::measure.active=false;
      if(!MapAuto::imuReady){Serial.println("MPU_NOT_READY");return;}
      MapAuto::biasCount=0;MapAuto::biasSum=0;MapAuto::bias=0;MapAuto::rate=0;
      Serial.println("GYRO_ZERO_KEEP_STILL_5_SECONDS");return;
    }
    if(!strcmp(line,"CALBEGIN")){
      enterPause();
      if(arming||!MapAuto::imuReady||MapAuto::biasCount<250){Serial.println("WAIT_MPU_READY_KEEP_STILL");return;}
      MapAuto::measure=MapAuto::MeasureJob();MapAuto::measure.beganAt=millis();MapAuto::measure.active=true;
      Serial.println("CAL_STARTED MOTORS_LOCKED. Move by hand; stop completely BEFORE #CALEND.");
      lastControl=millis();return;
    }
    if(!strcmp(line,"CALREAD")||!strcmp(line,"CALEND")){
      if(!strcmp(line,"CALEND"))MapAuto::measure.active=false;
      enterPause();
      const auto &m=MapAuto::measure;
      Serial.print("CAL RAW_L=");Serial.print(m.rawL);Serial.print(" RAW_R=");Serial.print(m.rawR);
      Serial.print(" FORWARD_L=");Serial.print(m.rawL*LEFT_TUNE.encoderForwardSign);
      Serial.print(" FORWARD_R=");Serial.print(m.rawR*RIGHT_TUNE.encoderForwardSign);
      Serial.print(" GYRO_RAW_DEG=");Serial.print(m.gyroDegrees,3);
      Serial.print(" BIAS_DPS=");Serial.print(MapAuto::bias,4);
      Serial.print(" TIME_MS=");Serial.print(millis()-m.beganAt);
      Serial.print(" BAD_READS=");Serial.print(m.badReads);
      Serial.print(" GAPS_GT30MS=");Serial.print(m.gapCount);
      Serial.print(" DT_MAX_MS=");Serial.println(m.maxDtMs);return;
    }
    if(MapAuto::measure.active){Serial.println("CAL_ACTIVE: #CALEND first");return;}

    if (!strcmp(line,"AUTO") || !strcmp(line,"RUN") || !strcmp(line,"A")) {
      MapAuto::start(); Serial.println(MapAuto::status); return;
    }
    if (!strcmp(line,"MAPRESET")) {enterPause();MapAuto::reset();Serial.println(MapAuto::status);return;}
    if (!strcmp(line,"MAP")) {
      Serial.print("MAP X=");Serial.print(MapAuto::x);Serial.print(" Y=");Serial.print(MapAuto::y);
      Serial.print(" H=");Serial.print(MapAuto::h*180/MapAuto::MAP_PI);Serial.print(" STATE=");Serial.println(MapAuto::status);return;
    }
    if (!strcmp(line,"FOLLOW")) {Serial.println("USE AUTO; HAND FOLLOW DISABLED IN MAP FIRMWARE");return;}
    if (!strcmp(line,"P") || !strcmp(line,"0") || !strcmp(line,"S")) {enterPause();return;}

    if(!strcmp(line,"STOP")||!strcmp(line,"PAUSE")){enterPause();return;}
    if(!strcmp(line,"SENSOR")){enterPause();logging=true;return;}
    if(!strcmp(line,"LOG 0")){logging=false;clearLog();return;}
    if(!strcmp(line,"LOG 1")){logging=true;return;}
    if(!strcmp(line,"SHOW")){showCalibration();return;}
    if(!strcmp(line,"SAVE")){enterPause();saveCalibration();return;}
    printHelp();
  }
  void readCommands() {
    static char line[64]; static uint8_t used = 0; static bool overflow = false, maintenance=false;
    uint8_t budget = 24;
    while (budget-- && Serial.available()) {
      char c = Serial.read();
      if(!maintenance){if(c=='#'){MapAuto::autoBootPending=false;enterPause();maintenance=true;used=0;overflow=false;}else MapAuto::rawCommand(c);continue;}
      if (c == '\r' || c == '\n') {
        if (overflow) Serial.println("COMMAND_TOO_LONG");
        else if (used) { line[used] = 0; executeLine(line); }
        used = 0; overflow = false;maintenance=false;
      } else if (!overflow) {
        if (used >= sizeof(line) - 1) overflow = true;
        else line[used++] = c;
      }
    }
  }

  // ---------------------------------------------------------------- setup / loop
  void setup() {
    digitalWrite(LED_PIN, HIGH); pinMode(LED_PIN, OUTPUT);
    holdLow(STBY_PIN); holdLow(LEFT_IN1); holdLow(LEFT_IN2);
    holdLow(RIGHT_IN1); holdLow(RIGHT_IN2);
    pinMode(LEFT_PWM, OUTPUT); pinMode(RIGHT_PWM, OUTPUT);
    analogWriteResolution(8);
  #if ENABLE_C4_PWM_20KHZ
    analogWriteFrequency(20000); // STM32duino >=1.5; shared TIM4 channels PB8/PB9
  #endif
    stopOutputs();
    for (uint8_t i = 0; i < COUNT; ++i) holdLow(XSHUT[i]);
    Serial.setRx(PA10); Serial.setTx(PA9); Serial.begin(MapAuto::BLUETOOTH_BAUD, SERIAL_8N1);
    initEncoders(); delay(200);
    if (!encoderConfigOK(TIM2) || !encoderConfigOK(TIM3) ||
        (AFIO->MAPR & (AFIO_MAPR_TIM2_REMAP | AFIO_MAPR_TIM3_REMAP | AFIO_MAPR_TIM4_REMAP))) latchFault("TIMER_CONFIG");
    Serial.println("MAP_AUTO_LEAN_F103C8 BT9600");
    loadCalibration();
    allReady = initSensors();
    MapAuto::init();
    MapAuto::loadRoute();
    paused=true;
    if (!allReady) latchFault("SENSOR_BOOT_FAIL");
    if (!tickSeen) latchFault("SYSTICK_GUARD_INACTIVE");
    if (!LEFT_TUNE.encoderForwardSign || !RIGHT_TUNE.encoderForwardSign)
      Serial.println("CANH BAO: encoderForwardSign chua do (MOTOR L 80 / MOTOR R 80). Kiem tra chieu encoder dang TAT.");
    printHelp();
    armAt = millis() + ARM_DELAY_MS;
    lastControl = millis();
    Serial.println("POWER SWITCH MODE: START A=TEACH; SQUARE S=SAVE. POWER OFF, REPLACE AT START, POWER ON=AUTO AFTER CALIBRATION+5s. #MAP newline=STATUS. KEEP POWER ON.");
  }

  void serviceControl() {
    const uint32_t now = millis();
    if (now - lastControl < CONTROL_MS) return;
    updateControl(now);
    makeLog(now);
  }
  void loop() {
    readCommands(); serviceControl();
    static uint8_t slot = 0;
    static uint32_t lastPoll = 0;
    const uint32_t now = millis();
    if (allReady && now - lastPoll >= 2) {
      lastPoll = now;
      // Front twice as often as sides. Keep probe gaps bounded at 50 kHz I2C.
      static const uint8_t order[6] = {FRONT_A, FRONT_B, SIDE_L, FRONT_A, FRONT_B, SIDE_R};
      const uint8_t i = order[slot];
      const uint32_t started = millis();
      pollSensor(i);
      const uint32_t elapsed = millis() - started;
      if (elapsed > maxPollMs[i]) maxPollMs[i] = elapsed;
      slot = (uint8_t)((slot + 1U) % 6U);
    }
    serviceControl();
    digitalWrite(LED_PIN, !fault && MapAuto::routeSaved && !MapAuto::active && !MapAuto::autoBootPending ? ((now/700)%2 ? LOW:HIGH) : fault ? ((now / 150) % 2 ? LOW : HIGH) :
      (target.index >= 0 && target.state != State::Far && !paused ? LOW : HIGH));
    pumpLog();
  }
  #endif




