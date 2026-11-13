



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
  bool ready[COUNT] = {}, allReady = false, paused = false, logging = true, arming = true;
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

  // ---------------------------------------------------------------- Vong dieu khien 10 ms
  void updateControl(uint32_t now) {
    controlDt = now - lastControl; lastControl = now;
    if (controlDt > maxControlDt) maxControlDt = controlDt;
    readEncoders(); encWindowLeft += deltaLeft; encWindowRight += deltaRight;
    const float dt = fmaxf(0.001f, float(controlDt) * 0.001f);
    wheelLeft.observe(deltaLeft, dt); wheelRight.observe(deltaRight, dt);
  #if ENABLE_SYNC_REQUEST_DIFF
    syncErrorCps = 0; syncRequestActive = false;
    wheelLeft.observeSync(deltaLeft, dt, now); wheelRight.observeSync(deltaRight, dt, now);
  #endif
    target = locate(reading, now);
    sideTurnLimit.apply(target, now);
    alphaBetaNow = now;
  #if ENABLE_FRONT_HANDOFF
    if (frontHandoff.prepare(target, reading, now)) alphaBetaDistance.reset();
  #endif
  #if ENABLE_A3_ALPHA_BETA
    if (target.hasFront && target.index >= FRONT_A && target.index <= FRONT_B) {
      // Only consume each new front frame ONCE. Bounded age includes I2C/probe latency.
      const Reading &src=reading[target.index];
      float measured=src.filtered;
      const uint8_t other=target.index==FRONT_A ? FRONT_B : FRONT_A;
      const Reading &peer=reading[other];
      if (peer.fresh(now) && peer.streak >= 2 &&
          fabsf(peer.filtered-src.filtered) <= PAIR_TOL_MM &&
          (uint32_t)(now-peer.receivedAt) <= PAIR_WINDOW_MS) {
        measured=0.5f*(src.filtered+peer.filtered); // only fuse when same target is plausible
      }
      alphaBetaDistance.accept(measured, src.receivedAt,
                              (uint32_t)(src.receivedAt + 1U));
      target.filtered=alphaBetaDistance.x;
      target.rate=alphaBetaDistance.v;
    } else if (!target.hasFront) alphaBetaDistance.reset();
  #endif
  #if ENABLE_FRONT_HANDOFF
    frontHandoff.apply(target, now);
  #endif
    predictedGapMm = (ENABLE_A3_ALPHA_BETA && alphaBetaDistance.ready && target.hasFront) ?
      alphaBetaDistance.predict(now, reading[target.index].sampleAt) : target.filtered;
    estimatedGapRate = target.hasFront ? target.rate : 0;
  #if ENABLE_D1_STEER_DYNAMIC
    const bool directionValid = target.state != State::Lost && target.state != State::Far &&
      target.state != State::Confirm && target.state != State::SideTimeout;
    float desiredAngle=directionValid ? target.steer*SIDE_LEFT_DEG : 0;
    if (directionValid) steerLostAt=0;
    if (!directionValid) {
      if (!steerLostAt) steerLostAt=now;
      desiredAngle=0;  // decay but NO movement when target data is invalid
    }
    if (steerEnabled && fabsf(desiredAngle) <= STEER_OFF_DEG) steerEnabled=false;
    if (!steerEnabled && fabsf(desiredAngle) >= STEER_ON_DEG) steerEnabled=true;
    const float goal=steerEnabled ? desiredAngle : 0;
    const float limitedAngle=towards(steerSmooth*SIDE_LEFT_DEG,goal,STEER_SLEW_DEG_S*dt);
    const float tau = directionValid ? 0.035f : STEER_LOST_GRACE_MS*0.001f;
    steerSmooth+=dt/(tau+dt)*(limitedAngle/SIDE_LEFT_DEG-steerSmooth);
    target.steer=directionValid ? steerSmooth : 0; // safety: no blind turn
  #else
    if (target.state == State::Lost || target.state == State::Far ||
        target.state == State::Confirm || target.state == State::SideTimeout) steerSmooth = 0;
  #if ENABLE_FAST_STEER_FILTER
    else steerSmooth = fastSteerStep(steerSmooth, target.steer, dt);
  #else
    else steerSmooth += dt / (STEER_TAU_S + dt) * (target.steer - steerSmooth);
  #endif
    target.steer = steerSmooth;
  #endif
    command = decide(target);
  #if !ENABLE_D2_DIRECT_STEER_MIX
    if (fabsf(command.turnCps) > 0.0f) command.cps *= 0.65f;
  #endif
    rawBodyCps=command.cps;
  #if ENABLE_B3_SPEED_SLEW
    if (command.cps==0 || !target.hasFront) bodySlew.reset();
    else command.cps=bodySlew.update(command.cps,dt);
  #endif
    slewBodyCps=command.cps;
  #if ENABLE_D4_GYRO_YAW
    // This update is optional and can slow the loop. If gyro fails, no correction.
    gyroUpdate();
    if (gyroOK && command.turnCps != 0) {
      const float yawRef=command.turnCps/(COUNTS_PER_MM_EST*WHEEL_TRACK_MM*0.5f)*57.2958f;
      command.turnCps=clip(command.turnCps+1.0f*(yawRef-gyroZDegS),-TURN_CPS_MAX,TURN_CPS_MAX);
    }
  #endif
    leftCommand = rightCommand = Command(); syncCps = 0;
    if (arming && (int32_t)(now - armAt) >= 0) arming = false;
    if (guardExpired && !fault) latchFault(guardExpired == 1 ? "WATCHDOG_CONTROL" : "WATCHDOG_SENSOR");
    if (fault) tryRecoverFault(now);
    if (!allReady || fault || calibration.active || arming) { stopOutputs(); return; }
    if (mode == Mode::MotorTest) { runMotorJob(now); return; }
    if (paused || mode != Mode::Follow) { stopOutputs(); return; }
    const char *lf = wheelLeft.motionFault(deltaLeft, now, controlDt, LEFT_TUNE, "STALL_L", "ENC_DIR_L");
    const char *rf = wheelRight.motionFault(deltaRight, now, controlDt, RIGHT_TUNE, "STALL_R", "ENC_DIR_R");
    if (lf || rf) { latchFault(lf ? lf : rf); return; }
    wheelTargets(command, wheelLeft, wheelRight, now, leftCommand, rightCommand, syncCps);
    const int l = wheelLeft.update(leftCommand, LEFT_TUNE, now, dt);
    const int r = wheelRight.update(rightCommand, RIGHT_TUNE, now, dt);
    applyOutputs(l, r, target.sampleAt,
                ENABLE_C5_COAST_OVERSPEED && wheelLeft.coastRequested,
                ENABLE_C5_COAST_OVERSPEED && wheelRight.coastRequested);
  }

  // ---------------------------------------------------------------- Log
  char logBuffer[1536]; // room for legacy fields + appended E2 diagnostics
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
    if (!logging || logSize || calibration.active || balJob.active || mode == Mode::MotorTest || now - lastLog < LOG_PERIOD_MS) return;
    lastLog = now;
    const Reading &a = reading[FRONT_A], &b = reading[FRONT_B];
    const Reading &sl = reading[SIDE_L], &sr = reading[SIDE_R];
    int n;
    if (mode == Mode::SensorTest) {
      const MmText m8(sl.mm, sl.fresh(now)), ma(a.mm, a.fresh(now));
      const MmText mb(b.mm, b.fresh(now)), m11(sr.mm, sr.fresh(now));
      // Full sensor diagnostics for all four, even when the car cannot see a target.
      n = snprintf(logBuffer, sizeof(logBuffer),
        "SENSOR_TEST "
        "J8_RAW=%d J8_STATUS=%u J8_SIGNAL=%u J8_AGE=%ld J8_VALID=%u J8_MM=%s "
        "J9_RAW=%d J9_STATUS=%u J9_SIGNAL=%u J9_AGE=%ld J9_VALID=%u J9_MM=%s "
        "J10_RAW=%d J10_STATUS=%u J10_SIGNAL=%u J10_AGE=%ld J10_VALID=%u J10_MM=%s "
        "J11_RAW=%d J11_STATUS=%u J11_SIGNAL=%u J11_AGE=%ld J11_VALID=%u J11_MM=%s "
        "BLOCK=%d,%d LIB16=%d,%d I2C_ERROR=%lu,%lu,%lu,%lu DT_MAX=%lu BLOCK13=%d,%d BYTES=%d,%d READ_MODE=%s\r\n",
        sl.seenFrame ? int(sl.raw) : -1, (unsigned)sl.deviceStatus, (unsigned)sl.signalQ7,
        sl.seenFrame ? (long)(now - sl.frameAt) : -1L, (unsigned)sl.fresh(now), m8.text,
        a.seenFrame ? int(a.raw) : -1, (unsigned)a.deviceStatus, (unsigned)a.signalQ7,
        a.seenFrame ? (long)(now - a.frameAt) : -1L, (unsigned)a.fresh(now), ma.text,
        b.seenFrame ? int(b.raw) : -1, (unsigned)b.deviceStatus, (unsigned)b.signalQ7,
        b.seenFrame ? (long)(now - b.frameAt) : -1L, (unsigned)b.fresh(now), mb.text,
        sr.seenFrame ? int(sr.raw) : -1, (unsigned)sr.deviceStatus, (unsigned)sr.signalQ7,
        sr.seenFrame ? (long)(now - sr.frameAt) : -1L, (unsigned)sr.fresh(now), m11.text,
        a.seenFrame ? int(a.rawBlock) : -1, b.seenFrame ? int(b.rawBlock) : -1,
        a.seenFrame ? int(a.rawLib) : -1, b.seenFrame ? int(b.rawLib) : -1,
        (unsigned long)i2cErrors[SIDE_L], (unsigned long)i2cErrors[FRONT_A],
        (unsigned long)i2cErrors[FRONT_B], (unsigned long)i2cErrors[SIDE_R],
        (unsigned long)maxControlDt,
        a.seenFrame ? int(a.rawBlock13) : -1, b.seenFrame ? int(b.rawBlock13) : -1,
        a.seenFrame ? int(a.rawBytes) : -1, b.seenFrame ? int(b.rawBytes) : -1,
        ENABLE_A2_BLOCK13 ? "BLOCK13" : "BYTES");
    } else {
      const bool has = target.index >= 0;
      const char *state = fault ? "FAULT" : paused ? "PAUSE" : arming ? "ARMING" : stateName(target.state);
      const char *reason = fault ? fault : guardExpired ? "WATCHDOG_PENDING" :
        (!paused && !arming && !has && allReady ? "NO_FRESH_FRONT" : "NONE");
      const MmText gap(target.gap, has), filt(target.filtered, has);
      n = snprintf(logBuffer, sizeof(logBuffer),
        "T_MS=%lu J8=%d J9=%d J10=%d J11=%d GAP_MM=%s FILTER_MM=%s STATE=%s DIR=%d MASK=%u TURN_CPS=%d DIST_RATE_MM_S=%d "
        "REF_CPS_L=%d REF_CPS_R=%d ACTUAL_CPS_L=%d ACTUAL_CPS_R=%d "
        "REQUEST_PWM_L=%d REQUEST_PWM_R=%d ACTUAL_PWM_L=%d ACTUAL_PWM_R=%d STBY=%u "
        "SENSOR_AGE_MS=%ld CONTROL_DT_MS=%lu FAULT=%s SRC=%s ENC_L=%ld ENC_R=%ld PAUSE_REV=%u "
        "OK=%u,%u STREAK=%u,%u STATUS=%u,%u SIG_Q7=%u,%u I2C_ERROR=%lu,%lu "
        "SIDE_OK=%u,%u SIDE_VALID=%u,%u SIDE_STATUS=%u,%u SIDE_STREAK=%u,%u SIDE_AGE=%ld,%ld SIDE_LIMIT=%u BAL=%u SYNC_CPS=%d DT_MAX=%lu "
  #if ENABLE_E2_EXTENDED_LOG
        "PRED_MM=%ld EST_V_MM_S=%ld SET_CPS_RAW=%ld SET_CPS_SLEW=%ld "
        "AGE9=%ld AGE10=%ld POLL_MAX_MS=%lu,%lu,%lu,%lu RECOVERIES=%lu RECV_WAIT=%u "
        "LOG_PERIOD_MS=%lu I2C_ALL=%lu,%lu,%lu,%lu"
  #endif
        "\r\n",
        (unsigned long)now, sl.seenFrame ? int(sl.raw) : -1, a.seenFrame ? int(a.raw) : -1,
        b.seenFrame ? int(b.raw) : -1, sr.seenFrame ? int(sr.raw) : -1,
        gap.text, filt.text, state, int(lroundf(target.steer * 100.0f)),
        (unsigned)target.mask, int(lroundf(command.turnCps)), int(lroundf(target.rate)),
        int(lroundf(leftCommand.cps)), int(lroundf(rightCommand.cps)),
        int(lroundf(wheelLeft.cps)), int(lroundf(wheelRight.cps)),
        wheelLeft.request, wheelRight.request, pwmReadback(true), pwmReadback(false),
        (unsigned)(GPIOB->ODR & 1U), has ? (long)(now - target.sampleAt) : -1L, (unsigned long)controlDt,
        reason, has ? NAME[target.index] : "NONE", (long)encWindowLeft, (long)encWindowRight,
        (unsigned)(wheelLeft.interlock || wheelRight.interlock),
        (unsigned)a.fresh(now), (unsigned)b.fresh(now), (unsigned)a.streak, (unsigned)b.streak,
        (unsigned)a.deviceStatus, (unsigned)b.deviceStatus, (unsigned)a.signalQ7, (unsigned)b.signalQ7,
        (unsigned long)i2cErrors[FRONT_A], (unsigned long)i2cErrors[FRONT_B],
        (unsigned)sl.fresh(now), (unsigned)sr.fresh(now),
        (unsigned)sl.valid, (unsigned)sr.valid,
        (unsigned)sl.deviceStatus, (unsigned)sr.deviceStatus,
        (unsigned)sl.streak, (unsigned)sr.streak,
        sl.fresh(now) ? (long)(now - sl.sampleAt) : -1L,
        sr.fresh(now) ? (long)(now - sr.sampleAt) : -1L,
        (unsigned)sideTurnLimit.blocked, (unsigned)frontBalMask,
        int(lroundf(syncCps)), (unsigned long)maxControlDt
  #if ENABLE_E2_EXTENDED_LOG
        ,(long)lroundf(predictedGapMm), (long)lroundf(estimatedGapRate),
        (long)lroundf(rawBodyCps), (long)lroundf(slewBodyCps),
        a.fresh(now) ? (long)(now-a.sampleAt) : -1L,
        b.fresh(now) ? (long)(now-b.sampleAt) : -1L,
        (unsigned long)maxPollMs[0], (unsigned long)maxPollMs[1],
        (unsigned long)maxPollMs[2], (unsigned long)maxPollMs[3],
        (unsigned long)recoveryCount, (unsigned)(fault && ENABLE_E1_RECOVERABLE_FAULTS),
        (unsigned long)LOG_PERIOD_MS,
        (unsigned long)i2cErrors[0], (unsigned long)i2cErrors[1],
        (unsigned long)i2cErrors[2], (unsigned long)i2cErrors[3]
  #endif
        );
    }
  #if ENABLE_SYNC_REQUEST_DIFF || ENABLE_FAST_STEER_FILTER || ENABLE_JUMP_PENDING || ENABLE_FRONT_HANDOFF
    if (n >= 2 && size_t(n) < sizeof(logBuffer) && mode != Mode::SensorTest) {
      // Reuse the existing nonblocking pump; no extra Serial.print in the loop.
      const size_t tail = size_t(n - 2); // replace the old CRLF, keeping every old field
      const int extra = snprintf(logBuffer + tail, sizeof(logBuffer) - tail,
          " V21_SYNC=%u SYNC_ERR_CPS=%ld STEER_TAU=%lu JUMP_PENDING=%u FRONT_HANDOFF=%u\r\n",
  #if ENABLE_SYNC_REQUEST_DIFF
          (unsigned)syncRequestActive, (long)lroundf(syncErrorCps),
  #else
          0U, 0L,
  #endif
          // D1 uses its own dynamic tau; FAST changes only the V2 one-pole path.
          (unsigned long)lroundf(1000.0f * (ENABLE_D1_STEER_DYNAMIC ?
            ((target.state == State::Lost || target.state == State::Far ||
              target.state == State::Confirm || target.state == State::SideTimeout) ?
              STEER_LOST_GRACE_MS * 0.001f : 0.035f) : STEER_TAU_S)),
  #if ENABLE_JUMP_PENDING
          (unsigned)jumpPendingMask(reading, now),
  #else
          0U,
  #endif
  #if ENABLE_FRONT_HANDOFF
          (unsigned)frontHandoff.active
  #else
          0U
  #endif
          );
      if (extra > 0 && size_t(extra) < sizeof(logBuffer) - tail) n = int(tail) + extra;
      else { logBuffer[tail] = '\r'; logBuffer[tail + 1] = '\n'; logBuffer[tail + 2] = 0; }
    }
  #endif
    if (n > 0 && size_t(n) < sizeof(logBuffer)) logSize = size_t(n);
    encWindowLeft = encWindowRight = 0; maxControlDt = 0;
    for (uint8_t i=0; i<COUNT; ++i) maxPollMs[i]=0;
  }

  // ---------------------------------------------------------------- Lenh UART
  void printHelp() {
    Serial.println("4 VL53: J8 trai 35deg, J9 truoc-trai, J10 truoc-phai, J11 phai 43deg.");
    Serial.println("Front: <100 LUI | 100..110 GIU/RE | >110..200 TIEN | >200 DUNG.");
    Serial.println("Only side sees hand 40..200mm: RE TAI CHO max 1.5s, new confirmed acquisition to rearm.");
    Serial.println("BAL 100 | BAL 150: center flat board; SAVE persists bias. SHOW displays it.");
    Serial.println("STOP | RUN | SENSOR (chi doc cam bien, khoa dong co)");
    Serial.println("MOTOR L|R|B <pwm -160..160> [ms 200..3000]  vd: MOTOR L 80  (KE BANH LEN truoc)");
    Serial.println("SWEEP F|R  (quet PWM 40..130 ca hai banh, KE BANH LEN truoc)");
    Serial.println("CAL J8 100 | CAL J9 100 | CAL J10 100 | CAL J11 100 | BAL 100 | BAL 150 | SAVE | SHOW | DEFAULT | LOG 0 | LOG 1 | HELP");
    Serial.println("A/B flags: edit #define ENABLE_* near top, rebuild, compare SENSOR age/POLL_MAX and PWM.");
  }
  void enterPause() {
    paused = true; mode = Mode::Follow; motorJob.active = false; balJob.active = false; stopOutputs(); clearLog();
  }
  bool testAllowed() {
    if (fault) { Serial.println("FAULT: reset board sau khi sua nguyen nhan."); return false; }
    if (calibration.active || balJob.active) { Serial.println("CAL_OR_BAL_BUSY"); return false; }
    if (arming) { Serial.println("ARMING: cho 3 giay sau khi bat nguon."); return false; }
    return true;
  }
  void executeLine(char *line) {
    if (!strcmp(line, "STOP") || !strcmp(line, "PAUSE")) {
      calibration.active = false; balJob.active = false; enterPause(); Serial.println("PAUSED");
    } else if (!strcmp(line, "RUN") || !strcmp(line, "FOLLOW")) {
      if (fault || !allReady) { Serial.println("FAULT: reset board sau khi sua nguyen nhan."); return; }
      if (calibration.active || balJob.active) { Serial.println("CAL_OR_BAL_BUSY"); return; }
      enterPause(); resetMeasurements(); paused = false;
      Serial.println("RUN: cho du lieu J8..J11 moi va duoc xac nhan.");
    } else if (!strncmp(line, "BAL ", 4)) {
      if (calibration.active || balJob.active) { Serial.println("CAL_OR_BAL_BUSY"); return; }
      int mm = 0; char extra = 0;
      if (sscanf(line, "BAL %d %c", &mm, &extra) != 1 || (mm != 100 && mm != 150)) {
        Serial.println("USE: BAL 100 or BAL 150, center flat card, motors off"); return;
      }
      if (!ready[FRONT_A] || !ready[FRONT_B]) { Serial.println("FRONT_SENSORS_NOT_READY"); return; }
      enterPause(); resetMeasurements();
      balJob = BalJob(); balJob.active = true; balJob.index = mm == 100 ? 0 : 1;
      balJob.beganAt = millis();
      Serial.print("BAL_START "); Serial.print(mm);
      Serial.println("mm: place a flat board centered, remain stationary until BAL_OK");
    } else if (!strcmp(line, "SENSOR")) {
      if (calibration.active || balJob.active) { Serial.println("CAL_OR_BAL_BUSY"); return; }
      enterPause(); resetMeasurements(); mode = Mode::SensorTest; logging = true;
      Serial.println("SENSOR_TEST: dong co bi khoa. RAW phai bang khoang cach that (mm) + sai so nho. STOP de thoat.");
    } else if (!strncmp(line, "MOTOR ", 6)) {
      char side[2] = {}; int pwm = 0, ms = 1500;
      const int got = sscanf(line, "MOTOR %1s %d %d", side, &pwm, &ms);
      const bool sideOK = side[0] == 'L' || side[0] == 'R' || side[0] == 'B';
      if (got < 2 || !sideOK || pwm == 0 || abs(pwm) > MOTOR_TEST_MAX_PWM || ms < 200 || ms > 3000) {
        Serial.println("USE: MOTOR L|R|B <pwm -160..160, khac 0> [ms 200..3000]"); return;
      }
      if (!testAllowed()) return;
      enterPause(); motorJob = MotorJob();
      motorJob.pwmL = side[0] != 'R' ? pwm : 0; motorJob.pwmR = side[0] != 'L' ? pwm : 0;
      motorJob.durationMs = (uint32_t)ms; motorJob.active = true;
      beginMotorStep(millis()); mode = Mode::MotorTest;
      Serial.println("MOTOR_START (STOP de dung ngay)");
    } else if (!strcmp(line, "SWEEP F") || !strcmp(line, "SWEEP R")) {
      if (!testAllowed()) return;
      enterPause(); motorJob = MotorJob();
      motorJob.sweep = true; motorJob.dir = line[6] == 'F' ? 1 : -1;
      motorJob.durationMs = SWEEP_STEP_MS; motorJob.active = true;
      beginMotorStep(millis()); mode = Mode::MotorTest;
      Serial.println("SWEEP_START (STOP de dung ngay)");
    } else if (!strcmp(line, "LOG 0")) { logging = false; clearLog(); }
    else if (!strcmp(line, "LOG 1")) logging = true;
    else if (!strcmp(line, "SAVE")) {
      if (calibration.active || balJob.active) { Serial.println("CAL_OR_BAL_BUSY"); return; }
      enterPause(); saveCalibration();
    } else if (!strcmp(line, "SHOW") || !strcmp(line, "HELP")) {
      enterPause();
      if (!strcmp(line, "SHOW")) showCalibration(); else printHelp();
    } else if (!strcmp(line, "DEFAULT")) {
      calibration.active = false; balJob.active = false; enterPause();
      for (uint8_t i = 0; i < COUNT; ++i) offsets[i] = DEFAULT_OFFSET_MM[i];
      frontBalMask = 0; frontPairBiasMm[0] = frontPairBiasMm[1] = 0.0f;
      resetMeasurements(); Serial.println("DEFAULT trong RAM. SAVE de luu. RUN de chay.");
    } else if (!strncmp(line, "CAL ", 4)) {
      if (calibration.active || balJob.active) { Serial.println("CAL_OR_BAL_BUSY"); return; }
      char name[8] = {}; int mm = 0; char extra = 0;
      if (sscanf(line, "CAL %7s %d %c", name, &mm, &extra) != 2 || mm < 70 || mm > 300) {
        Serial.println("USE: CAL J9 100 (70..300 mm)"); return;
      }
      int index = -1;
      for (uint8_t i = 0; i < COUNT; ++i) if (!strcmp(name, NAME[i])) index = i;
      if (index < 0 || !ready[index]) { Serial.println("SENSOR_NOT_READY"); return; }
      enterPause(); resetMeasurements();
      calibration = CalibrationJob();
      calibration.active = true; calibration.index = (uint8_t)index;
      calibration.knownMm = mm; calibration.beganAt = millis();
      Serial.print("CAL_START "); Serial.println(name);
    } else Serial.println("UNKNOWN_COMMAND. HELP dung xe va liet ke lenh.");
  }
  void readCommands() {
    static char line[64]; static uint8_t used = 0; static bool overflow = false;
    uint8_t budget = 24;
    while (budget-- && Serial.available()) {
      char c = Serial.read();
      if (c == '\r' || c == '\n') {
        if (overflow) Serial.println("COMMAND_TOO_LONG");
        else if (used) { line[used] = 0; executeLine(line); }
        used = 0; overflow = false;
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
    Serial.setRx(PA10); Serial.setTx(PA9); Serial.begin(115200, SERIAL_8N1);
    initEncoders(); delay(200);
    if (!encoderConfigOK(TIM2) || !encoderConfigOK(TIM3) ||
        (AFIO->MAPR & (AFIO_MAPR_TIM2_REMAP | AFIO_MAPR_TIM3_REMAP | AFIO_MAPR_TIM4_REMAP))) latchFault("TIMER_CONFIG");
    Serial.println("\nHAND_FOLLOW_4VL53 GAP=100..110mm TRACK<=200mm CONTROL=10ms RANGE_SELECTABLE");
    Serial.println("J8=-35.43deg J11=+35.43deg SIDE_SETBACK=15mm TRACK=120mm caster=yes");
    Serial.println("COUNTS_PER_MM_EST=10 NOT CALIBRATED. Front PD/PI retained.");
    Serial.print("BUDGET_US="); Serial.print(BUDGET_US);
    Serial.print(" I2C_HZ="); Serial.print(I2C_HZ);
    Serial.print(" FRESH_MS="); Serial.print(FRESH_MS);
    Serial.print(" GUARD_SAMPLE_MS="); Serial.print(GUARD_SAMPLE_MS);
    Serial.print(" WATCHDOG_MS="); Serial.println(WATCHDOG_MS);
    Serial.print("FEATURES A1,A2,A3,B,B3,B4,C1,C2,C3,C4,C5,D1,D2,D4,E1,E2=");
    Serial.print(ENABLE_A1_FAST_SENSOR_BUS); Serial.print(','); Serial.print(ENABLE_A2_BLOCK13); Serial.print(',');
    Serial.print(ENABLE_A3_ALPHA_BETA); Serial.print(','); Serial.print(ENABLE_B_CONTINUOUS_DISTANCE); Serial.print(',');
    Serial.print(ENABLE_B3_SPEED_SLEW); Serial.print(','); Serial.print(ENABLE_B4_FAST_ZONE_CONFIRM); Serial.print(',');
    Serial.print(ENABLE_C1_PWM_TABLE); Serial.print(','); Serial.print(ENABLE_C2_LOW_SPEED_COMP); Serial.print(',');
    Serial.print(ENABLE_C3_WHEEL_WINDOW); Serial.print(','); Serial.print(ENABLE_C4_PWM_20KHZ); Serial.print(',');
    Serial.print(ENABLE_C5_COAST_OVERSPEED); Serial.print(','); Serial.print(ENABLE_D1_STEER_DYNAMIC); Serial.print(',');
    Serial.print(ENABLE_D2_DIRECT_STEER_MIX); Serial.print(','); Serial.print(ENABLE_D4_GYRO_YAW); Serial.print(',');
    Serial.print(ENABLE_E1_RECOVERABLE_FAULTS); Serial.print(','); Serial.println(ENABLE_E2_EXTENDED_LOG);
    Serial.print("V2.1 SYNC,FAST_STEER,JUMP,HANDOFF=");
    Serial.print(ENABLE_SYNC_REQUEST_DIFF); Serial.print(',');
    Serial.print(ENABLE_FAST_STEER_FILTER); Serial.print(',');
    Serial.print(ENABLE_JUMP_PENDING); Serial.print(','); Serial.println(ENABLE_FRONT_HANDOFF);
    loadCalibration(); showCalibration();
    allReady = initSensors();
    gyroInit();
    if (!allReady) latchFault("SENSOR_BOOT_FAIL");
    if (!tickSeen) latchFault("SYSTICK_GUARD_INACTIVE");
    if (!LEFT_TUNE.encoderForwardSign || !RIGHT_TUNE.encoderForwardSign)
      Serial.println("CANH BAO: encoderForwardSign chua do (MOTOR L 80 / MOTOR R 80). Kiem tra chieu encoder dang TAT.");
    printHelp();
    armAt = millis() + ARM_DELAY_MS;
    lastControl = millis();
    Serial.println("ARMING 3s. Xe tu vao che do bam; gui STOP hoac SENSOR de kiem tra truoc.");
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
    serviceControl(); updateCalibration(); updateBalCalibration();
    digitalWrite(LED_PIN, fault ? ((now / 150) % 2 ? LOW : HIGH) :
      (target.index >= 0 && target.state != State::Far && !paused ? LOW : HIGH));
    pumpLog();
  }
  #endif


Phiên bản V2.1 được phát triển trên nền chương trình V2, giữ nguyên cấu hình phần cứng và bổ sung các chức năng nhằm hỗ trợ giảm trễ, giảm giật khi bám mục tiêu.

| Nội dung cải tiến | Phần đã thực hiện |
|---|---|
| **Đồng bộ hai bánh khi rẽ** | Bổ sung thuật toán so sánh chênh lệch tốc độ thực tế với chênh lệch tốc độ đặt, có xét chiều encoder; hỗ trợ rẽ và xoay tại chỗ. |
| **Điều chỉnh bộ lọc hướng** | Bổ sung tùy chọn giảm hằng số thời gian lọc từ 60 ms xuống 40 ms, giữ nguyên giới hạn tốc độ xoay. |
| **Xử lý mẫu khoảng cách nhảy bất thường** | Bổ sung cơ chế chờ xác nhận mẫu nhảy từ 35 mm, giữ dữ liệu đã xác nhận trong thời hạn cho phép. |
| **Làm mượt chuyển nguồn J9/J10** | Bổ sung xử lý chuyển tiếp giá trị lọc khi đổi cảm biến trước, hạn chế biến động do chuyển nguồn trong cùng vùng khoảng cách. |

Bốn cải tiến đã được tích hợp vào mã nguồn và có cờ bật/tắt riêng. **Mặc định cả bốn đều tắt** để duy trì cấu hình V2 đang hoạt động ổn định.

Các bộ lọc khoảng cách, Alpha–Beta, tốc độ encoder và hướng xoay được giữ lại. Đồng thời bảo toàn điều khiển khoảng cách PD, điều khiển tốc độ PI, watchdog, bảo vệ động cơ, lệnh UART và dữ liệu hiệu chuẩn EEPROM.

Mã nguồn hoàn chỉnh đã **biên dịch thành công cho STM32F103C8**, vượt qua kiểm tra C++11 và **7.800 phép kiểm tra trên 16 tổ hợp cờ**. Khi tắt các cải tiến mới, kết quả điều khiển trong các bài kiểm tra đối chiếu trùng với V2. Đã cung cấp thêm bản diff và hướng dẫn thử A/B.

**Hiệu quả giảm giật, giảm trễ và độ chính xác giữ khoảng cách vẫn cần đo trên xe thực tế.**


## CẢI TIẾN
Bạn là kỹ sư lập trình nhúng STM32 và điều khiển robot. Hãy nâng cấp file `main.cpp` V2.1 đính kèm lên V2.2, tập trung giảm giật khi bắt đầu rẽ và hạn chế bật–tắt xoay khi tay đứng yên.

**1. Nguyên tắc bắt buộc**

Lấy V2.1 làm nền duy nhất, chỉnh sửa có kiểm soát, không viết lại toàn bộ chương trình.

Giữ nguyên:
- Chân phần cứng, encoder TIM2/TIM3 và PWM TIM4.
- `LEFT_REVERSE=true`, `RIGHT_REVERSE=false`.
- Cách đọc VL53L0X từng byte, lịch polling và kiểm tra tuổi mẫu.
- I²C 100 kHz, timing budget 20 ms, chu kỳ điều khiển 10 ms.
- Điều khiển khoảng cách: dưới 100 mm lùi; 100–110 mm giữ; trên 110–200 mm tiến.
- Khi chỉ cảm biến bên thấy mục tiêu hợp lệ: chỉ xoay tại chỗ.
- `MAX_CPS=1600`, `TURN_CPS_MAX=600`, giới hạn PWM và các hệ số điều khiển hiện có.
- Watchdog, STALL, ENC_DIR, khóa đổi chiều, giới hạn xoay bên.
- Định dạng EEPROM, các lệnh UART và tên trường log.
- Trạng thái các cờ cải tiến V2.1 trong file được cung cấp.

**2. Cải tiến thứ nhất: làm mượt tốc độ xoay**

Hiện tại, dù đã lọc hướng, lệnh xoay có thể chuyển từ 0 lên 300 xung/s khi vượt vùng chết do `TURN_CPS_MIN`.

Bổ sung:

`#define ENABLE_TURN_SLEW 0`

Yêu cầu:
- Giới hạn tốc độ tăng/giảm của `command.turnCps`, trước khi trộn thành tốc độ hai bánh.
- Có hằng số tăng tốc và giảm tốc riêng, dễ điều chỉnh.
- Khi đổi hướng xoay, chuyển qua 0 và giữ cơ chế khóa đổi chiều động cơ hiện có.
- Kiểm tra tương tác với ngưỡng tốc độ bánh tối thiểu, tránh tạo chu kỳ dừng–khởi động hoặc tăng nguy cơ STALL.
- Giữ cách tính tốc độ tịnh tiến của V2.1; không tự thêm mức giảm tốc tiến.
- Không làm chậm quá mức khả năng bắt tay sang trái/phải.

**STOP, FAULT, mất mục tiêu, dữ liệu quá hạn hoặc hết giới hạn xoay bên phải ngắt chuyển động theo cơ chế an toàn hiện có và reset bộ slew. Không kéo dài chuyển động để làm mượt.**

Phân biệt mất mục tiêu hoàn toàn với trường hợp chỉ cảm biến bên còn thấy mục tiêu hợp lệ; trường hợp thứ hai vẫn được xoay tại chỗ.

**3. Cải tiến thứ hai: chống bật–tắt xoay quanh vùng chết**

Bổ sung:

`#define ENABLE_STEER_HYSTERESIS 0`

Yêu cầu:
- Có ngưỡng bắt đầu xoay và ngưỡng ngừng xoay khác nhau; ngưỡng ngừng thấp hơn ngưỡng bắt đầu.
- Giữ độ nhạy bắt đầu xoay gần cấu hình hiện tại.
- Hạn chế bật–tắt lệnh khi tín hiệu dao động sát vùng chết.
- Giữ đúng hướng trái/phải, không giữ hướng cũ khi mục tiêu đã đổi bên.
- Reset trạng thái khi STOP, lỗi, mất dữ liệu hoặc chuyển chế độ.
- Kiểm tra nhánh `ENABLE_D1_STEER_DYNAMIC` vốn có hysteresis; tránh chồng hai cơ chế gây mâu thuẫn.
- Hysteresis phải tác động thực sự đến điều kiện tạo lệnh xoay, không bị điều kiện vùng chết cũ vô hiệu hóa.

**4. Hiệu chuẩn động cơ và phanh**

Chuẩn bị quy trình đo bằng MOTOR, SWEEP và encoder để xác định:
- PWM khởi động và PWM duy trì của từng bánh, cả tiến và lùi.
- Quan hệ PWM–tốc độ.
- Số xung encoder trên mỗi mm.
- Thời gian hoặc quãng đường dừng.

Chưa có dữ liệu đo mới thì giữ nguyên các thông số đang dùng, gồm các mức PWM 78/42, `COUNTS_PER_MM_EST=10` và `LOOKAHEAD_S=0.08`.

Không tự tạo số liệu hoặc tuyên bố đã hiệu chuẩn. Hoàn thành hai cải tiến phần mềm trước; phần hiệu chuẩn cung cấp bảng ghi số liệu và hướng dẫn cập nhật sau khi đo.

**5. Cấu hình và log**

Hai cờ mới mặc định bằng 0. Khi cùng tắt, kết quả điều khiển phải tương đương V2.1.

Ưu tiên `#if/#else` để giữ nguyên nhánh cũ. Nếu cần, chỉ thêm cuối dòng log:
- `TURN_RAW_CPS`
- `TURN_SLEW_CPS`
- `STEER_ACTIVE`

Giữ LOG 0 và cơ chế in log không chặn hiện có.

**6. Kiểm chứng**

Giữ `HAND_FOLLOW_NATIVE_TEST` và bổ sung kiểm tra:
- Bắt đầu xoay trái/phải từ đứng yên.
- Đổi hướng xoay.
- Tín hiệu dao động quanh ngưỡng bật/tắt.
- Tiến kết hợp rẽ và chỉ xoay theo cảm biến bên.
- Một bánh có tốc độ đặt bằng 0 hoặc gần ngưỡng tối thiểu.
- STOP, mất mục tiêu, dữ liệu quá hạn và giới hạn xoay bên khi slew đang hoạt động.
- Hai cờ mới tắt cho kết quả tương đương V2.1.

Kiểm tra C++11 và build PlatformIO STM32duino nếu có môi trường. Phân biệt rõ kiểm tra phần mềm với thử trên xe thật.

**7. Đầu ra**

Cung cấp:
1. File `main.cpp` V2.2 hoàn chỉnh, thay trực tiếp vào `src/main.cpp`.
2. Bản diff so với V2.1.
3. Bảng ngắn các thay đổi và trạng thái mặc định.
4. Kết quả kiểm tra thực sự đã chạy.
5. Hướng dẫn thử A/B: baseline → chỉ TURN_SLEW → chỉ STEER_HYSTERESIS → kết hợp sau khi từng phần đạt yêu cầu.

Ưu tiên giữ khả năng bám hiện tại, giảm giật khi rẽ và bảo toàn các cơ chế an toàn. Không thêm tính năng ngoài phạm vi. Phần giải thích cuối cùng viết ngắn gọn.

****