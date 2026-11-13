## code v2:

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
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Optional PlatformIO build flags let the bus and ranging budget be tested independently.
#ifndef FOLLOW_I2C_HZ
#define FOLLOW_I2C_HZ 100000UL
#endif
#ifndef FOLLOW_BUDGET_US
#define FOLLOW_BUDGET_US 20000UL
#endif

namespace Follow {
float frontPairBiasMm[2] = {0.0f, 0.0f};
uint8_t frontBalMask = 0;
constexpr uint8_t COUNT = 4;
constexpr uint8_t FRONT_A = 1, FRONT_B = 2;
constexpr float LOW_MM = 100.0f, HIGH_MM = 110.0f, MAX_MM = 200.0f;
constexpr float MID_MM = (LOW_MM + HIGH_MM) * 0.5f;
constexpr uint32_t BASE_BUS_HZ = 50000UL, BASE_BUDGET_US = 33000UL;
constexpr uint32_t CONTROL_MS = 10, FRESH_MS = 100;
constexpr uint32_t WATCHDOG_MS = 100, GUARD_SAMPLE_MS = 125;
constexpr uint32_t MAX_POLL_GAP_MS = 70;  // 4 sensors on 50 kHz bus, validate DT_MAX + age in logs
constexpr uint32_t CONFIRM_WINDOW_MS = 120;   // 2 mau hop le cung vung trong cua so nay = xac nhan
constexpr uint32_t PAIR_WINDOW_MS = 24, REVERSE_PAUSE_MS = 25;
constexpr uint32_t STALL_MS = 850;
constexpr float PAIR_TOL_MM = 18.0f;
constexpr float FUSE_GAP_TOL_MM = 10.0f;  // avg only close, synchronized front distances
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
constexpr float MIN_CPS = 300.0f, MAX_CPS = 1600.0f;  // body speed cap from the road-tested 1600-line build
constexpr float BODY_ACCEL_CPS2 = 9000.0f;
constexpr float BRAKE_DECEL_CPS2 = 3500.0f;  // initial estimate, not measured
constexpr float LOOKAHEAD_S = 0.045f;               // do tre do + phanh (giay)
constexpr float EDGE_CPS = 250.0f, EDGE_CPS_PER_MM = 25.0f;  // giam toc som truoc vung 100..110
// Vong toc do tung banh (feedforward + PI) -> PWM.
constexpr float SPEED_KP = 0.010f, SPEED_KI = 0.080f, I_LIMIT_PWM = 25.0f;
constexpr float MAX_PWM = 125.0f, ACCEL_PWM_S = 850.0f, DECEL_PWM_S = 2400.0f;
constexpr float SYNC_KP = 0.18f, SYNC_MAX_CPS = 200.0f;
constexpr int32_t WRONG_DIR_COUNTS = 40;

// Hinh hoc va bo re: tinh theo do chenh toc do hai banh.
// COUNTS_PER_MM_EST CHUA DUOC DO => he so goc chi la xap xi de thu.
constexpr float WHEEL_TRACK_MM = 120.0f;
constexpr float SIDE_LEFT_DEG = 35.43f, SIDE_RIGHT_DEG = 35.43f;
constexpr float SIDE_SETBACK_MM = 15.0f;
constexpr float COUNTS_PER_MM_EST = 10.0f;
constexpr float TURN_RAD_S_MAX = 1.25f;
constexpr float TURN_CPS_MIN = 300.0f, TURN_CPS_MAX = 600.0f;
constexpr float TURN_DEADBAND = 0.12f;  // expanded steering deadband for noise
constexpr float FRONT_DIFF_DEADBAND_MM = 18.0f;
constexpr float FRONT_DIFF_NORMALIZE_FACTOR = 0.90f;
// BAL 100 and BAL 150 are deliberately optional: without both, keep stable front logic.
extern float frontPairBiasMm[2];
extern uint8_t frontBalMask;
constexpr float STEER_TAU_S = 0.060f;
constexpr float ARC_BODY_SCALE = 0.65f;
constexpr float PAIR_STEER_DEADBAND = 0.045f; // applied after the 18 mm BAL deadband
constexpr float PAIR_PIVOT_MIN_CPS = 180.0f;
constexpr float NEAR_EDGE_MIN_CPS = 180.0f; // starting estimate; verify under load
constexpr float ACQUIRE_DELTA_MM = 35.0f;  // reject unrelated consecutive ranges
constexpr float SIDE_ASSOC_TOL_MM = 85.0f;  // chi la cong dieu huong, khong phai nhan dang tay
// 1.5s hard limit for a side-only rotation episode; rearm only after confirmed target is lost.
constexpr uint32_t SIDE_TURN_MAX_MS = 1500;
constexpr uint32_t SIDE_REARM_ABSENT_MS = 200;
constexpr uint8_t SIDE_L = 0, SIDE_R = 3;


// ================= SMALL A/B OPTIONS (edit here, rebuild, upload) =============
// Conservative defaults: corrected sync ON; experimental smoothing OFF.
constexpr bool SYNC_REQUEST_DIFF_ENABLE = true;
constexpr bool FRONT_SINGLE_CONFIRM_ENABLE = false;
constexpr uint8_t FRONT_SINGLE_CONFIRM_FRAMES = 2; // 2 or 3 NEW exclusive frames
constexpr bool HOLD_HYSTERESIS_ENABLE = false;
constexpr float HOLD_EXIT_MARGIN_MM = 13.0f; // optional hold expands to 87..123mm
constexpr bool DIST_KD_TRIAL_ENABLE = false;
constexpr float DIST_KD_TRIAL = 1.8f;
constexpr bool RATE_FILTER_TRIAL_ENABLE = false;
constexpr float RATE_TAU_TRIAL_S = 0.120f;
constexpr bool PWM_RAMP_TRIAL_ENABLE = false;
constexpr float ACCEL_PWM_TRIAL_S = 450.0f, DECEL_PWM_TRIAL_S = 1300.0f;
constexpr bool EXTRA_LOG_ENABLE = true;
constexpr bool RECOVERABLE_FAULTS = false;
constexpr uint32_t RECOVERY_STABLE_MS = 300, RECOVERY_MAX_DT_MS = 25;
constexpr uint8_t RECOVERY_MAX_ATTEMPTS = 1, RECOVERY_MIN_NEW_FRAMES = 3;
// ENC_DIR_* stays latched: a stopped encoder cannot prove its sign is correct.
// STALL/watchdog may have ONE guarded retry per boot when enabled. Repeated faults latch.
// I2C uninitialized-buffer and reversal-time fixes are always ON (not unsafe A/B options).
static_assert(FRONT_SINGLE_CONFIRM_FRAMES >= 2 && FRONT_SINGLE_CONFIRM_FRAMES <= 3,
              "Single front confirmation requires 2 or 3 new frames");


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


enum class Zone { Near, Band, Track, Far };
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
  bool valid = false, seenFrame = false, jumpPending = false;
  float pendingMm = 0;
  uint32_t pendingAt = 0;
  uint8_t streak = 0, misses = 0, deviceStatus = 255;
  uint16_t raw = 0, rawBlock = 0, rawLib = 0, signalQ7 = 0;
  float mm = 0, filtered = 0, rate = 0;
  uint32_t receivedAt = 0, sampleAt = 0, frameAt = 0;

  void invalidate() { jumpPending = false; valid = false; streak = 0; misses = 0; rate = 0; }
  // Mot khung xau don le khong xoa xac nhan; mau cu van tu het han theo FRESH_MS.
  void miss() {
    // Only an established track may coast through one bad result.
    if (jumpPending) { invalidate(); return; }
    if (valid && (++misses >= 2 || streak < 2)) invalidate();
  }
  bool fresh(uint32_t now) const { return valid && !jumpPending && now - sampleAt < FRESH_MS; }

  // Chi duoc goi khi pollSensor() vua lay mot khung MOI, nen moi lan tang streak
  // va moi lan tinh dao ham deu ung voi mot phep do moi.
  bool accept(uint16_t rawMm, float offset, uint32_t received, uint32_t oldestStart) {
    const float next = fmaxf(0.0f, float(rawMm) + offset);
    const uint32_t elapsed = received - receivedAt;
    const bool adjacent = valid && elapsed > 0 && elapsed <= CONFIRM_WINDOW_MS;
    // Reject a lone range jump on an established in-range track. Save the
    // candidate separately; do not drive or feed it to the derivative.
    // An out-of-range reading is processed immediately, including during a jump.
    if (next <= MAX_MM && mm <= MAX_MM && adjacent && streak >= 2) {
      const float tolerance = clip(8.0f + 0.42f * elapsed, 16.0f, 32.0f);
      if (fabsf(next - mm) > tolerance) {
        const uint32_t delta = received - pendingAt;
        const float pairTolerance = clip(8.0f + 0.42f * delta, 16.0f, 32.0f);
        if (!jumpPending || delta == 0 || delta > CONFIRM_WINDOW_MS ||
            fabsf(next - pendingMm) > pairTolerance || zoneOf(next) != zoneOf(pendingMm)) {
          jumpPending = true; pendingMm = next; pendingAt = received;
          return false;
        }
        // Two coherent new readings. Use the candidate to confirm the zone,
        // and restart the velocity filter at the new range.
        mm = pendingMm; streak = 1; misses = 1;
      }
    }
    jumpPending = false;
    // A missed provisional frame was already invalidated by miss(). An
    // established track may resume after one dropout if the new range agrees.
    const Zone before = zoneOf(mm), after = zoneOf(next);
    // Carry an established track across a single hold boundary on a small,
    // clean step. Never carry confidence from Far or directly Near <-> Track.
    const bool nearBoundary = (before == Zone::Band || after == Zone::Band) &&
                              before != Zone::Far && after != Zone::Far;
    const bool trackedCrossing = streak >= 2 && !misses && nearBoundary &&
                                 fabsf(next - mm) <= 12.0f;
    const bool coherent = adjacent && fabsf(next - mm) <= ACQUIRE_DELTA_MM &&
                          (before == after || trackedCrossing);
    streak = coherent ? (streak < 3 ? streak + 1 : 3) : 1;
    if (!adjacent || misses || fabsf(next - mm) >= 25.0f) {
      filtered = next;
      rate = 0;  // bat lai muc tieu / nhay bac khong phai la phep do van toc
    } else {
      const float dt = elapsed * 0.001f;
      const float tau = fabsf(next - filtered) > 6.0f ? 0.012f : 0.032f;
      const float previous = filtered;
      filtered += dt / (tau + dt) * (next - filtered);
      const float slope = clip((filtered - previous) / dt, -400, 400);
      rate += dt / ((RATE_FILTER_TRIAL_ENABLE ? RATE_TAU_TRIAL_S : 0.060f) + dt) * (slope - rate);
    }
    raw = rawMm; mm = next; receivedAt = received; sampleAt = oldestStart;
    valid = true; misses = 0; return true;
  }
};

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
enum class SteerCue { None, FrontPair, SingleFront, Side };
struct Target {
  SteerCue cue = SteerCue::None;
  bool fusedGap = false;
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
  const bool pair = sameZone && !r[i].misses && !r[j].misses && skew <= PAIR_WINDOW_MS && fabsf(r[i].mm - r[j].mm) <= PAIR_TOL_MM;
  bool pairOnly = false;
  if (r[i].streak < 2) {
    if (pair) pairOnly = true;
    else if (sameZone && r[j].streak >= 2) { const int k = i; i = j; j = k; }  // dung mat da tu xac nhan
  }
  const bool confirmed = r[i].streak >= 2 || pairOnly;
  t.index = i; t.gap = r[i].mm; t.filtered = r[i].filtered; t.rate = r[i].rate;
  t.sampleAt = r[i].sampleAt;
  // A slanted hand may put one return just across a band edge. Average only
  // clean, near-simultaneous readings. Give a clearly close return priority.
  const float nearest = fminf(r[i].mm, r[j].mm);
  const float farthest = fmaxf(r[i].mm, r[j].mm);
  const bool closeConflict = nearest <= LOW_MM - 3.0f && farthest >= LOW_MM;
  if (both && !r[i].misses && !r[j].misses && skew <= PAIR_WINDOW_MS &&
      farthest <= MAX_MM && farthest - nearest <= FUSE_GAP_TOL_MM && !closeConflict &&
      (pair || (r[i].streak >= 2 && r[j].streak >= 2))) {
    t.gap = 0.5f * (r[i].mm + r[j].mm);
    t.filtered = 0.5f * (r[i].filtered + r[j].filtered);
    t.rate = 0.5f * (r[i].rate + r[j].rate);
    t.fusedGap = true;
  }
  const Zone z = zoneOf(t.gap);
  if (z == Zone::Band && confirmed) t.state = State::Hold;
  else if (z == Zone::Far) t.state = State::Far;
  else if (!confirmed) t.state = State::Confirm;
  else t.state = z == Zone::Near ? State::Reverse : State::Forward;
  // Neu hai khung dau tien la bang chung duy nhat thi het han theo khung cu hon.
  if ((pairOnly || t.fusedGap) && now - r[j].sampleAt > now - t.sampleAt) t.sampleAt = r[j].sampleAt;
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

bool pendingClose(const Reading &r, uint32_t now) {
  return r.jumpPending && now - r.pendingAt < FRESH_MS && r.pendingMm <= HIGH_MM;
}

Target locate(const Reading (&r)[COUNT], uint32_t now) {
  Target t = locateFront(r, now);
  // Do not continue forward on the older eye while the other is verifying a
  // sudden close return. A false near spike costs a pause, never blind reverse.
  if (t.state == State::Forward && (pendingClose(r[FRONT_A], now) || pendingClose(r[FRONT_B], now))) {
    t.state = State::Confirm;
    return t;
  }
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
    if (fl && fr) {
      const uint32_t ageA = now - r[FRONT_A].receivedAt, ageB = now - r[FRONT_B].receivedAt;
      const uint32_t skew = ageA > ageB ? ageA - ageB : ageB - ageA;
      if (!r[FRONT_A].misses && !r[FRONT_B].misses && skew <= PAIR_WINDOW_MS) {
        t.steer = calibratedFrontSteer(r[FRONT_A].filtered, r[FRONT_B].filtered, t.gap);
        if (t.steer != 0) t.cue = SteerCue::FrontPair;
      }
    } else if (fl != fr) {
      const Reading &other = r[fl ? FRONT_B : FRONT_A];
      if (!(other.jumpPending && now - other.pendingAt < FRESH_MS)) {
        t.steer = fl ? -0.38f : 0.38f;
        t.cue = SteerCue::SingleFront;
      }
    }
    if (sl && !sr) {
      t.cue = SteerCue::Side;
      t.steer = -SIDE_LEFT_DEG / SIDE_RIGHT_DEG;
      t.mask |= (uint8_t)(1U << SIDE_L); mergeSampleAge(t, r[SIDE_L], now);
    } else if (sr && !sl) {
      t.cue = SteerCue::Side;
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
    t.cue = SteerCue::Side;
    t.steer = left ? -SIDE_LEFT_DEG / SIDE_RIGHT_DEG : +1.0f;
    t.mask = (uint8_t)(1U << t.index);
    t.state = left ? State::TurnLeft : State::TurnRight;
  } else if (t.index >= 0 && t.state == State::Confirm) {
    // Do not drive a target lacking confirmation.
    t.steer = 0;
  }
  return t;
}

struct Command { float cps = 0, edgeMm = 0, turnCps = 0; };
Command decide(const Target &t) {
  Command c;
  const bool translating = t.state == State::Forward || t.state == State::Reverse;
  const bool pivot = t.state == State::Hold || t.state == State::TurnLeft || t.state == State::TurnRight;
  if (!translating && !pivot) return c;

  const bool pairCue = t.cue == SteerCue::FrontPair;
  const float deadband = pairCue ? PAIR_STEER_DEADBAND : TURN_DEADBAND;
  if (t.cue != SteerCue::None && fabsf(t.steer) > deadband) {
    const float yaw = fabsf(t.steer) * WHEEL_TRACK_MM * 0.5f * COUNTS_PER_MM_EST * TURN_RAD_S_MAX;
    const float minimum = pairCue ? (pivot ? PAIR_PIVOT_MIN_CPS : 0.0f) : TURN_CPS_MIN;
    c.turnCps = sign(t.steer) * clip(yaw, minimum, TURN_CPS_MAX);
    c.edgeMm = 20.0f;  // available breakaway torque for yaw
  }
  if (!translating) return c;  // zero longitudinal command through the entire 100..110 band

  const int d = t.state == State::Forward ? 1 : -1;
  const float edge = d > 0 ? t.gap - HIGH_MM : LOW_MM - t.gap;
  c.edgeMm = fmaxf(c.edgeMm, edge);
  const float error = clip(t.filtered - MID_MM, t.gap - MID_MM - 4.0f, t.gap - MID_MM + 4.0f);
  const float closing = fmaxf(0.0f, -d * t.rate);
  const float remaining = edge - closing * LOOKAHEAD_S;
  if (remaining <= 0.0f) return c;

  // Match the tested turn radius with a single confirmed eye; keep the faster
  // straight-line response when both eyes are centered.
  const bool tightArc = c.turnCps != 0.0f && !pairCue;
  const float urgency = tightArc ? 0.0f : clip((edge - 3.0f) / 15.0f, 0.0f, 1.0f);
  const float kp = DIST_KP + 10.0f * urgency;
  const float kd = DIST_KD_TRIAL_ENABLE ? DIST_KD_TRIAL : DIST_KD;
  const float pd = d * (kp * error + kd * t.rate);
  const float edgeCap = EDGE_CPS + (EDGE_CPS_PER_MM + 12.0f * urgency) * edge;
  const float floorCps = fminf(MIN_CPS, NEAR_EDGE_MIN_CPS + 8.0f * edge);
  float speed = clip(pd, floorCps, fminf(MAX_CPS, fmaxf(floorCps, edgeCap)));
  if (tightArc) speed = fminf(speed, fmaxf(NEAR_EDGE_MIN_CPS, ARC_BODY_SCALE * speed));
  // Cap stopping distance independently of yaw's PWM allowance. Decelerate
  // immediately on approach; do not floor the result back up after braking.
  const float stopCap = sqrtf(2.0f * BRAKE_DECEL_CPS2 * COUNTS_PER_MM_EST * remaining);
  c.cps = d * fminf(speed, stopCap);
  return c;
}

struct BodySlew {
  float value = 0;
  void reset() { value = 0; }
  float update(float goal, float dt) {
    if (goal == 0 || value * goal < 0) { reset(); return 0; }
    if (fabsf(goal) < fabsf(value)) value = goal;
    else value = towards(value, goal, BODY_ACCEL_CPS2 * dt);
    return value;
  }
};

struct Wheel {
  float cps = 0, integral = 0, applied = 0;
  int request = 0, lastDrive = 0, watched = 0;
  uint32_t zeroAt = 0, driveSince = 0, lastPulse = 0;
  int32_t wrongCounts = 0;
  int wrongMs = 0;
  bool interlock = false;

  void zero(uint32_t now) {
    if (applied != 0) { lastDrive = sign(applied); zeroAt = now; }
    applied = 0; integral = 0; request = 0; watched = 0; interlock = false;
    wrongCounts = 0; wrongMs = 0;
  }
  void observe(int32_t delta, float dt) {
    cps += dt / (0.060f + dt) * (float(delta) / dt - cps);
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
    if (sign(applied) && sign(applied) != d) zero(now);  // bo mo-men cu ngay trong tick nay
    const bool waiting = lastDrive && d != lastDrive && now - zeroAt < REVERSE_PAUSE_MS;
    interlock = waiting;
    if (waiting) { request = 0; return 0; }  // PWM = 0 va trang thai noi bo cung = 0
    const float speed = p.encoderForwardSign ? d * p.encoderForwardSign * cps : fabsf(cps);
    const float ref = fabsf(c.cps), error = ref - speed;
    // Banh dang nhanh hon han toc do yeu cau: phanh ngan mach thay vi giu PWM san.
    if (speed > 1.4f * ref + 150.0f) {
      if (applied != 0) { lastDrive = sign(applied); zeroAt = now; }
      applied = 0; request = 0; return 0;
    }
    const float start = d > 0 ? p.startFwd : p.startRev;
    const float limit = fminf(MAX_PWM, start + fminf(47.0f, 1.15f * c.edgeMm));
    const float ff = p.runPwm + (160.0f - p.runPwm) * ref / p.cpsAt160;
    // Chi tang dan mo-men khoi dong khi encoder khong co xung.
    const uint32_t quiet = watched == d ? now - lastPulse : 0;
    const float boost = p.runPwm + (start - p.runPwm) *
                        clip((float(quiet) - 25.0f) / 80.0f, 0, 1);
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
    const float rate = wanted < fabsf(applied)
      ? (PWM_RAMP_TRIAL_ENABLE ? DECEL_PWM_TRIAL_S : DECEL_PWM_S)
      : (PWM_RAMP_TRIAL_ENABLE ? ACCEL_PWM_TRIAL_S : ACCEL_PWM_S);
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
  if (fabsf(lc.cps) < MIN_CPS * 0.50f) lc.cps = 0;
  if (fabsf(rc.cps) < MIN_CPS * 0.50f) rc.cps = 0;
  if (SYNC_REQUEST_DIFF_ENABLE) {
    // lt,rt are signed forward-positive targets AFTER the original 150cps floor.
    const float lt = lc.cps, rt = rc.cps / RIGHT_COUNTS_PER_LEFT_COUNT;
    const int ld = sign(lt), rd = sign(rt);
    if (!ld || !rd || l.watched != ld || r.watched != rd ||
        now - l.driveSince <= 75 || now - r.driveSince <= 75) return;
    const float ls = LEFT_TUNE.encoderForwardSign * l.cps;
    const float rs = RIGHT_TUNE.encoderForwardSign * r.cps / RIGHT_COUNTS_PER_LEFT_COUNT;
    const float bound = fminf(SYNC_MAX_CPS, 0.25f * fminf(fabsf(lt), fabsf(rt)));
    trim = clip(SYNC_KP * ((ls - rs) - (lt - rt)), -bound, bound);
    lc.cps = lt - trim;
    rc.cps = (rt + trim) * RIGHT_COUNTS_PER_LEFT_COUNT;
    // No post-trim floor or zeroing: the 25% bound cannot reverse either wheel.
    return;
  }
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
}

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
      // New confirmed front observation disarms side-only timeout immediately.
      if (t.hasFront && (t.state == State::Forward || t.state == State::Reverse || t.state == State::Hold)) {
        reset();
      } else if (blocked || active) {
        // Continued side returns are NOT a new acquisition. Require 200ms without
        // confirmed side observations, then a fresh two-frame confirmation.
        if (absentAt == 0) absentAt = now;
        if ((uint32_t)(now - absentAt) >= SIDE_REARM_ABSENT_MS) reset();
      }
    }
  }
};

struct FrontHandoff {
  int source = -1, direction = 0, pending = -1;
  uint32_t pendingFrame = 0;
  void reset() { source = pending = -1; direction = 0; }
  void apply(Target &t, const Reading (&r)[COUNT]) {
    if (t.state == State::Confirm) return;  // preserve last trusted source
    const int next = t.state == State::Forward ? 1 : t.state == State::Reverse ? -1 : 0;
    if (!next) { reset(); return; }
    if (source >= 0 && source != t.index && direction != next) {
      const uint32_t frame = r[t.index].receivedAt;
      if (pending != t.index || frame == pendingFrame || r[t.index].misses) {
        pending = t.index; pendingFrame = frame;
        t.hasFront = false; t.steer = 0; t.state = State::Confirm;
        return;
      }
    }
    source = t.index; direction = next; pending = -1;
  }
};

bool motionState(State state) {
  return state == State::Hold || state == State::Forward || state == State::Reverse ||
         state == State::TurnLeft || state == State::TurnRight;
}
struct FrontSingleConfirm {
  int candidate = -1;
  uint8_t frames = 0;
  uint32_t lastFrame = 0;
  bool seen = false;
  void reset() { candidate = -1; frames = 0; lastFrame = 0; seen = false; }
  void apply(Target &t, const Reading (&r)[COUNT], uint32_t now) {
    if (!FRONT_SINGLE_CONFIRM_ENABLE) { reset(); return; }
    const bool a = r[FRONT_A].fresh(now) && r[FRONT_A].streak >= 2 && r[FRONT_A].mm <= MAX_MM;
    const bool b = r[FRONT_B].fresh(now) && r[FRONT_B].streak >= 2 && r[FRONT_B].mm <= MAX_MM;
    // Side override and calibrated two-front steering remain exactly as locate() chose.
    if (!t.hasFront || a == b || (t.mask & ((1U << SIDE_L) | (1U << SIDE_R)))) { reset(); return; }
    const int i = a ? FRONT_A : FRONT_B;
    if (candidate != i) { reset(); candidate = i; }
    if (r[i].misses) { frames = 0; seen = true; lastFrame = r[i].receivedAt; }
    else if (!seen || r[i].receivedAt != lastFrame) {
      seen = true; lastFrame = r[i].receivedAt;
      if (frames < FRONT_SINGLE_CONFIRM_FRAMES) ++frames;
    }
    if (frames < FRONT_SINGLE_CONFIRM_FRAMES) t.steer = 0;
  }
};
struct HoldHysteresis {
  bool held = false;
  void reset() { held = false; }
  void apply(Target &t) {
    if (!HOLD_HYSTERESIS_ENABLE || !t.hasFront || !motionState(t.state)) { reset(); return; }
    if (t.gap >= LOW_MM && t.gap <= HIGH_MM) held = true;
    else if (t.gap < LOW_MM - HOLD_EXIT_MARGIN_MM || t.gap > HIGH_MM + HOLD_EXIT_MARGIN_MM) held = false;
    if (held) t.state = State::Hold;
  }
};
struct TurnSmoother {
  float steer = 0;
  SteerCue previousCue = SteerCue::None;
  void reset() { steer = 0; previousCue = SteerCue::None; }
  Command update(Target &t, uint32_t now, float dt) {
    if (!motionState(t.state) || t.index < 0 || now - t.sampleAt >= FRESH_MS) {
      reset(); return Command();
    }
    // Avoid carrying a strong single-side cue into a weak calibrated pair.
    if (t.steer == 0 || steer * t.steer < 0 ||
        (t.cue == SteerCue::FrontPair && previousCue != t.cue)) steer = 0;
    steer += dt / (STEER_TAU_S + dt) * (t.steer - steer);
    previousCue = t.cue;
    t.steer = steer;
    return decide(t);
  }
};
// Stationary encoder samples only establish "quiet", not correct direction.
// Thus encoder-direction faults are not eligible for autonomous recovery.
struct RecoveryGate {
  bool timing = false, seenFrame = false;
  uint32_t since = 0, lastFrame = 0, attempts = 0;
  uint8_t frames = 0;
  void resetWindow() { timing = seenFrame = false; frames = 0; }
  static bool supported(const char *reason) {
    return reason && (!strcmp(reason,"STALL_L") || !strcmp(reason,"STALL_R") ||
      !strcmp(reason,"WATCHDOG_CONTROL") || !strcmp(reason,"WATCHDOG_SENSOR"));
  }
  bool update(const char *reason, bool healthy, uint32_t frame, uint32_t now) {
    if (!RECOVERABLE_FAULTS || !supported(reason) || attempts >= RECOVERY_MAX_ATTEMPTS || !healthy) {
      resetWindow(); return false;
    }
    if (!timing) { timing = true; since = now; }
    if (!seenFrame || frame != lastFrame) {
      seenFrame = true; lastFrame = frame;
      if (frames < RECOVERY_MIN_NEW_FRAMES) ++frames;
    }
    if (now - since < RECOVERY_STABLE_MS || frames < RECOVERY_MIN_NEW_FRAMES) return false;
    ++attempts; resetWindow(); return true;
  }
};

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
uint32_t activeBusHz = BASE_BUS_HZ;
uint32_t activeBudgetUs = BASE_BUDGET_US;
constexpr uint32_t ARM_DELAY_MS = 3000, LOG_PERIOD_MS = 200;
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
FrontSingleConfirm frontSingle;
HoldHysteresis holdHysteresis;
TurnSmoother turnSmoother;
BodySlew bodySlew;
RecoveryGate recovery;
SideTurnLimit sideTurnLimit;
FrontHandoff frontHandoff;
uint32_t armAt = 0, lastControl = 0, controlDt = 0, maxControlDt = 0, lastLog = 0;
int32_t encWindowLeft = 0, encWindowRight = 0;   // co dau: giu duoc chieu quay
uint16_t previousLeft = 0, previousRight = 0;
int32_t deltaLeft = 0, deltaRight = 0;
uint32_t lastProbe[COUNT] = {}, i2cErrors[COUNT] = {}, maxPollGap[COUNT] = {};
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
  bodySlew.reset();
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
  bodySlew.reset();
  for (uint8_t i = 0; i < COUNT; ++i) { reading[i].invalidate(); probeKnown[i] = false; }
  target = Target(); command = leftCommand = rightCommand = Command(); syncCps = 0; steerSmooth = 0; sideTurnLimit.reset(); frontHandoff.reset();
  frontSingle.reset(); holdHysteresis.reset(); turnSmoother.reset();
}
void sensorBusError(uint8_t i) {
  ++i2cErrors[i]; probeKnown[i] = false;
  reading[i].deviceStatus = 255; reading[i].invalidate();
}

void pollSensor(uint8_t i) {
  if (!ready[i]) return;
  const uint32_t probeAt = millis();
  const uint32_t precedingProbe = lastProbe[i];
  if (probeKnown[i] && probeAt - precedingProbe > maxPollGap[i]) maxPollGap[i] = probeAt - precedingProbe;
  const bool bounded = probeKnown[i] && probeAt - precedingProbe <= MAX_POLL_GAP_MS;
  uint8_t interrupt = 0;
  if (!readRegisters(i, VL53L0X::RESULT_INTERRUPT_STATUS, &interrupt, 1)) { sensorBusError(i); return; }
  if (!(interrupt & 0x07)) { lastProbe[i] = probeAt; probeKnown[i] = true; return; }

  // Khoi 12 byte: chi dung byte 0 (status) va byte 6..7 (signal), la cac byte KHONG nam cuoi.
  uint8_t block[12] = {};
  // Normal mode ends at byte 8, keeping signal bytes 6/7 away from the final byte.
  const bool diagnostics = mode == Mode::SensorTest;
  const bool blockOK = readRegisters(i, VL53L0X::RESULT_RANGE_STATUS, block, diagnostics ? 12 : 9);
  if (!blockOK) { clearReady(i); sensorBusError(i); return; }
  // Khoang cach: doc high -> low -> high AGAIN, 3 giao dich 1 byte.
  // Never use the last byte of a multi-byte I2C read for the control distance.
  const uint8_t rangeReg = (uint8_t)(VL53L0X::RESULT_RANGE_STATUS + 10);
  uint8_t hi = 0, lo = 0, hiAgain = 0;
  const bool bytesOK = readRegisters(i, rangeReg, &hi, 1) &&
                       readRegisters(i, (uint8_t)(rangeReg + 1), &lo, 1) &&
                       readRegisters(i, rangeReg, &hiAgain, 1);
  uint16_t lib16 = 0;
  if (mode == Mode::SensorTest) lib16 = sensor[i].readReg16Bit(rangeReg);  // cach doc cua thu vien Pololu
  const bool clearOK = clearReady(i);  // xoa co ngat SAU khi da doc xong ket qua
  if (!blockOK || !bytesOK || !clearOK) { sensorBusError(i); return; }

  const uint16_t raw = ((uint16_t)hi << 8) | lo;
  const uint8_t status = (block[0] & 0x78) >> 3;
  const uint16_t signal = ((uint16_t)block[6] << 8) | block[7];
  Reading &s = reading[i];
  s.deviceStatus = status; s.signalQ7 = signal; s.seenFrame = true;
  s.raw = raw; s.rawBlock = ((uint16_t)block[10] << 8) | block[11]; s.rawLib = lib16;
  s.frameAt = millis();
  lastProbe[i] = probeAt; probeKnown[i] = true;

  if (hi != hiAgain) { s.miss(); return; }  // different frames: drop once, not a bus error
  if (!bounded) { s.invalidate(); return; }        // khong biet khung nay cu bao lau: bo, cho khung ke tiep
  // Status 11 = phep do duoc cam bien chap nhan. 0 va >=8190 la gia tri "khong co muc tieu".
  if (status != 11 || raw == 0 || raw >= 8190 || signal < 32) { s.miss(); return; }
  if (!s.accept(raw, offsets[i], millis(), precedingProbe - (activeBudgetUs + 999U) / 1000U)) return;
  if (calibration.active && calibration.index == i) {
    if (calibration.discard) --calibration.discard;
    else if (calibration.count < 40) calibration.values[calibration.count++] = raw;
  }
}

bool configureTiming(uint32_t busHz, uint32_t budgetUs) {
  Wire.setClock(busHz);
  for (uint8_t i = 0; i < COUNT; ++i) {
    uint8_t status;
    if (!readRegisters(i, VL53L0X::RESULT_INTERRUPT_STATUS, &status, 1) ||
        !sensor[i].setMeasurementTimingBudget(budgetUs) || sensor[i].last_status != 0) return false;
  }
  activeBusHz = busHz;
  activeBudgetUs = budgetUs;
  return true;
}

bool initSensors() {
  // Assign addresses before increasing bus speed. Timing is set on all four
  // stopped sensors, so an unsuccessful fast configuration is reversible.
  Wire.setSCL(PB10); Wire.setSDA(PB11); Wire.begin(); Wire.setClock(BASE_BUS_HZ);
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
        !sensor[i].setSignalRateLimit(0.25f) || sensor[i].last_status != 0) {
      holdLow(XSHUT[i]); Serial.print(NAME[i]); Serial.println(" CONFIG_FAIL"); continue;
    }
    ready[i] = true; ++count;
    Serial.print(NAME[i]); Serial.println(" READY");
  }
  if (count != COUNT) return false;
  if (!configureTiming(FOLLOW_I2C_HZ, FOLLOW_BUDGET_US)) {
    if ((FOLLOW_I2C_HZ == BASE_BUS_HZ && FOLLOW_BUDGET_US == BASE_BUDGET_US) ||
        !configureTiming(BASE_BUS_HZ, BASE_BUDGET_US)) return false;
    Serial.println("TIMING_FALLBACK");
  }
  Serial.print("I2C_HZ="); Serial.print(activeBusHz);
  Serial.print(" BUDGET_US="); Serial.println(activeBudgetUs);
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
  s.frontBalMask = frontBalMask;
  for (uint8_t i = 0; i < 2; ++i) s.frontBiasMm[i] = frontPairBiasMm[i];
  for (uint8_t i = 0; i < COUNT; ++i) s.offsets[i] = offsets[i];
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
    if ((calibration.index == FRONT_A || calibration.index == FRONT_B) &&
        offsets[calibration.index] != offset) {
      frontBalMask = 0;
      frontPairBiasMm[0] = frontPairBiasMm[1] = 0;
      Serial.println("FRONT_OFFSET_CHANGED: repeat BAL 100 and BAL 150.");
    }
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
      Serial.println("SAVE stores offsets and BAL. RUN to resume.");
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
  stopOutputs(); turnSmoother.reset(); frontSingle.reset(); holdHysteresis.reset();
  if (!fault) { fault = reason; recovery.resetWindow(); }
}
bool encoderConfigOK(TIM_TypeDef *t) {
  return (t->CR1 & TIM_CR1_CEN) && (t->SMCR & 7U) == 3 && t->ARR == 65535 &&
         t->PSC == 0 && t->DIER == 0 && (t->CCMR1 & 0xffffU) == 0x8181U &&
         (t->CCER & 0xffffU) == (TIM_CCER_CC1E | TIM_CCER_CC2E);
}
// pwm > 0 = TIEN. Trai tien: IN1 LOW, IN2 HIGH. Phai tien: IN1 HIGH, IN2 LOW.
void setWheelPins(int pwm, bool left) {
  const uint32_t a = left ? LEFT_IN1 : RIGHT_IN1, b = left ? LEFT_IN2 : RIGHT_IN2;
  const uint32_t out = left ? LEFT_PWM : RIGHT_PWM;
  const bool reversed = left ? LEFT_REVERSE : RIGHT_REVERSE;
  if (!pwm) {
    analogWrite(out, 0); digitalWrite(a, HIGH); digitalWrite(b, HIGH);  // ca hai HIGH = phanh ngan mach
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
void applyOutputs(int l, int r, uint32_t sampleAt) {
  const bool moving = l != 0 || r != 0;
  if (moving && (guardExpired || millis() - sampleAt >= FRESH_MS)) { stopOutputs(); return; }
  setWheelPins(l, true); setWheelPins(r, false);
  const uint32_t saved = __get_PRIMASK(); __disable_irq();
  const uint32_t now = millis();
  const bool allowed = !guardExpired && (!moving || now - sampleAt < FRESH_MS);
  guardArmed = allowed && moving;
  if (guardArmed) { guardSampleAt = sampleAt; guardControlAt = now; }
  digitalWrite(STBY_PIN, allowed ? HIGH : LOW);
  __set_PRIMASK(saved);
  if (!allowed) stopOutputs();
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
  target = locate(reading, now);
  frontHandoff.apply(target, reading);
  frontSingle.apply(target, reading, now);
  holdHysteresis.apply(target);
  sideTurnLimit.apply(target, now);
  command = turnSmoother.update(target, now, dt);
  steerSmooth = turnSmoother.steer;
  leftCommand = rightCommand = Command(); syncCps = 0;
  if (arming && (int32_t)(now - armAt) >= 0) arming = false;
  if (guardExpired && !fault) latchFault(guardExpired == 1 ? "WATCHDOG_CONTROL" : "WATCHDOG_SENSOR");
  if (fault) {
    // Check with motors OFF. Quiet counts do not prove a jam is fixed: this is
    // a bounded retry; the original STALL and direction checks still monitor it.
    const bool healthy = allReady && !paused && !arming && mode == Mode::Follow &&
      !calibration.active && !balJob.active && tickSeen && controlDt <= RECOVERY_MAX_DT_MS &&
      encoderConfigOK(TIM2) && encoderConfigOK(TIM3) && deltaLeft == 0 && deltaRight == 0 &&
      motionState(target.state) && target.index >= 0 && reading[target.index].streak >= 2 &&
      now - target.sampleAt < FRESH_MS;
    const uint32_t frame = target.index >= 0 ? reading[target.index].receivedAt : 0;
    if (recovery.update(fault, healthy, frame, now)) {
      stopOutputs();
      const uint32_t saved = __get_PRIMASK(); __disable_irq();
      guardArmed = false; guardExpired = 0; fault = nullptr;
      __set_PRIMASK(saved);
      resetMeasurements(); // require NEW confirmation; no stale command resumes
    }
    stopOutputs(); return;
  }
  if (!allReady || calibration.active || balJob.active || arming) {
    turnSmoother.reset(); stopOutputs(); return;
  }
  if (mode == Mode::MotorTest) { runMotorJob(now); return; }
  if (paused || mode != Mode::Follow) { turnSmoother.reset(); stopOutputs(); return; }
  const char *lf = wheelLeft.motionFault(deltaLeft, now, controlDt, LEFT_TUNE, "STALL_L", "ENC_DIR_L");
  const char *rf = wheelRight.motionFault(deltaRight, now, controlDt, RIGHT_TUNE, "STALL_R", "ENC_DIR_R");
  if (lf || rf) { latchFault(lf ? lf : rf); return; }
  command.cps = bodySlew.update(command.cps, dt);
  wheelTargets(command, wheelLeft, wheelRight, now, leftCommand, rightCommand, syncCps);
  const int l = wheelLeft.update(leftCommand, LEFT_TUNE, now, dt);
  const int r = wheelRight.update(rightCommand, RIGHT_TUNE, now, dt);
  applyOutputs(l, r, target.sampleAt);  // 0 = phanh chu dong (mat du lieu / xa / cho xac nhan / giu)
}

// ---------------------------------------------------------------- Log
char logBuffer[1536];
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
      "BLOCK=%d,%d LIB16=%d,%d I2C_ERROR=%lu,%lu,%lu,%lu DT_MAX=%lu\r\n",
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
      (unsigned long)maxControlDt);
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
      "SIDE_OK=%u,%u SIDE_VALID=%u,%u SIDE_STATUS=%u,%u SIDE_STREAK=%u,%u SIDE_AGE=%ld,%ld SIDE_LIMIT=%u BAL=%u SYNC_CPS=%d DT_MAX=%lu\r\n",
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
      int(lroundf(syncCps)), (unsigned long)maxControlDt);
  }
  if (EXTRA_LOG_ENABLE && n > 2 && size_t(n) < sizeof(logBuffer)) {
    n -= 2;
    const int extra = snprintf(logBuffer + n, sizeof(logBuffer) - size_t(n),
      " POLL_MAX_MS=%lu,%lu,%lu,%lu J9_SAMPLE_AGE_MS=%ld J10_SAMPLE_AGE_MS=%ld"
      " J9_RX_AGE_MS=%ld J10_RX_AGE_MS=%ld STEER_FILT=%d TURN_OUT_CPS=%d"
      " HOLD_HYST=%u REC_COUNT=%lu REC_WAIT=%u GAP_PAIR=%u BODY_CPS=%d\r\n",
      (unsigned long)maxPollGap[0], (unsigned long)maxPollGap[1],
      (unsigned long)maxPollGap[2], (unsigned long)maxPollGap[3],
      a.valid ? (long)(now - a.sampleAt) : -1L, b.valid ? (long)(now - b.sampleAt) : -1L,
      a.valid ? (long)(now - a.receivedAt) : -1L, b.valid ? (long)(now - b.receivedAt) : -1L,
      int(lroundf(steerSmooth * 100.0f)), int(lroundf(command.turnCps)),
      (unsigned)holdHysteresis.held, (unsigned long)recovery.attempts, (unsigned)recovery.timing,
      (unsigned)target.fusedGap, int(lroundf(command.cps)));
    if (extra > 0 && size_t(extra) < sizeof(logBuffer) - size_t(n)) n += extra;
    else n = 0;
  }
  if (n > 0 && size_t(n) < sizeof(logBuffer)) logSize = size_t(n);
  encWindowLeft = encWindowRight = 0; maxControlDt = 0;
  for (uint8_t i = 0; i < COUNT; ++i) maxPollGap[i] = 0;
}

// ---------------------------------------------------------------- Lenh UART
void printHelp() {
  Serial.println("4 VL53: J8 trai, J9 truoc-trai, J10 truoc-phai, J11 phai.");
  Serial.println("Front: <100 LUI | 100..110 GIU/RE | >110..200 TIEN | >200 DUNG.");
  Serial.println("Only side sees hand 40..200mm: RE TAI CHO max 1.5s, new confirmed acquisition to rearm.");
  Serial.println("BAL 100 | BAL 150: center flat board; SAVE stores BAL and offsets. SHOW displays both.");
  Serial.println("STOP | RUN | SENSOR (chi doc cam bien, khoa dong co)");
  Serial.println("MOTOR L|R|B <pwm -160..160> [ms 200..3000]  vd: MOTOR L 80  (KE BANH LEN truoc)");
  Serial.println("SWEEP F|R  (quet PWM 40..130 ca hai banh, KE BANH LEN truoc)");
  Serial.println("CAL J8 100 | CAL J9 100 | CAL J10 100 | CAL J11 100 | BAL 100 | BAL 150 | SAVE | SHOW | DEFAULT | LOG 0 | LOG 1 | HELP");
}
void enterPause() {
  paused = true; mode = Mode::Follow; motorJob.active = false; balJob.active = false; stopOutputs(); clearLog();
  turnSmoother.reset(); frontSingle.reset(); holdHysteresis.reset(); recovery.resetWindow();
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
  analogWriteResolution(8); stopOutputs();
  for (uint8_t i = 0; i < COUNT; ++i) holdLow(XSHUT[i]);
  Serial.setRx(PA10); Serial.setTx(PA9); Serial.begin(115200, SERIAL_8N1);
  initEncoders(); delay(200);
  if (!encoderConfigOK(TIM2) || !encoderConfigOK(TIM3) ||
      (AFIO->MAPR & (AFIO_MAPR_TIM2_REMAP | AFIO_MAPR_TIM3_REMAP | AFIO_MAPR_TIM4_REMAP))) latchFault("TIMER_CONFIG");
  Serial.println("\nHAND_FOLLOW_HYBRID_V3 GAP=100..110mm TRACK<=200mm CONTROL=10ms RANGE=BYTE_READ");
  Serial.print("J8_DEG=-"); Serial.print(SIDE_LEFT_DEG, 2);
  Serial.print(" J11_DEG=+"); Serial.println(SIDE_RIGHT_DEG, 2);
  Serial.println("COUNTS_PER_MM_EST=10 NOT CALIBRATED. Front PD/PI retained.");
  Serial.println("FRESH=100ms WATCHDOG=100ms GUARD=125ms; verify AGE/DT_MAX on hardware.");
  loadCalibration(); showCalibration();
  allReady = initSensors();
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
    // The tested six-slot order prioritizes longitudinal feedback.
    static const uint8_t order[] = {FRONT_A, FRONT_B, SIDE_L, FRONT_A, FRONT_B, SIDE_R};
    pollSensor(order[slot]);
    slot = (uint8_t)((slot + 1U) % (sizeof(order) / sizeof(order[0])));
  }
  serviceControl(); updateCalibration(); updateBalCalibration();
  digitalWrite(LED_PIN, fault ? ((now / 150) % 2 ? LOW : HIGH) :
    (target.index >= 0 && target.state != State::Far && !paused ? LOW : HIGH));
  pumpLog();
}
#endif