/*
 * Xe test tu hanh rieng cho STM32F103C8T6. Khong dung duong da luu.
 * Goc toa do la goc duoi-trai ban do, truc X sang phai, truc Y huong vao sa ban.
 * Dat trung diem truc hai banh tai START_X_MM, START_Y_MM, mui xe theo +Y.
 * Don trong doan truoc mui xe: xe co the chay toi 50 mm truoc khi du mau VL53.
 * UART PA9/PA10 38400: ? xem trang thai, P dung. De chay lai, dat lai xe va bat nguon.
 *
 * COUNTS_PER_MM va GYRO_SIGN phai khop voi xe that. Gioi han bien duoc tinh tu
 * ban do, encoder va MPU6050. Bon VL53L0X ngang khong doc duoc vach den tren san.
 */
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#ifndef ONE_RUN_CALIBRATED
#define ONE_RUN_CALIBRATED 1
#endif
#if (ONE_RUN_CALIBRATED != 0) && (ONE_RUN_CALIBRATED != 1)
#error "ONE_RUN_CALIBRATED must be 0 or 1"
#endif

namespace AutoTest {

constexpr float PI = 3.14159265359f;
constexpr float DEG = PI / 180.0f;
constexpr float START_X_MM = 300, START_Y_MM = 100;
constexpr float MAP_WIDTH_MM = 1800, MAP_HEIGHT_MM = 1200;
constexpr float BODY_LENGTH_MM = 120, BODY_WIDTH_MM = 100, TRACK_MM = 120;
constexpr float COUNTS_PER_MM_L = 10, COUNTS_PER_MM_R = 10; // x4, gia tri uoc luong tu xe_full
constexpr float GYRO_SIGN = 1;
constexpr float MARGIN_MM = 17, MAX_DRIFT_MARGIN_MM = 24;
constexpr float SLOW_RANGE_MM = 250, FRONT_STOP_MM = 110, FRONT_RESUME_MM = 125;
constexpr float MIN_RANGE_MM = 45, MIN_FORWARD_MM = 80, LOOKAHEAD_MM = 320;
constexpr float AFTER_TURN_STEP_MM = 32, AFTER_TURN_CLEAR_MM = 48;
constexpr float MIN_SPEED_MM_S = 28, MAX_SPEED_MM_S = 55;
constexpr float LAUNCH_MM = 50, LAUNCH_SPEED_MM_S = 35;
constexpr uint32_t LAUNCH_TIMEOUT_MS = 2500;
constexpr uint32_t FRAME_AGE_MS = 100, FRAME_SKEW_MS = 60, CONTROL_MS = 10;
constexpr uint32_t CONTROL_GUARD_MS = 100, SENSOR_GUARD_MS = 125;
constexpr uint32_t TURN_SETTLE_MS = 180, TURN_TIMEOUT_MS = 6500;
constexpr uint8_t SENSOR_COUNT = 4;
enum SensorIndex : uint8_t { J8, J9, J10, J11 };

float clamp(float value, float low, float high) {
    return fmaxf(low, fminf(value, high));
}

float wrap(float angle) {
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}

struct Pose {
    float x = START_X_MM;
    float y = START_Y_MM;
    float heading = PI / 2;
    float margin = MARGIN_MM;
};

struct Range {
    float raw = 0;
    float median = 0;
    float previous = 0;
    float older = 0;
    uint32_t sampledAt = 0;
    uint8_t count = 0;
    uint8_t status = 255;
    uint16_t deviceMm = 0, signalQ7 = 0;
    int32_t rangeRegisterMm = -1;
    const char *problem = "NO_FRAME";
    bool rejected = false;

    void reject(const char *reason) {
        if (rejected) count = 0;
        rejected = true;
        problem = reason;
    }

    void accept(float mm, uint32_t sampled) {
        if (sampled - sampledAt >= FRAME_AGE_MS) count = 0;
        raw = mm;
        median = count >= 2
            ? fmaxf(fminf(previous, older), fminf(mm, fmaxf(previous, older)))
            : mm;
        older = previous;
        previous = mm;
        sampledAt = sampled;
        rejected = false;
        problem = nullptr;
        if (count < 3) ++count;
    }

    float clearance() const { return fminf(raw, median); }
};

struct Scan {
    Range ranges[SENSOR_COUNT];

    bool ready(uint32_t now, uint32_t &oldest) const {
        uint32_t oldestAge = 0, newestAge = FRAME_AGE_MS;
        for (const auto &r : ranges) {
            const uint32_t age = now - r.sampledAt;
            if (r.rejected || r.count < 3 || age >= FRAME_AGE_MS) return false;
            if (age > oldestAge) oldestAge = age;
            if (age < newestAge) newestAge = age;
        }
        oldest = now - oldestAge;
        return oldestAge - newestAge <= FRAME_SKEW_MS;
    }

    float front() const { return fminf(ranges[J9].clearance(), ranges[J10].clearance()); }
    float frontMedian() const { return fminf(ranges[J9].median, ranges[J10].median); }

    float nearest() const {
        float mm = ranges[J8].clearance();
        for (uint8_t i = J9; i <= J11; ++i) mm = fminf(mm, ranges[i].clearance());
        return mm;
    }
};

struct Box { float x, y, halfX, halfY; };

// Cac hop T1-T13 va vung cam giua ban do cua xe_full.
constexpr Box FIXED[] = {
    {900,300,300,300},
    {60,150,50,50}, {550,250,50,50}, {310,350,50,50}, {550,450,50,50},
    {60,550,50,50}, {60,750,50,50}, {360,900,50,50}, {460,1150,50,50},
    {660,750,50,50}, {760,650,50,50}, {860,1000,50,50}, {960,650,50,50},
    {1160,1150,50,50}
};

struct Footprint {
    float u, v, hl, hw, ex, ey;

    Footprint(float heading, float margin)
        : u(cosf(heading)), v(sinf(heading)),
          hl(BODY_LENGTH_MM / 2 + margin), hw(BODY_WIDTH_MM / 2 + margin),
          ex(hl * fabsf(u) + hw * fabsf(v)), ey(hl * fabsf(v) + hw * fabsf(u)) {}

    bool overlaps(float x, float y, const Box &box) const {
        const float dx = box.x - x, dy = box.y - y;
        return fabsf(dx) <= box.halfX + ex && fabsf(dy) <= box.halfY + ey &&
               fabsf(dx * u + dy * v) <= hl + box.halfX * fabsf(u) + box.halfY * fabsf(v) &&
               fabsf(-dx * v + dy * u) <= hw + box.halfX * fabsf(v) + box.halfY * fabsf(u);
    }
};

bool insideMap(float x, float y, const Footprint &shape) {
    return x - shape.ex >= 0 && x + shape.ex <= MAP_WIDTH_MM &&
           y - shape.ey >= 0 && y + shape.ey <= MAP_HEIGHT_MM;
}

bool freeOnMap(float x, float y, const Footprint &shape) {
    if (!insideMap(x, y, shape)) return false;
    for (const auto &box : FIXED) if (shape.overlaps(x, y, box)) return false;
    return true;
}

struct LocalMap {
    Box contacts[SENSOR_COUNT];
    uint8_t count = 0;

    LocalMap(const Pose &pose, const Scan &scan) {
        const float u = cosf(pose.heading), v = sinf(pose.heading);
        constexpr float angles[SENSOR_COUNT] = {35.43f * DEG, 0, 0, -35.43f * DEG};
        for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
            const float mm = scan.ranges[i].clearance();
            if (mm >= 450) continue;
            const float forward = i == J9 || i == J10 ? 60.0f : 45.0f;
            const float side = i < J10 ? 25.0f : -25.0f;
            const float a = pose.heading + angles[i], rayX = cosf(a), rayY = sinf(a);
            const float sx = pose.x + forward * u - side * v + mm * rayX;
            const float sy = pose.y + forward * v + side * u + mm * rayY;
            bool onFixed = false;
            for (const auto &box : FIXED) {
                const float dx = fabsf(sx - box.x) - box.halfX;
                const float dy = fabsf(sy - box.y) - box.halfY;
                if (fabsf(fmaxf(dx, dy)) <= 14) { onFixed = true; break; }
            }
            // Kich thuoc hop thi dau 100 mm. Giua lai ca vi tri mat phan xa.
            if (!onFixed) contacts[count++] = {sx + 45 * rayX, sy + 45 * rayY, 58, 58};
        }
    }

    bool free(float x, float y, const Footprint &shape) const {
        if (!freeOnMap(x, y, shape)) return false;
        for (uint8_t i = 0; i < count; ++i) if (shape.overlaps(x, y, contacts[i])) return false;
        return true;
    }

    float clearAhead(const Pose &pose, float heading, float limit) const {
        // Them 4 mm de bu viec lay mau mot doan thang o moi 8 mm.
        const Footprint shape(heading, pose.margin + 4);
        for (float step = 0; step <= limit; step += 8) {
            if (!free(pose.x + shape.u * step, pose.y + shape.v * step, shape))
                return fmaxf(0, step - 8);
        }
        return limit;
    }

    bool canArc(const Pose &pose, float speed, float yaw, float turn) const {
        constexpr float step = 8;
        float x = pose.x, y = pose.y, heading = pose.heading;
        for (float d = 0; d <= 48; d += step) {
            const Footprint shape(heading, pose.margin + 6);
            if (!free(x, y, shape)) return false;
            const float delta = copysignf(fminf(fabsf(turn), fabsf(yaw) * step / speed), yaw);
            x += step * cosf(heading + delta * 0.5f);
            y += step * sinf(heading + delta * 0.5f);
            heading += delta;
            turn -= delta;
        }
        return true;
    }

    bool canTurn(const Pose &pose, float target) const {
        const float angle = wrap(target - pose.heading);
        const unsigned steps = unsigned(ceilf(fabsf(angle) / (5 * DEG))) + 1;
        for (unsigned i = 0; i <= steps; ++i) {
            const Footprint shape(pose.heading + angle * i / steps, pose.margin + 5);
            if (!free(pose.x, pose.y, shape)) return false;
        }
        return true;
    }
};

struct Motion {
    float speed;
    float yaw;

    Motion(float speedMmS = 0, float yawRadS = 0) : speed(speedMmS), yaw(yawRadS) {}
};

struct Launch {
    bool done = false;
    bool started = false;
    uint32_t startedAt = 0;

    Motion command(const Pose &pose, float distance, const Scan &scan, uint32_t now) {
        if (done) return {};
        if (!started) {
            started = true;
            startedAt = now;
        }
        const float error = wrap(PI / 2 - pose.heading);
        if (distance >= LAUNCH_MM || now - startedAt >= LAUNCH_TIMEOUT_MS ||
            fabsf(error) > 10 * DEG || fabsf(pose.x - START_X_MM) > 12 ||
            pose.y < START_Y_MM - 5) {
            done = true;
            return {};
        }

        for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
            const auto &range = scan.ranges[i];
            const float stopMm = i == J9 || i == J10 ? FRONT_STOP_MM : MIN_RANGE_MM;
            if (range.count && !range.rejected && now - range.sampledAt < FRAME_AGE_MS &&
                range.clearance() < stopMm) {
                done = true;
                return {};
            }
        }
        const Footprint shape(pose.heading, pose.margin + 6);
        for (int step = 0; step <= 16; step += 8) {
            if (!freeOnMap(pose.x + shape.u * step, pose.y + shape.v * step, shape)) {
                done = true;
                return {};
            }
        }
        return {LAUNCH_SPEED_MM_S, clamp(error * 2, -0.16f, 0.16f)};
    }
};

struct Navigator {
    enum class Phase : uint8_t { Forward, Settle, Turn };
    Phase phase = Phase::Forward;
    float straightHeading = PI / 2, turnHeading = 0;
    float stepX = 0, stepY = 0, lastPath = 0;
    uint32_t phaseAt = 0;
    bool frontBlocked = false, stopped = false;
    bool afterTurnStep = false, rollingTurn = false;
    const char *state = "READY";

    void stop(const char *reason) {
        state = reason;
        stopped = true;
    }

    Motion tick(const Pose &pose, const Scan &scan, uint32_t now, uint32_t oldest) {
        if (stopped) return {};
        if (!insideMap(pose.x, pose.y, Footprint(pose.heading, pose.margin)) ||
            pose.margin > MARGIN_MM + MAX_DRIFT_MARGIN_MM) {
            stop("MAP_LIMIT");
            return {};
        }
        if (scan.nearest() < MIN_RANGE_MM) { state = "TOO_CLOSE"; return {}; }
        if (phase == Phase::Settle) {
            // Doi xe ngung troi va ca 4 mau deu duoc do sau khi dung.
            if (now - phaseAt < TURN_SETTLE_MS || now - oldest > now - phaseAt - TURN_SETTLE_MS) {
                state = "SCAN";
                return {};
            }
            straightHeading = pose.heading;
            stepX = pose.x;
            stepY = pose.y;
            afterTurnStep = true;
            phase = Phase::Forward;
        }

        const LocalMap map(pose, scan);
        if (phase == Phase::Turn) {
            const float error = wrap(turnHeading - pose.heading);
            if (fabsf(error) <= 4 * DEG && rollingTurn) {
                straightHeading = pose.heading;
                phase = Phase::Forward;
                rollingTurn = false;
            } else if (fabsf(error) <= 4 * DEG) {
                phase = Phase::Settle;
                phaseAt = now;
                state = "SCAN";
                return {};
            }
            if (phase == Phase::Turn) {
                if (now - phaseAt >= TURN_TIMEOUT_MS) { stop("TURN_TIMEOUT"); return {}; }
                const float arcYaw = copysignf(clamp(fabsf(error) * 2, 0.20f, 0.58f), error);
                const float side = scan.ranges[error > 0 ? J8 : J11].clearance();
                if (rollingTurn && scan.front() >= FRONT_STOP_MM && side >= 100 &&
                    map.canArc(pose, MAX_SPEED_MM_S, arcYaw, error)) {
                    state = error > 0 ? "ARC_LEFT" : "ARC_RIGHT";
                    return {MAX_SPEED_MM_S, arcYaw};
                }
                rollingTurn = false;
                const float overrun = error > 0 ? 4 * DEG : -4 * DEG;
                if (!map.canTurn(pose, wrap(turnHeading + overrun))) { state = "TURN_BLOCKED"; return {}; }
                state = error > 0 ? "TURN_LEFT" : "TURN_RIGHT";
                return {0, copysignf(clamp(fabsf(error) * 1.8f, 0.28f, 0.48f), error)};
            }
        }

        const float yawError = wrap(straightHeading - pose.heading);
        const float path = fminf(map.clearAhead(pose, pose.heading, LOOKAHEAD_MM),
                                 map.clearAhead(pose, straightHeading, LOOKAHEAD_MM));
        lastPath = path;
        const bool wasBlocked = frontBlocked;
        frontBlocked = scan.front() < (wasBlocked ? FRONT_RESUME_MM : FRONT_STOP_MM);
        const float dx = pose.x - stepX, dy = pose.y - stepY;
        if (dx * dx + dy * dy >= AFTER_TURN_STEP_MM * AFTER_TURN_STEP_MM) afterTurnStep = false;
        const float minPath = afterTurnStep ? AFTER_TURN_CLEAR_MM : MIN_FORWARD_MM;
        if (!afterTurnStep && !frontBlocked && (path < 150 || scan.front() < 200)) {
            float best = 0;
            float target = 0, yaw = 0;
            const int angles[] = {25, -25, 45, -45, 65, -65};
            for (const int degrees : angles) {
                const float side = scan.ranges[degrees > 0 ? J8 : J11].clearance();
                if (side < 120) continue;
                const float angle = degrees * DEG;
                const float heading = wrap(pose.heading + angle);
                // Favor the left corridor near the starting gate on this map.
                const float score = fminf(side, map.clearAhead(pose, heading, LOOKAHEAD_MM))
                                  - 0.7f * fabsf(float(degrees)) + (pose.y < 200 && degrees > 0 ? 35 : 0);
                const float rate = copysignf(0.55f, angle);
                if (score <= best || score < 95 || !map.canArc(pose, MAX_SPEED_MM_S, rate, angle)) continue;
                best = score;
                target = heading;
                yaw = rate;
            }
            if (best > 0) {
                turnHeading = target;
                phaseAt = now;
                phase = Phase::Turn;
                rollingTurn = true;
                state = yaw > 0 ? "ARC_LEFT" : "ARC_RIGHT";
                return {MAX_SPEED_MM_S, yaw};
            }
        }
        if (!frontBlocked && path >= minPath && fabsf(yawError) <= 17 * DEG) {
            if (afterTurnStep) {
                state = "STEP_FORWARD";
                return {MIN_SPEED_MM_S, clamp(yawError * 2, -0.05f, 0.05f)};
            }
            const float clearance = fminf(scan.nearest(), path + FRONT_STOP_MM);
            const float speed = clamp(MIN_SPEED_MM_S + (MAX_SPEED_MM_S - MIN_SPEED_MM_S) *
                                (clearance - FRONT_STOP_MM) / (SLOW_RANGE_MM - FRONT_STOP_MM),
                                MIN_SPEED_MM_S, MAX_SPEED_MM_S);
            const float yawLimit = 2 * (speed - 15) / TRACK_MM;
            state = clearance < SLOW_RANGE_MM ? "SLOW" : "FORWARD";
            return {speed, clamp(yawError * 2, -yawLimit, yawLimit)};
        }
        // Mau gan le loi la ly do phanh, chi chon huong neu mau trung vi xac nhan.
        if (frontBlocked && scan.frontMedian() >= FRONT_RESUME_MM && path >= minPath) {
            state = "FRONT_CHECK";
            return {};
        }

        float bestScore = 0;
        float bestHeading = pose.heading;
        constexpr int candidateAngles[] = {10, -10, 20, -20, 30, -30, 45, -45, 65, -65, 90, -90};
        for (const int angle : candidateAngles) {
            const auto side = angle > 0 ? J8 : J11;
            if (scan.ranges[side].clearance() < 120) continue;
            const float target = wrap(pose.heading + angle * DEG);
            const float pathAhead = map.clearAhead(pose, target, LOOKAHEAD_MM);
            if (pathAhead < MIN_FORWARD_MM + 8) continue;
            const float score = fminf(pathAhead, scan.ranges[side].median) - 0.5f * fabsf(float(angle));
            if (score <= bestScore || !map.canTurn(pose, wrap(target + copysignf(4 * DEG, float(angle))))) continue;
            bestScore = score;
            bestHeading = target;
        }
        if (bestScore <= 0) { state = "NO_SAFE_TURN"; return {}; }
        afterTurnStep = false;
        turnHeading = bestHeading;
        phase = Phase::Turn;
        phaseAt = now;
        state = "TURN_READY";
        return {};
    }
};

} // namespace AutoTest

#ifndef XE_AUTO_NATIVE_TEST
#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>

namespace AutoTest {

constexpr uint32_t STBY = PB0;
constexpr uint32_t MOTOR_IN1[] = {PB14, PB12}, MOTOR_IN2[] = {PB15, PB13};
constexpr uint32_t MOTOR_PWM[] = {PB9, PB8};
constexpr uint32_t XSHUT[] = {PA2, PA3, PA4, PA5};
constexpr uint8_t ADDRESS[] = {0x30, 0x31, 0x32, 0x33};
constexpr int ENCODER_SIGN[] = {-1, 1};
constexpr uint32_t RANGE_BUDGET_US = 33000;
constexpr float RANGE_OFFSET_MM[] = {0, -15, -15, 0};

VL53L0X sensors[SENSOR_COUNT];
Scan scan;
Pose pose;
Navigator navigator;
TwoWire imu(PB7, PB6);
uint8_t imuAddress = 0;
uint16_t gyroSamples = 0;
float gyroSum = 0, gyroBias = 0, lastYaw = 0, traveled = 0;
uint16_t lastCount[2];
uint32_t lastControl = 0, armAt = 0;
uint32_t probedAt[SENSOR_COUNT] = {};
bool probeKnown[SENSOR_COUNT] = {}, sensorsReady = false;
volatile bool tickSeen = false, motorArmed = false, guardExpired = false;
volatile bool guardControlLate = false;
const char *firstDriveStop = nullptr;
volatile uint32_t guardControlAt = 0, guardSampleAt = 0;
Launch launch;
#if !ONE_RUN_CALIBRATED
int32_t lockedLeftTicks = 0, lockedRightTicks = 0;
int lockedWho = -1;
#endif

struct Wheel {
    float cps = 0, integral = 0, pwm = 0;
    uint32_t lastPulse = 0, driveSince = 0, stoppedAt = 0;
    int8_t direction = 0;
    int32_t wrongCounts = 0;

    void stop(uint32_t now) {
        if (pwm > 0) stoppedAt = now;
        pwm = integral = 0;
        lastPulse = driveSince = now;
        wrongCounts = 0;
    }

    bool observe(int32_t count, float dt, uint32_t now) {
        cps += dt / (0.06f + dt) * (float(count) / dt - cps);
        if (!pwm) return true;
        if (count) lastPulse = now;
        const int32_t directed = count * direction;
        if (directed > 0) wrongCounts = 0;
        if (directed < 0) wrongCounts -= directed;
        return now - lastPulse < 850 && (now - driveSince < 150 || wrongCounts < 40);
    }

    int drive(float ref, float fullCps, float dt, uint32_t now) {
        const int8_t sign = ref > 0 ? 1 : -1;
        if (pwm && sign != direction) stop(now);
        if (direction && sign != direction && now - stoppedAt < 25) return 0;
        const float error = fabsf(ref) - sign * cps;
        if (!pwm) {
            lastPulse = driveSince = now;
            direction = sign;
        }
        const float boost = 112 * clamp((float(now - lastPulse) - 45) / 110, 0, 1);
        const float lower = fmaxf(42, boost);
        const float base = 42 + (160 - 42) * fabsf(ref) / fullCps + 0.01f * error;
        if ((error > 0 && base + integral < 112) || (error < 0 && base + integral > lower))
            integral = clamp(integral + 0.08f * error * dt, -25, 25);
        const float wanted = sign * cps > 1.4f * fabsf(ref) + 150
            ? 0 : clamp(base + integral, lower, 112);
        const float before = pwm;
        pwm += clamp(wanted - pwm, -850 * dt, 850 * dt);
        if (before > 0 && pwm <= 0) stoppedAt = now;
        return sign * int(lroundf(pwm));
    }
};

Wheel wheels[2];

extern "C" void HAL_SYSTICK_Callback(void) {
    tickSeen = true;
    const uint32_t now = HAL_GetTick();
    if (motorArmed && (now - guardControlAt >= CONTROL_GUARD_MS ||
                       now - guardSampleAt >= SENSOR_GUARD_MS)) {
        guardControlLate = now - guardControlAt >= CONTROL_GUARD_MS;
        GPIOB->BRR = 1U; // PB0/STBY LOW, ke ca neu I2C dang ket.
        motorArmed = false;
        guardExpired = true;
    }
}

void disableMotors() {
    const uint32_t irq = __get_PRIMASK();
    __disable_irq();
    if (motorArmed && !firstDriveStop) firstDriveStop = navigator.state;
    digitalWrite(STBY, LOW);
    motorArmed = false;
    __set_PRIMASK(irq);
    const uint32_t now = millis();
    for (uint8_t i = 0; i < 2; ++i) {
        analogWrite(MOTOR_PWM[i], 0);
        wheels[i].stop(now);
    }
}

void setMotor(uint8_t i, int pwm) {
    const bool in1High = (pwm > 0) != (i == 0);
    digitalWrite(MOTOR_IN1[i], pwm && in1High ? HIGH : LOW);
    digitalWrite(MOTOR_IN2[i], pwm && !in1High ? HIGH : LOW);
    analogWrite(MOTOR_PWM[i], pwm < 0 ? -pwm : pwm);
}

void applyMotors(int left, int right, uint32_t sampledAt) {
    if (millis() - sampledAt >= FRAME_AGE_MS) navigator.state = "SAMPLE_EXPIRED";
    if (guardExpired || (!left && !right) || millis() - sampledAt >= FRAME_AGE_MS) {
        disableMotors();
        return;
    }
    setMotor(0, left);
    setMotor(1, right);
    const uint32_t irq = __get_PRIMASK();
    __disable_irq();
    const uint32_t now = millis();
    motorArmed = !guardExpired && now - sampledAt < FRAME_AGE_MS;
    if (motorArmed) {
        guardSampleAt = sampledAt;
        guardControlAt = now;
    }
    digitalWrite(STBY, motorArmed ? HIGH : LOW);
    __set_PRIMASK(irq);
}

void initEncoder(TIM_TypeDef *timer) {
    timer->CR1 = timer->CR2 = timer->DIER = timer->CCER = timer->PSC = 0;
    timer->ARR = 65535;
    timer->CCMR1 = 1U | (8U << 4) | (1U << 8) | (8U << 12);
    timer->CCMR2 = 0;
    timer->SMCR = 3;
    timer->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    timer->EGR = TIM_EGR_UG;
    timer->CNT = timer->SR = 0;
    timer->CR1 = TIM_CR1_CEN;
}

void initEncoders() {
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
    AFIO->MAPR = (AFIO->MAPR & ~(AFIO_MAPR_TIM2_REMAP | AFIO_MAPR_TIM3_REMAP | AFIO_MAPR_SWJ_CFG)) |
                 AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
    pinMode(PA6, INPUT_PULLUP);
    pinMode(PA7, INPUT_PULLUP);
    pinMode(PA0, INPUT_PULLUP);
    pinMode(PA1, INPUT_PULLUP);
    initEncoder(TIM3);
    initEncoder(TIM2);
    lastCount[0] = uint16_t(TIM3->CNT);
    lastCount[1] = uint16_t(TIM2->CNT);
}

int32_t readCount(uint8_t i, TIM_TypeDef *timer) {
    const uint16_t next = uint16_t(timer->CNT);
    const uint16_t raw = uint16_t(next - lastCount[i]);
    lastCount[i] = next;
    const int32_t delta = raw < 32768 ? int32_t(raw) : int32_t(raw) - 65536;
    return ENCODER_SIGN[i] * delta;
}

bool readRegs(TwoWire &bus, uint8_t address, uint8_t reg, uint8_t *data, uint8_t size) {
    bus.beginTransmission(address);
    bus.write(reg);
    if (bus.endTransmission(false) || bus.requestFrom(address, size) != size) return false;
    for (uint8_t i = 0; i < size; ++i) {
        const int byte = bus.read();
        if (byte < 0) return false;
        data[i] = uint8_t(byte);
    }
    return true;
}

bool writeReg(TwoWire &bus, uint8_t address, uint8_t reg, uint8_t value) {
    bus.beginTransmission(address);
    bus.write(reg);
    bus.write(value);
    return bus.endTransmission() == 0;
}

bool initSensors() {
    Wire.setSCL(PB10);
    Wire.setSDA(PB11);
    Wire.begin();
    Wire.setClock(100000);
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        pinMode(XSHUT[i], INPUT);
        delay(100);
        auto &sensor = sensors[i];
        sensor.setBus(&Wire);
        sensor.setTimeout(100);
        if (!sensor.init()) return false;
        sensor.setAddress(ADDRESS[i]);
        if (sensor.last_status || !sensor.setMeasurementTimingBudget(RANGE_BUDGET_US) ||
            !sensor.setSignalRateLimit(0.25f) || sensor.last_status)
            return false;
    }
    for (auto &sensor : sensors) {
        sensor.startContinuous();
        if (sensor.last_status) return false;
    }
    return true;
}

void pollSensor(uint8_t i) {
    const uint32_t now = millis(), previousProbe = probedAt[i];
    const bool bounded = probeKnown[i] && now - previousProbe <= 65;
    auto &range = scan.ranges[i];
    const auto fail = [&]() {
        range.count = 0;
        range.status = 255;
        range.reject("I2C");
        probeKnown[i] = false;
    };
    uint8_t interrupt = 0;
    if (!readRegs(Wire, ADDRESS[i], VL53L0X::RESULT_INTERRUPT_STATUS, &interrupt, 1)) {
        fail();
        return;
    }
    probedAt[i] = now;
    probeKnown[i] = true;
    if (!(interrupt & 7)) return;

    // Match xe_full: read one extra register so the range bytes are not at
    // the end of the STM32 I2C burst.
    uint8_t result[13] = {};
    const bool read = readRegs(Wire, ADDRESS[i], VL53L0X::RESULT_RANGE_STATUS, result, sizeof(result));
    range.rangeRegisterMm = -1;
    if (read && result[10] == 0 && result[11] == 0) {
        uint8_t bytes[2];
        if (readRegs(Wire, ADDRESS[i], VL53L0X::RESULT_RANGE_STATUS + 10, bytes, sizeof(bytes)))
            range.rangeRegisterMm = (uint16_t(bytes[0]) << 8) | bytes[1];
    }
    const bool cleared = writeReg(Wire, ADDRESS[i], VL53L0X::SYSTEM_INTERRUPT_CLEAR, 1);
    if (!read || !cleared) { fail(); return; }
    const uint16_t mm = (uint16_t(result[10]) << 8) | result[11];
    const uint16_t signal = (uint16_t(result[6]) << 8) | result[7];
    range.deviceMm = mm;
    range.signalQ7 = signal;
    range.status = (result[0] >> 3) & 15;
    // Chi dung ket qua hop le; ma trang thai loi khong duoc coi la duong trong.
    if (!bounded) { range.reject("POLL_GAP"); return; }
    if (range.status != 11) { range.reject("STATUS"); return; }
    if (mm == 0 || mm >= 8190) { range.reject("INVALID_MM"); return; }
    if (signal < 32) { range.reject("LOW_SIGNAL"); return; }
    range.accept(fmaxf(0, mm + RANGE_OFFSET_MM[i]), previousProbe - RANGE_BUDGET_US / 1000);
}

bool initImu() {
    imu.begin();
    imu.setClock(100000);
    for (uint8_t address = 0x68; address <= 0x69; ++address) {
        uint8_t who = 0;
        if (!readRegs(imu, address, 0x75, &who, 1) || who != 0x68) continue;
        if (writeReg(imu, address, 0x6B, 1) && writeReg(imu, address, 0x1B, 0) &&
            writeReg(imu, address, 0x1A, 3) && writeReg(imu, address, 0x19, 9)) {
            imuAddress = address;
            return true;
        }
    }
    return false;
}

bool updatePose(int32_t left, int32_t right, float dt) {
    uint8_t bytes[2];
    if (!readRegs(imu, imuAddress, 0x47, bytes, 2)) {
        if (gyroSamples >= 250) navigator.stop("IMU_READ");
        else { gyroSamples = 0; gyroSum = 0; }
        return false;
    }
    const float raw = int16_t((uint16_t(bytes[0]) << 8) | uint16_t(bytes[1])) / 131.0f;
    if (gyroSamples < 250) {
        if (left || right || fabsf(raw) > 5) { gyroSamples = 0; gyroSum = 0; }
        else {
            gyroSum += raw;
            if (++gyroSamples == 250) gyroBias = gyroSum / 250;
        }
        navigator.state = "GYRO_KEEP_STILL";
        return false;
    }
    const float dl = left / COUNTS_PER_MM_L, dr = right / COUNTS_PER_MM_R;
    const float distance = (dl + dr) * 0.5f;
    const float corrected = raw - gyroBias;
    const bool still = !motorArmed && !left && !right && fabsf(corrected) < 0.35f;
    if (still) gyroBias += 0.005f * corrected;
    const float yaw = still ? 0 : corrected * GYRO_SIGN * DEG;
    const float dh = (yaw + lastYaw) * 0.5f * dt;
    lastYaw = yaw;
    if (fabsf(dh - (dr - dl) / TRACK_MM) > 0.16f) {
        navigator.stop("ODOM_MISMATCH");
        return false;
    }
    pose.x += distance * cosf(pose.heading + dh * 0.5f);
    pose.y += distance * sinf(pose.heading + dh * 0.5f);
    pose.heading = wrap(pose.heading + dh);
    traveled += fabsf(distance);
    pose.margin = MARGIN_MM + 0.009f * traveled;
    return true;
}

void control(uint32_t now) {
    const float dt = (now - lastControl) * 0.001f;
    lastControl = now;
    const int32_t left = readCount(0, TIM3), right = readCount(1, TIM2);
    if (guardExpired) {
        navigator.stop(guardControlLate ? "WATCH_CONTROL" : "WATCH_SAMPLE");
        if (!firstDriveStop) firstDriveStop = navigator.state;
    }
    if (navigator.stopped) { disableMotors(); return; }
    if (!updatePose(left, right, dt)) { disableMotors(); return; }
    if (int32_t(now - armAt) < 0) { disableMotors(); return; }
    uint32_t oldest = now;
    Motion command = launch.command(pose, traveled, scan, now);
    if (command.speed) {
        navigator.state = "LAUNCH";
    } else {
        if (!scan.ready(now, oldest)) {
            navigator.state = "WAIT_RANGES";
            disableMotors();
            return;
        }
        command = navigator.tick(pose, scan, now, oldest);
    }
    const bool leftOK = wheels[0].observe(left, dt, now), rightOK = wheels[1].observe(right, dt, now);
    if (!leftOK || !rightOK) navigator.stop(leftOK ? "WHEEL_R" : "WHEEL_L");
    if (navigator.stopped || (!command.speed && !command.yaw)) { disableMotors(); return; }
    const float turn = command.yaw * TRACK_MM * 0.5f;
    const int pwmL = wheels[0].drive((command.speed - turn) * COUNTS_PER_MM_L, 5666, dt, now);
    const int pwmR = wheels[1].drive((command.speed + turn) * COUNTS_PER_MM_R, 5563, dt, now);
    applyMotors(pwmL, pwmR, oldest);
}

void printStatus() {
    const uint32_t now = millis();
    Serial.print(navigator.state);
    Serial.print(" J8..11=");
    for (const auto &range : scan.ranges) {
        Serial.print(!range.rejected && range.count && now - range.sampledAt < FRAME_AGE_MS ? int(range.raw) : -1);
        Serial.print('/');
        Serial.print(range.status);
        Serial.print(' ');
    }
    Serial.print(" X="); Serial.print(int(lroundf(pose.x)));
    Serial.print(" Y="); Serial.print(int(lroundf(pose.y)));
    Serial.print(" H="); Serial.print(int(lroundf(pose.heading / DEG)));
    Serial.print(" PATH="); Serial.print(int(navigator.lastPath));
    Serial.print(" FIRST_STOP="); Serial.println(firstDriveStop ? firstDriveStop : "-");
    uint32_t oldest;
    if (motorArmed || scan.ready(now, oldest)) return;
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        const auto &range = scan.ranges[i];
        Serial.print('J'); Serial.print(8 + i);
        Serial.print(" RAW="); Serial.print(range.deviceMm);
        Serial.print(" REG="); Serial.print(range.rangeRegisterMm);
        Serial.print(" SIG="); Serial.print(range.signalQ7);
        Serial.print(" N="); Serial.print(range.count);
        Serial.print(' ');
        Serial.println(range.problem ? range.problem :
                       now - range.sampledAt >= FRAME_AGE_MS ? "STALE" :
                       range.count < 3 ? "COLLECTING" : "OK");
    }
}

#if !ONE_RUN_CALIBRATED
void zeroLockedEncoders() {
    lastCount[0] = uint16_t(TIM3->CNT);
    lastCount[1] = uint16_t(TIM2->CNT);
    lockedLeftTicks = lockedRightTicks = 0;
}

void printLockedTelemetry() {
    const uint32_t now = millis();
    Serial.print("ONE_RUN_LOCKED TIM3_L="); Serial.print(lockedLeftTicks);
    Serial.print(" TIM2_R="); Serial.print(lockedRightTicks);
    Serial.print(" MPU_WHO=");
    if (lockedWho < 0) Serial.print("NA");
    else { Serial.print("0x"); Serial.print(lockedWho, HEX); }
    Serial.print(" VL_INIT="); Serial.print(sensorsReady ? "OK" : "FAIL");
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        const auto &range = scan.ranges[i];
        Serial.print(" J"); Serial.print(8 + i); Serial.print('=');
        const bool fresh = sensorsReady && !range.rejected && range.count &&
                           now - range.sampledAt < FRAME_AGE_MS;
        Serial.print(fresh ? int(range.raw) : -1);
        Serial.print('/');
        Serial.print(!sensorsReady ? "INIT_FAIL" :
                     range.problem ? range.problem : fresh ? "OK" : "STALE");
    }
    Serial.println();
}
#endif

} // namespace AutoTest

void setup() {
    using namespace AutoTest;
    digitalWrite(STBY, LOW);
    pinMode(STBY, OUTPUT);
    digitalWrite(PC13, HIGH);
    pinMode(PC13, OUTPUT);
    for (uint8_t i = 0; i < 2; ++i) {
        pinMode(MOTOR_IN1[i], OUTPUT);
        pinMode(MOTOR_IN2[i], OUTPUT);
        pinMode(MOTOR_PWM[i], OUTPUT);
        setMotor(i, 0);
    }
    analogWriteResolution(8);
    for (const auto pin : XSHUT) {
        digitalWrite(pin, LOW);
        pinMode(pin, OUTPUT);
    }
    Serial.setRx(PA10);
    Serial.setTx(PA9);
    Serial.begin(38400, SERIAL_8N1);
#if !ONE_RUN_CALIBRATED
    disableMotors();
    initEncoders();
    imu.begin();
    imu.setClock(100000);
    for (uint8_t address = 0x68; address <= 0x69; ++address) {
        uint8_t who = 0;
        if (readRegs(imu, address, 0x75, &who, 1)) {
            lockedWho = who;
            break;
        }
    }
    sensorsReady = initSensors();
    Serial.println("ONE_RUN_LOCKED TELEMETRY ?=status Z=zero_ticks");
    printLockedTelemetry();
    return;
#endif
    initEncoders();
    sensorsReady = initSensors();
    if (!sensorsReady) navigator.stop("VL53_INIT");
    else if (!initImu()) navigator.stop("IMU_INIT");
    else if (!tickSeen || (AFIO->MAPR & (AFIO_MAPR_TIM2_REMAP | AFIO_MAPR_TIM3_REMAP | AFIO_MAPR_TIM4_REMAP)))
        navigator.stop("MCU_INIT");
    Serial.println("XE_AUTO_TEST ?=status P=stop");
    lastControl = millis();
    armAt = lastControl + 3000;
}

void loop() {
    using namespace AutoTest;
#if !ONE_RUN_CALIBRATED
    {
        GPIOB->BRR = 1U; // PB0/STBY stays LOW; motor-control code is unreachable.
        const uint32_t now = millis();
        static uint32_t lastEncoderSample = 0, lastPoll = 0, lastReport = 0;
        if (now - lastEncoderSample >= 20) {
            lastEncoderSample = now;
            lockedLeftTicks += readCount(0, TIM3);
            lockedRightTicks += readCount(1, TIM2);
        }
        static uint8_t nextSensor = 0;
        if (sensorsReady && now - lastPoll >= 2) {
            lastPoll = now;
            pollSensor(nextSensor);
            nextSensor = (nextSensor + 1) % SENSOR_COUNT;
        }
        bool report = false;
        for (uint8_t i = 0; i < 8 && Serial.available(); ++i) {
            const int command = Serial.read();
            if (command == '?') report = true;
            if (command == 'Z' || command == 'z') {
                zeroLockedEncoders();
                Serial.println("ENC_ZERO");
                report = true;
            }
        }
        if (report || now - lastReport >= 1000) {
            lastReport = now;
            printLockedTelemetry();
        }
        return;
    }
#endif
    for (uint8_t i = 0; i < 8 && Serial.available(); ++i) {
        const int command = Serial.read();
        if (command == 'P' || command == '0') { navigator.stop("PAUSED"); disableMotors(); }
        if (command == '?') printStatus();
    }
    const uint32_t now = millis();
    if (now - lastControl >= CONTROL_MS) control(now);
    static uint8_t nextSensor = 0;
    static uint32_t lastPoll = 0;
    if (sensorsReady && now - lastPoll >= 2) {
        lastPoll = now;
        pollSensor(nextSensor);
        nextSensor = (nextSensor + 1) % SENSOR_COUNT;
    }
    digitalWrite(PC13, navigator.stopped ? (now / 150 & 1 ? HIGH : LOW) : (motorArmed ? LOW : HIGH));
}
#endif
