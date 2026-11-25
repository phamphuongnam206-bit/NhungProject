"""Compile the actual two motor functions with mocked GPIO/PWM; no hardware.

Usage: python tools/test_hybrid_motor_lock.py --cxx g++
Also accepts --cxx /path/to/zig c++.
"""
import argparse
from pathlib import Path
import subprocess
import tempfile


def function(source, signature):
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cxx", nargs="+", default=["g++"])
    args = parser.parse_args()
    source = (Path(__file__).resolve().parents[1] / "src/hybrid_2laps.cpp").read_text()
    lease_functions = "\n".join(function(source, signature) for signature in (
        'extern "C" void osSystickHandler(void)',
        "void armMotorLease(uint8_t mode)",
    ))
    motor_functions = "\n".join(function(source, signature) for signature in (
        "void stopMotors(bool brake = false)",
        "void driveWheels(float leftCmS, float rightCmS, float dt)",
    ))
    harness = r'''
#include <assert.h>
#include <stdint.h>
enum PinStatus {LOW=0, HIGH=1};
enum {STBY_PIN, LEFT_PWM, RIGHT_PWM, LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2};
int levels[7], pwm[7], highStandbyCalls, nonzeroPwmCalls;
void digitalWrite(int pin, PinStatus value) {
    levels[pin] = value;
    if (pin == STBY_PIN && value == HIGH) ++highStandbyCalls;
}
void analogWrite(int pin, int value) {
    pwm[pin] = value;
    if (value != 0) ++nonzeroPwmCalls;
}
struct Wheel {
    int resetCount = 0, driveCount = 0;
    void reset() { ++resetCount; }
    int drive(float, float, float) { ++driveCount; return 80; }
} wheelL, wheelR;
float MAP_COUNTS_CM_L=45, MAP_COUNTS_CM_R=45;
float commandedSpeed = 10, lastCurve = 1;
uint32_t tickMs = 0;
uint32_t HAL_GetTick() { return tickMs; }
uint32_t __get_PRIMASK() { return 0; }
void __disable_irq() {}
void __enable_irq() {}
struct BrakeRegister {
    void operator=(uint32_t mask) { if (mask & 1U) levels[STBY_PIN] = LOW; }
};
struct MockGPIO { BrakeRegister BRR; } mockGPIO;
MockGPIO *GPIOB = &mockGPIO;
#if !HYBRID_MOTOR_LOCK
constexpr uint32_t DRIVE_STBY_LEASE_MS = 80, BRAKE_STBY_LEASE_MS = 120;
volatile uint32_t motorLeaseAtMs = 0;
volatile uint8_t motorLeaseMode = 0;
volatile uint8_t motorLeaseExpired = 0;
bool brakeEpisodeActive = false;
''' + lease_functions + r'''
#endif
''' + motor_functions + r'''
int main() {
    stopMotors();
    assert(levels[STBY_PIN] == LOW && pwm[LEFT_PWM] == 0 && pwm[RIGHT_PWM] == 0);
    assert(commandedSpeed == 0 && lastCurve == 0);
    driveWheels(10, 12, .01f);
#if HYBRID_MOTOR_LOCK
    stopMotors(true);
    assert(highStandbyCalls == 0 && nonzeroPwmCalls == 0);
    assert(wheelL.driveCount == 0 && wheelR.driveCount == 0);
    assert(levels[STBY_PIN] == LOW);
    assert(levels[LEFT_IN1] == LOW && levels[LEFT_IN2] == LOW);
    assert(levels[RIGHT_IN1] == LOW && levels[RIGHT_IN2] == LOW);
#else
    assert(levels[STBY_PIN] == HIGH && motorLeaseMode == 1U);
    tickMs = 79U; osSystickHandler();
    assert(levels[STBY_PIN] == HIGH);
    tickMs = 80U; osSystickHandler();
    assert(levels[STBY_PIN] == LOW && motorLeaseMode == 0U && motorLeaseExpired == 1U);
    armMotorLease(1U); // late main-loop continuation may not re-enable STBY
    assert(levels[STBY_PIN] == LOW && motorLeaseMode == 0U);
    motorLeaseExpired = 0U; // isolate the brake lease test from drive fault
    tickMs = 100U; stopMotors(true);
    assert(levels[STBY_PIN] == HIGH && motorLeaseMode == 2U);
    tickMs = 219U; osSystickHandler();
    assert(levels[STBY_PIN] == HIGH);
    tickMs = 220U; osSystickHandler();
    assert(levels[STBY_PIN] == LOW && motorLeaseMode == 0U && motorLeaseExpired == 0U);
    stopMotors(true); // repeated brake must not re-arm an expired lease
    assert(levels[STBY_PIN] == LOW && motorLeaseMode == 0U);
    tickMs = 221U; driveWheels(10, 12, .01f);
    stopMotors(true); // new drive allows a new bounded brake episode
    assert(levels[STBY_PIN] == HIGH && motorLeaseMode == 2U);
    tickMs += 120U; osSystickHandler();
    assert(levels[STBY_PIN] == LOW);
    assert(highStandbyCalls == 4 && nonzeroPwmCalls == 4);
    assert(wheelL.driveCount == 2 && wheelR.driveCount == 2);
    assert(levels[LEFT_IN1] == HIGH && levels[LEFT_IN2] == HIGH);
    assert(levels[RIGHT_IN1] == HIGH && levels[RIGHT_IN2] == HIGH);
    stopMotors(false);
    tickMs = UINT32_MAX - 40U;
    driveWheels(10, 12, .01f);
    tickMs = 38U; osSystickHandler();
    assert(levels[STBY_PIN] == HIGH);
    tickMs = 39U; osSystickHandler();
    assert(levels[STBY_PIN] == LOW && motorLeaseExpired == 1U);
#endif
    stopMotors(false);
    assert(levels[STBY_PIN] == LOW && pwm[LEFT_PWM] == 0 && pwm[RIGHT_PWM] == 0);
    assert(wheelL.resetCount >= 3 && wheelR.resetCount >= 3);
}
'''
    with tempfile.TemporaryDirectory(prefix="nhung_motor_lock_") as temporary:
        root = Path(temporary)
        test = root / "motor_io.cpp"
        test.write_text(harness)
        for locked in (0, 1):
            executable = root / f"motor_io_{locked}.exe"
            subprocess.run(args.cxx + ["-std=c++11", "-Wall", "-Wextra", "-Werror",
                           f"-DHYBRID_MOTOR_LOCK={locked}", str(test), "-o", str(executable)],
                           check=True)
            subprocess.run([str(executable)], check=True)
            print(f"PASS actual motor functions: HYBRID_MOTOR_LOCK={locked}")


if __name__ == "__main__":
    main()
