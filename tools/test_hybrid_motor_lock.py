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
    actual = "\n".join(function(source, signature) for signature in (
        "void stopMotors(bool brake = false)",
        "void driveWheels(float leftCmS, float rightCmS, float dt)",
    ))
    harness = r'''
#include <assert.h>
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
''' + actual + r'''
int main() {
    stopMotors();
    assert(levels[STBY_PIN] == LOW && pwm[LEFT_PWM] == 0 && pwm[RIGHT_PWM] == 0);
    assert(commandedSpeed == 0 && lastCurve == 0);
    driveWheels(10, 12, .01f);
    stopMotors(true); // Braking used to re-enable STBY, so test it explicitly.
#if HYBRID_MOTOR_LOCK
    assert(highStandbyCalls == 0 && nonzeroPwmCalls == 0);
    assert(wheelL.driveCount == 0 && wheelR.driveCount == 0);
    assert(levels[STBY_PIN] == LOW);
    assert(levels[LEFT_IN1] == LOW && levels[LEFT_IN2] == LOW);
    assert(levels[RIGHT_IN1] == LOW && levels[RIGHT_IN2] == LOW);
#else
    assert(highStandbyCalls == 2 && nonzeroPwmCalls == 2);
    assert(wheelL.driveCount == 1 && wheelR.driveCount == 1);
    assert(levels[STBY_PIN] == HIGH);
    assert(levels[LEFT_IN1] == HIGH && levels[LEFT_IN2] == HIGH);
    assert(levels[RIGHT_IN1] == HIGH && levels[RIGHT_IN2] == HIGH);
#endif
    stopMotors(false);
    assert(levels[STBY_PIN] == LOW && pwm[LEFT_PWM] == 0 && pwm[RIGHT_PWM] == 0);
    assert(wheelL.resetCount == 3 && wheelR.resetCount == 3);
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
