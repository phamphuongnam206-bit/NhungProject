"""Run the firmware's VL53 pause/recovery function against mocked board I/O."""

import argparse
from pathlib import Path
import subprocess
import tempfile


def extract_function(source, signature):
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
    actual = extract_function(source, "bool rangesPermitMotion(uint32_t now, bool fresh)")
    harness = r'''
#include <assert.h>
#include <stdint.h>
enum class RunMode { WAIT_SENSORS, AUTO_RUN, LOCKED };
enum class Fault { SENSOR };
enum { LED_PIN };
enum { HIGH };
RunMode mode = RunMode::WAIT_SENSORS;
uint32_t autoStartMs = 0, frameAt = 0, lastPlanMs = 0, clockMs = 0;
uint8_t rangeFaultMask = 0, rangeRestarts = 0;
const uint32_t SENSOR_REINIT_DELAY_MS = 250, SENSOR_RECOVERY_LIMIT_MS = 500;
const uint8_t SENSOR_REINIT_LIMIT = 2, SENSOR_RECOVERY_FRAMES = 2;
int brakeCalls = 0, initCalls = 0, haltCalls = 0;
bool stopped = false, initOk = true;
const char *status = nullptr;
uint8_t staleRangeMask(uint32_t) { return 0x04; }
bool wheelsSettled() { return stopped; }
bool initArcSensors(bool restart) { assert(restart); ++initCalls; return initOk; }
void rememberStop(const char *) {}
void stopMotors(bool brake) { assert(brake); ++brakeCalls; }
void digitalWrite(int, int) {}
void halt(const char *, Fault) { ++haltCalls; mode = RunMode::LOCKED; }
uint32_t millis() { return clockMs; }
''' + actual + r'''
int main() {
    assert(!rangesPermitMotion(849, false));
    assert(haltCalls == 0);
    assert(!rangesPermitMotion(850, false));
    assert(haltCalls == 1 && mode == RunMode::LOCKED);

    // A separate boot epoch models a fresh run; the function was not paused
    // while WAIT_SENSORS was active.
    mode = RunMode::AUTO_RUN;
    assert(rangesPermitMotion(1000, true));
    assert(!rangesPermitMotion(1010, false));
    assert(brakeCalls == 1 && initCalls == 0);
    frameAt = 1020;
    assert(!rangesPermitMotion(1020, true)); // wheels still moving
    stopped = true;
    assert(!rangesPermitMotion(1021, true)); // duplicate frame
    frameAt = 1030;
    assert(rangesPermitMotion(1030, true)); // two distinct frames
    assert(lastPlanMs == 1000);

    assert(!rangesPermitMotion(1100, false));
    clockMs = 1350;
    assert(!rangesPermitMotion(1350, false));
    assert(initCalls == 1 && rangeRestarts == 1 && brakeCalls == 2);
    frameAt = 1360;
    assert(!rangesPermitMotion(1360, true));
    frameAt = 1370;
    assert(rangesPermitMotion(1370, true));

    // Without a usable range frame the function latches a sensor fault.
    assert(!rangesPermitMotion(1500, false));
    stopped = false; // no restart while wheels appear to move
    assert(!rangesPermitMotion(1999, false));
    assert(initCalls == 1);
    assert(!rangesPermitMotion(2000, false));
    assert(haltCalls == 2 && mode == RunMode::LOCKED);
}
'''
    with tempfile.TemporaryDirectory(prefix="nhung_range_recovery_") as folder:
        path = Path(folder)
        cpp = path / "range_recovery.cpp"
        exe = path / "range_recovery.exe"
        cpp.write_text(harness, encoding="utf-8")
        subprocess.run(args.cxx + ["-std=c++11", "-Wall", "-Wextra", "-Werror",
                                  "-DHYBRID_COMPACT_DEBUG=1",
                                  str(cpp), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
        print("PASS firmware VL53 pause, restart, fresh-frame recovery and timeout")


if __name__ == "__main__":
    main()
