"""Host check of the SWD diagnostic's timer wrap and measured-distance gate."""

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
    source = (Path(__file__).resolve().parents[1] / "src/swd_quick_encoder_mpu.cpp").read_text()
    extracted = "\n".join(function(source, sig) for sig in (
        "static int32_t signedCounterDiff(uint16_t current, uint16_t previous)",
        "static void updateMeasuredScale()",
    ))
    harness = r'''
#include <assert.h>
#include <stdint.h>
#include <math.h>
struct Result {
    uint32_t straight_done;
    int32_t straight_left, straight_right;
    float measured_distance_cm, left_counts_per_cm, right_counts_per_cm;
};
volatile Result swd_quick = {};
''' + extracted + r'''
int main() {
    assert(signedCounterDiff(2U, 65534U) == 4);
    assert(signedCounterDiff(65534U, 2U) == -4);
    assert(signedCounterDiff(100U, 101U) == -1);
    swd_quick.straight_done = 1;
    swd_quick.straight_left = 600;
    swd_quick.straight_right = -570;
    swd_quick.measured_distance_cm = 20.0f;
    updateMeasuredScale();
    assert(swd_quick.left_counts_per_cm == 30.0f);
    assert(swd_quick.right_counts_per_cm == 28.5f);
    swd_quick.measured_distance_cm = 9.0f;
    updateMeasuredScale();
    assert(swd_quick.left_counts_per_cm == 0.0f);
    swd_quick.measured_distance_cm = 31.0f;
    updateMeasuredScale();
    assert(swd_quick.right_counts_per_cm == 0.0f);
    swd_quick.measured_distance_cm = NAN;
    updateMeasuredScale();
    assert(swd_quick.left_counts_per_cm == 0.0f);
}
'''
    with tempfile.TemporaryDirectory(prefix="swd_quick_math_") as folder:
        cpp = Path(folder) / "test.cpp"
        exe = Path(folder) / "test.exe"
        cpp.write_text(harness, encoding="utf-8")
        subprocess.run(args.cxx + ["-std=c++11", "-Wall", "-Wextra", "-Werror",
                                  str(cpp), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
        print("PASS SWD encoder wrap and 10-30 cm scale gate")


if __name__ == "__main__":
    main()
