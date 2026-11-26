"""Host-check actual hybrid MPU init/readback with an I2C register mock."""

import argparse
from pathlib import Path
import subprocess
import tempfile

from test_hybrid_motor_lock import function


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cxx", nargs="+", required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    source = (root / "src/hybrid_2laps.cpp").read_text()
    functions = "\n".join(function(source, signature) for signature in (
        "bool writeMpuReg(uint8_t r, uint8_t v)",
        "bool readMpuReg(uint8_t r, uint8_t &value)",
        "bool initGyro()",
    ))
    harness = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "v1_mpu_profile.h"
enum { PB6, PB7, MPU_ADDR = 0x68 };
struct MockWire {
  uint8_t regs[256] = {};
  uint8_t reg = 0;
  unsigned writes = 0;
  bool bad_range_readback = false, fail_read = false;
  void setSDA(int) {} void setSCL(int) {} void begin() {} void setClock(int) {}
  void beginTransmission(uint8_t) { writes = 0; }
  void write(uint8_t v) {
    if (!writes) reg = v;
    else regs[reg++] = v;
    ++writes;
  }
  int endTransmission(bool = true) { return 0; }
  int requestFrom(uint8_t, uint8_t count) { return fail_read ? 0 : count; }
  int read() { return bad_range_readback && reg == 0x1B ? 0x10 : regs[reg]; }
} Wire;
float gyroBias = 0, currentYawDeg = 0;
uint32_t lastGyroGoodMs = 0, lastGyroMs = 0;
unsigned rawReads = 0;
int16_t secondSample = 100;
void delay(unsigned) {}
uint32_t millis() { return 123U; }
bool readGyroRawZ(int16_t &raw) { raw = ++rawReads == 2 ? secondSample : 100; return true; }
''' + functions + r'''
int main() {
  assert(initGyro());
  assert(Wire.regs[0x1A] == 0x03 && Wire.regs[0x19] == 0x09);
  assert(Wire.regs[0x1B] == 0x08 && gyroBias == 100.0f);
  assert(rawReads == 50U && lastGyroGoodMs == 123U);
  rawReads = 0; Wire.bad_range_readback = true;
  assert(!initGyro() && rawReads == 0U);
  Wire.bad_range_readback = false; Wire.fail_read = true;
  assert(!initGyro() && rawReads == 0U);
  Wire.fail_read = false; secondSample = 240;
  assert(!initGyro() && rawReads == 2U && gyroBias == 100.0f);
  puts("PASS MPU config readback and stationary bias rejection");
}
'''
    with tempfile.TemporaryDirectory(prefix="nhung_mpu_config_") as folder:
        path = Path(folder)
        cpp = path / "mpu_config.cpp"
        exe = path / "mpu_config.exe"
        cpp.write_text(harness, encoding="utf-8")
        subprocess.run(args.cxx + ["-std=c++11", "-Wall", "-Wextra", "-Werror",
                                   "-I", str(root / "include"), str(cpp), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
