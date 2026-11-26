#pragma once

#include <stdint.h>

namespace V1Mpu {

// Profile used on the assembled robot by the hybrid and SWD builds.
constexpr uint8_t filter = 0x03, sampleDivider = 0x09, gyroConfig = 0x08;
constexpr float gyroLsbPerDps = 65.5f;
constexpr unsigned biasSamples = 50;
constexpr unsigned samplePeriodMs = 10;

// Adapt the teammate driver's Z-axis bias checks to this full scale.
// A stable mean is committed only after all samples succeed.
struct StationaryBias {
  int32_t sum = 0;
  int16_t low = 0, high = 0;
  unsigned count = 0;

  bool add(int16_t raw) {
    if (count == 0) low = high = raw;
    if (raw < low) low = raw;
    if (raw > high) high = raw;
    if (raw < -10.0f * gyroLsbPerDps || raw > 10.0f * gyroLsbPerDps ||
        int32_t(high) - low > 2.0f * gyroLsbPerDps) return false;
    sum += raw;
    ++count;
    return true;
  }

  float mean() const { return float(sum) / float(count); }
};

} // namespace V1Mpu
