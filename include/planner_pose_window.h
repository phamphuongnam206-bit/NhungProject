#pragma once
#include <math.h>
#include <stdint.h>

struct PlannerPoseSample {
  float x_cm, y_cm, yaw_rad;
  uint32_t at_ms;
};

// A VL53 result does not report its exposure time. A read-time pose is usable
// only if the preceding 100 ms of sampled motion stayed inside this envelope.
struct PlannerPoseWindow {
  PlannerPoseSample samples[16] = {};
  uint8_t next = 0, count = 0;

  void record(const PlannerPoseSample &sample) {
    samples[next] = sample;
    next = (next + 1U) % 16U;
    if (count < 16U) ++count;
  }

  // Interpolate only between actual adjacent control samples. The caller must
  // supply a measurement timestamp; a VL53 register-read time is not one.
  bool interpolateAt(uint32_t measurement_ms, PlannerPoseSample *origin) const {
    if (!origin || !count) return false;
    const uint8_t oldest = (next + 16U - count) % 16U;
    const PlannerPoseSample &first = samples[oldest];
    if (measurement_ms == first.at_ms && isfinite(first.x_cm) &&
        isfinite(first.y_cm) && isfinite(first.yaw_rad)) {
      *origin = first;
      return true;
    }
    for (uint8_t i = 1; i < count; ++i) {
      const PlannerPoseSample &before = samples[(oldest + i - 1U) % 16U];
      const PlannerPoseSample &after = samples[(oldest + i) % 16U];
      const uint32_t span = after.at_ms - before.at_ms;
      if (!span || span > 25U) return false;
      const uint32_t offset = measurement_ms - before.at_ms;
      if (offset > span) continue;
      if (!isfinite(before.x_cm) || !isfinite(before.y_cm) ||
          !isfinite(before.yaw_rad) || !isfinite(after.x_cm) ||
          !isfinite(after.y_cm) || !isfinite(after.yaw_rad)) return false;
      const float fraction = float(offset) / float(span);
      const float yaw_delta = remainderf(after.yaw_rad - before.yaw_rad,
                                         6.28318530718f);
      *origin = {
        before.x_cm + fraction * (after.x_cm - before.x_cm),
        before.y_cm + fraction * (after.y_cm - before.y_cm),
        remainderf(before.yaw_rad + fraction * yaw_delta, 6.28318530718f),
        measurement_ms,
      };
      return true;
    }
    return false;
  }

  bool stableOrigin(uint32_t now_ms, PlannerPoseSample *origin) const {
    if (!origin || !count) return false;
    const PlannerPoseSample &latest = samples[(next + 15U) % 16U];
    const uint32_t newest_age = now_ms - latest.at_ms;
    if (newest_age > 25U) return false;
    uint32_t previous_age = newest_age;
    for (uint8_t n = 0; n < count; ++n) {
      const PlannerPoseSample &sample = samples[(next + 15U - n) % 16U];
      const uint32_t age = now_ms - sample.at_ms;
      if (age < previous_age || age - previous_age > 25U ||
          !isfinite(sample.x_cm) || !isfinite(sample.y_cm) ||
          !isfinite(sample.yaw_rad) ||
          fabsf(sample.x_cm - latest.x_cm) > 0.5f ||
          fabsf(sample.y_cm - latest.y_cm) > 0.5f ||
          fabsf(remainderf(sample.yaw_rad - latest.yaw_rad,
                            6.28318530718f)) > 0.034906585f) return false;
      if (age >= 100U) {
        *origin = latest;
        return true;
      }
      previous_age = age;
    }
    return false;
  }
};
