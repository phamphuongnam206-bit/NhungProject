#pragma once
#include <stdint.h>
extern "C" {
#include "nav_bootstrap.h"
}

// Conservative qualification guard; re-evaluate after measuring ARM WCET.
constexpr uint32_t START_LOCALIZATION_MAX_CANDIDATES = 512U;
constexpr uint16_t START_LOCALIZATION_MAX_BLOCKED_RECTS = 18U;
constexpr uint32_t START_LOCALIZATION_RETRY_MS = 500U;

inline bool StartLocalization_FrameDue(uint32_t frame_ms,
                                      uint32_t last_attempt_frame_ms) {
  return frame_ms != 0U &&
      (last_attempt_frame_ms == 0U ||
       uint32_t(frame_ms - last_attempt_frame_ms) >=
           START_LOCALIZATION_RETRY_MS);
}

// Configuration is supplied by the caller from measured course and mount data.
// No default START position or guessed sensor angle can authorize a pose.
struct StartLocalizationInput {
  NavBootConfig scan;
  NavBootRange ranges[4];
  uint32_t sample_ms[4];
  uint32_t now_ms;
  bool course_verified;
  bool start_region_verified;
  bool mounts_verified;
  bool stationary;
};

extern "C" NavBootResult StartLocalization_Estimate(
    const StartLocalizationInput *input);
