#pragma once
#include <stdint.h>
extern "C" {
#include "nav_bootstrap.h"
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
