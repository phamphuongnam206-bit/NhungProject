#include "start_localization.h"
#include <math.h>

static bool candidateBudgetValid(const NavBootConfig &c) {
  if (c.blocked_count > START_LOCALIZATION_MAX_BLOCKED_RECTS ||
      !isfinite(c.xmin_cm) || !isfinite(c.xmax_cm) ||
      !isfinite(c.ymin_cm) || !isfinite(c.ymax_cm) ||
      !isfinite(c.xy_step_cm) || !isfinite(c.heading_span_rad) ||
      !isfinite(c.heading_step_rad) || c.xmax_cm < c.xmin_cm ||
      c.ymax_cm < c.ymin_cm || c.xy_step_cm <= 0 ||
      c.heading_span_rad < 0 || c.heading_step_rad <= 0) return false;
  const float fx = (c.xmax_cm - c.xmin_cm) / c.xy_step_cm;
  const float fy = (c.ymax_cm - c.ymin_cm) / c.xy_step_cm;
  const float fh = 2.0f * c.heading_span_rad / c.heading_step_rad;
  if (!isfinite(fx) || !isfinite(fy) || !isfinite(fh) ||
      fx > START_LOCALIZATION_MAX_CANDIDATES ||
      fy > START_LOCALIZATION_MAX_CANDIDATES ||
      fh > START_LOCALIZATION_MAX_CANDIDATES) return false;
  // Values above are bounded by 512. Each count is at most 513, so the
  // three-factor product fits uint32_t without bringing in 64-bit helpers.
  const uint32_t nx = uint32_t(fx + 1.0e-5f) + 1U;
  const uint32_t ny = uint32_t(fy + 1.0e-5f) + 1U;
  const uint32_t nh = uint32_t(fh + 1.0e-5f) + 1U;
  return nx * ny * nh <= START_LOCALIZATION_MAX_CANDIDATES;
}

extern "C" NavBootResult StartLocalization_Estimate(
    const StartLocalizationInput *input) {
  NavBootResult rejected{};
  rejected.status = NAV_BOOT_BAD_INPUT;
  if (!input || !input->course_verified || !input->start_region_verified ||
      !input->mounts_verified) return rejected;
  if (!candidateBudgetValid(input->scan)) return rejected;
  if (!input->stationary) {
    rejected.status = NAV_BOOT_NOT_ENOUGH_DATA;
    return rejected;
  }
  uint32_t oldest = 0, newest = UINT32_MAX;
  for (unsigned i = 0; i < 4; ++i) {
    const uint32_t age = input->now_ms - input->sample_ms[i];
    if (!input->ranges[i].valid || !isfinite(input->ranges[i].measured_cm) ||
        age > 100U) {
      rejected.status = NAV_BOOT_NOT_ENOUGH_DATA;
      return rejected;
    }
    if (age > oldest) oldest = age;
    if (age < newest) newest = age;
  }
  if (oldest - newest > 45U) {
    rejected.status = NAV_BOOT_NOT_ENOUGH_DATA;
    return rejected;
  }
  return NavBoot_Estimate(&input->scan, input->ranges, 4U);
}
