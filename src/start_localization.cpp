#include "start_localization.h"
#include <math.h>

extern "C" NavBootResult StartLocalization_Estimate(
    const StartLocalizationInput *input) {
  NavBootResult rejected{};
  rejected.status = NAV_BOOT_BAD_INPUT;
  if (!input || !input->course_verified || !input->start_region_verified ||
      !input->mounts_verified) return rejected;
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
