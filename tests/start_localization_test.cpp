#include "start_localization.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main() {
  assert(StartLocalization_FrameDue(1000U, 0U));
  assert(!StartLocalization_FrameDue(1000U, 1000U));
  assert(!StartLocalization_FrameDue(1499U, 1000U));
  assert(StartLocalization_FrameDue(1500U, 1000U));
  assert(StartLocalization_FrameDue(300U, UINT32_MAX - 200U));
  assert(!StartLocalization_FrameDue(0U, UINT32_MAX - 200U));
  const NavBootRect block[] = {{68.0f, 55.0f, 8.0f, 8.0f}};
  StartLocalizationInput input{};
  auto &c = input.scan;
  c.arena_width_cm = c.arena_height_cm = 100.0f;
  c.arena_edges_reflect = 1U; // synthetic fixture has reflective walls
  c.blocked = block; c.blocked_count = 1;
  c.robot_length_cm = 15.0f; c.robot_width_cm = 10.0f;
  c.body_margin_cm = 0.5f;
  c.xmin_cm = 20.0f; c.xmax_cm = 85.0f;
  c.ymin_cm = 15.0f; c.ymax_cm = 35.0f;
  c.xy_step_cm = 5.0f;
  c.heading_center_rad = 1.57079632679f;
  c.heading_span_rad = 0.261799388f;
  c.heading_step_rad = 0.087266463f;
  c.max_mean_error_cm = 1.0f; c.ambiguity_slack_cm = 0.5f;
  c.ambiguity_distance_cm = 7.0f; c.ambiguity_angle_rad = 0.13962634f;
  const NavBootPose actual = {70.0f, 20.0f, 1.57079632679f};
  const float yaw[] = {0.0f, -1.57079632679f,
                       1.57079632679f, -0.7853981634f};
  input.now_ms = 1000U;
  for (unsigned i = 0; i < 4; ++i) {
    input.ranges[i] = {0, 0, yaw[i], 0, 120, 0, 1};
    input.ranges[i].measured_cm = NavBoot_PredictRange(&c, &actual,
                                                       &input.ranges[i]);
    input.sample_ms[i] = 990U + i;
  }
  input.stationary = true;
  assert(StartLocalization_Estimate(&input).status == NAV_BOOT_BAD_INPUT);
  input.course_verified = input.start_region_verified =
      input.mounts_verified = true;
  assert(StartLocalization_Estimate(&input).status == NAV_BOOT_CONFIDENT);
  input.stationary = false;
  assert(StartLocalization_Estimate(&input).status == NAV_BOOT_NOT_ENOUGH_DATA);
  input.stationary = true;
  input.sample_ms[0] = 800U;
  assert(StartLocalization_Estimate(&input).status == NAV_BOOT_NOT_ENOUGH_DATA);
  input.sample_ms[0] = 990U;
  input.ranges[0].valid = 0;
  assert(StartLocalization_Estimate(&input).status == NAV_BOOT_NOT_ENOUGH_DATA);
  input.ranges[0].valid = 1;
  const NavBootConfig saved = c;
  c.xmin_cm = 20.0f; c.xmax_cm = 71.1f; c.xy_step_cm = 0.1f;
  c.ymin_cm = c.ymax_cm = 20.0f; c.heading_span_rad = 0.0f;
  assert(StartLocalization_Estimate(&input).status != NAV_BOOT_BAD_INPUT);
  c.xmax_cm = 71.3f;
  assert(StartLocalization_Estimate(&input).status == NAV_BOOT_BAD_INPUT);
  c = saved;
  assert(StartLocalization_Estimate(&input).status == NAV_BOOT_CONFIDENT);
  c.blocked_count = 19U; // rejected before the one-element test pointer is read
  assert(StartLocalization_Estimate(&input).status == NAV_BOOT_BAD_INPUT);
  puts("PASS START adapter: verified, fresh, four ranges and bounded scan");
}
