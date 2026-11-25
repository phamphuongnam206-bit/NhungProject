#include "planner_motion_health.h"
#include <assert.h>
#include <stdio.h>

static PlannerHealthConfig config() {
  PlannerHealthConfig c{};
  c.verified = true; c.max_cross_track_cm = 5.0f;
  c.min_command_cm_s = 2.0f; c.min_progress_cm = 1.0f;
  c.max_yaw_delta_error_rad = 0.2f; c.track_cm = 10.0f;
  c.no_progress_ms = 500U; c.stall_ms = 300U; c.yaw_error_ms = 200U;
  return c;
}
static PlannerHealthSample sample(uint32_t t) {
  PlannerHealthSample s{};
  s.now_ms = t; s.cross_track_cm = 1.0f;
  s.route_progress_cm = 10.0f; s.command_cm_s = 5.0f;
  s.left_cm = s.right_cm = 0.2f; s.route_active = true;
  return s;
}
int main() {
  PlannerMotionHealth h{};
  auto c = config(); auto s = sample(100U);
  c.verified = false;
  assert(h.update(c,s) == PlannerHealthFault::UNVERIFIED);
  h.reset(); c = config();
  assert(h.update(c,s) == PlannerHealthFault::NONE);
  s.cross_track_cm = 6.0f;
  assert(h.update(c,s) == PlannerHealthFault::OFF_ROUTE);
  s.cross_track_cm = 0.0f;
  assert(h.update(c,s) == PlannerHealthFault::OFF_ROUTE); // latched
  h.reset(); s = sample(100U);
  assert(h.update(c,s) == PlannerHealthFault::NONE);
  s.now_ms = 600U;
  assert(h.update(c,s) == PlannerHealthFault::NO_PROGRESS);
  h.reset(); s = sample(100U); h.update(c,s);
  s.now_ms = 200U; s.route_progress_cm = 12.0f;
  assert(h.update(c,s) == PlannerHealthFault::NONE);
  h.reset(); s = sample(100U); s.left_cm = 0.0f;
  h.update(c,s); s.now_ms = 400U;
  assert(h.update(c,s) == PlannerHealthFault::LEFT_STALL);
  h.reset(); s = sample(100U); s.right_cm = 0.0f;
  h.update(c,s); s.now_ms = 400U;
  assert(h.update(c,s) == PlannerHealthFault::RIGHT_STALL);
  h.reset(); s = sample(100U); s.gyro_delta_rad = 0.3f;
  h.update(c,s); s.now_ms = 300U;
  assert(h.update(c,s) == PlannerHealthFault::YAW_DISAGREEMENT);
  h.reset(); s = sample(100U); s.route_active = false;
  assert(h.update(c,s) == PlannerHealthFault::NONE);
  puts("PASS route deviation, no progress, wheel stalls and yaw disagreement");
}
