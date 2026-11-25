#include "planner_pose_window.h"
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>

static PlannerPoseWindow filled(uint32_t start_ms) {
  PlannerPoseWindow history{};
  for (uint32_t i = 0; i <= 12; ++i)
    history.record({10.0f, 20.0f, 0.0f, start_ms + i * 10U});
  return history;
}

int main() {
  PlannerPoseSample origin{};
  PlannerPoseWindow history{};
  assert(!history.stableOrigin(125U, &origin));
  history = filled(0U);
  assert(history.stableOrigin(125U, &origin));
  assert(origin.x_cm == 10.0f && origin.y_cm == 20.0f && origin.at_ms == 120U);
  assert(!history.stableOrigin(150U, &origin)); // stale last pose
  assert(!history.stableOrigin(125U, nullptr));

  history = filled(0U);
  history.samples[7].x_cm += 0.6f;
  assert(!history.stableOrigin(125U, &origin));
  history = filled(0U);
  history.samples[7].yaw_rad = 0.04f;
  assert(!history.stableOrigin(125U, &origin));
  history = filled(0U);
  history.samples[7].x_cm = NAN;
  assert(!history.stableOrigin(125U, &origin));

  history = PlannerPoseWindow{};
  history.record({10.0f, 20.0f, 0.0f, 0U});
  history.record({10.0f, 20.0f, 0.0f, 10U});
  history.record({10.0f, 20.0f, 0.0f, 100U});
  history.record({10.0f, 20.0f, 0.0f, 110U});
  history.record({10.0f, 20.0f, 0.0f, 120U});
  assert(!history.stableOrigin(125U, &origin)); // missed control samples

  const uint32_t start = UINT32_MAX - 120U;
  history = filled(start);
  assert(history.stableOrigin(start + 125U, &origin)); // millis rollover
  puts("PASS stable pose window, movement, stale/gap/NaN, rollover");
}
