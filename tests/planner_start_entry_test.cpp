#include "planner_start_entry.h"
#include <assert.h>
#include <stdio.h>

int main() {
  PlannerStartEntryConfig c{};
  c.gate_xmin_cm = 0; c.gate_xmax_cm = 60; c.line_y_cm = 0;
  c.half_length_cm = 6.5f; c.half_width_cm = 5.0f;
  c.heading_tolerance_rad = 0.2f; // Synthetic reviewed tolerance for this test.
  const auto ready = [&](bool pose, bool fresh, bool ranges, bool stopped,
                         bool clear, float x, float y, float yaw) {
    return PlannerStartEntry_HandoffPermitted(c, pose, fresh, ranges,
                                               stopped, clear, x, y, yaw);
  };
  assert(!ready(true, true, true, true, true, 30, 20, 0));
  c.verified = true;
  assert(ready(true, true, true, true, true, 30, 20, 0));
  assert(!ready(false, true, true, true, true, 30, 20, 0));
  assert(!ready(true, false, true, true, true, 30, 20, 0));
  assert(!ready(true, true, false, true, true, 30, 20, 0));
  assert(!ready(true, true, true, false, true, 30, 20, 0));
  assert(!ready(true, true, true, true, false, 30, 20, 0));
  assert(!ready(true, true, true, true, true, 30, -8, 0));
  assert(!ready(true, true, true, true, true, 30, 6.5f, 0));
  assert(!ready(true, true, true, true, true, 4.9f, 20, 0));
  assert(!ready(true, true, true, true, true, 30, 20, 0.3f));
  assert(!ready(true, true, true, true, true, 30, 20, 3.14f));
  c.heading_tolerance_rad = 0;
  assert(!ready(true, true, true, true, true, 30, 20, 0));
  puts("PASS START handoff: reviewed config, full body, pose, sensors, stop and collision");
}
