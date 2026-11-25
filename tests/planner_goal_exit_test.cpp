#include "planner_goal_exit.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main() {
  PlannerGoalExitConfig c{};
  c.gate_xmin_cm = 120; c.gate_xmax_cm = 180;
  c.line_y_cm = 0; c.exit_ymin_cm = -25;
  c.half_length_cm = 6.5f; c.half_width_cm = 5.0f;
  c.obstacle_margin_cm = 1.5f;
  c.heading_tolerance_rad = 0.1f;
  c.terminal_center_y_cm = -10;
  c.brake_overrun_cm = 2; // Test fixture, not an operational calibration.
  const PlannerGoalExitRect fixed[] = {
      {60, 0, 60, 60}, // forbidden tape zone: collision constraint
      {140, 10, 10, 10}, // T16
      {170, 20, 10, 10}, // T17
  };
  const auto clear = [&](bool mission, bool pose, bool ranges, bool stopped,
                         float x, float y, float yaw,
                         const PlannerGoalExitRect *boxes, size_t count) {
    return PlannerGoalExit_CorridorClear(c, mission, pose, ranges, stopped,
                                         x, y, yaw, boxes, count);
  };
  assert(!clear(true, true, true, true, 162.5f, 12.5f, 3.14159265f,
                fixed, 3));
  c.verified = true;
  assert(clear(true, true, true, true, 162.5f, 12.5f, 3.14159265f,
               fixed, 3));
  assert(!clear(false, true, true, true, 162.5f, 12.5f, 3.14159265f,
                fixed, 3));
  assert(!clear(true, false, true, true, 162.5f, 12.5f, 3.14159265f,
                fixed, 3));
  assert(!clear(true, true, false, true, 162.5f, 12.5f, 3.14159265f,
                fixed, 3));
  assert(!clear(true, true, true, false, 162.5f, 12.5f, 3.14159265f,
                fixed, 3));
  assert(!clear(true, true, true, true, 162.5f, 12.5f, 0, fixed, 3));
  assert(!clear(true, true, true, true, 162.5f, 6.5f, 3.14159265f,
                fixed, 3));
  assert(!clear(true, true, true, true, 163.6f, 12.5f, 3.14159265f,
                fixed, 3)); // T17 closes the narrow corridor.
  c.brake_overrun_cm = 15;
  assert(!clear(true, true, true, true, 162.5f, 12.5f, 3.14159265f,
                fixed, 3)); // Insufficient reviewed exit envelope.
  c.brake_overrun_cm = 2;
  const PlannerGoalExitRect dynamic[] = {{160, -3, 5, 5}};
  assert(!clear(true, true, true, true, 162.5f, 12.5f, 3.14159265f,
                dynamic, 1));
  assert(!clear(true, true, true, true, 162.5f, 12.5f, 3.14159265f,
                nullptr, 1));
  c.gate_xmax_cm = INFINITY;
  assert(!clear(true, true, true, true, 162.5f, 12.5f, 3.14159265f,
                fixed, 3));
  puts("PASS GOAL exit: full-body crossing, brake envelope, fixed/dynamic collision and gates");
}
