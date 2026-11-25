#include "planner_goal_gate.h"
#include <assert.h>
#include <stdio.h>

int main() {
  PlannerGoalGateConfig c{};
  c.xmin_cm = 120; c.xmax_cm = 180;
  c.ymin_cm = 0; c.ymax_cm = 25;
  c.heading_rad = 3.14159265f; c.heading_tolerance_rad = 0.2f;
  c.half_length_cm = 6; c.half_width_cm = 4;
  assert(!PlannerGoalGate_Complete(c, true, true, 150, 12, 3.14f));
  c.verified = true;
  assert(PlannerGoalGate_Complete(c, true, true, 150, 12, 3.14f));
  assert(!PlannerGoalGate_Complete(c, false, true, 150, 12, 3.14f));
  assert(!PlannerGoalGate_Complete(c, true, false, 150, 12, 3.14f));
  assert(!PlannerGoalGate_Complete(c, true, true, 121, 12, 3.14f));
  assert(!PlannerGoalGate_Complete(c, true, true, 150, 12, 0));
  assert(!PlannerGoalGate_Complete(c, true, true, 150, 2, 3.14f));
  puts("PASS goal: reviewed config, mission complete, stop, footprint and heading");
}
