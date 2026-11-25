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
  assert(!PlannerGoalPulseQuiet(79U, 0U, 80U));
  assert(PlannerGoalPulseQuiet(80U, 0U, 80U));
  assert(PlannerGoalPulseQuiet(20U, UINT32_MAX - 79U, 80U));
  assert(PlannerGoalGate_Decide(c, true, false, 30, 500, 150, 12, 3.14f) ==
         PlannerGoalDecision::WAIT);
  assert(PlannerGoalGate_Decide(c, true, true, 100, 500, 150, 12, 3.14f) ==
         PlannerGoalDecision::ACCEPT);
  assert(PlannerGoalGate_Decide(c, true, true, 100, 500, 121, 12, 3.14f) ==
         PlannerGoalDecision::REJECT);
  assert(PlannerGoalGate_Decide(c, true, false, 501, 500, 150, 12, 3.14f) ==
         PlannerGoalDecision::TIMEOUT);
  assert(PlannerGoalGate_Decide(c, false, false, 999, 500, 150, 12, 3.14f) ==
         PlannerGoalDecision::WAIT);
  c.verified = false;
  assert(PlannerGoalGate_Decide(c, true, false, 30, 500, 150, 12, 3.14f) ==
         PlannerGoalDecision::REJECT);
  puts("PASS goal: settle, timeout, reviewed config, footprint and heading");
}
