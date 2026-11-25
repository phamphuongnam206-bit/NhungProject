#include "planner_goal_exit_motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static PlannerGoalExitMotionConfig fixture() {
  PlannerGoalExitMotionConfig m{};
  m.corridor.verified = true;
  m.corridor.gate_xmin_cm = 120;
  m.corridor.gate_xmax_cm = 180;
  m.corridor.line_y_cm = 0;
  m.corridor.exit_ymin_cm = -25;
  m.corridor.half_length_cm = 6.5f;
  m.corridor.half_width_cm = 5;
  m.corridor.heading_tolerance_rad = 0.1f;
  m.corridor.terminal_center_y_cm = -10;
  m.corridor.brake_overrun_cm = 2;
  m.speed_cm_s = 8;
  m.timeout_ms = 6000;
  m.stall_ms = 700;
  m.progress_epsilon_cm = 0.2f;
  return m; // All numeric values are host fixtures, not vehicle calibration.
}

int main() {
  auto m = fixture();
  PlannerGoalExitMotion state{};
  assert(!PlannerGoalExitMotion_Begin(&state, m, false, 100, 12.5f));
  m.speed_cm_s = 0;
  assert(!PlannerGoalExitMotion_Begin(&state, m, true, 100, 12.5f));
  m = fixture();
  m.corridor.heading_tolerance_rad = 3.0f;
  assert(!PlannerGoalExitMotion_Begin(&state, m, true, 100, 12.5f));
  m = fixture();
  m.corridor.gate_xmax_cm = m.corridor.gate_xmin_cm;
  assert(!PlannerGoalExitMotion_Begin(&state, m, true, 100, 12.5f));
  m = fixture();
  m.corridor.half_width_cm = 0;
  assert(!PlannerGoalExitMotion_Begin(&state, m, true, 100, 12.5f));
  m = fixture();
  const PlannerGoalExitRect blocks[] = {
      {60, 0, 60, 60}, {140, 10, 10, 10}, {170, 20, 10, 10}};
  assert(PlannerGoalExitMotion_PathClear(m, 162.5f, 12.5f, 3.14159265f,
                                         blocks, 3, 1.5f));
  const PlannerGoalExitRect moving[] = {{159, -5, 6, 6}};
  assert(!PlannerGoalExitMotion_PathClear(m, 162.5f, 8, 3.14159265f,
                                          moving, 1, 4.3f));
  assert(PlannerGoalExitMotion_Begin(&state, m, true, 100, 12.5f));
  assert(PlannerGoalExitMotion_Step(&state, m, 200, 162.5f, 10, 3.14159265f,
                                    true, true, true, false) == PlannerGoalExitPhase::DRIVE);
  assert(PlannerGoalExitMotion_Step(&state, m, 1000, 162.5f, -8, 3.14159265f,
                                    true, true, true, false) == PlannerGoalExitPhase::BRAKE);
  assert(PlannerGoalExitMotion_Step(&state, m, 1100, 162.5f, -10, 3.14159265f,
                                    true, true, true, true) == PlannerGoalExitPhase::COMPLETE);
  assert(PlannerGoalExitMotion_Step(&state, m, 1200, 162.5f, -10, 3.14159265f,
                                    true, true, true, true) == PlannerGoalExitPhase::COMPLETE);

  state = {};
  assert(PlannerGoalExitMotion_Begin(&state, m, true, 100, 12.5f));
  assert(PlannerGoalExitMotion_Step(&state, m, 900, 162.5f, 12.5f, 3.14159265f,
                                    true, true, true, false) == PlannerGoalExitPhase::FAULT);
  state = {};
  assert(PlannerGoalExitMotion_Begin(&state, m, true, 100, 12.5f));
  assert(PlannerGoalExitMotion_Step(&state, m, 200, 162.5f, 10, 3.14159265f,
                                    true, false, true, false) == PlannerGoalExitPhase::FAULT);
  state = {};
  assert(PlannerGoalExitMotion_Begin(&state, m, true, 100, 12.5f));
  assert(PlannerGoalExitMotion_Step(&state, m, 200, 162.5f, 10, 3.14159265f,
                                    true, true, false, false) == PlannerGoalExitPhase::FAULT);
  state = {};
  assert(PlannerGoalExitMotion_Begin(&state, m, true, 100, 12.5f));
  assert(PlannerGoalExitMotion_Step(&state, m, 200, 162.5f, 10, 0,
                                    true, true, true, false) == PlannerGoalExitPhase::FAULT);
  state = {};
  assert(PlannerGoalExitMotion_Begin(&state, m, true, 100, 12.5f));
  assert(PlannerGoalExitMotion_Step(&state, m, 200, 162.5f, -8, 3.14159265f,
                                    true, true, true, false) == PlannerGoalExitPhase::BRAKE);
  assert(PlannerGoalExitMotion_Step(&state, m, 300, 162.5f, -7, 3.14159265f,
                                    true, true, true, true) == PlannerGoalExitPhase::FAULT);
  puts("PASS GOAL exit motion: progress, full-body finish, timeout and safety faults");
}
