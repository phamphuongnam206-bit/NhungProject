#include "planner_start_entry_motion.h"
#include <assert.h>
#include <stdio.h>

static PlannerStartEntryMotionConfig fixture() {
  PlannerStartEntryMotionConfig c{};
  c.verified = true;
  c.gate_xmin_cm = 0; c.gate_xmax_cm = 60;
  c.line_y_cm = 0; c.outside_ymin_cm = -20;
  c.half_length_cm = 6.5f; c.half_width_cm = 5;
  c.heading_tolerance_rad = 0.1f;
  c.terminal_center_y_cm = 15;
  c.brake_overrun_cm = 2;
  c.speed_cm_s = 8; c.progress_epsilon_cm = 0.2f;
  c.timeout_ms = 6000; c.stall_ms = 700;
  return c; // Host fixture only; no PCB start pose or braking data is verified.
}

int main() {
  auto c = fixture();
  PlannerStartEntryMotion state{};
  const PlannerGoalExitRect forbidden[] = {{60, 0, 60, 60}};
  assert(PlannerStartEntryMotion_PathClear(c, 30, -7, 0, forbidden, 1, 1.5f));
  assert(!PlannerStartEntryMotion_PathClear(c, 56, -7, 0, forbidden, 1, 1.5f));
  const PlannerGoalExitRect dynamic[] = {{27, 5, 6, 6}};
  assert(!PlannerStartEntryMotion_PathClear(c, 30, -7, 0, dynamic, 1, 4.3f));
  assert(!PlannerStartEntryMotion_Begin(&state, c, false, true, true, true,
                                        true, 100, 30, -7, 0));
  assert(!PlannerStartEntryMotion_Begin(&state, c, true, true, true, true,
                                        true, 100, 30, 0, 0));
  assert(PlannerStartEntryMotion_Begin(&state, c, true, true, true, true,
                                       true, 100, 30, -7, 0));
  assert(PlannerStartEntryMotion_Step(&state, c, 200, 30, -4, 0,
                                      true, true, true, false) == PlannerStartEntryPhase::DRIVE);
  assert(PlannerStartEntryMotion_Step(&state, c, 1000, 30, 13, 0,
                                      true, true, true, false) == PlannerStartEntryPhase::BRAKE);
  assert(PlannerStartEntryMotion_Step(&state, c, 1100, 30, 15, 0,
                                      true, true, true, true) == PlannerStartEntryPhase::COMPLETE);
  state = {};
  assert(PlannerStartEntryMotion_Begin(&state, c, true, true, true, true,
                                       true, 100, 30, -7, 0));
  assert(PlannerStartEntryMotion_Step(&state, c, 900, 30, -7, 0,
                                      true, true, true, false) == PlannerStartEntryPhase::FAULT);
  state = {};
  assert(PlannerStartEntryMotion_Begin(&state, c, true, true, true, true,
                                       true, 100, 30, -7, 0));
  assert(PlannerStartEntryMotion_Step(&state, c, 200, 30, -4, 0,
                                      true, false, true, false) == PlannerStartEntryPhase::FAULT);
  state = {};
  assert(PlannerStartEntryMotion_Begin(&state, c, true, true, true, true,
                                       true, 100, 30, -7, 0));
  assert(PlannerStartEntryMotion_Step(&state, c, 200, 30, -4, 0,
                                      true, true, false, false) == PlannerStartEntryPhase::FAULT);
  c.speed_cm_s = 0;
  state = {};
  assert(!PlannerStartEntryMotion_Begin(&state, c, true, true, true, true,
                                        true, 100, 30, -7, 0));
  puts("PASS START entry motion: surveyed start, gate, progress, stop and faults");
}
