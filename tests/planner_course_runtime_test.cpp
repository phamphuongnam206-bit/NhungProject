#include "planner_runtime.h"
#include "planner_route_view.h"
#include "planner_snapshot.h"
#include "planner_start_entry_motion.h"
#include "planner_goal_exit_motion.h"
#include "course_fixture.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static MapGrid course_map;

int main() {
  assert(MapGrid_InitBase(&course_map) == MAP_GRID_OK);
  for (unsigned i = 0; i < COURSE_RECT_COUNT; ++i) {
    const unsigned short *r = course_rects[i];
    assert(MapGrid_SetRectangle(&course_map, r[1] / 5, r[0] / 5,
        r[3] / 5, r[2] / 5,
        i == 0 ? MAP_CELL_FORBIDDEN : MAP_CELL_OBSTACLE) == MAP_GRID_OK);
  }
  OneRunConfig config{};
  config.planner.search = {4U, 2U};
  config.planner.min_los_clearance_cells = 1U;
  config.planner.footprint = {COURSE_HALF_LENGTH_CM / 5.0f,
                              COURSE_HALF_WIDTH_CM / 5.0f};
  config.navigation.follower = {5, 20, .35f, 5, 20, 3, 8, 20, 2};
  config.navigation.wheels = {10, 30, 40, 80};
  config.navigation.safety = {100, 100, 120};
  config.replan = {100, 2, 2};
  config.sensors.min_range_mm = 30;
  config.sensors.max_range_mm = 2000;
  config.sensors.confirmations_required = 2;
  // Nominal J8..J11 mounts from firmware; physical angles remain unverified.
  config.sensors.mounts[0] = {4.5f, 2.5f, 0.61837f};
  config.sensors.mounts[1] = {4.5f, 2.5f, 0.0f};
  config.sensors.mounts[2] = {4.5f, -2.5f, 0.0f};
  config.sensors.mounts[3] = {4.5f, -2.5f, -0.61837f};
  config.inflation_cells = 1U;
  config.goal = {2U, 32U}; // Interior approach, not competition finish.

  PlannerSnapshotSource source{};
  source.x_cm = 42.5f; source.y_cm = 52.5f;
  source.hybrid_yaw_rad = 0.0f; // +Y in hybrid frame.
  source.dt_s = .05f;
  source.now_ms = 1000U;
  source.pose_verified = source.encoder_valid = source.gyro_valid = true;
  for (auto &range : source.ranges)
    range = {2000U, 1000U, true, 42.5f, 52.5f, 0.0f};
  PlannerSnapshotLimits limits{100U, 100U, 30U};
  OneRunInput input{};
  const auto refresh = [&](uint32_t elapsed_ms) {
    source.now_ms += elapsed_ms;
    source.encoder_ms = source.gyro_ms = source.now_ms;
    for (auto &range : source.ranges) range.at_ms = source.now_ms;
    return PlannerSnapshot_Build(&source, &limits, &input);
  };
  assert(refresh(0U));
  assert(PlannerRuntime_Begin(&course_map, &config, true, true));
  assert(PlannerRuntime_Poll(&input, true, true) == ONE_RUN_STOPPED);
  const OneRunMission *mission = OneRunFirmwareMission();
  assert(mission->state == ONE_RUN_FOLLOWING);
  assert(mission->waypoint_count >= 2U);
  assert(mission->waypoints[mission->waypoint_count - 1U].row == config.goal.row);
  assert(mission->waypoints[mission->waypoint_count - 1U].col == config.goal.col);
  const unsigned initial_count = mission->waypoint_count;
  for (unsigned i = 0; i < initial_count; ++i) {
    const MapCellPosition p = mission->waypoints[i];
    assert(mission->runtime_map.cells[p.row][p.col] == MAP_CELL_FREE);
  }
  const OneRunStatus following = PlannerRuntime_Poll(&input, true, true);
  assert(following == ONE_RUN_OK || following == ONE_RUN_STOPPED);

  // One synthetic VL53 hit on the narrow first corridor. All origins and
  // timestamps pass through the real snapshot adapter before sensor mapping.
  source.ranges_have_origins = true;
  source.ranges[1].mm = 200U;
  for (unsigned pass = 0; pass < 2; ++pass) {
    assert(refresh(10U));
    assert(input.ranges_ready && input.range_poses_ready);
    const OneRunStatus status = PlannerRuntime_Poll(&input, true, false);
    assert(status == ONE_RUN_OK || status == ONE_RUN_STOPPED);
  }
  assert(mission->state == ONE_RUN_REPLAN_STOPPED);
  assert(!PlannerRuntime_PreviewPermitted());
  MapCell dynamic_cell{};
  assert(MapGrid_GetCell(&mission->runtime_map, 15U, 8U, &dynamic_cell) == MAP_GRID_OK);
  assert(dynamic_cell == MAP_CELL_OBSTACLE);

  // The hit remains on the map after the sensor reports open space. On this
  // nominal narrow course there is no safe alternate route from this pose.
  source.ranges_have_origins = false;
  source.ranges[1].mm = 2000U;
  assert(refresh(100U));
  assert(PlannerRuntime_Poll(&input, true, true) == ONE_RUN_STOPPED);
  assert(mission->replan.failure_count == 1U);
  assert(!PlannerRuntime_PreviewPermitted());
  assert(refresh(100U));
  assert(PlannerRuntime_Poll(&input, true, true) == ONE_RUN_STOPPED);
  assert(mission->replan.failure_count == 2U);
  assert(refresh(10U));
  assert(PlannerRuntime_Poll(&input, true, true) == ONE_RUN_REPLAN_EXHAUSTED);
  assert(mission->state == ONE_RUN_FAULT);
  assert(mission->waypoint_count == 0U);
  assert(!PlannerRuntime_PreviewPermitted());

  // A second synthetic run covers the whole boundary-to-boundary chain. The
  // pose trace follows the actual C11 waypoints; it is not a dynamics model.
  PlannerStartEntryMotionConfig entry{};
  entry.verified = true;
  entry.gate_xmin_cm = 0; entry.gate_xmax_cm = 60;
  entry.line_y_cm = 0; entry.outside_ymin_cm = -20;
  entry.half_length_cm = 6.5f; entry.half_width_cm = 5;
  entry.heading_tolerance_rad = .005f;
  entry.terminal_center_y_cm = 52.5f; entry.brake_overrun_cm = 2;
  entry.speed_cm_s = 8; entry.progress_epsilon_cm = .2f;
  entry.timeout_ms = 10000; entry.stall_ms = 800;
  PlannerGoalExitRect fixed_rects[COURSE_RECT_COUNT]{};
  for (unsigned i = 0; i < COURSE_RECT_COUNT; ++i) {
    const unsigned short *r = course_rects[i];
    fixed_rects[i] = {float(r[0]), float(r[1]), float(r[2]), float(r[3])};
  }
  assert(PlannerStartEntryMotion_PathClear(entry, 42.5f, -7, 0,
      fixed_rects, COURSE_RECT_COUNT, 1.5f));
  PlannerStartEntryMotion boundary{};
  assert(PlannerStartEntryMotion_Begin(&boundary, entry, true, true, true,
                                      true, true, 2000, 42.5f, -7, 0));
  for (unsigned step = 1; step <= 29; ++step) {
    const float y = -7.0f + 2.0f * step;
    const auto phase = PlannerStartEntryMotion_Step(&boundary, entry,
        2000 + 100 * step, 42.5f, y, 0, true, true, true, false);
    assert(phase == PlannerStartEntryPhase::DRIVE ||
           phase == PlannerStartEntryPhase::BRAKE);
    if (phase == PlannerStartEntryPhase::BRAKE) break;
  }
  assert(boundary.phase == PlannerStartEntryPhase::BRAKE);
  assert(PlannerStartEntryMotion_Step(&boundary, entry, 5100, 42.5f, 52.5f,
      0, true, true, true, true) == PlannerStartEntryPhase::COMPLETE);

  source.x_cm = 42.5f; source.y_cm = 52.5f; source.hybrid_yaw_rad = 0;
  source.ranges[1].mm = 2000U;
  source.ranges_have_origins = false;
  assert(refresh(100U));
  assert(PlannerRuntime_Begin(&course_map, &config, true, true));
  assert(PlannerRuntime_Poll(&input, true, true) == ONE_RUN_STOPPED);
  mission = OneRunFirmwareMission();
  assert(mission->state == ONE_RUN_FOLLOWING);
  assert(mission->waypoint_count >= 2U);
  bool reached = false;
  for (unsigned segment = 1; segment < mission->waypoint_count && !reached; ++segment) {
    const MapCellPosition a = mission->waypoints[segment - 1U];
    const MapCellPosition b = mission->waypoints[segment];
    const float ax = (a.col + .5f) * 5, ay = (a.row + .5f) * 5;
    const float bx = (b.col + .5f) * 5, by = (b.row + .5f) * 5;
    const float length = hypotf(bx - ax, by - ay);
    const unsigned steps = unsigned(ceilf(length / 2.0f));
    for (unsigned j = 1; j <= steps; ++j) {
      const float t = float(j) / float(steps);
      source.x_cm = ax + t * (bx - ax);
      source.y_cm = ay + t * (by - ay);
      source.hybrid_yaw_rad = 1.57079632679f - atan2f(by - ay, bx - ax);
      assert(refresh(50U));
      const OneRunStatus step_status = PlannerRuntime_Poll(&input, true, false);
      assert(step_status == ONE_RUN_OK || step_status == ONE_RUN_STOPPED ||
             step_status == ONE_RUN_COMPLETE);
      if (step_status == ONE_RUN_COMPLETE) { reached = true; break; }
    }
  }
  assert(reached);
  assert(!PlannerRuntime_PreviewPermitted());
  assert(fabsf(source.x_cm - 162.5f) < 2.1f);
  assert(fabsf(source.y_cm - 12.5f) < 2.1f);

  PlannerGoalExitMotionConfig exit{};
  exit.corridor.verified = true;
  exit.corridor.gate_xmin_cm = 120; exit.corridor.gate_xmax_cm = 180;
  exit.corridor.line_y_cm = 0; exit.corridor.exit_ymin_cm = -25;
  exit.corridor.half_length_cm = 6.5f; exit.corridor.half_width_cm = 5;
  exit.corridor.heading_tolerance_rad = .02f;
  exit.corridor.terminal_center_y_cm = -10;
  exit.corridor.brake_overrun_cm = 2;
  exit.speed_cm_s = 8; exit.timeout_ms = 6000;
  exit.stall_ms = 700; exit.progress_epsilon_cm = .2f;
  assert(PlannerGoalExitMotion_PathClear(exit, source.x_cm, source.y_cm,
      3.14159265f, fixed_rects, COURSE_RECT_COUNT, 1.5f));
  boundary = {};
  assert(PlannerGoalExitMotion_Begin(&boundary, exit, true, 100,
                                     source.y_cm));
  assert(PlannerGoalExitMotion_Step(&boundary, exit, 200, 162.5f, 10,
      3.14159265f, true, true, true, false) == PlannerGoalExitPhase::DRIVE);
  assert(PlannerGoalExitMotion_Step(&boundary, exit, 1000, 162.5f, -8,
      3.14159265f, true, true, true, false) == PlannerGoalExitPhase::BRAKE);
  assert(PlannerGoalExitMotion_Step(&boundary, exit, 1100, 162.5f, -10,
      3.14159265f, true, true, true, true) == PlannerGoalExitPhase::COMPLETE);
  printf("PASS 17-block runtime: %u interior waypoints, timed ToF map, safe no-path fault\n",
         initial_count);
  puts("PASS synthetic START_ENTRY -> C11 A* -> FOLLOWING -> GOAL_EXIT chain");
}
