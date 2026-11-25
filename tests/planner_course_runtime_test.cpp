#include "planner_runtime.h"
#include "planner_route_view.h"
#include "planner_snapshot.h"
#include "course_fixture.h"
#include <assert.h>
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
  printf("PASS 17-block runtime: %u interior waypoints, timed ToF map, safe no-path fault\n",
         initial_count);
}
