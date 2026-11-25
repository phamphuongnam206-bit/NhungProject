#include "planner_snapshot.h"
#include "planner_route_view.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static MapGrid base_map;

int main() {
  OneRunConfig config{};
  config.goal = {12U, 20U};
  config.navigation.follower = {5, 20, .5f, 5, 20, 3, 8, 20, 2};
  config.navigation.wheels = {10, 30, 40, 80};
  config.navigation.safety = {50, 100, 100};
  config.replan = {100, 2, 2};
  config.sensors.min_range_mm = 30;
  config.sensors.max_range_mm = 1000;
  config.sensors.confirmations_required = 2;
  config.sensors.mounts[0].yaw_rad = 1.57079632679f;  // J8, left
  config.sensors.mounts[3].yaw_rad = -1.57079632679f; // J11, right
  assert(MapGrid_InitBase(&base_map) == MAP_GRID_OK);

  PlannerSnapshotSource source{};
  source.x_cm = source.y_cm = 62.5f;
  source.hybrid_yaw_rad = 1.57079632679f; // core heading +X
  source.dt_s = .05f;
  source.now_ms = source.encoder_ms = source.gyro_ms = 1000U;
  source.pose_verified = source.encoder_valid = source.gyro_valid = true;
  for (unsigned i = 0; i < 4; ++i)
    source.ranges[i] = {800U, 1000U, true, 62.5f, 62.5f,
                        1.57079632679f};
  source.ranges[0].mm = source.ranges[3].mm = 100U;
  PlannerSnapshotLimits limits{50U, 100U, 30U};
  OneRunInput snapshot{};
  assert(PlannerSnapshot_Build(&source, &limits, &snapshot));
  assert(snapshot.ranges_ready == 0U);
  assert(PlannerRuntime_Begin(&base_map, &config, true, true));
  assert(PlannerRuntime_Poll(&snapshot, true, true) == ONE_RUN_STOPPED);
  assert(PlannerRuntime_Poll(&snapshot, true, true) == ONE_RUN_OK);

  source.ranges_have_origins = true;
  for (unsigned pass = 0; pass < 2; ++pass) {
    source.now_ms += 10U;
    source.encoder_ms = source.gyro_ms = source.now_ms;
    for (auto &range : source.ranges) range.at_ms = source.now_ms;
    assert(PlannerSnapshot_Build(&source, &limits, &snapshot));
    assert(snapshot.ranges_ready && snapshot.range_poses_ready);
    for (unsigned i = 0; i < 4; ++i) {
      assert(snapshot.ranges[i].valid);
      assert(snapshot.ranges[i].range_mm == source.ranges[i].mm);
      assert(fabsf(snapshot.range_poses[i].x_cm - 62.5f) < .001f);
    }
    const OneRunStatus status = PlannerRuntime_Poll(&snapshot, true, true);
    assert(status == ONE_RUN_OK || status == ONE_RUN_STOPPED);
  }
  MapCell left{}, right{};
  const MapGrid *runtime = &OneRunFirmwareMission()->runtime_map;
  assert(MapGrid_GetCell(runtime, 14U, 12U, &left) == MAP_GRID_OK);
  assert(MapGrid_GetCell(runtime, 10U, 12U, &right) == MAP_GRID_OK);
  assert(left == MAP_CELL_OBSTACLE && right == MAP_CELL_OBSTACLE);
  source.ranges[1].mm = source.ranges[2].mm = 100U;
  for (unsigned pass = 0; pass < 2; ++pass) {
    source.now_ms += 10U;
    source.encoder_ms = source.gyro_ms = source.now_ms;
    for (auto &range : source.ranges) range.at_ms = source.now_ms;
    assert(PlannerSnapshot_Build(&source, &limits, &snapshot));
    const OneRunStatus status = PlannerRuntime_Poll(&snapshot, true, false);
    assert(status == ONE_RUN_OK || status == ONE_RUN_STOPPED);
  }
  MapCell ahead{};
  assert(MapGrid_GetCell(runtime, 12U, 14U, &ahead) == MAP_GRID_OK);
  assert(ahead == MAP_CELL_OBSTACLE);
  assert(OneRunFirmwareMission()->state == ONE_RUN_REPLAN_STOPPED);
  assert(!PlannerRuntime_PreviewPermitted());
  puts("PASS four timed VL53 ranges map left/right and front obstacle requests replan");
}
