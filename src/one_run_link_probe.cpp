// Locked runtime storage. No synthetic pose is sent to PI.
#include "planner_runtime.h"
extern "C" {
#include "one_run_adapter.h"
}

#if !defined(HYBRID_MOTOR_LOCK) || HYBRID_MOTOR_LOCK != 1
#error "Planner link qualification requires the compile-time motor lock"
#endif

struct OneRunFirmwareStorage {
  OneRunMission mission;
  OneRunAdapter adapter;
  OneRunConfig config;
  OneRunInput snapshot;
  OneRunAdapterOutput output;
};
extern "C" {
OneRunFirmwareStorage one_run_link_storage;
}
static bool runtimeReady;

void PlannerRuntime_Invalidate() {
  runtimeReady = false;
  one_run_link_storage.mission.state = ONE_RUN_FAULT;
  one_run_link_storage.mission.fault_status = ONE_RUN_INVALID_ARGUMENT;
  one_run_link_storage.mission.waypoint_count = 0;
  one_run_link_storage.mission.navigation.previous_targets = {};
  one_run_link_storage.output = {};
  OneRunAdapter_Init(&one_run_link_storage.adapter);
}

bool PlannerRuntime_Begin(const MapGrid *map, const OneRunConfig *config,
                          bool inputs_verified, bool stopped) {
  PlannerRuntime_Invalidate();
  if (!map || !config || !inputs_verified || !stopped ||
      config->goal.row >= MAP_GRID_ROWS || config->goal.col >= MAP_GRID_COLS)
    return false;
  if (map == &one_run_link_storage.mission.runtime_map ||
      map == &one_run_link_storage.mission.planner_workspace.planning_grid)
    return false;
  one_run_link_storage.config = *config;
  if (OneRunMission_Init(&one_run_link_storage.mission, map) != ONE_RUN_STOPPED)
    return false;
  runtimeReady = true;
  return true;
}

OneRunStatus PlannerRuntime_Poll(const OneRunInput *snapshot,
                                 bool global_pose_verified, bool stopped) {
  if (!runtimeReady || !snapshot || !global_pose_verified) {
    PlannerRuntime_Invalidate();
    return ONE_RUN_STOPPED;
  }
  one_run_link_storage.snapshot = *snapshot;
  return OneRunAdapter_Poll(&one_run_link_storage.adapter,
      &one_run_link_storage.mission, &one_run_link_storage.config,
      &one_run_link_storage.snapshot, stopped ? 1U : 0U,
      &one_run_link_storage.output);
}

extern "C" void *OneRunFirmwareLinkProbe() {
  return &one_run_link_storage;
}

extern "C" const OneRunMission *OneRunFirmwareMission() {
  return &one_run_link_storage.mission;
}

// Preview permission only; never changes the compile-time actuator lock.
bool PlannerRuntime_PreviewPermitted() {
  const auto &o = one_run_link_storage.output.preview;
  return runtimeReady && one_run_link_storage.mission.state == ONE_RUN_FOLLOWING &&
         o.status == ONE_RUN_OK && o.navigation.safety.motion_permitted != 0 &&
         o.navigation.safety.reasons == 0;
}
