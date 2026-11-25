#include "planner_runtime.h"
#include "planner_route_view.h"
#include <assert.h>
#include <stdio.h>
static MapGrid map;
int main() {
  OneRunConfig cfg{};
  cfg.goal = {2,8};
  cfg.navigation.follower = {5,20,.5f,5,20,3,8,20,2};
  cfg.navigation.wheels = {10,30,40,80};
  cfg.navigation.safety = {50,100,100};
  cfg.replan = {100,2,2};
  cfg.sensors.min_range_mm=30; cfg.sensors.max_range_mm=1000;
  cfg.sensors.confirmations_required=2;
  assert(MapGrid_InitBase(&map) == MAP_GRID_OK);
  OneRunInput input{};
  input.pose = {12.5f,12.5f,0}; input.dt_s=.05f;
  input.safety.now_ms=1000; input.safety.pose_sample_ms=1000;
  input.safety.sensor_sample_ms=1000; input.safety.front_min_mm=300;
  input.safety.pose_valid=1; input.safety.sensor_valid=1;
  input.safety.control_deadline_ok=1;
  assert(PlannerRuntime_Poll(&input,true,true)==ONE_RUN_STOPPED);
  assert(!PlannerRuntime_Begin(&map,&cfg,false,true));
  assert(!PlannerRuntime_Begin(&map,&cfg,true,false));
  assert(PlannerRuntime_Begin(&map,&cfg,true,true));
  assert(PlannerRuntime_Poll(&input,true,false)==ONE_RUN_STOPPED);
  assert(OneRunFirmwareMission()->waypoint_count==0);
  assert(PlannerRuntime_Poll(&input,true,true)==ONE_RUN_STOPPED);
  assert(OneRunFirmwareMission()->waypoint_count>=2);
  PlannerRouteTarget target{};
  assert(PlannerRoute_Target(OneRunFirmwareMission(),12.5f,12.5f,10,&target));
  assert(PlannerRuntime_Poll(&input,true,true)==ONE_RUN_OK);
  assert(PlannerRuntime_PreviewPermitted());
  input.ranges_ready=1; input.range_poses_ready=1;
  input.ranges[0]={150,1}; input.range_poses[0]={12.5f,12.5f,0};
  assert(PlannerRuntime_Poll(&input,true,true)==ONE_RUN_OK);
  input.safety.now_ms+=10; input.safety.pose_sample_ms+=10;
  input.safety.sensor_sample_ms+=10;
  assert(PlannerRuntime_Poll(&input,true,false)==ONE_RUN_STOPPED);
  assert(OneRunFirmwareMission()->state==ONE_RUN_REPLAN_STOPPED);
  assert(!PlannerRuntime_PreviewPermitted());
  input.ranges_ready=0;
  assert(PlannerRuntime_Poll(&input,true,false)==ONE_RUN_STOPPED);
  assert(PlannerRuntime_Poll(&input,true,true)==ONE_RUN_STOPPED);
  assert(OneRunFirmwareMission()->state==ONE_RUN_FOLLOWING);
  assert(OneRunFirmwareMission()->waypoint_count>=2);
  // Safety pause must retain route even if the new path needs reorientation.
  input.safety.emergency_stop_requested=1;
  assert(PlannerRuntime_Poll(&input,true,true)==ONE_RUN_STOPPED);
  assert(!PlannerRuntime_PreviewPermitted());
  assert(OneRunFirmwareMission()->state==ONE_RUN_FOLLOWING);
  assert(PlannerRuntime_Poll(&input,false,true)==ONE_RUN_STOPPED);
  assert(!PlannerRoute_Target(OneRunFirmwareMission(),12.5f,12.5f,10,&target));
  assert(PlannerRuntime_Poll(&input,true,true)==ONE_RUN_STOPPED);
  assert(PlannerRuntime_Begin(&map,&cfg,true,true));
  input.safety.pose_sample_ms=0;
  assert(PlannerRuntime_Poll(&input,true,true)==ONE_RUN_STOPPED);
  assert(OneRunFirmwareMission()->waypoint_count==0);
  assert(PlannerRuntime_Poll(nullptr,true,true)==ONE_RUN_STOPPED);
  puts("PASS real planner runtime, stationary gate, stale pose, invalidation and route consumer");
}
