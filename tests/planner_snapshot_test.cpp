#include "planner_snapshot.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main() {
  PlannerSnapshotSource s{};
  s.x_cm=12.5f; s.y_cm=12.5f; s.dt_s=.01f; s.now_ms=1000;
  s.encoder_ms=995; s.gyro_ms=990;
  s.pose_verified=s.encoder_valid=s.gyro_valid=true;
  for (auto &r:s.ranges) r={300,980,true,0,0,0};
  s.ranges[1].mm=200; s.ranges[2].at_ms=970;
  PlannerSnapshotLimits limits{50,100,30};
  OneRunInput out{};
  assert(PlannerSnapshot_Build(&s,&limits,&out));
  assert(fabsf(out.pose.heading_rad-1.57079632679f)<1e-5f);
  assert(out.safety.pose_sample_ms==990 && out.safety.sensor_sample_ms==970);
  assert(out.safety.front_min_mm==200 && out.ranges_ready==0);
  s.hybrid_yaw_rad=1.57079632679f;
  assert(PlannerSnapshot_Build(&s,&limits,&out) && fabsf(out.pose.heading_rad)<1e-5f);
  s.ranges_have_origins=true;
  for (auto &r:s.ranges) { r.x_cm=12.5f; r.y_cm=12.5f; r.hybrid_yaw_rad=0; }
  assert(PlannerSnapshot_Build(&s,&limits,&out) && out.ranges_ready &&
         out.range_poses_ready && fabsf(out.range_poses[0].heading_rad-1.57079632679f)<1e-5f);
  for(unsigned i=0;i<4;++i) {
    s.ranges[i].mm=100+20*i; s.ranges[i].x_cm=62.5f+5*i;
    s.ranges[i].y_cm=62.5f;
  }
  assert(PlannerSnapshot_Build(&s,&limits,&out));
  MapGrid grid{}; SensorMapState state{}; SensorMapResult result{};
  SensorMapConfig cfg{}; cfg.min_range_mm=30; cfg.max_range_mm=1000;
  cfg.confirmations_required=1;
  // hybrid left=-90 degrees -> core left=+90 degrees
  cfg.mounts[0]={0,0,1.57079632679f};
  cfg.mounts[3]={0,0,-1.57079632679f};
  assert(MapGrid_InitBase(&grid)==MAP_GRID_OK);
  for(unsigned i=0;i<4;++i) {
    assert(out.ranges[i].valid && out.ranges[i].range_mm==100+20*i);
    assert(out.range_poses[i].x_cm==62.5f+5*i);
  }
  assert(SensorMap_UpdateTimed(&grid,&cfg,&state,out.range_poses,out.ranges,&result)==SENSOR_MAP_UPDATED);
  assert(result.projected_cell[0].col<12);
  assert(result.projected_cell[3].col>15);
  for(unsigned i=0;i<4;++i) assert(result.status[i]==SENSOR_MAP_UPDATED);
  s.ranges[3].x_cm=NAN;
  assert(!PlannerSnapshot_Build(&s,&limits,&out));
  assert(!out.safety.pose_valid && !out.ranges_ready && !out.range_poses_ready);
  s.ranges[3].x_cm=77.5f;
  s.ranges_have_origins=false;
  s.ranges[0].at_ms=930; // fresh but skewed
  assert(!PlannerSnapshot_Build(&s,&limits,&out) && !out.safety.pose_valid);
  s.ranges[0]={300,980,true,0,0,0}; s.gyro_ms=900;
  assert(!PlannerSnapshot_Build(&s,&limits,&out));
  s.gyro_ms=990; s.pose_verified=false;
  assert(!PlannerSnapshot_Build(&s,&limits,&out));
  s.pose_verified=true; s.now_ms=10; s.encoder_ms=5; s.gyro_ms=UINT32_MAX-4;
  for(auto &r:s.ranges) r={300,UINT32_MAX-9,true,0,0,0};
  assert(PlannerSnapshot_Build(&s,&limits,&out));
  assert(out.safety.sensor_sample_ms==UINT32_MAX-9);
  s.ranges[0].at_ms=20; // future timestamp, not fresh
  assert(!PlannerSnapshot_Build(&s,&limits,&out));
  s.hybrid_yaw_rad=NAN;
  assert(!PlannerSnapshot_Build(&s,&limits,&out));
  assert(!PlannerSnapshot_Build(nullptr,&limits,&out));
  puts("PASS snapshot axes, ages, skew, rollover, invalid data and mapping gate");
}
