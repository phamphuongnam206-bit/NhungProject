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
