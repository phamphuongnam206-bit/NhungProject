#pragma once
#include "planner_runtime.h"

struct PlannerTimedRange { uint16_t mm; uint32_t at_ms; bool valid; };
struct PlannerSnapshotSource {
  float x_cm, y_cm, hybrid_yaw_rad, speed_cm_s, dt_s;
  uint32_t now_ms, encoder_ms, gyro_ms;
  bool pose_verified, encoder_valid, gyro_valid, emergency_stop;
  PlannerTimedRange ranges[4]; // J8..J11; J9/J10 forward
};
struct PlannerSnapshotLimits { uint32_t pose_age_ms, range_age_ms, frame_skew_ms; };
// Geometry: hybrid yaw=0 at +Y, positive clockwise; core=0 at +X, CCW.
// Mapping disabled until individual range origins can be passed to the core.
extern "C" bool PlannerSnapshot_Build(const PlannerSnapshotSource *source,
    const PlannerSnapshotLimits *limits, OneRunInput *output);
