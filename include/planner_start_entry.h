#pragma once
#include <math.h>

// Handoff into the in-arena planner. This does not drive the robot across START.
// The surveyed start region and the collision check must be supplied by callers.
struct PlannerStartEntryConfig {
  bool verified = false;
  float gate_xmin_cm = 0, gate_xmax_cm = 0, line_y_cm = 0;
  float half_length_cm = 0, half_width_cm = 0;
  float heading_tolerance_rad = 0;
};

inline bool PlannerStartEntry_HandoffPermitted(
    const PlannerStartEntryConfig &c, bool pose_verified, bool pose_fresh,
    bool ranges_fresh, bool wheels_stopped, bool footprint_clear,
    float x_cm, float y_cm, float hybrid_yaw_rad) {
  if (!c.verified || !pose_verified || !pose_fresh || !ranges_fresh ||
      !wheels_stopped || !footprint_clear ||
      !(c.gate_xmin_cm < c.gate_xmax_cm) ||
      !(c.half_length_cm > 0) || !(c.half_width_cm > 0) ||
      !(c.heading_tolerance_rad > 0 &&
        c.heading_tolerance_rad < 1.57079632679f) ||
      !isfinite(x_cm) || !isfinite(y_cm) ||
      !isfinite(hybrid_yaw_rad)) return false;
  // Hybrid yaw zero points into the arena (+Y); positive yaw turns right.
  const float s = fabsf(sinf(hybrid_yaw_rad));
  const float co = fabsf(cosf(hybrid_yaw_rad));
  const float extent_x = c.half_length_cm * s + c.half_width_cm * co;
  const float extent_y = c.half_length_cm * co + c.half_width_cm * s;
  return fabsf(remainderf(hybrid_yaw_rad, 6.28318530718f)) <=
             c.heading_tolerance_rad &&
         x_cm - extent_x >= c.gate_xmin_cm &&
         x_cm + extent_x <= c.gate_xmax_cm &&
         y_cm - extent_y > c.line_y_cm;
}
