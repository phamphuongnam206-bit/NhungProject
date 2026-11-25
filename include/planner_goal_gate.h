#pragma once
#include <math.h>

// A planner cell alone is not proof that the whole robot reached the finish.
// Bounds and heading must come from the reviewed competition rule.
struct PlannerGoalGateConfig {
  bool verified = false;
  float xmin_cm = 0, xmax_cm = 0, ymin_cm = 0, ymax_cm = 0;
  float heading_rad = 0, heading_tolerance_rad = 0;
  float half_length_cm = 0, half_width_cm = 0;
};

inline bool PlannerGoalGate_Complete(const PlannerGoalGateConfig &c,
                                     bool mission_complete, bool wheels_stopped,
                                     float x_cm, float y_cm, float heading_rad) {
  if (!c.verified || !mission_complete || !wheels_stopped ||
      !(c.xmin_cm < c.xmax_cm) || !(c.ymin_cm < c.ymax_cm) ||
      !(c.heading_tolerance_rad > 0) || !(c.half_length_cm > 0) ||
      !(c.half_width_cm > 0) || !isfinite(x_cm) || !isfinite(y_cm) ||
      !isfinite(heading_rad) || !isfinite(c.heading_rad)) return false;
  const float s = sinf(heading_rad), co = cosf(heading_rad);
  const float extent_x = c.half_length_cm * fabsf(s) + c.half_width_cm * fabsf(co);
  const float extent_y = c.half_length_cm * fabsf(co) + c.half_width_cm * fabsf(s);
  return x_cm - extent_x >= c.xmin_cm && x_cm + extent_x <= c.xmax_cm &&
         y_cm - extent_y >= c.ymin_cm && y_cm + extent_y <= c.ymax_cm &&
         fabsf(remainderf(heading_rad - c.heading_rad, 6.28318530718f)) <=
             c.heading_tolerance_rad;
}
