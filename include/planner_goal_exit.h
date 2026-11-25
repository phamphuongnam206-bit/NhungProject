#pragma once
#include <math.h>
#include <stddef.h>

struct PlannerGoalExitRect {
  float x_cm, y_cm, width_cm, height_cm;
};

// Geometry only: a straight, fixed-heading exit and its measured braking
// overrun. The caller must verify the destination and stopping envelope.
struct PlannerGoalExitConfig {
  bool verified = false;
  float gate_xmin_cm = 0, gate_xmax_cm = 0;
  float line_y_cm = 0, exit_ymin_cm = 0;
  float half_length_cm = 0, half_width_cm = 0;
  float obstacle_margin_cm = 0;
  float heading_tolerance_rad = 0;
  float terminal_center_y_cm = 0;
  float brake_overrun_cm = 0;
};

inline bool PlannerGoalExit_CorridorClear(
    const PlannerGoalExitConfig &c, bool mission_complete, bool pose_verified,
    bool ranges_fresh, bool wheels_stopped, float x_cm, float y_cm,
    float hybrid_yaw_rad, const PlannerGoalExitRect *obstacles,
    size_t obstacle_count) {
  if (!c.verified || !mission_complete || !pose_verified || !ranges_fresh ||
      !wheels_stopped || !(c.gate_xmin_cm < c.gate_xmax_cm) ||
      !(c.exit_ymin_cm < c.line_y_cm) ||
      !(c.half_length_cm > 0) || !(c.half_width_cm > 0) ||
      !(c.obstacle_margin_cm >= 0) ||
      !(c.heading_tolerance_rad > 0 &&
        c.heading_tolerance_rad < 1.57079632679f) ||
      !(c.brake_overrun_cm > 0) ||
      !isfinite(c.gate_xmin_cm) || !isfinite(c.gate_xmax_cm) ||
      !isfinite(c.line_y_cm) || !isfinite(c.exit_ymin_cm) ||
      !isfinite(c.half_length_cm) || !isfinite(c.half_width_cm) ||
      !isfinite(c.obstacle_margin_cm) ||
      !isfinite(c.heading_tolerance_rad) ||
      !isfinite(x_cm) || !isfinite(y_cm) ||
      !isfinite(hybrid_yaw_rad) ||
      !isfinite(c.terminal_center_y_cm) ||
      !isfinite(c.brake_overrun_cm) ||
      (obstacle_count && !obstacles)) return false;
  // Hybrid yaw pi faces the exit (-Y). AABB projection is conservative for
  // the fixed heading, including small reviewed angular errors.
  if (fabsf(remainderf(hybrid_yaw_rad - 3.14159265359f,
                       6.28318530718f)) > c.heading_tolerance_rad)
    return false;
  const float s = fabsf(sinf(hybrid_yaw_rad));
  const float co = fabsf(cosf(hybrid_yaw_rad));
  const float ex = c.half_length_cm * s + c.half_width_cm * co;
  const float ey = c.half_length_cm * co + c.half_width_cm * s;
  const float left = x_cm - ex, right = x_cm + ex;
  const float lowest_center = c.terminal_center_y_cm - c.brake_overrun_cm;
  if (left <= c.gate_xmin_cm || right >= c.gate_xmax_cm ||
      y_cm - ey <= c.line_y_cm ||
      c.terminal_center_y_cm >= y_cm ||
      c.terminal_center_y_cm + ey >= c.line_y_cm ||
      lowest_center - ey < c.exit_ymin_cm) return false;
  const float bottom = lowest_center - ey;
  const float top = y_cm + ey;
  for (size_t i = 0; i < obstacle_count; ++i) {
    const PlannerGoalExitRect &b = obstacles[i];
    if (!isfinite(b.x_cm) || !isfinite(b.y_cm) ||
        !(b.width_cm > 0) || !(b.height_cm > 0) ||
        !isfinite(b.width_cm) || !isfinite(b.height_cm)) return false;
    if (right >= b.x_cm - c.obstacle_margin_cm &&
        left <= b.x_cm + b.width_cm + c.obstacle_margin_cm &&
        top >= b.y_cm - c.obstacle_margin_cm &&
        bottom <= b.y_cm + b.height_cm + c.obstacle_margin_cm) return false;
  }
  return true;
}
