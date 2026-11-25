#pragma once
#include <math.h>
#include <stdint.h>

enum class PlannerHealthFault : uint8_t {
  NONE, UNVERIFIED, INVALID, OFF_ROUTE, NO_PROGRESS,
  LEFT_STALL, RIGHT_STALL, YAW_DISAGREEMENT
};

struct PlannerHealthConfig {
  bool verified = false;
  float max_cross_track_cm = 0;
  float min_command_cm_s = 0;
  float min_progress_cm = 0;
  float max_yaw_delta_error_rad = 0;
  float track_cm = 0;
  uint32_t no_progress_ms = 0;
  uint32_t stall_ms = 0;
  uint32_t yaw_error_ms = 0;
};

struct PlannerHealthSample {
  uint32_t now_ms;
  float cross_track_cm, route_progress_cm, command_cm_s;
  float left_cm, right_cm, gyro_delta_rad;
  bool route_active;
};

struct PlannerMotionHealth {
  PlannerHealthFault latched = PlannerHealthFault::NONE;
  uint32_t progress_at_ms = 0, left_stall_at_ms = 0;
  uint32_t right_stall_at_ms = 0, yaw_error_at_ms = 0;
  float progress_mark_cm = 0;
  bool timing = false, left_stalling = false, right_stalling = false;
  bool yaw_disagreeing = false;

  void reset() { *this = PlannerMotionHealth{}; }

  PlannerHealthFault update(const PlannerHealthConfig &c,
                            const PlannerHealthSample &s) {
    if (latched != PlannerHealthFault::NONE) return latched;
    if (!c.verified || !(c.max_cross_track_cm > 0) ||
        !(c.min_command_cm_s > 0) || !(c.min_progress_cm > 0) ||
        !(c.max_yaw_delta_error_rad > 0) || !(c.track_cm > 0) ||
        !c.no_progress_ms || !c.stall_ms || !c.yaw_error_ms)
      return latched = PlannerHealthFault::UNVERIFIED;
    if (!isfinite(s.cross_track_cm) || !isfinite(s.route_progress_cm) ||
        !isfinite(s.command_cm_s) || !isfinite(s.left_cm) ||
        !isfinite(s.right_cm) || !isfinite(s.gyro_delta_rad) ||
        s.cross_track_cm < 0)
      return latched = PlannerHealthFault::INVALID;
    if (!s.route_active || s.command_cm_s < c.min_command_cm_s) {
      timing = left_stalling = right_stalling = yaw_disagreeing = false;
      return PlannerHealthFault::NONE;
    }
    if (s.cross_track_cm > c.max_cross_track_cm)
      return latched = PlannerHealthFault::OFF_ROUTE;
    if (!timing || s.route_progress_cm >= progress_mark_cm + c.min_progress_cm) {
      progress_mark_cm = s.route_progress_cm;
      progress_at_ms = s.now_ms;
      timing = true;
    } else if (uint32_t(s.now_ms - progress_at_ms) >= c.no_progress_ms) {
      return latched = PlannerHealthFault::NO_PROGRESS;
    }
    const float wheel_epsilon_cm = c.min_progress_cm * 0.1f;
    if (fabsf(s.left_cm) <= wheel_epsilon_cm) {
      if (!left_stalling) { left_stalling = true; left_stall_at_ms = s.now_ms; }
      else if (uint32_t(s.now_ms - left_stall_at_ms) >= c.stall_ms)
        return latched = PlannerHealthFault::LEFT_STALL;
    } else left_stalling = false;
    if (fabsf(s.right_cm) <= wheel_epsilon_cm) {
      if (!right_stalling) { right_stalling = true; right_stall_at_ms = s.now_ms; }
      else if (uint32_t(s.now_ms - right_stall_at_ms) >= c.stall_ms)
        return latched = PlannerHealthFault::RIGHT_STALL;
    } else right_stalling = false;
    // This checks rotational wheel/gyro disagreement. Translational tire slip
    // cannot be established from these two sensors without an external pose.
    const float wheel_yaw = (s.left_cm - s.right_cm) / c.track_cm;
    if (fabsf(remainderf(wheel_yaw - s.gyro_delta_rad,
                          6.28318530718f)) > c.max_yaw_delta_error_rad) {
      if (!yaw_disagreeing) { yaw_disagreeing = true; yaw_error_at_ms = s.now_ms; }
      else if (uint32_t(s.now_ms - yaw_error_at_ms) >= c.yaw_error_ms)
        return latched = PlannerHealthFault::YAW_DISAGREEMENT;
    } else yaw_disagreeing = false;
    return PlannerHealthFault::NONE;
  }
};
