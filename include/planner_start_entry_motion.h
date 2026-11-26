#pragma once
#include "planner_goal_exit_motion.h"

using PlannerStartEntryMotion = PlannerBoundaryMotion;
using PlannerStartEntryPhase = PlannerBoundaryPhase;

struct PlannerStartEntryMotionConfig {
  bool verified = false;
  float gate_xmin_cm = 0, gate_xmax_cm = 0;
  float line_y_cm = 0, outside_ymin_cm = 0;
  float half_length_cm = 0, half_width_cm = 0;
  float heading_tolerance_rad = 0;
  float terminal_center_y_cm = 0, brake_overrun_cm = 0;
  float speed_cm_s = 0, progress_epsilon_cm = 0;
  uint32_t timeout_ms = 0, stall_ms = 0;
};

inline bool PlannerStartEntryMotion_ConfigReady(const PlannerStartEntryMotionConfig &c) {
  return c.verified && isfinite(c.gate_xmin_cm) && isfinite(c.gate_xmax_cm) &&
      c.gate_xmin_cm < c.gate_xmax_cm && isfinite(c.line_y_cm) &&
      isfinite(c.outside_ymin_cm) && c.outside_ymin_cm < c.line_y_cm &&
      isfinite(c.half_length_cm) && c.half_length_cm > 0 &&
      isfinite(c.half_width_cm) && c.half_width_cm > 0 &&
      isfinite(c.heading_tolerance_rad) && c.heading_tolerance_rad > 0 &&
      c.heading_tolerance_rad < 1.57079632679f &&
      isfinite(c.terminal_center_y_cm) && isfinite(c.brake_overrun_cm) &&
      c.brake_overrun_cm > 0 &&
      c.terminal_center_y_cm - c.brake_overrun_cm - c.half_length_cm > c.line_y_cm &&
      isfinite(c.speed_cm_s) && c.speed_cm_s > 0 &&
      isfinite(c.progress_epsilon_cm) && c.progress_epsilon_cm > 0 &&
      c.timeout_ms > 0 && c.stall_ms > 0 && c.stall_ms < c.timeout_ms;
}

inline bool PlannerStartEntryMotion_PathClear(
    const PlannerStartEntryMotionConfig &c, float x_cm, float y_cm,
    float yaw_rad, const PlannerGoalExitRect *obstacles, size_t count,
    float margin_cm) {
  if (!PlannerStartEntryMotion_ConfigReady(c) || !isfinite(x_cm) ||
      !isfinite(y_cm) || !isfinite(yaw_rad) || !isfinite(margin_cm) ||
      margin_cm < 0 || (count && !obstacles) ||
      fabsf(remainderf(yaw_rad, 6.28318530718f)) >
          c.heading_tolerance_rad) return false;
  // Straight motion can drift laterally throughout the remaining travel.
  // Bound the entire footprint for any heading inside the approved tolerance.
  const float max_y = c.terminal_center_y_cm + c.brake_overrun_cm;
  const float drift = fmaxf(0.0f, max_y - y_cm) *
                      tanf(c.heading_tolerance_rad);
  const float s = sinf(c.heading_tolerance_rad);
  const float ex = c.half_width_cm + c.half_length_cm * s;
  const float ey = c.half_length_cm + c.half_width_cm * s;
  if (!isfinite(drift)) return false;
  const float left = x_cm - ex - drift, right = x_cm + ex + drift;
  const float bottom = y_cm - ey;
  const float top = max_y + ey;
  if (left <= c.gate_xmin_cm || right >= c.gate_xmax_cm ||
      bottom < c.outside_ymin_cm || top < bottom) return false;
  for (size_t i = 0; i < count; ++i) {
    const PlannerGoalExitRect &b = obstacles[i];
    if (!isfinite(b.x_cm) || !isfinite(b.y_cm) ||
        !isfinite(b.width_cm) || !isfinite(b.height_cm) ||
        b.width_cm <= 0 || b.height_cm <= 0) return false;
    if (right >= b.x_cm - margin_cm && left <= b.x_cm + b.width_cm + margin_cm &&
        top >= b.y_cm - margin_cm && bottom <= b.y_cm + b.height_cm + margin_cm)
      return false;
  }
  return true;
}

inline bool PlannerStartEntryMotion_Begin(
    PlannerStartEntryMotion *state, const PlannerStartEntryMotionConfig &c,
    bool pose_verified, bool pose_fresh, bool ranges_fresh, bool wheels_stopped,
    bool path_clear, uint32_t now_ms, float x_cm, float y_cm, float yaw_rad) {
  if (!state || state->phase != PlannerStartEntryPhase::IDLE ||
      !PlannerStartEntryMotion_ConfigReady(c) || !pose_verified || !pose_fresh ||
      !ranges_fresh || !wheels_stopped || !path_clear ||
      !isfinite(x_cm) || !isfinite(y_cm) || !isfinite(yaw_rad) ||
      fabsf(remainderf(yaw_rad, 6.28318530718f)) > c.heading_tolerance_rad)
    return false;
  const float s = fabsf(sinf(yaw_rad)), co = fabsf(cosf(yaw_rad));
  const float ex = c.half_length_cm * s + c.half_width_cm * co;
  const float ey = c.half_length_cm * co + c.half_width_cm * s;
  if (x_cm - ex <= c.gate_xmin_cm || x_cm + ex >= c.gate_xmax_cm ||
      y_cm + ey > c.line_y_cm || y_cm - ey < c.outside_ymin_cm)
    return false;
  state->phase = PlannerStartEntryPhase::DRIVE;
  state->started_ms = state->last_progress_ms = now_ms;
  state->start_y_cm = state->last_y_cm = y_cm;
  return true;
}

inline PlannerStartEntryPhase PlannerStartEntryMotion_Step(
    PlannerStartEntryMotion *state, const PlannerStartEntryMotionConfig &c,
    uint32_t now_ms, float x_cm, float y_cm, float yaw_rad, bool pose_fresh,
    bool ranges_fresh, bool path_clear, bool wheels_stopped) {
  if (!state || state->phase == PlannerStartEntryPhase::IDLE ||
      state->phase == PlannerStartEntryPhase::FAULT ||
      state->phase == PlannerStartEntryPhase::COMPLETE)
    return state ? state->phase : PlannerStartEntryPhase::FAULT;
  if (!PlannerStartEntryMotion_ConfigReady(c) || !pose_fresh || !ranges_fresh ||
      !path_clear || !isfinite(x_cm) || !isfinite(y_cm) || !isfinite(yaw_rad) ||
      fabsf(remainderf(yaw_rad, 6.28318530718f)) > c.heading_tolerance_rad ||
      uint32_t(now_ms - state->started_ms) > c.timeout_ms) {
    state->phase = PlannerStartEntryPhase::FAULT;
    return state->phase;
  }
  const float s = fabsf(sinf(yaw_rad)), co = fabsf(cosf(yaw_rad));
  const float ex = c.half_length_cm * s + c.half_width_cm * co;
  const float ey = c.half_length_cm * co + c.half_width_cm * s;
  if (x_cm - ex <= c.gate_xmin_cm || x_cm + ex >= c.gate_xmax_cm ||
      y_cm - ey < c.outside_ymin_cm ||
      y_cm < state->start_y_cm - c.progress_epsilon_cm) {
    state->phase = PlannerStartEntryPhase::FAULT;
  } else if (state->phase == PlannerStartEntryPhase::DRIVE) {
    if (y_cm > state->last_y_cm + c.progress_epsilon_cm) {
      state->last_y_cm = y_cm;
      state->last_progress_ms = now_ms;
    }
    if (y_cm >= c.terminal_center_y_cm - c.brake_overrun_cm) {
      state->phase = PlannerStartEntryPhase::BRAKE;
      state->brake_ms = now_ms;
    } else if (uint32_t(now_ms - state->last_progress_ms) > c.stall_ms) {
      state->phase = PlannerStartEntryPhase::FAULT;
    }
  } else if (state->phase == PlannerStartEntryPhase::BRAKE && wheels_stopped &&
             uint32_t(now_ms - state->brake_ms) >= 80U) {
    state->phase = (y_cm - ey > c.line_y_cm &&
                    y_cm >= c.terminal_center_y_cm - c.brake_overrun_cm &&
                    y_cm <= c.terminal_center_y_cm + c.brake_overrun_cm)
                       ? PlannerStartEntryPhase::COMPLETE : PlannerStartEntryPhase::FAULT;
  }
  return state->phase;
}
