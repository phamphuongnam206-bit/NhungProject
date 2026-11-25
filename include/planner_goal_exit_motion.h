#pragma once
#include "planner_goal_exit.h"
#include <stdint.h>

enum class PlannerGoalExitPhase : uint8_t { IDLE, DRIVE, BRAKE, COMPLETE, FAULT };

struct PlannerGoalExitMotion {
  PlannerGoalExitPhase phase = PlannerGoalExitPhase::IDLE;
  uint32_t started_ms = 0;
  uint32_t brake_ms = 0;
  float start_y_cm = 0;
  float last_y_cm = 0;
  uint32_t last_progress_ms = 0;
};

struct PlannerGoalExitMotionConfig {
  PlannerGoalExitConfig corridor{};
  float speed_cm_s = 0;       // Measured speed at which brake_overrun_cm applies.
  uint32_t timeout_ms = 0;   // Measured upper bound, including brake settling.
  uint32_t stall_ms = 0;     // Measured no-progress limit.
  float progress_epsilon_cm = 0;
};

inline bool PlannerGoalExitMotion_ConfigReady(const PlannerGoalExitMotionConfig &m) {
  return m.corridor.verified && isfinite(m.speed_cm_s) && m.speed_cm_s > 0 &&
         m.timeout_ms > 0 && m.stall_ms > 0 && m.stall_ms < m.timeout_ms &&
         isfinite(m.progress_epsilon_cm) && m.progress_epsilon_cm > 0 &&
         isfinite(m.corridor.brake_overrun_cm) && m.corridor.brake_overrun_cm > 0 &&
         isfinite(m.corridor.terminal_center_y_cm) &&
         m.corridor.terminal_center_y_cm < m.corridor.line_y_cm;
}

// Conservative swept corridor from the current footprint to the lowest
// possible stopping point. Tape boundaries are checked as geometry, not ToF.
inline bool PlannerGoalExitMotion_PathClear(
    const PlannerGoalExitMotionConfig &m, float x_cm, float y_cm, float yaw_rad,
    const PlannerGoalExitRect *obstacles, size_t count, float margin_cm) {
  const PlannerGoalExitConfig &c = m.corridor;
  if (!PlannerGoalExitMotion_ConfigReady(m) || !isfinite(x_cm) ||
      !isfinite(y_cm) || !isfinite(yaw_rad) || !isfinite(margin_cm) ||
      margin_cm < 0 || (count && !obstacles)) return false;
  const float s = fabsf(sinf(yaw_rad)), co = fabsf(cosf(yaw_rad));
  const float ex = c.half_length_cm * s + c.half_width_cm * co;
  const float ey = c.half_length_cm * co + c.half_width_cm * s;
  const float left = x_cm - ex, right = x_cm + ex;
  const float bottom = c.terminal_center_y_cm - c.brake_overrun_cm - ey;
  const float top = y_cm + ey;
  if (left <= c.gate_xmin_cm || right >= c.gate_xmax_cm ||
      bottom < c.exit_ymin_cm || top < bottom) return false;
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

inline bool PlannerGoalExitMotion_Begin(
    PlannerGoalExitMotion *state, const PlannerGoalExitMotionConfig &m,
    bool preflight_clear, uint32_t now_ms, float y_cm) {
  if (!state || state->phase != PlannerGoalExitPhase::IDLE ||
      !PlannerGoalExitMotion_ConfigReady(m) || !preflight_clear || !isfinite(y_cm))
    return false;
  state->phase = PlannerGoalExitPhase::DRIVE;
  state->started_ms = state->last_progress_ms = now_ms;
  state->start_y_cm = state->last_y_cm = y_cm;
  return true;
}

// Returns a phase; only DRIVE permits a straight motor command. Caller must
// keep checking live fixed/dynamic collision, sensor age and motor lease.
inline PlannerGoalExitPhase PlannerGoalExitMotion_Step(
    PlannerGoalExitMotion *state, const PlannerGoalExitMotionConfig &m,
    uint32_t now_ms, float x_cm, float y_cm, float yaw_rad,
    bool pose_fresh, bool ranges_fresh, bool path_clear, bool wheels_stopped) {
  if (!state || state->phase == PlannerGoalExitPhase::IDLE ||
      state->phase == PlannerGoalExitPhase::FAULT ||
      state->phase == PlannerGoalExitPhase::COMPLETE) return state ? state->phase : PlannerGoalExitPhase::FAULT;
  const PlannerGoalExitConfig &c = m.corridor;
  const float s = fabsf(sinf(yaw_rad)), co = fabsf(cosf(yaw_rad));
  const float ex = c.half_length_cm * s + c.half_width_cm * co;
  const float ey = c.half_length_cm * co + c.half_width_cm * s;
  if (!PlannerGoalExitMotion_ConfigReady(m) || !pose_fresh || !ranges_fresh ||
      !path_clear || !isfinite(x_cm) || !isfinite(y_cm) || !isfinite(yaw_rad) ||
      fabsf(remainderf(yaw_rad - 3.14159265359f, 6.28318530718f)) > c.heading_tolerance_rad ||
      x_cm - ex <= c.gate_xmin_cm || x_cm + ex >= c.gate_xmax_cm ||
      y_cm - ey < c.exit_ymin_cm || y_cm > state->start_y_cm + m.progress_epsilon_cm ||
      uint32_t(now_ms - state->started_ms) > m.timeout_ms) {
    state->phase = PlannerGoalExitPhase::FAULT;
    return state->phase;
  }
  if (state->phase == PlannerGoalExitPhase::DRIVE) {
    if (y_cm < state->last_y_cm - m.progress_epsilon_cm) {
      state->last_y_cm = y_cm;
      state->last_progress_ms = now_ms;
    }
    if (y_cm <= c.terminal_center_y_cm + c.brake_overrun_cm) {
      state->phase = PlannerGoalExitPhase::BRAKE;
      state->brake_ms = now_ms;
    } else if (uint32_t(now_ms - state->last_progress_ms) > m.stall_ms) {
      state->phase = PlannerGoalExitPhase::FAULT;
    }
  } else if (state->phase == PlannerGoalExitPhase::BRAKE &&
             wheels_stopped && uint32_t(now_ms - state->brake_ms) >= 80U) {
    // Whole footprint must cross y=0, while the measured overrun envelope
    // must contain the final center. Never finish merely at the waypoint.
    state->phase = (y_cm + ey < c.line_y_cm &&
                    y_cm >= c.terminal_center_y_cm - c.brake_overrun_cm &&
                    y_cm <= c.terminal_center_y_cm + c.brake_overrun_cm)
                       ? PlannerGoalExitPhase::COMPLETE : PlannerGoalExitPhase::FAULT;
  }
  return state->phase;
}
