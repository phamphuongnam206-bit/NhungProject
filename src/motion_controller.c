#include "motion_controller.h"
#include "static_map.h"
#include "odometry.h"
#include "robot_config.h"
#include <math.h>

static float bounded(float x, float lo, float hi) { return fminf(hi,fmaxf(lo,x)); }

void motion_begin(MotionController *motion, uint32_t now_ms) {
    *motion = (MotionController){0};
    motion->phase = MOTION_ALIGN;
    motion->next_waypoint = 1;
    motion->phase_at_ms = now_ms;
}

void motion_update(MotionController *motion, RobotPose pose, uint32_t now_ms) {
    motion->target_left_mm_s = motion->target_right_mm_s = 0;
    if (motion->next_waypoint >= robot_route_count) {
        motion->phase = MOTION_FINISHED;
        return;
    }
    const RobotPoint from = robot_route[motion->next_waypoint-1u];
    const RobotPoint to = robot_route[motion->next_waypoint];
    const float segment_dx = to.x_mm - from.x_mm;
    const float segment_dy = to.y_mm - from.y_mm;
    const float segment_len = hypotf(segment_dx, segment_dy);
    if (!isfinite(segment_len) || segment_len < 1.0f) { motion->phase = MOTION_ERROR; return; }
    const float ux = segment_dx / segment_len, uy = segment_dy / segment_len;
    const float dx = to.x_mm - pose.x_mm, dy = to.y_mm - pose.y_mm;
    const float distance = hypotf(dx,dy);
    const float remaining = dx*ux + dy*uy;
    const float cross = (pose.x_mm-from.x_mm)*uy - (pose.y_mm-from.y_mm)*ux;
    const float segment_heading = atan2f(uy,ux);
    const float error = angle_wrap(segment_heading - pose.heading_rad);
    if (motion->phase == MOTION_ALIGN) {
        if ((uint32_t)(now_ms-motion->phase_at_ms) > 8000u) {
            motion->phase = MOTION_ERROR;
            return;
        }
        if (fabsf(error) < 0.045f) {
            motion->phase = MOTION_TURN_SETTLE;
            motion->phase_at_ms = now_ms;
            return;
        }
        const float omega = bounded(2.0f*error,-1.3f,1.3f);
        motion->target_left_mm_s = -omega*ROBOT_TRACK_MM*0.5f;
        motion->target_right_mm_s = omega*ROBOT_TRACK_MM*0.5f;
        return;
    }
    if (motion->phase == MOTION_TURN_SETTLE) {
        if ((uint32_t)(now_ms-motion->phase_at_ms) < 250u) return;
        motion->phase = fabsf(error) < 0.08f ? MOTION_DRIVE : MOTION_ALIGN;
        motion->phase_at_ms = now_ms;
        return;
    }
    if (motion->phase == MOTION_DRIVE) {
        if ((uint32_t)(now_ms-motion->phase_at_ms) > 30000u ||
            fabsf(cross) > 35.0f || fabsf(error) > 0.25f || remaining < -30.0f) {
            motion->phase = MOTION_ERROR;
            return;
        }
        if (remaining <= 10.0f) {
            motion->phase = MOTION_SETTLE;
            motion->phase_at_ms = now_ms;
            return;
        }
        const float v = bounded(remaining*0.9f, 40.0f, 90.0f);
        const float desired = segment_heading + atan2f(cross, fmaxf(150.0f,remaining));
        const float omega = bounded(2.0f*angle_wrap(desired-pose.heading_rad),-0.75f,0.75f);
        motion->target_left_mm_s = bounded(v-omega*ROBOT_TRACK_MM*0.5f, 15.0f, 135.0f);
        motion->target_right_mm_s = bounded(v+omega*ROBOT_TRACK_MM*0.5f, 15.0f, 135.0f);
        return;
    }
    if (motion->phase == MOTION_SETTLE) {
        if ((uint32_t)(now_ms-motion->phase_at_ms) < 350u) return;
        if (distance > 20.0f) { motion->phase = MOTION_ERROR; return; }
        if (++motion->next_waypoint == robot_route_count) motion->phase = MOTION_FINISHED;
        else motion->phase = MOTION_ALIGN;
        motion->phase_at_ms = now_ms;
    }
}
