#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "robot_types.h"

typedef enum { MOTION_ALIGN, MOTION_TURN_SETTLE, MOTION_DRIVE,
               MOTION_SETTLE, MOTION_FINISHED, MOTION_ERROR } MotionPhase;
typedef struct {
    MotionPhase phase;
    uint8_t next_waypoint;
    uint32_t phase_at_ms;
    float target_left_mm_s, target_right_mm_s;
} MotionController;

void motion_begin(MotionController *motion, uint32_t now_ms);
void motion_update(MotionController *motion, RobotPose pose, uint32_t now_ms);
