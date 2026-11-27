#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "robot_types.h"

typedef struct {
    RobotPose pose;
    float left_mm_s, right_mm_s;
    float encoder_yaw_rad_s;
    float gyro_yaw_rad_s;
} Odometry;

float angle_wrap(float angle_rad);
void odometry_reset(Odometry *odo, RobotPose start);
bool odometry_step(Odometry *odo, int32_t left_counts, int32_t right_counts,
                   float gyro_rad_s, float dt_s);
