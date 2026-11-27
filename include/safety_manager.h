#pragma once
#include <stdint.h>
#include "robot_types.h"
#include "odometry.h"

typedef struct {
    uint32_t left_last_pulse_ms, right_last_pulse_ms;
    uint32_t yaw_mismatch_since_ms;
} SafetyManager;

void safety_reset(SafetyManager *manager, uint32_t now_ms);
RobotFault safety_check(SafetyManager *manager, const Odometry *odo,
                        int32_t left_delta, int32_t right_delta,
                        float target_left, float target_right,
                        uint32_t imu_last_ok_ms, uint32_t now_ms);
