#include "safety_manager.h"
#include "static_map.h"
#include "robot_config.h"
#include <math.h>

void safety_reset(SafetyManager *manager, uint32_t now_ms) {
    *manager = (SafetyManager){now_ms,now_ms,0};
}

RobotFault safety_check(SafetyManager *manager, const Odometry *odo,
                        int32_t left_delta, int32_t right_delta,
                        float target_left, float target_right,
                        uint32_t imu_last_ok_ms, uint32_t now_ms) {
    if ((uint32_t)(now_ms-imu_last_ok_ms) > 100u) return FAULT_IMU;
    if (!isfinite(odo->pose.x_mm) || !isfinite(odo->pose.y_mm) ||
        !isfinite(odo->pose.heading_rad)) return FAULT_POSE;
    if (!map_body_clear(odo->pose, ROBOT_CLEARANCE_MM)) return FAULT_CLEARANCE;
    if (left_delta != 0) manager->left_last_pulse_ms = now_ms;
    if (right_delta != 0) manager->right_last_pulse_ms = now_ms;
    if (fabsf(target_left) > 40.0f &&
        (uint32_t)(now_ms-manager->left_last_pulse_ms) > 600u) return FAULT_ENCODER_L;
    if (fabsf(target_right) > 40.0f &&
        (uint32_t)(now_ms-manager->right_last_pulse_ms) > 600u) return FAULT_ENCODER_R;
    if (fabsf(odo->encoder_yaw_rad_s) > 0.6f && fabsf(odo->gyro_yaw_rad_s) < 0.12f) {
        if (manager->yaw_mismatch_since_ms == 0) manager->yaw_mismatch_since_ms = now_ms;
        else if ((uint32_t)(now_ms-manager->yaw_mismatch_since_ms) > 500u) return FAULT_POSE;
    } else manager->yaw_mismatch_since_ms = 0;
    return FAULT_NONE;
}
