#include "odometry.h"
#include "robot_config.h"
#include <math.h>

float angle_wrap(float angle_rad) { return remainderf(angle_rad, 6.28318530718f); }

void odometry_reset(Odometry *odo, RobotPose start) {
    *odo = (Odometry){0};
    odo->pose = start;
}

bool odometry_step(Odometry *odo, int32_t left_counts, int32_t right_counts,
                   float gyro_rad_s, float dt_s) {
    if (!isfinite(gyro_rad_s) || !isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.1f)
        return false;
    const float left = left_counts * ROBOT_MM_PER_COUNT_L;
    const float right = right_counts * ROBOT_MM_PER_COUNT_R;
    const float encoder_rate = (right - left) / (ROBOT_TRACK_MM * dt_s);
    const float rate = fabsf(encoder_rate - gyro_rad_s) > 1.0f ?
                       gyro_rad_s : 0.98f * gyro_rad_s + 0.02f * encoder_rate;
    const float delta = rate * dt_s;
    const float halfway = odo->pose.heading_rad + 0.5f * delta;
    const float centre = 0.5f * (left + right);
    odo->pose.x_mm += centre * cosf(halfway);
    odo->pose.y_mm += centre * sinf(halfway);
    odo->pose.heading_rad = angle_wrap(odo->pose.heading_rad + delta);
    odo->left_mm_s = left / dt_s;
    odo->right_mm_s = right / dt_s;
    odo->encoder_yaw_rad_s = encoder_rate;
    odo->gyro_yaw_rad_s = gyro_rad_s;
    return isfinite(odo->pose.x_mm) && isfinite(odo->pose.y_mm) &&
           isfinite(odo->pose.heading_rad);
}
