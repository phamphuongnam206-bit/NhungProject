#pragma once
#include <stdint.h>

// Board and measured values from the running PCB port. Units are millimetres,
// seconds, radians and normalized PWM duty unless a name states otherwise.
#define ROBOT_TRACK_MM 90.0f
#define ROBOT_MM_PER_COUNT_L (10.0f / 56.66f)
#define ROBOT_MM_PER_COUNT_R (10.0f / 51.24f)
#define ROBOT_ENCODER_SIGN_L (-1)
#define ROBOT_ENCODER_SIGN_R (1)
#define ROBOT_MOTOR_SIGN_L (-1)
#define ROBOT_MOTOR_SIGN_R (1)
#define ROBOT_GYRO_SIGN (1.0f)
#define ROBOT_FF_GAIN_L (1000.0f)
#define ROBOT_FF_GAIN_R (1085.68f)
#define ROBOT_FF_OFFSET (0.16f)
#define ROBOT_SPEED_KP (0.0006f)
#define ROBOT_SPEED_KI (0.003f)
#define ROBOT_PI_LIMIT (0.3f)
#define ROBOT_PWM_LIMIT (140.0f / 255.0f)
#define ROBOT_SPEED_PERIOD_MS (5u)
#define ROBOT_CONTROL_PERIOD_MS (20u)
#define ROBOT_COMMAND_LEASE_MS (150u)
#define ROBOT_RUN_LIMIT_MS (240000u)

#define ROBOT_START_X_MM (300.0f)
#define ROBOT_START_Y_MM (-80.0f)
#define ROBOT_START_HEADING_RAD (1.5707963268f)
#define ROBOT_BODY_HALF_LENGTH_MM (65.0f)
#define ROBOT_BODY_HALF_WIDTH_MM (50.0f)
#define ROBOT_CLEARANCE_MM (15.0f)
