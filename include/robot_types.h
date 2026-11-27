#pragma once
#include <stdint.h>

typedef enum {
    ROBOT_BOOT, ROBOT_SELF_TEST, ROBOT_CALIBRATION, ROBOT_IDLE,
    ROBOT_READY, ROBOT_COUNTDOWN, ROBOT_ALIGN_HEADING,
    ROBOT_NAVIGATING, ROBOT_BRAKING, ROBOT_REPLANNING,
    ROBOT_GOAL, ROBOT_FAULT
} RobotState;

typedef enum {
    FAULT_NONE, FAULT_BOARD, FAULT_IMU, FAULT_ENCODER_L,
    FAULT_ENCODER_R, FAULT_STALL, FAULT_TIMING, FAULT_COMMAND,
    FAULT_ROUTE, FAULT_CLEARANCE, FAULT_POSE, FAULT_TIMEOUT,
    FAULT_UNSUPPORTED_PROFILE
} RobotFault;

typedef struct { float x_mm, y_mm, heading_rad; } RobotPose;
typedef struct { float x_mm, y_mm; } RobotPoint;

typedef struct {
    volatile uint32_t magic;
    volatile uint32_t build_id;
    volatile RobotState state;
    volatile RobotFault fault;
    volatile uint32_t uptime_ms;
    volatile uint32_t run_ms;
    volatile int32_t encoder_l, encoder_r;
    volatile float gyro_rad_s, gyro_bias_rad_s;
    volatile float x_mm, y_mm, heading_rad;
    volatile float target_l_mm_s, target_r_mm_s;
    volatile float measured_l_mm_s, measured_r_mm_s;
    volatile float duty_l, duty_r;
    volatile uint8_t waypoint;
    volatile uint8_t imu_ok;
    volatile uint8_t motor_enabled;
    volatile uint8_t reserved;
    volatile uint32_t max_loop_ms;
} RobotDiagnostic;

extern volatile RobotDiagnostic g_robot_status;
extern volatile RobotDiagnostic g_robot_fault_snapshot;
