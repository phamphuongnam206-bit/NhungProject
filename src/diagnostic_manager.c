#include "diagnostic_manager.h"

volatile RobotDiagnostic g_robot_status;
volatile RobotDiagnostic g_robot_fault_snapshot;

void diagnostic_init(void) {
    g_robot_status.magic = 0x524f424fu; // ROBO
    g_robot_status.build_id = 0x20261127u;
    g_robot_status.state = ROBOT_BOOT;
    g_robot_status.fault = FAULT_NONE;
    g_robot_fault_snapshot.magic = 0;
}

void diagnostic_capture_fault(void) {
    // This image is deliberately immutable until power-cycle.
    if (g_robot_fault_snapshot.magic != 0x524f424fu)
        g_robot_fault_snapshot = g_robot_status;
}

void diagnostic_publish(uint32_t now_ms, uint32_t run_started_ms,
                        const Odometry *odo, int32_t dl, int32_t dr,
                        float gyro_rad_s, const float measured[2],
                        const float duty[2], uint8_t motor_enabled) {
    g_robot_status.uptime_ms = now_ms;
    g_robot_status.run_ms = run_started_ms ? now_ms-run_started_ms : 0;
    g_robot_status.encoder_l += dl;
    g_robot_status.encoder_r += dr;
    g_robot_status.gyro_rad_s = gyro_rad_s;
    g_robot_status.x_mm = odo->pose.x_mm;
    g_robot_status.y_mm = odo->pose.y_mm;
    g_robot_status.heading_rad = odo->pose.heading_rad;
    g_robot_status.measured_l_mm_s = measured[0];
    g_robot_status.measured_r_mm_s = measured[1];
    g_robot_status.duty_l = duty[0];
    g_robot_status.duty_r = duty[1];
    g_robot_status.motor_enabled = motor_enabled;
}
