#pragma once
#include <stdint.h>
#include "robot_types.h"
#include "odometry.h"

void diagnostic_init(void);
void diagnostic_capture_fault(void);
void diagnostic_publish(uint32_t now_ms, uint32_t run_started_ms,
                        const Odometry *odo, int32_t dl, int32_t dr,
                        float gyro_rad_s, const float measured[2],
                        const float duty[2], uint8_t motor_enabled);
