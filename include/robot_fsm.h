#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "robot_types.h"
#include "motion_controller.h"
#include "safety_manager.h"
#include "odometry.h"

typedef struct {
    RobotState state;
    RobotFault fault;
    MotionController motion;
    SafetyManager safety;
    uint32_t entered_ms, run_started_ms;
    uint8_t button_samples;
    bool button_was_stable;
    bool button_released;
} RobotFsm;

void robot_fsm_init(RobotFsm *fsm, uint32_t now_ms);
void robot_fsm_ready(RobotFsm *fsm, uint32_t now_ms);
void robot_fsm_fail(RobotFsm *fsm, RobotFault fault, uint32_t now_ms);
void robot_fsm_step(RobotFsm *fsm, Odometry *odo, int32_t dl, int32_t dr,
                    uint32_t imu_last_ok_ms, bool button_down, uint32_t now_ms);
