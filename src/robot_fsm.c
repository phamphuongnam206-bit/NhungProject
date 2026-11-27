#include "robot_fsm.h"
#include "motor_control.h"
#include "robot_config.h"
#include "static_map.h"
#include "diagnostic_manager.h"
#include <math.h>

static void enter(RobotFsm *fsm, RobotState state, uint32_t now_ms) {
    fsm->state = state;
    fsm->entered_ms = now_ms;
    g_robot_status.state = state;
    motor_control_stop();
}

void robot_fsm_init(RobotFsm *fsm, uint32_t now_ms) {
    *fsm = (RobotFsm){0};
    fsm->state = ROBOT_BOOT;
    fsm->entered_ms = now_ms;
    g_robot_status.state = fsm->state;
}

void robot_fsm_ready(RobotFsm *fsm, uint32_t now_ms) {
    fsm->fault = FAULT_NONE;
    fsm->button_released = false;
    enter(fsm, ROBOT_READY, now_ms);
}

void robot_fsm_fail(RobotFsm *fsm, RobotFault fault, uint32_t now_ms) {
    if (fsm->state == ROBOT_FAULT) return;
    motor_control_stop();
    fsm->fault = fault;
    fsm->state = ROBOT_FAULT;
    fsm->entered_ms = now_ms;
    g_robot_status.state = ROBOT_FAULT;
    g_robot_status.fault = fault;
    // Freeze a compact fault snapshot for SWD inspection until power-cycle.
    diagnostic_capture_fault();
}

static bool button_event(RobotFsm *fsm, bool down) {
    if (!down) {
        fsm->button_samples = 0;
        fsm->button_was_stable = false;
        fsm->button_released = true;
        return false;
    }
    if (!fsm->button_released) return false;
    if (fsm->button_samples < 3u) ++fsm->button_samples;
    if (fsm->button_samples >= 3u && !fsm->button_was_stable) {
        fsm->button_was_stable = true;
        return true;
    }
    return false;
}

void robot_fsm_step(RobotFsm *fsm, Odometry *odo, int32_t dl, int32_t dr,
                    uint32_t imu_last_ok_ms, bool button_down, uint32_t now_ms) {
    if (fsm->state == ROBOT_FAULT) return;
    const bool pressed = button_event(fsm, button_down);
    if (fsm->state == ROBOT_READY || fsm->state == ROBOT_GOAL || fsm->state == ROBOT_IDLE) {
        if (pressed) {
            odometry_reset(odo, (RobotPose){ROBOT_START_X_MM,ROBOT_START_Y_MM,
                                              ROBOT_START_HEADING_RAD});
            safety_reset(&fsm->safety, now_ms);
            enter(fsm, ROBOT_COUNTDOWN, now_ms);
        }
        return;
    }
    if (fsm->state == ROBOT_COUNTDOWN) {
        if (pressed) { enter(fsm, ROBOT_IDLE, now_ms); return; }
        if (dl || dr || fabsf(odo->gyro_yaw_rad_s) > 0.08f) {
            enter(fsm, ROBOT_IDLE, now_ms);
            return;
        }
        if ((uint32_t)(now_ms-fsm->entered_ms) >= 3000u) {
            // The immutable route was swept during boot before READY.
            odometry_reset(odo, (RobotPose){ROBOT_START_X_MM,ROBOT_START_Y_MM,
                                              ROBOT_START_HEADING_RAD});
            motion_begin(&fsm->motion, now_ms);
            safety_reset(&fsm->safety, now_ms);
            fsm->run_started_ms = now_ms;
            enter(fsm, ROBOT_ALIGN_HEADING, now_ms);
        }
        return;
    }
    if (pressed) { enter(fsm, ROBOT_IDLE, now_ms); return; } // STOP
    if ((uint32_t)(now_ms-fsm->run_started_ms) > ROBOT_RUN_LIMIT_MS) {
        robot_fsm_fail(fsm, FAULT_TIMEOUT, now_ms); return;
    }
    if (!motor_control_enabled() &&
        (fsm->state == ROBOT_ALIGN_HEADING || fsm->state == ROBOT_NAVIGATING) &&
        (uint32_t)(now_ms-fsm->entered_ms) > 100u) {
        robot_fsm_fail(fsm, FAULT_COMMAND, now_ms); return;
    }
    RobotFault fault = safety_check(&fsm->safety, odo, dl, dr,
                                    fsm->motion.target_left_mm_s,
                                    fsm->motion.target_right_mm_s,
                                    imu_last_ok_ms, now_ms);
    if (fault != FAULT_NONE) { robot_fsm_fail(fsm, fault, now_ms); return; }
    motion_update(&fsm->motion, odo->pose, now_ms);
    g_robot_status.waypoint = fsm->motion.next_waypoint;
    if (fsm->motion.phase == MOTION_ERROR) {
        robot_fsm_fail(fsm, FAULT_ROUTE, now_ms); return;
    }
    if (fsm->motion.phase == MOTION_FINISHED) {
        enter(fsm, ROBOT_GOAL, now_ms); return;
    }
    if (fsm->motion.phase == MOTION_SETTLE ||
        fsm->motion.phase == MOTION_TURN_SETTLE) {
        if (fsm->state != ROBOT_BRAKING) enter(fsm, ROBOT_BRAKING, now_ms);
        return;
    }
    const RobotState next = fsm->motion.phase == MOTION_ALIGN ?
                            ROBOT_ALIGN_HEADING : ROBOT_NAVIGATING;
    if (fsm->state != next) enter(fsm, next, now_ms);
    motor_control_set(fsm->motion.target_left_mm_s,
                      fsm->motion.target_right_mm_s, now_ms);
    g_robot_status.target_l_mm_s = fsm->motion.target_left_mm_s;
    g_robot_status.target_r_mm_s = fsm->motion.target_right_mm_s;
}
