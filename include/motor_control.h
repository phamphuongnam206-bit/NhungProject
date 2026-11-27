#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float target_mm_s;
    float measured_mm_s;
    float duty;
    float integral;
    int32_t counts_window[4];
    uint8_t window_index;
    int8_t applied_direction;
    uint16_t low_motion_ms;
} WheelControl;

void motor_control_init(void);
void motor_control_set(float left_mm_s, float right_mm_s, uint32_t now_ms);
void motor_control_stop(void);
void motor_control_tick_isr(void);
void motor_control_lease_isr(uint32_t now_ms);
bool motor_control_enabled(void);
void motor_control_snapshot(float measured[2], float duty[2]);
