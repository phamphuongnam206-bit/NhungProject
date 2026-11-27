#include "motor_control.h"
#include "bsp.h"
#include "encoder_driver.h"
#include "robot_config.h"
#include "speed_pi.h"
#include "stm32f1xx_hal.h"
#include <math.h>

static WheelControl wheels[2];
static volatile uint32_t command_at;
static volatile uint8_t enabled;

void motor_control_init(void) {
    for (unsigned i = 0; i < 2; ++i) wheels[i] = (WheelControl){0};
    command_at = 0;
    enabled = 0;
    bsp_motor_output(0, 0);
    bsp_motor_output(1, 0);
    bsp_motor_standby(false);
}

void motor_control_stop(void) {
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    enabled = 0;
    bsp_motor_standby(false);
    bsp_motor_output(0, 0);
    bsp_motor_output(1, 0);
    for (unsigned i = 0; i < 2; ++i) {
        wheels[i].target_mm_s = 0;
        wheels[i].integral = 0;
        wheels[i].duty = 0;
        wheels[i].applied_direction = 0;
    }
    __set_PRIMASK(mask);
}

void motor_control_set(float left_mm_s, float right_mm_s, uint32_t now_ms) {
    if (!isfinite(left_mm_s) || !isfinite(right_mm_s) ||
        fabsf(left_mm_s) > 200.0f || fabsf(right_mm_s) > 200.0f) {
        motor_control_stop();
        return;
    }
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    // A commanded reversal starts at zero duty. The motion FSM decides when to pivot.
    const float requested[2] = {left_mm_s, right_mm_s};
    const uint32_t elapsed = enabled ? (uint32_t)(now_ms-command_at) : ROBOT_CONTROL_PERIOD_MS;
    const float dt = fminf(elapsed*0.001f,0.020f);
    for (unsigned i = 0; i < 2; ++i) {
        WheelControl *w = &wheels[i];
        const float limit = fabsf(requested[i]) > fabsf(w->target_mm_s) ?
                            400.0f*dt : 600.0f*dt;
        const float change = fmaxf(-limit,fminf(limit,requested[i]-w->target_mm_s));
        float next = w->target_mm_s + change;
        if (w->target_mm_s * requested[i] < 0 && fabsf(w->target_mm_s) > limit)
            next = w->target_mm_s > 0 ? w->target_mm_s-limit : w->target_mm_s+limit;
        const int8_t direction = next > 1.0f ? 1 : next < -1.0f ? -1 : 0;
        if (direction != w->applied_direction) {
            bsp_motor_output(i, 0);
            w->integral = 0;
            w->applied_direction = direction;
        }
        w->target_mm_s = next;
    }
    command_at = now_ms;
    enabled = 1;
    bsp_motor_standby(true);
    __set_PRIMASK(mask);
}

void motor_control_lease_isr(uint32_t now_ms) {
    if (enabled && (uint32_t)(now_ms - command_at) >= ROBOT_COMMAND_LEASE_MS) {
        enabled = 0;
        bsp_motor_standby(false);
        bsp_motor_output(0, 0);
        bsp_motor_output(1, 0);
    }
}

void motor_control_tick_isr(void) {
    int32_t counts[2];
    encoder_speed_delta(&counts[0], &counts[1]);
    for (unsigned i = 0; i < 2; ++i) {
        WheelControl *w = &wheels[i];
        w->counts_window[w->window_index] = counts[i];
        w->window_index = (uint8_t)((w->window_index + 1u) % 4u);
        int32_t sum = 0;
        for (unsigned k = 0; k < 4; ++k) sum += w->counts_window[k];
        const float mm_per_count = i == 0 ? ROBOT_MM_PER_COUNT_L : ROBOT_MM_PER_COUNT_R;
        w->measured_mm_s = sum * mm_per_count / 0.020f;
        if (!enabled || fabsf(w->target_mm_s) < 10.0f) {
            w->integral = 0;
            w->duty = 0;
            bsp_motor_output(i, 0);
            continue;
        }
        const float gain = i == 0 ? ROBOT_FF_GAIN_L : ROBOT_FF_GAIN_R;
        w->duty = speed_pi_step(w->target_mm_s,w->measured_mm_s,
                                 gain,0.005f,&w->integral);
        bsp_motor_output(i, w->duty);
    }
}

bool motor_control_enabled(void) { return enabled != 0; }
void motor_control_snapshot(float measured[2], float duty[2]) {
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    for (unsigned i = 0; i < 2; ++i) {
        measured[i] = wheels[i].measured_mm_s;
        duty[i] = wheels[i].duty;
    }
    __set_PRIMASK(mask);
}
