#include "encoder_driver.h"
#include "bsp.h"
#include "robot_config.h"

static uint16_t speed_previous[2], pose_previous[2];

int32_t encoder_wrap_delta(uint16_t current, uint16_t previous) {
    const uint16_t delta = (uint16_t)(current - previous);
    return delta < 32768u ? (int32_t)delta : (int32_t)delta - 65536;
}

void encoder_init(void) {
    for (unsigned i = 0; i < 2; ++i)
        speed_previous[i] = pose_previous[i] = bsp_encoder_raw(i);
}

static void sample(uint16_t previous[2], int32_t *left, int32_t *right) {
    const uint16_t l = bsp_encoder_raw(0), r = bsp_encoder_raw(1);
    *left = encoder_wrap_delta(l, previous[0]) * ROBOT_ENCODER_SIGN_L;
    *right = encoder_wrap_delta(r, previous[1]) * ROBOT_ENCODER_SIGN_R;
    previous[0] = l;
    previous[1] = r;
}

void encoder_speed_delta(int32_t *left, int32_t *right) { sample(speed_previous, left, right); }
void encoder_pose_delta(int32_t *left, int32_t *right) { sample(pose_previous, left, right); }
