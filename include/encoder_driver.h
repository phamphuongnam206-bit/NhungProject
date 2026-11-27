#pragma once
#include <stdint.h>

void encoder_init(void);
void encoder_speed_delta(int32_t *left, int32_t *right);
void encoder_pose_delta(int32_t *left, int32_t *right);
int32_t encoder_wrap_delta(uint16_t current, uint16_t previous);
