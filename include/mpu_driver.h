#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float bias_raw;
    float last_rate_rad_s;
    uint32_t last_ok_ms;
    bool ready;
} MpuDriver;

bool mpu_init(MpuDriver *imu);
bool mpu_calibrate(MpuDriver *imu);
bool mpu_read_rate(MpuDriver *imu, float *rate_rad_s, uint32_t now_ms);
