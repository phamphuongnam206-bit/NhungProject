#include "mpu_driver.h"
#include "bsp.h"
#include "robot_config.h"
#include "stm32f1xx_hal.h"
#include <math.h>

static bool reg_is(uint8_t reg, uint8_t expected) {
    uint8_t value = 0;
    return bsp_mpu_read(reg, &value, 1) && value == expected;
}

static bool raw_z(int16_t *value) {
    uint8_t bytes[2];
    if (!bsp_mpu_read(0x47, bytes, 2)) return false;
    *value = (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
    return true;
}

bool mpu_init(MpuDriver *imu) {
    *imu = (MpuDriver){0};
    if (!reg_is(0x75, 0x68)) return false;
    if (!bsp_mpu_write(0x6b, 0x01) || // PLL from gyro X
        !bsp_mpu_write(0x1a, 0x03) || // DLPF
        !bsp_mpu_write(0x19, 0x09) || // divider
        !bsp_mpu_write(0x1b, 0x08)) return false; // 500 dps, 65.5 LSB/dps
    if (!reg_is(0x1a, 0x03) || !reg_is(0x19, 0x09) || !reg_is(0x1b, 0x08)) return false;
    imu->ready = true;
    return true;
}

bool mpu_calibrate(MpuDriver *imu) {
    if (!imu->ready) return false;
    HAL_Delay(2000); // startup only, motors remain disabled
    double sum = 0, squares = 0;
    for (unsigned i = 0; i < 400; ++i) {
        int16_t value;
        if (!raw_z(&value)) return false;
        sum += value;
        squares += (double)value * value;
        HAL_Delay(5);
    }
    const double mean = sum / 400.0;
    const double variance = squares / 400.0 - mean * mean;
    if (variance > (0.01 * 65.5 * 180.0 / 3.141592653589793) *
                   (0.01 * 65.5 * 180.0 / 3.141592653589793) ||
        fabs(mean) > (0.1 * 65.5 * 180.0 / 3.141592653589793)) return false;
    imu->bias_raw = (float)mean;
    imu->last_ok_ms = bsp_millis();
    return true;
}

bool mpu_read_rate(MpuDriver *imu, float *rate_rad_s, uint32_t now_ms) {
    if (!imu->ready) return false;
    int16_t raw;
    if (!raw_z(&raw)) return false;
    *rate_rad_s = ROBOT_GYRO_SIGN * ((float)raw - imu->bias_raw) *
                  (0.01745329252f / 65.5f);
    if (!isfinite(*rate_rad_s)) return false;
    imu->last_rate_rad_s = *rate_rad_s;
    imu->last_ok_ms = now_ms;
    return true;
}
