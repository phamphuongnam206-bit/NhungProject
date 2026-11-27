#pragma once
#include <stdbool.h>
#include <stdint.h>

bool bsp_init(void);
uint32_t bsp_millis(void);
bool bsp_start_pressed(void);
void bsp_led(bool on);
void bsp_motor_standby(bool on);
void bsp_motor_output(unsigned wheel, float signed_duty);
uint16_t bsp_encoder_raw(unsigned wheel);
bool bsp_mpu_read(uint8_t reg, uint8_t *data, uint16_t len);
bool bsp_mpu_write(uint8_t reg, uint8_t value);
