#include "bsp.h"
#include "robot_config.h"
#include "stm32f1xx_hal.h"
#include <math.h>

static TIM_HandleTypeDef pwm_timer, left_counter, right_counter;
#ifndef ROBOT_MOTOR_DIAG
static TIM_HandleTypeDef speed_timer;
#endif
static I2C_HandleTypeDef imu_bus;
volatile uint8_t g_bsp_stage;

static bool clock_start(void) {
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) return false;
    RCC_ClkInitTypeDef clocks = {0};
    clocks.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clocks.APB1CLKDivider = RCC_HCLK_DIV2;
    clocks.APB2CLKDivider = RCC_HCLK_DIV1;
    return HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_2) == HAL_OK;
}

static void gpio_start(void) {
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
    GPIO_InitTypeDef io = {0};
    io.Mode = GPIO_MODE_OUTPUT_PP;
    io.Speed = GPIO_SPEED_FREQ_HIGH;
    io.Pin = GPIO_PIN_0 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_WritePin(GPIOB, io.Pin, GPIO_PIN_RESET);
    HAL_GPIO_Init(GPIOB, &io);
    io.Pin = GPIO_PIN_13;
    HAL_GPIO_WritePin(GPIOC, io.Pin, GPIO_PIN_SET);
    HAL_GPIO_Init(GPIOC, &io);
    io.Mode = GPIO_MODE_AF_PP;
    io.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &io);
    io.Mode = GPIO_MODE_INPUT;
    io.Pull = GPIO_PULLUP;
    io.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &io);
    io.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &io);
    io.Mode = GPIO_MODE_AF_OD;
    io.Pull = GPIO_NOPULL;
    io.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &io);
}

static bool pwm_start(void) {
    __HAL_RCC_TIM4_CLK_ENABLE();
    pwm_timer.Instance = TIM4;
    pwm_timer.Init.Prescaler = 0;
    pwm_timer.Init.Period = 3599; // 72 MHz / 3600 = 20 kHz
    pwm_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    pwm_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    pwm_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&pwm_timer) != HAL_OK) return false;
    TIM_OC_InitTypeDef channel = {0};
    channel.OCMode = TIM_OCMODE_PWM1;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    return HAL_TIM_PWM_ConfigChannel(&pwm_timer, &channel, TIM_CHANNEL_3) == HAL_OK &&
           HAL_TIM_PWM_ConfigChannel(&pwm_timer, &channel, TIM_CHANNEL_4) == HAL_OK &&
           HAL_TIM_PWM_Start(&pwm_timer, TIM_CHANNEL_3) == HAL_OK &&
           HAL_TIM_PWM_Start(&pwm_timer, TIM_CHANNEL_4) == HAL_OK;
}

static bool counter_start(TIM_HandleTypeDef *timer, TIM_TypeDef *instance) {
    timer->Instance = instance;
    timer->Init.Prescaler = 0;
    timer->Init.Period = 0xffff;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM_Encoder_InitTypeDef channels = {0};
    channels.EncoderMode = TIM_ENCODERMODE_TI12;
    channels.IC1Polarity = TIM_ICPOLARITY_RISING;
    channels.IC2Polarity = TIM_ICPOLARITY_RISING;
    channels.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    channels.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    channels.IC1Prescaler = TIM_ICPSC_DIV1;
    channels.IC2Prescaler = TIM_ICPSC_DIV1;
    channels.IC1Filter = 6;
    channels.IC2Filter = 6;
    if (HAL_TIM_Encoder_Init(timer, &channels) != HAL_OK) return false;
    __HAL_TIM_SET_COUNTER(timer, 0);
    return HAL_TIM_Encoder_Start(timer, TIM_CHANNEL_ALL) == HAL_OK;
}

static bool imu_bus_start(void) {
    // Release a slave left midway through a transfer by an earlier reset.
    GPIO_InitTypeDef pins = {0};
    pins.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    pins.Mode = GPIO_MODE_OUTPUT_OD;
    pins.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_WritePin(GPIOB,pins.Pin,GPIO_PIN_SET);
    HAL_GPIO_Init(GPIOB,&pins);
    for (unsigned i=0; i<9 && HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)==GPIO_PIN_RESET; ++i) {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
        for (volatile unsigned wait=0;wait<400;++wait) {}
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
        for (volatile unsigned wait=0;wait<400;++wait) {}
    }
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
    // Test the actual released lines while GPIO still owns the open-drain pins.
    // In AF mode with I2C disabled, the pins need not read HIGH yet.
    const bool released = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)!=GPIO_PIN_RESET &&
                          HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)!=GPIO_PIN_RESET;
    pins.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(GPIOB,&pins);
    if (!released) return false;
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    imu_bus.Instance = I2C1;
    imu_bus.Init.ClockSpeed = 400000;
    imu_bus.Init.DutyCycle = I2C_DUTYCYCLE_2;
    imu_bus.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    imu_bus.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    imu_bus.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    imu_bus.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    return HAL_I2C_Init(&imu_bus) == HAL_OK;
}

bool bsp_init(void) {
    if (HAL_Init() != HAL_OK) return false;
    g_bsp_stage = 1;
    if (!clock_start()) return false;
    g_bsp_stage = 2;
    gpio_start();
    g_bsp_stage = 3;
    if (!pwm_start()) return false;
    g_bsp_stage = 4;
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    if (!counter_start(&left_counter, TIM3)) return false;
    g_bsp_stage = 5;
    if (!counter_start(&right_counter, TIM2)) return false;
    g_bsp_stage = 6;
    if (!imu_bus_start()) return false;
    g_bsp_stage = 7;
#ifndef ROBOT_MOTOR_DIAG
    __HAL_RCC_TIM1_CLK_ENABLE();
    speed_timer.Instance = TIM1;
    speed_timer.Init.Prescaler = 7199;
    speed_timer.Init.Period = 49; // 72 MHz / 7200 / 50 = 200 Hz
    speed_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    speed_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&speed_timer) != HAL_OK) return false;
    g_bsp_stage = 8;
    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
    if (HAL_TIM_Base_Start_IT(&speed_timer) != HAL_OK) return false;
    g_bsp_stage = 9;
#endif
    return true;
}

uint32_t bsp_millis(void) { return HAL_GetTick(); }
bool bsp_start_pressed(void) { return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET; }
void bsp_led(bool on) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, on ? GPIO_PIN_RESET : GPIO_PIN_SET); }
void bsp_motor_standby(bool on) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }

void bsp_motor_output(unsigned wheel, float duty) {
    if (wheel > 1u) return;
    if (!isfinite(duty)) duty = 0.0f;
    if (duty > ROBOT_PWM_LIMIT) duty = ROBOT_PWM_LIMIT;
    if (duty < -ROBOT_PWM_LIMIT) duty = -ROBOT_PWM_LIMIT;
    duty *= wheel == 0u ? ROBOT_MOTOR_SIGN_L : ROBOT_MOTOR_SIGN_R;
    const uint16_t a = wheel == 0u ? GPIO_PIN_14 : GPIO_PIN_12;
    const uint16_t b = wheel == 0u ? GPIO_PIN_15 : GPIO_PIN_13;
    // Clear duty before changing direction; controller calls this only after a target-direction transition.
    __HAL_TIM_SET_COMPARE(&pwm_timer, wheel == 0u ? TIM_CHANNEL_4 : TIM_CHANNEL_3, 0);
    HAL_GPIO_WritePin(GPIOB, a, duty > 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, b, duty < 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&pwm_timer, wheel == 0u ? TIM_CHANNEL_4 : TIM_CHANNEL_3,
                          (uint32_t)(fabsf(duty) * 3600.0f));
}

uint16_t bsp_encoder_raw(unsigned wheel) {
    return (uint16_t)__HAL_TIM_GET_COUNTER(wheel == 0u ? &left_counter : &right_counter);
}

bool bsp_mpu_read(uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Read(&imu_bus, 0x68u << 1, reg, I2C_MEMADD_SIZE_8BIT, data, len, 3) == HAL_OK;
}
bool bsp_mpu_write(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(&imu_bus, 0x68u << 1, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 3) == HAL_OK;
}
