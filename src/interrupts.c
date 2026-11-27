#include "stm32f1xx_hal.h"
#include "motor_control.h"

void SysTick_Handler(void) {
    HAL_IncTick();
    motor_control_lease_isr(HAL_GetTick());
}

void TIM1_UP_IRQHandler(void) {
    if (TIM1->SR & TIM_SR_UIF) {
        TIM1->SR &= ~TIM_SR_UIF;
        motor_control_tick_isr();
    }
}
