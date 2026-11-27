#include "bsp.h"
#include "encoder_driver.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// One-shot bench diagnostic. Lift both wheels before pressing PB1.
// Result: 1 blink = both directions correct, 2 = reverse did not rotate,
// 3 = reverse encoder sign wrong, 4 = forward did not rotate, 5 = BSP failure.
typedef struct {
    volatile uint32_t magic;
    volatile uint8_t phase, result;
    volatile int32_t reverse_counts, forward_counts;
    volatile uint32_t elapsed_ms;
} MotorDiag;
volatile MotorDiag g_motor_diag = {.magic=0x524d4447u};

void SysTick_Handler(void) { HAL_IncTick(); }

static void stop(void) {
    bsp_motor_standby(false);
    bsp_motor_output(0,0);
    bsp_motor_output(1,0);
}

int main(void) {
    if (!bsp_init()) {
        g_motor_diag.result=5;
        for (;;) {}
    }
    stop();
    encoder_init();
    uint32_t at=bsp_millis();
    bool released=false;
    uint32_t press_at=0;
    IWDG->KR=0xCCCCu;
    IWDG->KR=0x5555u;
    IWDG->PR=4u;
    IWDG->RLR=625u;
    while(IWDG->SR!=0u) {}
    IWDG->KR=0xAAAAu;
    g_motor_diag.phase=1; // READY
    for (;;) {
        IWDG->KR=0xAAAAu;
        const uint32_t now=bsp_millis();
        if (g_motor_diag.phase==1) {
            bsp_led((now/500u)%2u==0);
            if (!bsp_start_pressed()) { released=true;press_at=0; }
            else if (released) {
                if (!press_at) press_at=now;
                if ((uint32_t)(now-press_at)>=50u) {
                    g_motor_diag.phase=2;at=now;stop();
                }
            }
        } else if (g_motor_diag.phase==2) {
            bsp_led((now/100u)%2u==0);
            if ((uint32_t)(now-at)>=3000u) {
                encoder_init();
                g_motor_diag.reverse_counts=0;
                g_motor_diag.phase=3;at=now;
                bsp_motor_output(1,-0.50f);
                bsp_motor_standby(true);
            }
        } else if (g_motor_diag.phase==3) {
            int32_t l,r;encoder_pose_delta(&l,&r);
            g_motor_diag.reverse_counts+=r;
            if ((uint32_t)(now-at)>=700u) {
                stop();g_motor_diag.phase=4;at=now;
            }
        } else if (g_motor_diag.phase==4) {
            if ((uint32_t)(now-at)>=500u) {
                encoder_init();
                g_motor_diag.forward_counts=0;
                g_motor_diag.phase=5;at=now;
                bsp_motor_output(1,0.50f);
                bsp_motor_standby(true);
            }
        } else if (g_motor_diag.phase==5) {
            int32_t l,r;encoder_pose_delta(&l,&r);
            g_motor_diag.forward_counts+=r;
            if ((uint32_t)(now-at)>=700u) {
                stop();g_motor_diag.phase=6;at=now;
                g_motor_diag.result=g_motor_diag.reverse_counts < -10 ?
                    (g_motor_diag.forward_counts > 10 ? 1 : 4) :
                    (g_motor_diag.reverse_counts > 10 ? 3 : 2);
            }
        } else {
            uint32_t n=g_motor_diag.result;
            uint32_t period=n*300u+1200u,t=now%period;
            bsp_led(t<n*300u && (t%300u)<150u);
        }
        g_motor_diag.elapsed_ms=now;
    }
}
