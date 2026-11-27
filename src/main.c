#include "bsp.h"
#include "encoder_driver.h"
#include "motor_control.h"
#include "mpu_driver.h"
#include "odometry.h"
#include "robot_fsm.h"
#include "robot_config.h"
#include "static_map.h"
#include "diagnostic_manager.h"
#include "stm32f1xx_hal.h"
#include <math.h>

static RobotFsm fsm;
static Odometry odometry;
static MpuDriver imu;

static void led_update(uint32_t now_ms) {
    switch (fsm.state) {
        case ROBOT_READY: bsp_led((now_ms/500u)%2u == 0); break;
        case ROBOT_COUNTDOWN: bsp_led((now_ms/100u)%2u == 0); break;
        case ROBOT_FAULT: {
            const uint32_t code = (uint32_t)fsm.fault;
            const uint32_t t = now_ms % (300u*code+1200u);
            bsp_led(t < 300u*code && (t%300u) < 150u);
            break;
        }
        case ROBOT_GOAL: bsp_led(true); break;
        case ROBOT_IDLE: bsp_led((now_ms/1000u)%2u == 0); break;
        default: bsp_led(false); break;
    }
}

int main(void) {
    diagnostic_init();
    const bool watchdog_reset = (RCC->CSR & RCC_CSR_IWDGRSTF) != 0;
    RCC->CSR |= RCC_CSR_RMVF;
    robot_fsm_init(&fsm, 0);
    if (!bsp_init()) {
        robot_fsm_fail(&fsm, FAULT_BOARD, 0);
        for (;;) {}
    }
    motor_control_init();
    encoder_init();
    fsm.state = ROBOT_SELF_TEST;
    g_robot_status.state = fsm.state;
#ifdef ROBOT_PROFILE_V2
    (void)watchdog_reset;
    // V2 is intentionally disabled until V1 has passed physical acceptance.
    // Building this profile checks the shared platform, never authorizes motion.
    robot_fsm_fail(&fsm, FAULT_UNSUPPORTED_PROFILE, bsp_millis());
#else
    if (watchdog_reset) robot_fsm_fail(&fsm,FAULT_COMMAND,bsp_millis());
    else if (!map_route_preflight()) robot_fsm_fail(&fsm,FAULT_ROUTE,bsp_millis());
    else if (!mpu_init(&imu)) robot_fsm_fail(&fsm,FAULT_IMU,bsp_millis());
    if (fsm.state != ROBOT_FAULT) {
        fsm.state = ROBOT_CALIBRATION;
        g_robot_status.state = fsm.state;
        if (!mpu_calibrate(&imu)) robot_fsm_fail(&fsm, FAULT_IMU, bsp_millis());
        else {
            g_robot_status.imu_ok = 1;
            g_robot_status.gyro_bias_rad_s = imu.bias_raw*0.01745329252f/65.5f;
            odometry_reset(&odometry,(RobotPose){ROBOT_START_X_MM,ROBOT_START_Y_MM,
                                                  ROBOT_START_HEADING_RAD});
            encoder_init();
            robot_fsm_ready(&fsm,bsp_millis());
        }
    }
#endif

    // Hardware watchdog covers a halted CPU even if SysTick cannot expire the
    // software motor lease. It starts only after the blocking boot calibration.
    IWDG->KR = 0xCCCCu;
    IWDG->KR = 0x5555u;
    IWDG->PR = 4u; // LSI / 64
    IWDG->RLR = 2499u; // nominal four seconds
    while (IWDG->SR != 0u) {}
    IWDG->KR = 0xAAAAu;

    uint32_t next_gyro = bsp_millis();
    uint32_t next_control = next_gyro;
    uint32_t last_control = next_control;
    float gyro_rate = 0;
    for (;;) {
        IWDG->KR = 0xAAAAu;
        const uint32_t now = bsp_millis();
        if ((int32_t)(now-next_gyro) >= 0) {
            next_gyro = now + 5u;
            if (fsm.state != ROBOT_FAULT && !mpu_read_rate(&imu,&gyro_rate,now) &&
                (uint32_t)(now-imu.last_ok_ms) > 100u)
                robot_fsm_fail(&fsm,FAULT_IMU,now);
        }
        if ((int32_t)(now-next_control) < 0) continue;
        next_control = now + ROBOT_CONTROL_PERIOD_MS;
        const uint32_t elapsed = now-last_control;
        last_control = now;
        if (elapsed > g_robot_status.max_loop_ms) g_robot_status.max_loop_ms = elapsed;
        if (fsm.state != ROBOT_FAULT && elapsed > 100u)
            robot_fsm_fail(&fsm,FAULT_TIMING,now);
        int32_t dl=0,dr=0;
        encoder_pose_delta(&dl,&dr);
        if (fsm.state == ROBOT_ALIGN_HEADING || fsm.state == ROBOT_NAVIGATING ||
            fsm.state == ROBOT_BRAKING) {
            if (!odometry_step(&odometry,dl,dr,gyro_rate,elapsed*0.001f))
                robot_fsm_fail(&fsm,FAULT_POSE,now);
        }
        if (fsm.state != ROBOT_FAULT)
            robot_fsm_step(&fsm,&odometry,dl,dr,imu.last_ok_ms,bsp_start_pressed(),now);
        if (fsm.state != ROBOT_FAULT &&
            (fsm.state == ROBOT_ALIGN_HEADING || fsm.state == ROBOT_NAVIGATING) &&
            !motor_control_enabled() && (uint32_t)(now-fsm.entered_ms) > 100u)
            robot_fsm_fail(&fsm,FAULT_COMMAND,now);
        float measured[2],duty[2];
        motor_control_snapshot(measured,duty);
        diagnostic_publish(now,fsm.run_started_ms,&odometry,dl,dr,
                           gyro_rate,measured,duty,motor_control_enabled());
        led_update(now);
    }
}
