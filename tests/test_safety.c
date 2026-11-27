#include "safety_manager.h"
#include "robot_config.h"
#include <assert.h>
#include <stdio.h>

int main(void) {
    Odometry odo={0};
    odometry_reset(&odo,(RobotPose){ROBOT_START_X_MM,ROBOT_START_Y_MM,ROBOT_START_HEADING_RAD});
    SafetyManager safety;
    safety_reset(&safety,100);
    assert(safety_check(&safety,&odo,0,0,80,80,100,120)==FAULT_NONE);
    assert(safety_check(&safety,&odo,0,0,80,80,700,710)==FAULT_ENCODER_L);
    safety_reset(&safety,100);
    assert(safety_check(&safety,&odo,1,0,80,80,700,710)==FAULT_ENCODER_R);
    safety_reset(&safety,100);
    assert(safety_check(&safety,&odo,0,0,0,0,0,200)==FAULT_IMU);
    safety_reset(&safety,100);
    odo.pose.x_mm=900;
    odo.pose.y_mm=300;
    assert(safety_check(&safety,&odo,0,0,0,0,200,200)==FAULT_CLEARANCE);
    odo.pose.x_mm=300;
    odo.pose.y_mm=-80;
    odo.encoder_yaw_rad_s=1.0f;
    odo.gyro_yaw_rad_s=0.0f;
    assert(safety_check(&safety,&odo,0,0,0,0,200,200)==FAULT_NONE);
    assert(safety_check(&safety,&odo,0,0,0,0,730,730)==FAULT_POSE);
    puts("safety MPU, encoder, map and yaw mismatch PASS");
}
