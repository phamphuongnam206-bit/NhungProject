#include "robot_fsm.h"
#include "motor_control.h"
#include "robot_config.h"
#include <assert.h>
#include <stdio.h>

static bool mock_enabled;
void motor_control_stop(void) { mock_enabled=false; }
void motor_control_set(float l,float r,uint32_t now) {
    (void)l;(void)r;(void)now;mock_enabled=true;
}
bool motor_control_enabled(void) { return mock_enabled; }

int main(void) {
    RobotFsm fsm;
    Odometry odo;
    odometry_reset(&odo,(RobotPose){ROBOT_START_X_MM,ROBOT_START_Y_MM,ROBOT_START_HEADING_RAD});
    robot_fsm_init(&fsm,0);
    robot_fsm_ready(&fsm,100);
    robot_fsm_step(&fsm,&odo,0,0,120,false,120);
    for(uint32_t t=140;t<=180;t+=20) robot_fsm_step(&fsm,&odo,0,0,t,true,t);
    assert(fsm.state==ROBOT_COUNTDOWN && !mock_enabled);
    robot_fsm_step(&fsm,&odo,0,0,3200,true,3200);
    assert(fsm.state==ROBOT_ALIGN_HEADING && !mock_enabled);
    robot_fsm_step(&fsm,&odo,0,0,3220,false,3220);
    for(uint32_t t=3240;t<=3280;t+=20) robot_fsm_step(&fsm,&odo,0,0,t,true,t);
    assert(fsm.state==ROBOT_IDLE && !mock_enabled); // operator STOP
    robot_fsm_fail(&fsm,FAULT_IMU,3300);
    robot_fsm_step(&fsm,&odo,0,0,3400,false,3400);
    assert(fsm.state==ROBOT_FAULT && g_robot_fault_snapshot.fault==FAULT_IMU);
    puts("FSM START, countdown, STOP and latched fault PASS");
}
