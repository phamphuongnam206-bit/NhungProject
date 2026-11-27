#include "speed_pi.h"
#include "robot_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void) {
    float integral=0;
    for (unsigned i=0;i<200;++i) {
        float duty=speed_pi_step(90.0f,400.0f,ROBOT_FF_GAIN_L,0.005f,&integral);
        assert(duty >= 0 && duty <= ROBOT_PWM_LIMIT);
    }
    for (unsigned i=0;i<200;++i) {
        float duty=speed_pi_step(-90.0f,-400.0f,ROBOT_FF_GAIN_R,0.005f,&integral);
        assert(duty <= 0 && duty >= -ROBOT_PWM_LIMIT);
    }
    assert(speed_pi_step(0,0,ROBOT_FF_GAIN_L,0.005f,&integral)==0);
    assert(fabsf(integral)<0.00001f);
    puts("speed PI sign, saturation and reset PASS");
}
