#include "speed_pi.h"
#include "robot_config.h"
#include <math.h>

static float bound(float x,float lo,float hi) { return fminf(hi,fmaxf(lo,x)); }

float speed_pi_step(float target, float measured, float gain, float dt, float *integral) {
    if (!integral || !isfinite(target) || !isfinite(measured) ||
        !isfinite(gain) || gain <= 0 || !isfinite(dt) || dt <= 0 ||
        fabsf(target) < 10.0f) {
        if (integral) *integral = 0;
        return 0;
    }
    const float sign = target > 0 ? 1.0f : -1.0f;
    const float error = fabsf(target)-sign*measured;
    const float ff = fabsf(target)/gain+ROBOT_FF_OFFSET;
    const float candidate = bound(*integral+ROBOT_SPEED_KI*error*dt,
                                  -ROBOT_PI_LIMIT,ROBOT_PI_LIMIT);
    const float raw = ff+ROBOT_SPEED_KP*error+candidate;
    if (raw > 0 && raw < ROBOT_PWM_LIMIT) *integral = candidate;
    return sign*bound(ff+ROBOT_SPEED_KP*error+*integral,0,ROBOT_PWM_LIMIT);
}
