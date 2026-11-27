#pragma once

// Pure control law: duty has the same sign as a nonzero target. Caller owns
// the integrator, sample timing, motor direction transition and PWM output.
float speed_pi_step(float target_mm_s, float measured_mm_s, float gain_mm_s,
                    float dt_s, float *integral);
