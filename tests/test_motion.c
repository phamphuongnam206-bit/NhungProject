#include "motion_controller.h"
#include "odometry.h"
#include "static_map.h"
#include "robot_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static float slew(float current, float target, float step) {
    if (target-current > step) return current+step;
    if (target-current < -step) return current-step;
    return target;
}

int main(void) {
    const float dt = 0.020f;
    RobotPose truth = {ROBOT_START_X_MM,ROBOT_START_Y_MM,ROBOT_START_HEADING_RAD};
    Odometry estimate;
    odometry_reset(&estimate,truth);
    MotionController motion;
    motion_begin(&motion,0);
    float vl=0,vr=0,residual_l=0,residual_r=0;
    unsigned changes=0;
    MotionPhase previous=motion.phase;
    for (uint32_t now=0;now<240000;now+=20) {
        motion_update(&motion,estimate.pose,now);
        if(motion.phase!=previous) { ++changes; previous=motion.phase; }
        if(motion.phase==MOTION_ERROR) {
            fprintf(stderr,"motion error at %u ms, waypoint %u, pose %.1f %.1f %.3f\n",
                    now,motion.next_waypoint,estimate.pose.x_mm,estimate.pose.y_mm,estimate.pose.heading_rad);
            return 1;
        }
        if(motion.phase==MOTION_FINISHED) break;
        vl=slew(vl,motion.target_left_mm_s,8.0f);
        vr=slew(vr,motion.target_right_mm_s,8.0f);
        const float dl=vl*dt,dr=vr*dt;
        const float yaw=(dr-dl)/ROBOT_TRACK_MM/dt;
        const float mid=truth.heading_rad+0.5f*yaw*dt;
        truth.x_mm+=0.5f*(dl+dr)*cosf(mid);
        truth.y_mm+=0.5f*(dl+dr)*sinf(mid);
        truth.heading_rad=angle_wrap(truth.heading_rad+yaw*dt);
        residual_l+=dl/ROBOT_MM_PER_COUNT_L;
        residual_r+=dr/ROBOT_MM_PER_COUNT_R;
        const int32_t counts_l=(int32_t)residual_l,counts_r=(int32_t)residual_r;
        residual_l-=counts_l;residual_r-=counts_r;
        assert(odometry_step(&estimate,counts_l,counts_r,yaw,dt));
        if(!map_body_clear(truth,ROBOT_CLEARANCE_MM)) {
            fprintf(stderr,"clearance error at %u ms, wp %u, truth %.1f %.1f %.3f phase %u\n",
                    now,motion.next_waypoint,truth.x_mm,truth.y_mm,truth.heading_rad,(unsigned)motion.phase);
            return 1;
        }
    }
    assert(motion.phase==MOTION_FINISHED);
    assert(truth.y_mm < -90.0f - ROBOT_BODY_HALF_LENGTH_MM);
    printf("motion PASS: %u phase changes, finish (%.1f, %.1f), error %.1f mm\n",
           changes,truth.x_mm,truth.y_mm,
           hypotf(truth.x_mm-estimate.pose.x_mm,truth.y_mm-estimate.pose.y_mm));
}
