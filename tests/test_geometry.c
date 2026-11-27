#include "static_map.h"
#include "odometry.h"
#include "robot_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void) {
    assert(map_route_preflight());
    assert(!map_body_clear((RobotPose){900,300,0}, ROBOT_CLEARANCE_MM));
    assert(!map_body_clear((RobotPose){20,1150,0}, ROBOT_CLEARANCE_MM));
    assert(fabsf(angle_wrap(3.5f) + 2.7831853f) < 0.0001f);
    puts("geometry and route preflight PASS");
    return 0;
}
