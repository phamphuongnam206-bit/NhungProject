#include "static_map.h"
#include "robot_config.h"
#include <math.h>

// Rectangles are the 17 fixed blocks plus the central forbidden zone.
// Their coordinates are in mm, in the surveyed map frame.
const MapRect robot_obstacles[18] = {
    {600,0,1200,600},
    {0,100,100,200}, {500,200,600,300}, {250,300,350,400},
    {500,400,600,500}, {0,500,100,600}, {0,700,100,800},
    {300,850,400,950}, {400,1100,500,1200}, {600,700,700,800},
    {700,600,800,700}, {800,940,900,1040}, {900,600,1000,700},
    {1100,1100,1200,1200}, {1200,800,1300,900},
    {1400,400,1500,500}, {1400,100,1500,200}, {1700,200,1800,300}
};

// Offline route drawn from the verified fixed-map corridor. A runtime
// footprint sweep below rejects it if geometry or dimensions change.
const RobotPoint robot_route[] = {
    {300,-80}, {425,-70}, {425,625}, {575,925},
    {1600,575}, {1575,300}, {1575,-165}
};
const uint8_t robot_route_count = sizeof(robot_route) / sizeof(robot_route[0]);

static bool hits_rect(RobotPose pose, MapRect box, float clearance) {
    const float c = cosf(pose.heading_rad), s = sinf(pose.heading_rad);
    const float hl = ROBOT_BODY_HALF_LENGTH_MM, hw = ROBOT_BODY_HALF_WIDTH_MM;
    const float hx = 0.5f * (box.x1 - box.x0) + clearance;
    const float hy = 0.5f * (box.y1 - box.y0) + clearance;
    const float dx = 0.5f * (box.x0 + box.x1) - pose.x_mm;
    const float dy = 0.5f * (box.y0 + box.y1) - pose.y_mm;
    // Separating axes: map x/y, vehicle length/width.
    if (fabsf(dx) > hx + hl*fabsf(c) + hw*fabsf(s)) return false;
    if (fabsf(dy) > hy + hl*fabsf(s) + hw*fabsf(c)) return false;
    if (fabsf(dx*c + dy*s) > hl + hx*fabsf(c) + hy*fabsf(s)) return false;
    if (fabsf(-dx*s + dy*c) > hw + hx*fabsf(s) + hy*fabsf(c)) return false;
    return true;
}

bool map_body_clear(RobotPose pose, float clearance_mm) {
    if (!isfinite(pose.x_mm) || !isfinite(pose.y_mm) ||
        !isfinite(pose.heading_rad) || clearance_mm < 0) return false;
    const float c = fabsf(cosf(pose.heading_rad));
    const float s = fabsf(sinf(pose.heading_rad));
    const float ex = ROBOT_BODY_HALF_LENGTH_MM*c + ROBOT_BODY_HALF_WIDTH_MM*s;
    const float ey = ROBOT_BODY_HALF_LENGTH_MM*s + ROBOT_BODY_HALF_WIDTH_MM*c;
    if (pose.x_mm - ex < clearance_mm || pose.x_mm + ex > 1800-clearance_mm ||
        pose.y_mm + ey > 1200-clearance_mm || pose.y_mm - ey < -250+clearance_mm)
        return false;
    if (pose.y_mm - ey < 0 && pose.x_mm + ex >= 600-clearance_mm &&
        pose.x_mm - ex <= 1200+clearance_mm) return false;
    for (unsigned i = 0; i < 18; ++i)
        if (hits_rect(pose, robot_obstacles[i], clearance_mm)) return false;
    return true;
}

bool map_route_preflight(void) {
    for (uint8_t i = 0; i < robot_route_count; ++i) {
        const RobotPoint p = robot_route[i];
        // Stop-turn-go can pivot at intermediate waypoints; GOAL stops in place.
        if (i + 1u == robot_route_count) continue;
        for (unsigned deg = 0; deg < 360; deg += 5)
            if (!map_body_clear((RobotPose){p.x_mm,p.y_mm,deg*0.01745329252f},
                                ROBOT_CLEARANCE_MM)) return false;
        const RobotPoint q = robot_route[i+1u];
        const float dx = q.x_mm-p.x_mm, dy = q.y_mm-p.y_mm;
        const float length = hypotf(dx,dy);
        const unsigned samples = (unsigned)ceilf(length / 5.0f);
        const float heading = atan2f(dy,dx);
        for (unsigned j = 0; j <= samples; ++j) {
            const float t = (float)j / (float)samples;
            if (!map_body_clear((RobotPose){p.x_mm+t*dx,p.y_mm+t*dy,heading},
                                ROBOT_CLEARANCE_MM)) return false;
        }
    }
    return true;
}
