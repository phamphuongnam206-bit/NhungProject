#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "robot_types.h"

typedef struct { float x0, y0, x1, y1; } MapRect;

extern const MapRect robot_obstacles[18];
extern const RobotPoint robot_route[];
extern const uint8_t robot_route_count;

bool map_body_clear(RobotPose pose, float clearance_mm);
bool map_route_preflight(void);
