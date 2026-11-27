#pragma once
#include <stdint.h>

namespace StaticMap {

// =====================================
// 1. KICH THUOC MAP (mm)
// =====================================

constexpr int MAP_WIDTH  = 1800;
constexpr int MAP_HEIGHT = 1200;
constexpr int CELL_SIZE  = 25;

constexpr int BOX_SIZE   = 100;
constexpr int BOX_HEIGHT = 100;

// =====================================
// 2. KICH THUOC XE - CAN HIEU CHUAN
// =====================================

constexpr int ROBOT_LENGTH = 136;
constexpr int ROBOT_WIDTH  = 116;
constexpr int SAFE_MARGIN  = 15;

// =====================================
// 3. VUNG CAM
// =====================================

struct Rectangle {
    int16_t xmin;
    int16_t ymin;
    int16_t xmax;
    int16_t ymax;
};

constexpr Rectangle FORBIDDEN = {
    600, 0, 1200, 600
};

// =====================================
// 4. START VA FINISH
// =====================================

constexpr int START_X = 300;
constexpr int START_Y = 120;
constexpr int START_HEADING_DEG = 90;

constexpr int START_LEFT  = 0;
constexpr int START_RIGHT = 600;

constexpr int FINISH_LEFT  = 1200;
constexpr int FINISH_RIGHT = 1800;
constexpr int FINISH_LINE  = 0;

#ifndef MAP_STATIC_CRUISE_MM_S
#define MAP_STATIC_CRUISE_MM_S 150
#endif
constexpr float CRUISE_MM_S=MAP_STATIC_CRUISE_MM_S;
constexpr float NARROW_MM_S=85;
constexpr bool LOCALIZE_AT_START = false;
constexpr int START_SEARCH_MM = 50;
constexpr int START_SEARCH_DEG = 15;

constexpr int GOAL_X = 1600;
constexpr int GOAL_Y = -100;

// =====================================
// 5. VAT CAN CO DINH
// Toa do tam vat can
// =====================================

struct Obstacle {
    uint8_t id;
    int16_t x;
    int16_t y;
    int16_t width;
    int16_t length;
};

constexpr Obstacle FIXED_OBSTACLES[] = {
    { 1,   50,  150, BOX_SIZE, BOX_SIZE},
    { 2,  550,  250, BOX_SIZE, BOX_SIZE},
    { 3,  300,  350, BOX_SIZE, BOX_SIZE},
    { 4,  550,  450, BOX_SIZE, BOX_SIZE},
    { 5,   50,  550, BOX_SIZE, BOX_SIZE},
    { 6,   50,  750, BOX_SIZE, BOX_SIZE},
    { 7,  350,  900, BOX_SIZE, BOX_SIZE},
    { 8,  450, 1150, BOX_SIZE, BOX_SIZE},
    { 9,  650,  750, BOX_SIZE, BOX_SIZE},
    {10,  750,  650, BOX_SIZE, BOX_SIZE},
    {11,  850, 1000, BOX_SIZE, BOX_SIZE},
    {12,  950,  650, BOX_SIZE, BOX_SIZE},
    {13, 1150, 1150, BOX_SIZE, BOX_SIZE}
};

constexpr int FIXED_COUNT = sizeof(FIXED_OBSTACLES)/sizeof(FIXED_OBSTACLES[0]);

// =====================================
// 6. VI TRI VAT CAN NGAU NHIEN
// =====================================

constexpr Obstacle POSSIBLE_OBSTACLES[] = {
    {14, 1250, 850, BOX_SIZE, BOX_SIZE},
    {15, 1450, 450, BOX_SIZE, BOX_SIZE},
    {16, 1450, 150, BOX_SIZE, BOX_SIZE},
    {17, 1750, 250, BOX_SIZE, BOX_SIZE}
};

constexpr int POSSIBLE_COUNT = sizeof(POSSIBLE_OBSTACLES)/sizeof(POSSIBLE_OBSTACLES[0]);

// =====================================
// 7. LO TRINH CO DINH
// =====================================

struct Waypoint {
    int16_t x;
    int16_t y;
};

constexpr Waypoint ROUTE[] = {
    {START_X, START_Y},
    {395, 140},
    {425, 300},
    {425, 680},
    {560, 895},
    {970, 845},
    {1170, 715},
    {1430, 715},
    {1600, 560},
    {GOAL_X, GOAL_Y}
};

constexpr int ROUTE_COUNT =
    sizeof(ROUTE) / sizeof(ROUTE[0]);

}