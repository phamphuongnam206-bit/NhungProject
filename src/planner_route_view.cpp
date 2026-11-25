#include "planner_route_view.h"
#include <math.h>

extern "C" bool PlannerRoute_Sample(const OneRunMission *mission,
                                    float distance_cm,
                                    PlannerRouteSample *output) {
  if (!output) return false;
  *output = {};
  if (!mission || mission->state != ONE_RUN_FOLLOWING ||
      mission->last_planner_status != PLANNER_OK ||
      mission->waypoint_count < 2 || mission->waypoint_count > ONE_RUN_MAX_WAYPOINTS ||
      !isfinite(distance_cm) || distance_cm < 0) return false;

  float accumulated = 0;
  bool sampled = false;
  PlannerRouteSample candidate{};
  // Validate the WHOLE route even when the requested point is near its start.
  for (unsigned i = 0; i < mission->waypoint_count; ++i) {
    const auto b = mission->waypoints[i];
    if (b.row >= MAP_GRID_ROWS || b.col >= MAP_GRID_COLS) return false;
    if (!i) continue;
    const auto a = mission->waypoints[i - 1];
    const float dx = (float(b.col) - float(a.col)) * MAP_GRID_CELL_CM;
    const float dy = (float(b.row) - float(a.row)) * MAP_GRID_CELL_CM;
    const float length = hypotf(dx, dy);
    if (!(length > 0)) return false;
    if (!sampled && distance_cm <= accumulated + length) {
      const float t = (distance_cm - accumulated) / length;
      candidate.x_cm = (float(a.col) + .5f) * MAP_GRID_CELL_CM + t * dx;
      candidate.y_cm = (float(a.row) + .5f) * MAP_GRID_CELL_CM + t * dy;
      candidate.segment = i - 1;
      sampled = true;
    }
    accumulated += length;
  }
  if (!sampled) {
    const auto goal = mission->waypoints[mission->waypoint_count - 1];
    candidate.x_cm = (float(goal.col) + .5f) * MAP_GRID_CELL_CM;
    candidate.y_cm = (float(goal.row) + .5f) * MAP_GRID_CELL_CM;
    candidate.segment = mission->waypoint_count - 2U;
  }
  candidate.length_cm = accumulated;
  *output = candidate;
  return true;
}
