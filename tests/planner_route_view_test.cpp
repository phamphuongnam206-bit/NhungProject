#include "planner_route_view.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static OneRunMission mission;
int main() {
  mission.state = ONE_RUN_FOLLOWING;
  mission.last_planner_status = PLANNER_OK;
  mission.waypoint_count = 3;
  mission.waypoints[0] = {2,2};
  mission.waypoints[1] = {2,6};
  mission.waypoints[2] = {6,6};
  PlannerRouteSample sample{};
  PlannerRouteTarget target{};
  assert(PlannerRoute_Target(&mission, 27.5f, 14.5f, 10, &target));
  assert(target.progress_cm == 15 && target.cross_track_cm == 2);
  assert(target.current_segment == 0 && target.lookahead.segment == 1);
  assert(target.lookahead.x_cm == 32.5f && target.lookahead.y_cm == 17.5f);
  assert(!PlannerRoute_Target(&mission, NAN, 10, 10, &target));
  assert(target.progress_cm == 0);
  assert(PlannerRoute_Sample(&mission, 10, &sample));
  assert(sample.x_cm == 22.5f && sample.y_cm == 12.5f && sample.length_cm == 40);
  assert(PlannerRoute_Sample(&mission, 25, &sample));
  assert(sample.x_cm == 32.5f && sample.y_cm == 17.5f && sample.segment == 1);
  assert(PlannerRoute_Sample(&mission, 200, &sample));
  assert(sample.x_cm == 32.5f && sample.y_cm == 32.5f);
  mission.state = ONE_RUN_REPLAN_STOPPED;
  assert(!PlannerRoute_Sample(&mission, 10, &sample) && sample.length_cm == 0);
  mission.state = ONE_RUN_FOLLOWING;
  mission.waypoints[2] = {99,6};
  assert(!PlannerRoute_Sample(&mission, 1, &sample));
  mission.waypoints[2] = mission.waypoints[1];
  assert(!PlannerRoute_Sample(&mission, 1, &sample));
  mission.waypoints[2] = {6,6};
  assert(!PlannerRoute_Sample(&mission, NAN, &sample));
  assert(!PlannerRoute_Sample(&mission, -1, &sample));
  mission.waypoint_count = 129;
  assert(!PlannerRoute_Sample(&mission, 1, &sample));
  mission.waypoint_count = 2;
  mission.waypoints[1] = {6,2};
  assert(PlannerRoute_Sample(&mission, 10, &sample));
  assert(sample.x_cm == 12.5f && sample.y_cm == 22.5f); // new route, no stale cache
  assert(PlannerRoute_Target(&mission, 12.5f, 17.5f, 5, &target));
  assert(target.progress_cm == 5 && target.lookahead.y_cm == 22.5f);
  assert(!PlannerRoute_Sample(nullptr, 0, &sample));
  assert(!PlannerRoute_Sample(&mission, 0, nullptr));
  puts("PASS planner route interpolation, bounds, errors and replan replacement");
}
