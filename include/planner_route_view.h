#pragma once

extern "C" {
#include "one_run_mission.h"
}

struct PlannerRouteSample {
  float x_cm, y_cm, length_cm;
  unsigned segment;
};

// Main context only: mission must not be modified concurrently. No copied
// waypoint buffer or cached pointer survives replanning. This is geometry,
// not authorization to drive or evidence of crossing the finish line.
extern "C" bool PlannerRoute_Sample(const OneRunMission *mission,
                                    float distance_cm,
                                    PlannerRouteSample *output);

struct PlannerRouteTarget {
  PlannerRouteSample lookahead;
  float progress_cm, cross_track_cm;
  unsigned current_segment;
};
extern "C" bool PlannerRoute_Target(const OneRunMission *mission,
    float x_cm, float y_cm, float lookahead_cm, PlannerRouteTarget *output);
extern "C" const OneRunMission *OneRunFirmwareMission();
