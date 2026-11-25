#pragma once
extern "C" {
#include "one_run_adapter.h"
}
// Main context. External map/config must be surveyed/reviewed, not aliases of
// runtime storage. This flag records caller verification, not calibration.
extern "C" bool PlannerRuntime_Begin(const MapGrid *map, const OneRunConfig *config,
                                     bool inputs_verified, bool stopped);
extern "C" OneRunStatus PlannerRuntime_Poll(const OneRunInput *snapshot,
                                            bool global_pose_verified, bool stopped);
extern "C" void PlannerRuntime_Invalidate();

extern "C" bool PlannerRuntime_PreviewPermitted();
