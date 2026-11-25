// Retain the complete C planner and realistic static storage in the ARM link.
// This function is NEVER called from setup/loop. No synthetic pose is sent to PI.
extern "C" {
#include "one_run_adapter.h"
}

#if !defined(HYBRID_MOTOR_LOCK) || HYBRID_MOTOR_LOCK != 1
#error "Planner link qualification requires the compile-time motor lock"
#endif

struct OneRunFirmwareStorage {
  OneRunMission mission;
  OneRunAdapter adapter;
  OneRunConfig config;
  OneRunInput snapshot;
  OneRunAdapterOutput output;
};
extern "C" {
OneRunFirmwareStorage one_run_link_storage;
}

extern "C" void *OneRunFirmwareLinkProbe() {
  return &one_run_link_storage;
}

extern "C" const OneRunMission *OneRunFirmwareMission() {
  return &one_run_link_storage.mission;
}
