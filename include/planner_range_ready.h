#pragma once
#include <stdint.h>

// Brackets the transition of the VL53 ready bit as seen by I2C polling.
// This is a completion bracket, NOT a measurement/exposure timestamp.
struct PlannerRangeReadyTracker {
  uint32_t last_not_ready_ms[4] = {};
  uint16_t ready_bracket_ms[4] = {UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX};
  uint8_t observed_not_ready = 0;

  void reset() {
    *this = PlannerRangeReadyTracker{};
  }

  void noteNotReady(uint8_t sensor, uint32_t now_ms) {
    if (sensor >= 4U) return;
    last_not_ready_ms[sensor] = now_ms;
    observed_not_ready |= uint8_t(1U << sensor);
  }

  void noteReady(uint8_t sensor, uint32_t now_ms) {
    if (sensor >= 4U) return;
    const uint8_t bit = uint8_t(1U << sensor);
    const uint32_t elapsed = now_ms - last_not_ready_ms[sensor];
    ready_bracket_ms[sensor] =
        (observed_not_ready & bit) && elapsed < UINT16_MAX ?
        uint16_t(elapsed) : UINT16_MAX;
    observed_not_ready &= uint8_t(~bit);
  }
};
