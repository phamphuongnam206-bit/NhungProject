#include "planner_range_ready.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>

int main() {
  PlannerRangeReadyTracker tracker{};
  assert(tracker.ready_bracket_ms[0] == UINT16_MAX);
  tracker.noteReady(0U, 10U);
  assert(tracker.ready_bracket_ms[0] == UINT16_MAX); // no preceding low status
  tracker.noteNotReady(0U, 20U);
  tracker.noteReady(0U, 24U);
  assert(tracker.ready_bracket_ms[0] == 4U);
  tracker.noteReady(0U, 28U);
  assert(tracker.ready_bracket_ms[0] == UINT16_MAX); // same result is not new
  tracker.noteNotReady(1U, UINT32_MAX - 2U);
  tracker.noteReady(1U, 2U);
  assert(tracker.ready_bracket_ms[1] == 5U); // millis rollover
  tracker.reset();
  assert(tracker.ready_bracket_ms[0] == UINT16_MAX);
  assert(tracker.ready_bracket_ms[1] == UINT16_MAX);
  puts("PASS VL53 ready-bit completion brackets and rollover");
}
