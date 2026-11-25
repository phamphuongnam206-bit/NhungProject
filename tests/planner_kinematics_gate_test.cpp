#include "map_one_run_calibration.h"
#include <assert.h>
#include <stdio.h>

using namespace MapOneRunCalibration;

int main() {
  const PlannerKinematicsExpected expected{
      45.0f, 46.0f, -1.0f, 1.0f, 90.0f, 130.0f, 100.0f,
      -1.0f, 350.0f}; // Synthetic fixture, never a robot calibration.
  assert(!PlannerKinematicsReady(candidate, expected));
  Calibration c{};
  const auto value = [](float n) { return Value{n, State::VERIFIED}; };
  c.left.countsPerCm = value(45.0f);
  c.right.countsPerCm = value(46.0f);
  c.left.forwardEncoderSign = value(-1.0f);
  c.right.forwardEncoderSign = value(1.0f);
  c.effectiveTrackMm = value(90.0f);
  c.bodyLengthMm = value(130.0f);
  c.bodyWidthMm = value(100.0f);
  c.gyroSign = value(-1.0f);
  c.decelerationMmPerSecond2 = value(350.0f);
  c.brakingDistanceMm = value(80.0f);
  assert(PlannerKinematicsReady(c, expected));
  c.left.countsPerCm.state = State::NEEDS_MEASUREMENT;
  assert(!PlannerKinematicsReady(c, expected));
  c.left.countsPerCm = value(45.0f);
  c.right.countsPerCm = value(47.0f);
  assert(!PlannerKinematicsReady(c, expected));
  c.right.countsPerCm = value(46.0f);
  c.left.forwardEncoderSign = value(1.0f);
  assert(!PlannerKinematicsReady(c, expected));
  c.left.forwardEncoderSign = value(-1.0f);
  c.decelerationMmPerSecond2 = value(0.0f);
  assert(!PlannerKinematicsReady(c, expected));
  c.decelerationMmPerSecond2 = value(350.0f);
  c.brakingDistanceMm = value(0.0f);
  assert(!PlannerKinematicsReady(c, expected));
  puts("PASS kinematics: review states, compiled values, signs and brake measurement");
}
