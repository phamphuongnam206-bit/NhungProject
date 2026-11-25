#pragma once

#include <stdint.h>
#include <math.h>

// Moc 0B: all values are placeholders. No field below is a driving constant.
// A measured value needs a recorded method, raw log and review before arming.
namespace MapOneRunCalibration {

enum class State : uint8_t { NEEDS_MEASUREMENT, VERIFIED };

struct Value {
    float number;
    State state;
};

struct WheelCalibration {
    Value countsPerCm;
    Value forwardEncoderSign;
    Value wheelDiameterMm;
    Value startupPwmForward;
    Value startupPwmReverse;
};

struct PwmSpeedPoint {
    Value pwm;
    Value speedMmPerSecond;
};

struct SensorMount {
    Value xMm;
    Value yMm;
    Value yawDegrees;
};

struct Calibration {
    WheelCalibration left;
    WheelCalibration right;
    Value effectiveTrackMm;
    Value bodyLengthMm;
    Value bodyWidthMm;
    Value gyroSign;
    Value gyroBiasDegreesPerSecond;
    Value gyroDriftDegreesPerMinute;
    Value accelerationMmPerSecond2;
    Value decelerationMmPerSecond2;
    Value brakingDistanceMm;
    PwmSpeedPoint leftPwmSpeed[5];
    PwmSpeedPoint rightPwmSpeed[5];
    SensorMount vl53[4]; // J8, J9, J10, J11 in PCB order.
};

// The user measured the assembled body at 13 cm along travel and 10 cm
// across the wheels. Other zeros are placeholders, never operating values.
constexpr Calibration candidate{
    {}, {}, {},
    {130.0f, State::VERIFIED}, {100.0f, State::VERIFIED},
    {}, {}, {}, {}, {}, {}, {}, {}, {}
};

struct PlannerKinematicsExpected {
    float leftCountsPerCm, rightCountsPerCm;
    float leftForwardSign, rightForwardSign;
    float effectiveTrackMm, bodyLengthMm, bodyWidthMm;
    float gyroSign, decelerationMmPerSecond2;
};

inline bool verifiedMatches(const volatile Value &value, float compiled) {
    const State state = value.state;
    const float number = value.number;
    return state == State::VERIFIED && isfinite(number) &&
           isfinite(compiled) && compiled != 0.0f &&
           fabsf(number - compiled) <=
               0.001f * fmaxf(1.0f, fabsf(compiled));
}

// A review flag alone cannot authorize old firmware constants. Every input
// below must have a measured, reviewed counterpart matching the compiled
// value; braking distance must also have its own measured positive result.
inline bool PlannerKinematicsReady(const volatile Calibration &cal,
                                   const PlannerKinematicsExpected &expected) {
    const volatile Value &brake = cal.brakingDistanceMm;
    const State brakeState = brake.state;
    const float brakeMm = brake.number;
    return verifiedMatches(cal.left.countsPerCm, expected.leftCountsPerCm) &&
           verifiedMatches(cal.right.countsPerCm, expected.rightCountsPerCm) &&
           verifiedMatches(cal.left.forwardEncoderSign, expected.leftForwardSign) &&
           verifiedMatches(cal.right.forwardEncoderSign, expected.rightForwardSign) &&
           verifiedMatches(cal.effectiveTrackMm, expected.effectiveTrackMm) &&
           verifiedMatches(cal.bodyLengthMm, expected.bodyLengthMm) &&
           verifiedMatches(cal.bodyWidthMm, expected.bodyWidthMm) &&
           verifiedMatches(cal.gyroSign, expected.gyroSign) &&
           verifiedMatches(cal.decelerationMmPerSecond2,
                           expected.decelerationMmPerSecond2) &&
           brakeState == State::VERIFIED && isfinite(brakeMm) &&
           brakeMm > 0.0f;
}

} // namespace MapOneRunCalibration
