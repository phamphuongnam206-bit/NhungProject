#pragma once

#include <stdint.h>
#include <math.h>

namespace MapOneRunCalibration {

enum class State : uint8_t { NEEDS_MEASUREMENT, VERIFIED, USER_MEASURED };

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

// USER_MEASURED is usable for locked diagnostics and still requires review
// before satisfying PlannerKinematicsReady. See V1_CALIBRATION_MATRIX.md.
constexpr float mechanicalTrackMm = 90.0f;
constexpr Calibration candidate{
    {{56.66f, State::USER_MEASURED}, {-1.0f, State::USER_MEASURED}, {}, {}, {}},
    {{51.24f, State::USER_MEASURED}, { 1.0f, State::USER_MEASURED}, {}, {}, {}},
    {},
    {130.0f, State::VERIFIED}, {100.0f, State::VERIFIED},
    {1.0f, State::USER_MEASURED}, {}, {}, {}, {}, {}, {}, {}, {}
};

constexpr float leftCountsPerCm = candidate.left.countsPerCm.number;
constexpr float rightCountsPerCm = candidate.right.countsPerCm.number;
constexpr int32_t leftEncoderSign = int32_t(candidate.left.forwardEncoderSign.number);
constexpr int32_t rightEncoderSign = int32_t(candidate.right.forwardEncoderSign.number);
constexpr float hybridYawSign = candidate.gyroSign.number; // +Y forward, clockwise positive
constexpr float steeringTrackCm = (candidate.effectiveTrackMm.state == State::VERIFIED &&
    candidate.effectiveTrackMm.number > 0.0f ? candidate.effectiveTrackMm.number :
    mechanicalTrackMm) / 10.0f;

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
