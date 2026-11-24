#pragma once

#include <stdint.h>

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

// Zero is a storage placeholder, not a measurement. Every State stays NEEDS_MEASUREMENT.
constexpr Calibration candidate{};

} // namespace MapOneRunCalibration
