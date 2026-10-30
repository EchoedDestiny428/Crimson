#pragma once

#include "lemlib/chassis/trackingWheel.hpp"

namespace config {

struct PidConstants {
    float kP;
    float kI;
    float kD;
    float windupRange;
    float smallError;
    float smallErrorTimeout;
    float largeError;
    float largeErrorTimeout;
    float slew;
};

struct CurveConstants {
    float deadband;
    float minOutput;
    float curve;
};

inline constexpr float kTrackWidthIn = 12.5f;
inline constexpr float kWheelDiameterIn = lemlib::Omniwheel::NEW_325;
inline constexpr float kDriveRpm = 360.0f;
inline constexpr float kHorizontalDrift = 2.0f;

inline constexpr float kTrackingWheelDiameterIn = 2.0f;
inline constexpr float kVerticalWheelOffsetIn = 0.5f;
inline constexpr float kHorizontalWheelOffsetIn = -0.7f;

inline constexpr PidConstants kLinearPid{9.5f, 0.0f, 12.0f, 1.0f, 0.2f, 100.0f, 1.0f, 500.0f, 40.0f};
inline constexpr PidConstants kAngularPid{5.5f, 0.0f, 40.0f, 3.0f, 1.0f, 50.0f, 3.0f, 300.0f, 0.0f};

inline constexpr CurveConstants kThrottleCurve{2.0f, 2.5f, 1.019f};
inline constexpr CurveConstants kSteerCurve{2.0f, 2.5f, 1.0f};

}
