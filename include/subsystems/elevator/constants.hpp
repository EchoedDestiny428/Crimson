#pragma once

// Shared by the elevator extension, arm pivot and claw pivot.
// Every value marked TODO still has to be measured on the new robot.

namespace subsystems::elevator::constants {

// Geometry and gearing.
inline constexpr double kArmLengthCm = 0.0;                          // TODO: arm pivot axis to claw
inline constexpr double kArmPivotMotorRotationsPerArmRotation = 0.0; // TODO: arm pivot gear ratio
inline constexpr double kExtensionMotorRotationsPerCm = 0.0;         // TODO: extension motor rotations per cm of lift

// Elevator extension setpoints, in cm of lift.
inline constexpr double kExtensionBootupCm = 0.0;     // TODO
inline constexpr double kExtensionFlipOutMinCm = 0.0; // TODO
inline constexpr double kExtensionHomeCm = 0.0;       // TODO
inline constexpr double kExtensionDualSetupCm = 0.0;  // TODO
inline constexpr double kExtensionOffsetCm = 0.0;     // TODO
inline constexpr double kExtensionFlipOutCm = 0.0;    // TODO
inline constexpr double kExtensionStage1Cm = 0.0;     // TODO
inline constexpr double kExtensionStage2Cm = 0.0;     // TODO
inline constexpr double kExtensionStage3Cm = 0.0;     // TODO
inline constexpr double kExtensionStage4Cm = 0.0;     // TODO
inline constexpr double kExtensionStage5Cm = 0.0;     // TODO

// Arm pivot setpoints, in degrees of arm rotation.
inline constexpr double kArmHomeDeg = 0.0;       // TODO
inline constexpr double kArmFlippedDeg = 0.0;    // TODO
inline constexpr double kArmTopDeg = 0.0;        // TODO
inline constexpr double kArmDualSetupDeg = 0.0;  // TODO
inline constexpr double kArmDualPickupDeg = 0.0; // TODO

// Claw pivot, in motor degrees until its gearing is known.
inline constexpr double kClawPivotHomeMotorDeg = 0.0; // TODO

// Unit conversions. These divide by the ratios above, so fill those in first.
constexpr double extension_cm_to_motor_deg(double cm) {
    return cm * kExtensionMotorRotationsPerCm * 360.0;
}

constexpr double extension_motor_deg_to_cm(double motor_deg) {
    return motor_deg / 360.0 / kExtensionMotorRotationsPerCm;
}

constexpr double arm_deg_to_motor_deg(double arm_deg) {
    return arm_deg * kArmPivotMotorRotationsPerArmRotation;
}

constexpr double arm_motor_deg_to_deg(double motor_deg) {
    return motor_deg / kArmPivotMotorRotationsPerArmRotation;
}

}
