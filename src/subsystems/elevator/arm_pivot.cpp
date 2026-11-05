#include "subsystems/elevator/arm_pivot.hpp"
#include "config/controls.hpp"
#include "lemlib/smartMotor.hpp"
#include "robotconfig.hpp"
#include <cmath>

namespace subsystems::elevator::arm_pivot {

namespace {

constexpr int kManualPower = 127;

// SmartMotor PID on the motor's encoder, in motor degrees (= arm degrees, direct drive).
// kG carries the arm's weight; low kP so snaps don't jerk the claw. I (only within
// kWindupDeg, reset on overshoot) pulls out what's left of the sag; D damps the stop.
constexpr float kP = 7.5f;
constexpr float kI = 0.0f;
constexpr float kD = 3.0f;
constexpr float kWindupDeg = 30.0f;
// Gravity: power to hold the arm horizontal. Added as kG * sin(arm angle from straight
// down): full at horizontal, none straight up or down, negative behind the pivot.
constexpr float kG = 11.0f;
constexpr float kSettleRange = 2.0f;            // motor deg
constexpr std::uint32_t kSettleTimeMs = 150;     // brakes (HOLD) after this long inside kSettleRange
// Red cartridge: 1800 raw encoder counts per output turn, independent of the gearset
// setting (which wasn't taking, so PROS degrees read wrong).
constexpr float kDegreesPerCount = 360.0f / 1800.0f;
// Slow zone going down: the last kDownSlowRangeDeg of a downward move capped at 35%.
constexpr float kDownSlowRangeDeg = 30.0f;
constexpr float kDownSlowPower = 44.0f; // 35% of 127

lemlib::SmartMotor motor(&arm_pivot_motor, lemlib::PID(kP, kI, kD, kWindupDeg, true), kSettleRange);
bool manual_active = false;

}

void init() {
    arm_pivot_motor.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    motor.useRawCounts(kDegreesPerCount);
    motor.setSettleTime(kSettleTimeMs);
    motor.reset(); // motor 0 = constants::kArmStartDeg; the conversions add the offset
    motor.setFeedforward([](float motor_deg) {
        const double arm_deg = constants::arm_motor_deg_to_deg(motor_deg);
        return kG * static_cast<float>(std::sin(arm_deg * 3.14159265358979323846 / 180.0));
    });
    motor.start();
}

void update() {
    const bool forward = controller.get_digital(controls::kArmPivotForward);
    const bool reverse = controller.get_digital(controls::kArmPivotReverse);

    if (forward || reverse) {
        manual_active = true;
        motor.setManual(forward ? kManualPower : -kManualPower);
    } else if (manual_active) {
        manual_active = false;
        motor.holdCurrent();
    }
}

void stop() {
    manual_active = false;
    motor.holdCurrent();
}

void set_pos(double motor_deg) {
    motor.setPosition(static_cast<float>(motor_deg));
}

void set_down_slow_zone(bool enabled) {
    motor.setReverseSlowZone(enabled ? kDownSlowRangeDeg : 0.0f, kDownSlowPower);
}

void set_power(int power) {
    motor.setManual(power);
}

void move_to(double motor_deg) {
    motor.setTarget(static_cast<float>(motor_deg));
}

bool is_settled() {
    return motor.isSettled();
}

bool wait_until_settled(std::uint32_t timeout_ms) {
    return motor.waitUntilSettled(timeout_ms);
}

double get_pos() {
    return motor.getRotation();
}

double target() {
    return motor.getTarget();
}

}
