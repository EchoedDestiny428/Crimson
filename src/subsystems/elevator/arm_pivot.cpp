#include "subsystems/elevator/arm_pivot.hpp"
#include "config/controls.hpp"
#include "lemlib/smartMotor.hpp"
#include "robotconfig.hpp"
#include <algorithm>
#include <cmath>

namespace subsystems::elevator::arm_pivot {

namespace {

// PID on the motor's encoder, in arm degrees (direct drive), with a minimum speed that stays
// on the whole move: the PID is the main controller (kP 1.15 is full power until ~110 deg out;
// kD 6 per 10 ms tick damps the approach so home <-> L1 doesn't run into the crossbar at 315),
// and whenever it asks for less than kMinPower toward the target (outside the deadband) the
// push is raised to kMinPower, a bang-bang that finishes the move and keeps friction and
// gravity from stalling it short. No handoffs or ranges. No kI, no kG (counterbanded).
constexpr float kP = 1.15f;
constexpr float kD = 6.0f;
constexpr float kMinPower = 30.0f; // above gravity + friction (20 stalled at ~235 climbing toward 180)
// Two ways to settle: inside kSettleRange (1 deg) for kSettleTimeMs, or inside the wider
// kHoldRange (2 deg) for kHoldTimeMs. Either brakes (HOLD); the minimum speed stops at kSettleRange.
constexpr float kSettleRange = 1.0f;         // motor deg
constexpr std::uint32_t kSettleTimeMs = 150;
constexpr float kHoldRange = 2.0f;           // motor deg
constexpr std::uint32_t kHoldTimeMs = 350;
// Red cartridge: 1800 raw encoder counts per output turn, independent of the gearset
// setting (which wasn't taking, so PROS degrees read wrong).
constexpr float kDegreesPerCount = 360.0f / 1800.0f;
// Slow zone: the last kSlowRangeDeg of a move capped at 35%, either direction (the arm
// falls into stage 1 behind going forward, and into home going back).
constexpr float kSlowRangeDeg = 35.0f;
constexpr float kSlowPower = 30.0f; // 35% of 127

// Soft limits, arm degrees from straight down: setpoints are clamped into this range, and
// manual power that would push past a limit holds at the limit instead.
constexpr double kMinArmDeg = 27.0; // the lower hard stop
constexpr double kMaxArmDeg = 315.0;
// Hard slowdown: within kLimitSlowRangeDeg of a soft limit, anything driving toward it
// (manual or PID) is capped at kLimitSlowPower.
constexpr double kLimitSlowRangeDeg = 15.0;
constexpr float kLimitSlowPower = 25.0f; // ~20% of 127

lemlib::SmartMotor motor(&arm_pivot_motor, lemlib::PID(kP, 0.0f, kD), kSettleRange);

double clamp_motor_deg(double motor_deg) {
    return std::clamp(motor_deg, constants::arm_deg_to_motor_deg(kMinArmDeg), constants::arm_deg_to_motor_deg(kMaxArmDeg));
}

// Open-loop power unless it pushes past a soft limit; then hold at that limit.
void drive(int power) {
    const double arm_deg = constants::arm_motor_deg_to_deg(motor.getRotation());
    if (power > 0 && arm_deg >= kMaxArmDeg) {
        motor.setTarget(static_cast<float>(constants::arm_deg_to_motor_deg(kMaxArmDeg)));
    } else if (power < 0 && arm_deg <= kMinArmDeg) {
        motor.setTarget(static_cast<float>(constants::arm_deg_to_motor_deg(kMinArmDeg)));
    } else {
        motor.setManual(power);
    }
}

}

void init() {
    arm_pivot_motor.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    motor.useRawCounts(kDegreesPerCount);
    motor.setSettleTime(kSettleTimeMs);
    motor.setSettleHold(kHoldRange, kHoldTimeMs);
    motor.setMinPower(kMinPower);
    motor.reset(); // motor 0 = constants::kArmStartDeg; the conversions add the offset
    motor.setOutputFilter([](float motor_deg, float output) {
        const double arm_deg = constants::arm_motor_deg_to_deg(motor_deg);
        if (output > 0.0f && arm_deg >= kMaxArmDeg - kLimitSlowRangeDeg) {
            return std::min(output, kLimitSlowPower);
        }
        if (output < 0.0f && arm_deg <= kMinArmDeg + kLimitSlowRangeDeg) {
            return std::max(output, -kLimitSlowPower);
        }
        return output;
    });
    motor.start();
}

void stop() {
    motor.holdCurrent();
}

void set_slow_zone(bool enabled) {
    motor.setSlowZone(enabled ? kSlowRangeDeg : 0.0f, kSlowPower);
}

void set_power(int power) {
    drive(power);
}

void move_to(double motor_deg) {
    motor.setTarget(static_cast<float>(clamp_motor_deg(motor_deg)));
}

double get_pos() {
    return motor.getRotation();
}

double target() {
    return motor.getTarget();
}

}
