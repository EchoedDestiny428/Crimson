#include "subsystems/elevator/arm_pivot.hpp"
#include "config/controls.hpp"
#include "lemlib/smartMotor.hpp"
#include "robotconfig.hpp"
#include <algorithm>
#include <cmath>

namespace subsystems::elevator::arm_pivot {

namespace {

constexpr int kManualPower = 127;

// SmartMotor ramp on the motor's encoder, in motor degrees (= arm degrees, direct drive):
// full power until kDecelerationDeg from the target, then straight down to 50%, linear from
// there to 10% at the target, then brake (HOLD) inside kSettleRange. No PID or kG for now.
constexpr std::int32_t kMaxVelocity = 100; // red cartridge rpm
constexpr float kDecelerationDeg = 80.0f;
constexpr float kRampStartPower = 63.5f; // 50% of 127
constexpr float kRampEndPower = 12.7f;   // 10% of 127
// Careful moves: anything over kCarefulMoveDeg (home -> L1 is 275 deg, ending by the
// crossbar at 315) slows over kCarefulDecelerationDeg instead, from 40% down to 6%, so it
// comes in gently and doesn't overshoot.
constexpr double kCarefulMoveDeg = 200.0;
constexpr float kCarefulDecelerationDeg = 150.0f;
constexpr float kCarefulRampStartPower = 50.8f; // 40% of 127
constexpr float kCarefulRampEndPower = 7.6f;    // 6% of 127
// Lifting behind (arm angle going down while past straight up, e.g. L3 310 -> L4 230):
// against gravity, and the normal 80 deg ramp would cover the whole 80 deg move, so it
// ramps late and hard instead: full power until 30 deg out, then 80% down to 25%.
constexpr float kLiftBehindDecelerationDeg = 30.0f;
constexpr float kLiftBehindRampStartPower = 101.6f; // 80% of 127
constexpr float kLiftBehindRampEndPower = 31.8f;    // 25% of 127
constexpr float kSettleRange = 1.0f;       // motor deg: 1 deg deadband, brakes inside it
// Red cartridge: 1800 raw encoder counts per output turn, independent of the gearset
// setting (which wasn't taking, so PROS degrees read wrong).
constexpr float kDegreesPerCount = 360.0f / 1800.0f;
// Slow zone: the last kSlowRangeDeg of a move capped at 35%, either direction (the arm
// falls into stage 1 behind going forward, and into home going back).
constexpr float kSlowRangeDeg = 30.0f;
constexpr float kSlowPower = 44.0f; // 35% of 127

// Soft limits, arm degrees from straight down: setpoints are clamped into this range, and
// manual power that would push past a limit holds at the limit instead.
constexpr double kMinArmDeg = 35.0;
constexpr double kMaxArmDeg = 315.0;
// Hard slowdown: within kLimitSlowRangeDeg of a soft limit, anything driving toward it
// (manual or ramp) is capped at kLimitSlowPower.
constexpr double kLimitSlowRangeDeg = 15.0;
constexpr float kLimitSlowPower = 25.0f; // ~20% of 127

lemlib::SmartMotor motor(&arm_pivot_motor, kMaxVelocity, kSettleRange, kDecelerationDeg);
bool manual_active = false;

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
    motor.setRampPowers(kRampStartPower, kRampEndPower);
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

void update() {
    const bool forward = controller.get_digital(controls::kArmPivotForward);
    const bool reverse = controller.get_digital(controls::kArmPivotReverse);

    if (forward || reverse) {
        manual_active = true;
        drive(forward ? kManualPower : -kManualPower);
    } else if (manual_active) {
        manual_active = false;
        motor.holdCurrent();
    }
}

void stop() {
    manual_active = false;
    motor.holdCurrent();
}

void set_slow_zone(bool enabled) {
    motor.setSlowZone(enabled ? kSlowRangeDeg : 0.0f, kSlowPower);
}

void set_power(int power) {
    drive(power);
}

void move_to(double motor_deg) {
    const double target = clamp_motor_deg(motor_deg);
    const double current = motor.getRotation();
    const bool careful = std::fabs(target - current) > kCarefulMoveDeg;
    const bool lifting_behind = target < current && constants::arm_motor_deg_to_deg(target) > 180.0;
    if (careful) {
        motor.setDecelerationDistance(kCarefulDecelerationDeg);
        motor.setRampPowers(kCarefulRampStartPower, kCarefulRampEndPower);
    } else if (lifting_behind) {
        motor.setDecelerationDistance(kLiftBehindDecelerationDeg);
        motor.setRampPowers(kLiftBehindRampStartPower, kLiftBehindRampEndPower);
    } else {
        motor.setDecelerationDistance(kDecelerationDeg);
        motor.setRampPowers(kRampStartPower, kRampEndPower);
    }
    motor.setTarget(static_cast<float>(target));
}

double get_pos() {
    return motor.getRotation();
}

double target() {
    return motor.getTarget();
}

}
