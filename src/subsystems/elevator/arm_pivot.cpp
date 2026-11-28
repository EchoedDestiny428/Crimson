#include "subsystems/elevator/arm_pivot.hpp"
#include "config/controls.hpp"
#include "lemlib/smartMotor.hpp"
#include "robotconfig.hpp"
#include "subsystems/elevator/extension.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

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
// 45, not 30 (arm-tuning run 49, 2026-11-28): after an L3 overshoot with a cup-pin, 30 (-2.8 V) could barely lift the
// arm back against gravity (it stayed 4-9 deg past); 45 brings it back within 2 deg every time. (Empty, 20 stalled at
// ~235 climbing toward 180.)
constexpr float kMinPower = 45.0f;
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
constexpr double kMinArmDeg = constants::kArmHardstopDeg; // the lower hardstop
constexpr double kMaxArmDeg = 315.0;
// Hard slowdown: within kLimitSlowRangeDeg of a soft limit, anything driving toward it
// (manual or PID) is capped at kLimitSlowPower.
constexpr double kLimitSlowRangeDeg = 15.0;
constexpr float kLimitSlowPower = 25.0f; // ~20% of 127

// Swing profile: the arm speeds up and slows down by at most a set rate (deg/s^2) instead of full power until the PID
// lets go. Measured with a cup-pin (arm-swing-testing's automatic test, 2026-11-26):
// - Lift down (L3): full power took 0.77 s to get within 2 deg (it overshoots and hunts) and slid the robot 0.9 in;
//   3000 took 0.59 s and slid it 0.4 in, and barely tipped it.
// - Lift all the way up (L7): the robot rocks past its lean 2.3 deg at 1000, 4.6 at 1500, 4.9 at 2000 and 7.5 at
//   3000 (at a claw ~1.2 m up, 2.3 deg is ~5 cm). The arm's shove tips the robot about the wheels through the pivot,
//   so the higher the pivot, the gentler the swing has to be.
// So the rate scales with 1 / pivot height: kSwingAccelLiftDownDegPerS2 with the lift down, about 1100 at L7, never
// under kSwingAccelMinDegPerS2 (the gentlest measured). The height is the higher of where the lift is and where it is
// going, updated every 10 ms: a swing out to a high stage is gentle from the start, a swing home speeds up as the lift
// comes down.
constexpr double kSwingAccelLiftDownDegPerS2 = 3000.0;
constexpr double kSwingAccelMinDegPerS2 = 1000.0;
constexpr float kSwingMaxSpeedDegPerS = 480.0f; // a red motor tops out at 600 deg/s; the rest is room for the PID
constexpr float kSwingKV = 127.0f / 600.0f;     // power per deg/s of setpoint speed (full power is ~600 deg/s)
constexpr float kSwingMaxLagDeg = 15.0f;        // the setpoint waits for the arm if it gets this far ahead
// The speed push fades out over the first kSwingAheadFadeDeg the arm gets ahead of its setpoint (gravity carrying it
// down the far side of a swing): full push wasn't letting the PID brake, and every swing in run 37 overshot 12-25 deg.
constexpr float kSwingAheadFadeDeg = 5.0f;
// And while the arm is ahead and faster than its setpoint, push back kSwingAheadBrake per deg/s of the difference: the
// fade alone left L3 (44 deg past straight up, gravity pulling it on) overshooting ~13 deg in a model fitted to run 37;
// with this ~7-9 deg, the low-angle swings 3-5 deg, and none stopping short, across +-20-30% model error. 0.4 started
// ending past the 2 deg hold range.
constexpr float kSwingAheadBrake = 0.3f;
// Swinging home (from a stage back to the front): the last 40% at 40% speed, so the arm eases into the hardstop
// instead of slamming.
constexpr float kHomeFinishFraction = 0.4f;
constexpr float kHomeFinishSpeedDegPerS = 0.4f * kSwingMaxSpeedDegPerS;
// Swings out (move_to): the last kOutFinishFraction at kOutFinishSpeed. The fade and brake above turned out not to stop
// the overshoot (arm-tuning, 2026-11-28, cup-pin): past the top the arm and pin run ahead of the motor into the arm's
// slack, where the motor's encoder can't see them (it pushes on at 0 mA), then the slack closes and the arm drags the
// motor 10-15 deg past whatever the brake. Arriving slower leaves less to carry it past: with the last 20% at 40%,
// L1 / L2 / L2N overshoot 2-3 deg (was 10-17), L3 ~10 deg and comes back (with kMinPower 45), every swing ends within
// 1.7 deg, and the claw can open after ~1.1-1.2 s. 15% was 0.1 s faster in all but rocked the robot 4.5 deg; 25-35%
// and 60% speed were worse.
constexpr float kOutFinishFraction = 0.2f;
constexpr float kOutFinishSpeedDegPerS = 0.4f * kSwingMaxSpeedDegPerS;

std::atomic<float> swing_accel_now{static_cast<float>(kSwingAccelLiftDownDegPerS2)};
// The fade and brake in use, from kSwingAheadFadeDeg / kSwingAheadBrake; set_swing_tuning changes them (arm-tuning).
std::atomic<float> swing_fade_now{kSwingAheadFadeDeg};
std::atomic<float> swing_brake_now{kSwingAheadBrake};
// The swing-out slowdown in use, from kOutFinishFraction / kOutFinishSpeedDegPerS; set_swing_finish changes it.
std::atomic<float> out_finish_fraction{kOutFinishFraction};
std::atomic<float> out_finish_speed{kOutFinishSpeedDegPerS};

lemlib::SmartMotor motor(&arm_pivot_motor, lemlib::PID(kP, 0.0f, kD), kSettleRange);

double swing_accel_for(double lift_cm) {
    const double pivot_cm = constants::kPivotStartHeightCm + std::max(0.0, lift_cm);
    return std::clamp(kSwingAccelLiftDownDegPerS2 * constants::kPivotStartHeightCm / pivot_cm,
                      kSwingAccelMinDegPerS2, kSwingAccelLiftDownDegPerS2);
}

void update_profile() {
    const double lift_cm = constants::extension_motor_deg_to_cm(std::max(extension::height(), extension::target()));
    const double accel = std::isfinite(lift_cm) ? swing_accel_for(lift_cm) : kSwingAccelMinDegPerS2;
    swing_accel_now = static_cast<float>(accel);
    motor.setProfile(static_cast<float>(accel), kSwingMaxSpeedDegPerS, kSwingKV, kSwingMaxLagDeg, swing_fade_now,
                     swing_brake_now);
}

// Homing: PID to the home angle, then once within kHomeHandoffDeg of it, push down at
// kHomePushPower until the arm sits on the hardstop: it has moved less than kStallDeg over
// kStallWindowMs (or kHomePushTimeoutMs passed), then hold there. Stalled within
// kRezeroWindowDeg of where the encoder thinks the hardstop is, the encoder is reset to it
// (new-robot), so a slipped arm comes back right after every home; stalled further off, it's
// more likely caught on something, and the encoder is left alone.
constexpr float kHomeHandoffDeg = 2.0f;
constexpr int kHomePushPower = -20; // of 127, ~1.9 V
constexpr float kStallDeg = 0.5f;
constexpr std::uint32_t kStallWindowMs = 100;
constexpr std::uint32_t kHomePushTimeoutMs = 1500;
constexpr std::uint32_t kHomeLoopMs = 10;
constexpr double kRezeroWindowDeg = 10.0;

enum class Home { Idle, Approach, Push };
std::atomic<Home> home_state{Home::Idle};

void home_loop() {
    std::uint32_t push_start = 0;
    std::uint32_t window_start = 0;
    float window_pos = 0.0f;
    std::uint32_t now = pros::millis();
    while (true) {
        update_profile();
        const Home state = home_state.load();
        const float pos = motor.getRotation();
        if (state == Home::Approach && std::isfinite(pos) && pos - motor.getTarget() < kHomeHandoffDeg) {
            Home expected = Home::Approach; // unless a new command cancelled homing meanwhile
            if (home_state.compare_exchange_strong(expected, Home::Push)) {
                motor.setManual(kHomePushPower);
                push_start = window_start = now;
                window_pos = pos;
            }
        } else if (state == Home::Push && std::isfinite(pos)) {
            if (now - window_start >= kStallWindowMs) {
                const bool stalled = std::fabs(pos - window_pos) < kStallDeg;
                if (stalled || now - push_start >= kHomePushTimeoutMs) {
                    Home expected = Home::Push;
                    if (home_state.compare_exchange_strong(expected, Home::Idle)) {
                        const double hardstop = constants::arm_deg_to_motor_deg(constants::kArmHardstopDeg);
                        if (stalled && std::fabs(pos - hardstop) < kRezeroWindowDeg) {
                            motor.setPosition(static_cast<float>(hardstop));
                        }
                        motor.holdCurrent(); // brakes (HOLD) on the hardstop
                    }
                }
                window_start = now;
                window_pos = pos;
            }
        }
        pros::Task::delay_until(&now, kHomeLoopMs);
    }
}

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
    arm_pivot_motor.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST); // starts disabled: coast until auton/driver
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
    update_profile();
    motor.start();
    pros::Task home_task(home_loop, "Arm home");
}

double swing_accel() {
    return swing_accel_now;
}

void set_swing_tuning(double fade_deg, double brake) {
    swing_fade_now = static_cast<float>(std::max(0.0, fade_deg));
    swing_brake_now = static_cast<float>(std::max(0.0, brake));
}

double swing_fade() {
    return swing_fade_now;
}

double swing_brake() {
    return swing_brake_now;
}

void set_swing_finish(double fraction, double speed_fraction) {
    out_finish_speed = static_cast<float>(std::clamp(speed_fraction, 0.0, 1.0) * kSwingMaxSpeedDegPerS);
    out_finish_fraction = static_cast<float>(std::clamp(fraction, 0.0, 1.0));
}

double swing_finish_fraction() {
    return out_finish_fraction;
}

double swing_finish_speed_fraction() {
    return out_finish_speed / kSwingMaxSpeedDegPerS;
}

namespace {
std::atomic<float> hold_power_now{kMinPower};
}

void set_hold_power(double power) {
    hold_power_now = static_cast<float>(std::clamp(power, 0.0, 127.0));
    motor.setMinPower(hold_power_now);
}

double hold_power() {
    return hold_power_now;
}

void stop() {
    home_state = Home::Idle;
    motor.holdCurrent();
}

void set_hold(bool hold) {
    home_state = Home::Idle;
    arm_pivot_motor.set_brake_mode_all(hold ? pros::E_MOTOR_BRAKE_HOLD : pros::E_MOTOR_BRAKE_COAST);
    if (hold) {
        motor.holdCurrent();
    }
}

void home(double motor_deg) {
    motor.setProfileFinish(kHomeFinishFraction, kHomeFinishSpeedDegPerS);
    motor.setTarget(static_cast<float>(clamp_motor_deg(motor_deg)));
    home_state = Home::Approach;
}

bool homing() {
    return home_state != Home::Idle;
}

bool is_settled() {
    return home_state == Home::Idle && motor.isSettled();
}

void set_slow_zone(bool enabled) {
    motor.setSlowZone(enabled ? kSlowRangeDeg : 0.0f, kSlowPower);
}

void set_power(int power) {
    home_state = Home::Idle;
    drive(power);
}

void move_to(double motor_deg) {
    home_state = Home::Idle;
    motor.setProfileFinish(out_finish_fraction, out_finish_speed); // the last 20% at 40% (kOutFinish*)
    motor.setTarget(static_cast<float>(clamp_motor_deg(motor_deg)));
}

double get_pos() {
    return motor.getRotation();
}

double target() {
    return motor.getTarget();
}

}
