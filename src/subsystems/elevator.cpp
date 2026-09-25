#include "subsystems/elevator.hpp"
#include "config/controls.hpp"
#include "lemlib/smartMotor.hpp"
#include "robotconfig.hpp"
#include <algorithm>
#include <array>

namespace subsystems::elevator {

namespace {

constexpr float kP = 0.3f;
constexpr float kI = 0.0f;
constexpr float kD = 1.0f;
constexpr float kFeedforward = 0.0f;
constexpr float kSettleRange = 5.0f;
constexpr int kManualPower = 127;

constexpr double kMin = 0.0;
constexpr double kMax = 2000.0;

constexpr double kDown = 0.0;
constexpr double kLoader = 250.0;
constexpr double kAlliance = 400.0;
constexpr double kShort = 700.0;
constexpr double kTall = 1000.0;
constexpr std::array kPresets{kDown, kLoader, kAlliance, kShort, kTall};
constexpr double kPresetTolerance = 1.0;

lemlib::SmartMotor motor(&elevator_motors, lemlib::PID(kP, kI, kD), kSettleRange, kFeedforward);
bool manual_active = false;

double preset_above(double height) {
    for (const double preset : kPresets) {
        if (preset > height + kPresetTolerance) {
            return preset;
        }
    }
    return kPresets.back();
}

double preset_below(double height) {
    for (auto it = kPresets.rbegin(); it != kPresets.rend(); ++it) {
        if (*it < height - kPresetTolerance) {
            return *it;
        }
    }
    return kPresets.front();
}

}

void init() {
    elevator_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    motor.reset();
    motor.setLogging(true);
    motor.start();
}

void update() {
    const bool up = controller.get_digital(controls::kElevatorUp);
    const bool down = controller.get_digital(controls::kElevatorDown);
    const bool step_up = controller.get_digital_new_press(controls::kElevatorStepUp);
    const bool step_down = controller.get_digital_new_press(controls::kElevatorStepDown);
    const bool home = controller.get_digital_new_press(controls::kElevatorHome);

    const int direction = up ? 1 : down ? -1 : 0;
    const double height = motor.getRotation();
    const bool at_limit = (direction > 0 && height >= kMax) || (direction < 0 && height <= kMin);

    if (direction != 0 && !at_limit) {
        motor.setManual(direction * kManualPower);
        manual_active = true;
        return;
    }

    if (manual_active) {
        manual_active = false;
        motor.holdCurrent();
    }

    if (step_up) {
        set_target(preset_above(target()));
    } else if (step_down) {
        set_target(preset_below(target()));
    } else if (home) {
        set_target(kDown);
    }
}

void stop() {
    manual_active = false;
    motor.holdCurrent();
}

void set_target(double height) {
    motor.setTarget(static_cast<float>(std::clamp(height, kMin, kMax)));
}

void raise_by(double delta) {
    set_target(target() + delta);
}

double target() {
    return motor.getTarget();
}

bool wait_until_settled(std::uint32_t timeout_ms) {
    return motor.waitUntilSettled(timeout_ms);
}

}
