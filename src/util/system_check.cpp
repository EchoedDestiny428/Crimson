#include "util/system_check.hpp"
#include "config/controls.hpp"
#include "macros.hpp"
#include "robotconfig.hpp"
#include "subsystems/claw.hpp"
#include "subsystems/elevator.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/pivot.hpp"
#include "util/driver_task.hpp"
#include <atomic>
#include <cmath>
#include <cstdint>

namespace util::system_check {

namespace {

constexpr int kSpinPower = 127;
constexpr double kHomeTolerance = 5.0;
constexpr std::uint32_t kArmMs = 200;
constexpr std::uint32_t kIntakeMs = 700;
constexpr std::uint32_t kTravelTimeoutMs = 8000;

std::atomic<bool> active{false};

bool button_pressed(controls::Button button) {
    return controller.get_digital(button);
}

bool bumpers_held() {
    return button_pressed(controls::kElevatorUp) && button_pressed(controls::kElevatorDown) &&
           button_pressed(controls::kIntakeIn) && button_pressed(controls::kIntakeOut);
}

bool any_bumper_held() {
    return button_pressed(controls::kElevatorUp) || button_pressed(controls::kElevatorDown) ||
           button_pressed(controls::kIntakeIn) || button_pressed(controls::kIntakeOut);
}

bool wait_ms(std::uint32_t ms) {
    const std::uint32_t start = pros::millis();
    while (driver_control_active() && pros::millis() - start < ms) {
        pros::delay(kLoopMs);
    }
    return driver_control_active();
}

bool at_top() {
    const double height = subsystems::elevator::height();
    return std::isfinite(height) && height >= subsystems::elevator::top_stage();
}

bool at_home() {
    const double height = subsystems::elevator::height();
    return std::isfinite(height) &&
           std::fabs(subsystems::elevator::target() - subsystems::elevator::kHome) <= kHomeTolerance &&
           subsystems::elevator::is_settled() && subsystems::pivot::is_settled();
}

bool hold(bool up, bool down, bool down_pressed, bool (*done)()) {
    const std::uint32_t start = pros::millis();
    while (driver_control_active() && pros::millis() - start < kTravelTimeoutMs) {
        macros::press(up, down, down_pressed);
        down_pressed = false;
        if (done()) {
            return true;
        }
        pros::delay(kLoopMs);
    }
    return false;
}

void run() {
    controller.rumble("-");

    subsystems::intake::spin(kSpinPower);
    if (!wait_ms(kIntakeMs)) {
        return;
    }
    subsystems::intake::spin(-kSpinPower);
    if (!wait_ms(kIntakeMs)) {
        return;
    }
    subsystems::intake::stop();

    subsystems::claw::automatic();
    if (hold(true, false, false, at_top)) {
        macros::press(false, false, false);
    }
    if (!driver_control_active()) {
        return;
    }
    if (hold(false, true, true, at_home) && driver_control_active()) {
        controller.rumble(".");
    }
}

void finish() {
    subsystems::intake::stop();
    subsystems::claw::automatic();
    if (!driver_control_active()) {
        subsystems::elevator::hold();
    }
}

}

bool running() {
    return active.load() || bumpers_held();
}

void update() {
    if (!bumpers_held()) {
        return;
    }

    subsystems::intake::stop();
    subsystems::claw::automatic();
    subsystems::claw::stop();
    subsystems::elevator::hold();
    if (!wait_ms(kArmMs) || !bumpers_held()) {
        subsystems::claw::automatic();
        return;
    }

    active = true;
    run();
    finish();
    while (driver_control_active() && any_bumper_held()) {
        pros::delay(kLoopMs);
    }
    active = false;
}

void stop() {
    active = false;
    subsystems::intake::stop();
    subsystems::elevator::hold();
    subsystems::claw::automatic();
}

}
