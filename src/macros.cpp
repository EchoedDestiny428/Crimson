#include "macros.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include "subsystems/claw.hpp"
#include "subsystems/pivot.hpp"
#include <algorithm>

namespace macros {

namespace claw = subsystems::claw;
namespace elevator = subsystems::elevator;
namespace pivot = subsystems::pivot;

namespace {

enum class Mode { Stowed, Deployed };

constexpr int kManualPower = 127;
constexpr double kTopArrivalTolerance = 50.0;

constexpr std::uint32_t kElevatorTimeoutMs = 1500;
constexpr std::uint32_t kPivotTimeoutMs = 1000;
constexpr std::uint32_t kDualPickupDwellMs = 200;
constexpr int kDualPickupClawPower = 127;

Mode mode = Mode::Stowed;
bool manual_active = false;
bool tilted = false;

void tilt_if_at_top() {
    if (!tilted && elevator::height() >= elevator::top_stage() - kTopArrivalTolerance) {
        tilted = true;
        pivot::move_to(pivot::kTopMotorDeg);
    }
}

void untilt_pivot() {
    if (tilted) {
        tilted = false;
        pivot::move_to(pivot::kFlippedMotorDeg);
    }
}

}

void flip_out(double height) {
    mode = Mode::Deployed;
    manual_active = false;
    tilted = false;
    pivot::move_to(pivot::kFlippedMotorDeg);
    elevator::set_target(std::max(height, elevator::kFlipOut));
}

void go_to(double height) {
    if (height < elevator::kFlipOut) {
        home();
        return;
    }
    if (mode == Mode::Stowed) {
        flip_out(height);
        return;
    }
    manual_active = false;
    if (height < elevator::top_stage()) {
        untilt_pivot();
    }
    elevator::set_target(height);
}

void home() {
    mode = Mode::Stowed;
    manual_active = false;
    tilted = false;
    pivot::move_to(pivot::kHomeMotorDeg);
    elevator::set_target(elevator::kHome);
}

bool wait_until_done(std::uint32_t timeout_ms) {
    const std::uint32_t start = pros::millis();
    const bool elevator_done = elevator::wait_until_settled(timeout_ms);
    if (mode == Mode::Deployed) {
        tilt_if_at_top();
    }

    const std::uint32_t elapsed = pros::millis() - start;
    const std::uint32_t remaining = elapsed < timeout_ms ? timeout_ms - elapsed : 0;
    const bool pivot_done = pivot::wait_until_settled(remaining);
    return elevator_done && pivot_done;
}

void dual_setup() {
    mode = Mode::Deployed;
    manual_active = false;
    tilted = false;
    elevator::set_target(elevator::kDualSetup);
    elevator::wait_until_settled(kElevatorTimeoutMs);
    pivot::move_to(pivot::kDualSetupDeg);
    pivot::wait_until_settled(kPivotTimeoutMs);
}

void dual_pickup() {
    claw::spin(kDualPickupClawPower);
    pivot::move_to(pivot::kDualPickupDeg);
    pivot::wait_until_settled(kPivotTimeoutMs);
    pros::delay(kDualPickupDwellMs);
    pivot::move_to(pivot::kFlippedMotorDeg);
    pivot::wait_until_settled(kPivotTimeoutMs);
    claw::stop();
}

void update() {
    const bool up = controller.get_digital(controls::kElevatorUp);
    const bool down = controller.get_digital(controls::kElevatorDown);
    const bool down_pressed = controller.get_digital_new_press(controls::kElevatorDown);

    if (mode == Mode::Stowed) {
        if (up) {
            flip_out();
        } else if (down_pressed) {
            home();
        }
        return;
    }

    if (down && elevator::height() < elevator::kFlipOut) {
        home();
        return;
    }

    if (up || down) {
        if (down) {
            untilt_pivot();
        }
        manual_active = true;
        elevator::set_manual(up ? kManualPower : -kManualPower);
    } else if (manual_active) {
        manual_active = false;
        elevator::snap_to_stage();
    }

    if (!down) {
        tilt_if_at_top();
    }
}

void stop() {
    manual_active = false;
    elevator::hold();
}

}
