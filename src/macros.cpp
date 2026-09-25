#include "macros.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include "subsystems/elevator.hpp"
#include "subsystems/pivot.hpp"

namespace macros {

namespace elevator = subsystems::elevator;
namespace pivot = subsystems::pivot;

namespace {

enum class Mode { Stowed, Deployed };

constexpr int kManualPower = 127;
constexpr double kTopArrivalTolerance = 50.0;

Mode mode = Mode::Stowed;
bool manual_active = false;
bool tilted = false;

void untilt_pivot() {
    if (tilted) {
        tilted = false;
        pivot::move_to(pivot::kFlippedMotorDeg);
    }
}

}

void flip_out() {
    mode = Mode::Deployed;
    manual_active = false;
    tilted = false;
    pivot::move_to(pivot::kFlippedMotorDeg);
    elevator::set_target(elevator::kFlipOut);
}

void home() {
    mode = Mode::Stowed;
    manual_active = false;
    tilted = false;
    pivot::move_to(pivot::kHomeMotorDeg);
    elevator::set_target(elevator::kHome);
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

    if (!down && !tilted && elevator::height() >= elevator::top_stage() - kTopArrivalTolerance) {
        tilted = true;
        pivot::move_to(pivot::kTopMotorDeg);
    }
}

void stop() {
    manual_active = false;
    elevator::hold();
}

}
