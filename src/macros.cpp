#include "macros.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include "subsystems/elevator.hpp"
#include "subsystems/pivot.hpp"

namespace macros {

namespace elevator = subsystems::elevator;
namespace pivot = subsystems::pivot;

namespace {

enum class Mode { Stowed, FlippingOut, Deployed };

constexpr int kManualPower = 127;
constexpr double kArrivalTolerance = 10.0;

Mode mode = Mode::Stowed;
bool manual_active = false;

}

void flip_out() {
    mode = Mode::FlippingOut;
    manual_active = false;
    pivot::move_to(pivot::kFlippedDeg);
    elevator::set_target(elevator::kFlipOut);
}

void home() {
    mode = Mode::Stowed;
    manual_active = false;
    pivot::move_to(pivot::kHomeDeg);
    elevator::set_target(elevator::kHome);
}

void update() {
    const bool up = controller.get_digital(controls::kElevatorUp);
    const bool down = controller.get_digital(controls::kElevatorDown);
    const bool down_pressed = controller.get_digital_new_press(controls::kElevatorDown);
    const double height = elevator::height();

    if (mode == Mode::Stowed) {
        if (up) {
            flip_out();
        } else if (down_pressed) {
            home();
        }
        return;
    }

    if (down && height < elevator::kFlipOut) {
        home();
        return;
    }

    if (mode == Mode::FlippingOut) {
        if (height < elevator::kFlipOut - kArrivalTolerance) {
            return;
        }
        mode = Mode::Deployed;
    }

    if (up || down) {
        manual_active = true;
        elevator::set_manual(up ? kManualPower : -kManualPower);
    } else if (manual_active) {
        manual_active = false;
        elevator::snap_to_stage();
    }
}

void stop() {
    manual_active = false;
    elevator::hold();
}

}
