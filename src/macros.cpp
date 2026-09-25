#include "macros.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include "subsystems/elevator.hpp"
#include "subsystems/pivot.hpp"

namespace macros {

namespace {

constexpr double kPivotTurnDeg = 90.0;
constexpr double kElevatorRise = 150.0;

}

void pivot_and_raise() {
    subsystems::pivot::rotate_by(kPivotTurnDeg);
    subsystems::elevator::raise_by(kElevatorRise);
}

void update() {
    if (controller.get_digital_new_press(controls::kPivotAndRaise)) {
        pivot_and_raise();
    }
}

}
