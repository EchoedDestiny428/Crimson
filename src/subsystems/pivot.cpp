#include "subsystems/pivot.hpp"
#include "subsystems/elevator.hpp"
#include "robotconfig.hpp"
#include "pros/rtos.hpp"

namespace subsystems {

constexpr double kPivotMacroDeg = 90.0;  // how far one press turns the pivot
constexpr double kPivotGearRatio = 1.0;  // motor degrees per 1 degree of pivot arm
                                         // e.g. 12T motor gear -> 60T arm gear = 5.0
constexpr int kPivotMacroVelocity = 300; // rpm (blue cartridge max is 600)
constexpr double kElevMacroRise = 150.0; // TODO: TUNE — how far the elevator rises on the A macro
constexpr int kLoopDelayMs = 20;

void pivot_init() {
    pivot.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void pivot_loop() {
    bool macro_active = false;
    double macro_target = 0.0;

    while (!pros::competition::is_disabled() && !pros::competition::is_autonomous())
    {
        const bool fwd = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
        const bool rev = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

        if (fwd || rev)
        {
            // Manual control always overrides the macro
            macro_active = false;
            pivot.move(fwd ? 127 : -127);
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
        {
            // Pivot: exactly +90 degrees
            const double base = macro_active ? macro_target : pivot.get_position();
            macro_target = base + kPivotMacroDeg * kPivotGearRatio;
            macro_active = true;
            pivot.move_absolute(macro_target, kPivotMacroVelocity);

            // Elevator: rise by kElevMacroRise at the same time
            subsystems::elev_goto(subsystems::elev_setpoint() + kElevMacroRise);
        }
        else if (!macro_active)
        {
            pivot.brake();
        }

        pros::delay(kLoopDelayMs);
    }

    pivot.brake();
}

} // namespace subsystems
