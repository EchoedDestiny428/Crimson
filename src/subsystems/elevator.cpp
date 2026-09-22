#include "subsystems/elevator.hpp"
#include "robotconfig.hpp"
#include "pros/rtos.h"

namespace subsystems {

// Elevator tuning constants
constexpr double kElevMin = 0.0;    // min height
constexpr double kElevMax = 2000.0; // max height

// Manual control speeds
constexpr int kElevManualUp = 80;
constexpr int kElevManualDown = -80;

// Elevator height presets
constexpr double kDown = 0.0;       // elev collapsed: also the starting position
constexpr double kLoader = 250.0;   // match-load chute — NOT the same as down
constexpr double kAlliance = 400.0; // 3.25" rim (alliance goal)
constexpr double kShort = 700.0;    // 5.8" rim (neutral quadrant)
constexpr double kTall = 1000.0;    // 8.7" rim (center goal)

// SmartMotor for elevator PID control
// PID gains: kP=1.2, kI=0, kD=0.4
// Create with static initialization to avoid pointer allocation
static lemlib::SmartMotor elevator_motor(&elevator, lemlib::PID(1.2, 0, 0.4, 0, false));

void elevator_init() {
    elevator.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    elevator_motor.reset();
}

void elev_goto(double target, float timeout) {
    if (target < kElevMin) target = kElevMin;
    if (target > kElevMax) target = kElevMax;
    
    // Move to target using SmartMotor PID (async mode)
    elevator_motor.movePID(target, timeout, 0.5, true);
}

void elevator_update() {
    // Preset buttons - trigger async movement to preset heights
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
        elev_goto(kDown);
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
        elev_goto(kTall);
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        elev_goto(kLoader);
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
        elev_goto(kAlliance);
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        elev_goto(kShort);

    // Manual elevator controls (Hold L1 = up, Hold L2 = down)
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        elevator.move(kElevManualUp);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        elevator.move(kElevManualDown);
    }
    // Note: When neither L1 nor L2 is pressed, the SmartMotor's async task
    // will continue running and holding the last setpoint via its PID loop
}

} // namespace subsystems
