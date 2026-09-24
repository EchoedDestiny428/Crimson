#include "subsystems/elevator.hpp"
#include "robotconfig.hpp"
#include "pros/rtos.h"

namespace subsystems {

// Elevator tuning constants
constexpr int kElevMove = 127;   // elevator movement speed
// constexpr int kElevHold = 10;    // holding constant to offset gravity
constexpr double kElevKp = 1.2;  // kP for elevator height control TODO: tune
constexpr double kElevKd = 0.4;  // kD for elevator height control TODO: tune
constexpr double kElevMin = 0.0;    // min height
constexpr double kElevMax = 2000.0; // max height

// Elevator height presets
constexpr double kDown = 0.0;       // elev collapsed: also the starting position
constexpr double kLoader = 250.0;   // match-load chute — NOT the same as down
constexpr double kAlliance = 400.0; // 3.25" rim (alliance goal)
constexpr double kShort = 700.0;    // 5.8" rim (neutral quadrant)
constexpr double kTall = 1000.0;    // 8.7" rim (center goal)

// Last height commanded by elev_goto
static double current_target = 0.0;

// SmartMotor for elevator PID control
// Create with static initialization to avoid pointer allocation
static lemlib::SmartMotor elevator_motor(
    &elevator, lemlib::PID(static_cast<float>(kElevKp), 0, static_cast<float>(kElevKd), 0, false));

void elevator_init() {
    elevator.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    elevator_motor.reset();
    
    // Start logging commanded and real voltage
    elevator_motor.startLogging(true);
}

void elev_goto(double target, float timeout) {
    if (target < kElevMin) target = kElevMin;
    if (target > kElevMax) target = kElevMax;

    current_target = target;

    // Move to target using SmartMotor PID (async mode)
    elevator_motor.movePID(target, timeout, 0.5, true);
}

double elev_setpoint() {
    return current_target;
}

void elevator_update() {
    // Preset buttons - trigger async movement to preset heights
    // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
        elev_goto(kDown);
    // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
        elev_goto(kTall);
    // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        elev_goto(kLoader);
    // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
        elev_goto(kAlliance);
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        elev_goto(kShort);

    // Hold L1 = up at kElevMove, Hold L2 = down. On release, hold that height.
    static bool manual = false;
    const bool up = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    const bool down = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

    if (up) {
        elevator.move(kElevMove);
        current_target = elevator_motor.getRotation();
        manual = true;
    } else if (down) {
        elevator.move(-kElevMove);
        current_target = elevator_motor.getRotation();
        manual = true;
    } else if (manual) {
        manual = false;
        elev_goto(current_target);
    }
}

} // namespace subsystems
