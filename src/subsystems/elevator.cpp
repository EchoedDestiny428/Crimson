#include "subsystems/elevator.hpp"
#include "robotconfig.hpp"
#include "pros/rtos.h"

namespace subsystems {

// Elevator tuning constants
constexpr int kElevMove = 80;       // Elevator movement speed
constexpr int kElevHold = 10;       // Holding constant to offset gravity
constexpr double kElevKp = 1.2;     // kP for elevator height control
constexpr double kElevKd = 0.4;     // kD for elevator height control
constexpr double kElevMin = 0.0;    // min height
constexpr double kElevMax = 2000.0; // max height

// Elevator height presets
constexpr double kDown = 0.0;       // elev collapsed: also the starting position
constexpr double kLoader = 250.0;   // match-load chute — NOT the same as down
constexpr double kAlliance = 400.0; // 3.25" rim (alliance goal)
constexpr double kShort = 700.0;    // 5.8" rim (neutral quadrant)
constexpr double kTall = 1000.0;    // 8.7" rim (center goal)

// State variables
double elev_setpoint = 0;
double elev_last_error = 0;

// Clamp PID output to motor range
int clamp(int x) {
    if (x > 127) return 127;
    if (x < -127) return -127;
    return x;
}

void elevator_init() {
    elevator.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    elevator.tare_position();
    elev_setpoint = elevator.get_position();
}

void elev_goto(double target) {
    if (target < kElevMin) target = kElevMin;
    if (target > kElevMax) target = kElevMax;
    elev_setpoint = target;
}

void elev_hold_pid(double pos) {
    double error = pos - elev_setpoint;
    double derivative = error - elev_last_error;
    elev_last_error = error;
    int pid_output = clamp(static_cast<int>(kElevKp * error + kElevKd * derivative + kElevHold));
    elevator.move(pid_output);
}

void elevator_update(double pos) {
    // Elevator preset buttons
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

    // Elevator manual controls (Hold L1 = up, Hold L2 = down)
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        elevator.move(kElevMove);
        elev_setpoint = pos; // hold this height
        elev_last_error = 0.0; // for D-term
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        elevator.move(-kElevMove);
        elev_setpoint = pos; // hold this height
        elev_last_error = 0.0; // for D-term
    } else {
        elev_hold_pid(pos);
    }
}

} // namespace subsystems
