#include "main.h"
#include "display/dashboard.hpp"
#include "lemlib/api.hpp"
#include "pros/motors.h"
#include "vision/crimson.hpp"
#include "robotconfig.hpp"
#include "subsystems/elevator.hpp"
#include <cstdint>

// --------------------------------------------------
// Hardware Configuration & LemLib Setup
// --------------------------------------------------
constexpr std::uint8_t kLoopDelayMs = 20;
constexpr std::int8_t kIntakePort = 3; // Change to your intake port (use negative if reversed, e.g. -3)
// below values are all placeholders
constexpr int kElevMove = 127; // Elevator movement speed
constexpr int kElevHold = 10; // Holding constant to offset gravity
constexpr double kElevKp = 1.2; // kP for elevator height control
constexpr double kElevKd = 0.4; // kd for elevator height control
constexpr double kElevMin = 0.0; // min height
constexpr double kElevMax = 2000.0; // max height
constexpr double kDown = 0.0;        // elev collapsed: also the starting position
constexpr double kLoader = 250.0;    // match-load chute — NOT the same as down
constexpr double kAlliance = 400.0;  // 3.25" rim (alliance goal)
constexpr double kShort = 700.0;     // 5.8" rim (neutral quadrant)
constexpr double kTall = 1000.0;     // 8.7" rim(center goal)

double elev_setpoint = 0;
double elev_last_error = 0;

// Pivot macro
constexpr double kPivotMacroDeg = 90.0;    // how far one press turns the pivot
constexpr double kPivotGearRatio = 1.0;    // motor degrees per 1 degree of pivot arm
                                           // e.g. 12T motor gear -> 60T arm gear = 5.0
constexpr int kPivotMacroVelocity = 300;   // rpm (blue cartridge max is 600)
constexpr double kElevMacroRise = 150.0; // TODO: TUNE — how far the elevator rises on the A macro (encoder degrees)

//cap for PID output
int clamp(int x) 
{
    if (x>127) return 127;
    if (x<-127) return -127;
    return x;
}
// elevator setpoint cap
void elev_goto(double target)
{
    if (target < kElevMin) target = kElevMin;
    if (target > kElevMax) target = kElevMax;
    elev_setpoint = target;
}


pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor intake(1, pros::MotorGears::blue);
crimson::Crimson crimson_cam(1);
pros::Imu imu(2);
display::Dashboard dashboard;

pros::MotorGroup left_motors({3, -20}, pros::MotorGears::blue);
pros::MotorGroup right_motors({-2, 19}, pros::MotorGears::blue);
pros::MotorGroup elevator({4, -18}, pros::MotorGears::blue); // Change to the elevator ports once we wire them up, motors are gree
pros::Motor claw(12,pros::MotorGears::blue); // Change to the claw port once we wire it up  
pros::Motor pivot(11,pros::MotorGears::blue); // Change to the pivot port once we wire it up
// elevator hold PID
void elev_hold_pid(double pos)
{
    double error = pos - elevator.get_position();
    double derivative = error - elev_last_error;
    elev_last_error = error;
    int pid_output = clamp(static_cast<int>(kElevKp * error + kElevKd * derivative + kElevHold));
    elevator.move(pid_output);
}

// Drivetrain settings
lemlib::Drivetrain drivetrain(
    &left_motors,                // left motor group
    &right_motors,               // right motor group
    12.5,                        // 12.5 inch track width
    lemlib::Omniwheel::NEW_325,  // using new 3.25" omnis
    360,                         // drivetrain rpm
    2                            // horizontal drift is 2 (for omni wheels)
);

// Lateral motion controller (forward/backward)
lemlib::ControllerSettings linearController(
    10,   // proportional gain (kP)
    0,    // integral gain (kI)
    12,   // derivative gain (kD)
    1,    // anti windup
    0.2,  // small error range, in inches
    100,  // small error range timeout, in milliseconds
    1,    // large error range, in inches
    500,  // large error range timeout, in milliseconds
    40    // maximum acceleration (slew)
);

// Angular motion controller (turning)
lemlib::ControllerSettings angularController(
    5.5,  // proportional gain (kP)
    0,    // integral gain (kI)
    40,   // derivative gain (kD)
    3,    // anti windup
    1,    // small error range, in degrees
    50,   // small error range timeout, in milliseconds
    3,    // large error range, in degrees
    300,  // large error range timeout, in milliseconds
    0     // maximum acceleration (slew)
);

// Sensors for odometry
lemlib::OdomSensors sensors(
    nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2
    &imu     // inertial sensor
);

// Throttle input curve for driver control
lemlib::ExpoDriveCurve throttleCurve(
    2,    // joystick deadband out of 127
    2.5,  // minimum output where drivetrain will move
    1.019 // expo curve gain
);

// Steer input curve for driver control
lemlib::ExpoDriveCurve steerCurve(
    2,    // joystick deadband out of 127
    2.5,  // minimum output where drivetrain will move
    1.0   // expo curve gain
);

// LemLib Chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

void update_dashboard()
{
    double heading = imu.get_heading();
    auto pose = crimson_cam.estimate_global_pose(heading);

    int tag_id = -1;
    if (crimson_cam.has_target())
    {
        tag_id = crimson_cam.get_raw_target().id;
    }

    dashboard.update(pose, crimson_cam.get_tv(), tag_id);
}

// --------------------------------------------------
// Mechanism Task Helper
// --------------------------------------------------
// Runs a two-button motor (hold fwd = +127, hold rev = -127, else brake)
// in its own loop. Templated so it works with pros::Motor or pros::MotorGroup.
//
// The loop exits on its own when the robot leaves driver control. PROS kills
// opcontrol() when the competition state changes, but NOT the tasks it
// spawned, so without this check they'd keep running during autonomous and
// stack up duplicates every time opcontrol() restarts.
template <typename MotorT>
static void mechanism_loop(MotorT& motor,
                           pros::controller_digital_e_t fwd_btn,
                           pros::controller_digital_e_t rev_btn)
{
    while (!pros::competition::is_disabled() && !pros::competition::is_autonomous())
    {
        if (controller.get_digital(fwd_btn))
        {
            motor.move(127);
        }
        else if (controller.get_digital(rev_btn))
        {
            motor.move(-127);
        }
        else
        {
            motor.brake();
        }

        pros::delay(kLoopDelayMs);
    }

    motor.brake();
}

// Pivot: Hold X = +, Hold B = -, Tap A = rotate exactly +90 degrees and raise elevator
static void pivot_loop()
{
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
            elev_goto(elev_setpoint + kElevMacroRise);
        }
        else if (!macro_active)
        {
            pivot.brake();
        }

        pros::delay(kLoopDelayMs);
    }

    pivot.brake();
}

void initialize()
{
    crimson_cam.initialize();
    crimson_cam.set_camera_mount_metrics(100.0, 0.0);
    dashboard.initialize();

    pros::screen::set_pen(0xFFFFFF);
    pros::screen::print(pros::E_TEXT_MEDIUM, 250, 100, "Calibrating chassis...");

    // Calibrate chassis (also calibrates the IMU connected in sensors)
    chassis.calibrate();

    // Initialize elevator subsystem
    subsystems::elevator_init();
}

void disabled() {}

void competition_initialize() {}

void autonomous()
{
    // Example LemLib autonomous routine:
    // chassis.setPose(0, 0, 0);
    // chassis.moveToPoint(0, 24, 2000);
}

void opcontrol()
{
    elevator.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pivot.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // --- Mechanism tasks (each runs independently) ---

    // Intake: Hold R1 = in, Hold R2 = out
    pros::Task intake_task([] {
        mechanism_loop(intake, pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_R2);
    }, "Intake");
    // Claw: Hold UP = +, Hold DOWN = -
    pros::Task claw_task([] {
        mechanism_loop(claw, pros::E_CONTROLLER_DIGITAL_UP, pros::E_CONTROLLER_DIGITAL_DOWN);
    }, "Claw");

        // Pivot: Hold X = +, Hold B = -, Tap A = +90 degrees and raise elevator
    pros::Task pivot_task(pivot_loop, "Pivot");

    // --- Main thread: drive + camera + dashboard ---
    while (true)
    {
        // LemLib Arcade Drive (split arcade: left stick Y for throttle, right stick X for turn)
        int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(throttle, turn);

        crimson_cam.update();
        update_dashboard();

        // Update elevator control
        subsystems::elevator_update();

        pros::delay(kLoopDelayMs);
    }
}
