#include "main.h"
#include "display/dashboard.hpp"
#include "lemlib/api.hpp"
#include "pros/motors.h"
#include "vision/crimson.hpp"
#include "robotconfig.hpp"
// --------------------------------------------------
// Hardware Configuration & LemLib Setup
// --------------------------------------------------
constexpr std::uint8_t kLoopDelayMs = 20;
constexpr std::int8_t kIntakePort = 3; // Change to your intake port (use negative if reversed, e.g. -3)
// below values are all placeholders
constexpr int kElevMove = 80; // Elevator movement speed
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


// elevator hold PID
void elev_hold_pid(double pos)
{
    double error = pos - elevator.get_position();
    double derivative = error - elev_last_error;
    elev_last_error = error;
    int pid_output = clamp(static_cast<int>(kElevKp * error + kElevKd * derivative + kElevHold));
    elevator.move(pid_output);
}

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
        else if (controller.get_digit   al(rev_btn))
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

void initialize()
{
    crimson_cam.initialize();
    crimson_cam.set_camera_mount_metrics(100.0, 0.0);
    dashboard.initialize();

    pros::screen::set_pen(0xFFFFFF);
    pros::screen::print(pros::E_TEXT_MEDIUM, 250, 100, "Calibrating chassis...");

    // Calibrate chassis (also calibrates the IMU connected in sensors)
    chassis.calibrate();

    //tare the elevator encoders
    elevator.tare_position();
    elevator.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    elev_setpoint = elevator.get_position(); 
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

    // Elevator: Hold L1 = up, Hold L2 = down
    pros::Task elevator_task([] {
        mechanism_loop(elevator, pros::E_CONTROLLER_DIGITAL_L1, pros::E_CONTROLLER_DIGITAL_L2);
    }, "Elevator");

    // Claw: Hold UP = +, Hold DOWN = -
    pros::Task claw_task([] {
        mechanism_loop(claw, pros::E_CONTROLLER_DIGITAL_UP, pros::E_CONTROLLER_DIGITAL_DOWN);
    }, "Claw");

    // Pivot: Hold X = +, Hold B = -
    pros::Task pivot_task([] {
        mechanism_loop(pivot, pros::E_CONTROLLER_DIGITAL_X, pros::E_CONTROLLER_DIGITAL_B);
    }, "Pivot");

    // --- Main thread: drive + camera + dashboard ---
    while (true)
    {
        // LemLib Arcade Drive (split arcade: left stick Y for throttle, right stick X for turn)
        int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(throttle, turn);

        crimson_cam.update();
        update_dashboard();

        // Intake controls (Hold R1 = in, Hold R2 = out)
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            intake.move(127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            intake.move(-127);
        }
        else
        {
            intake.move(0);
        }

        const double pos = elevator.get_position();

        //presets: kvalues are all constants. Must tune for each height
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

        // Elevator controls (Hold L1 = up, Hold L2 = down)
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            elevator.move(kElevMove);
            elev_setpoint = pos; // hold this height
            elev_last_error = 0.0; // for D-term
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            elevator.move(-kElevMove);
            elev_setpoint = pos; // hold this height
            elev_last_error = 0.0; // for D-term
        }
        else
        {
            elev_hold_pid(pos);
        }
        pros::delay(kLoopDelayMs);
    }
}