#include "main.h"
#include "display/dashboard.hpp"
#include "lemlib/api.hpp"
#include "pros/motors.h"
#include "vision/crimson.hpp"
#include "robotconfig.hpp"
#include "subsystems/elevator.hpp"

// --------------------------------------------------
// Hardware Configuration & LemLib Setup
// --------------------------------------------------
constexpr std::uint8_t kLoopDelayMs = 20;
constexpr std::int8_t kIntakePort = 3; // Change to your intake port (use negative if reversed, e.g. -3)

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

        // Update elevator control
        subsystems::elevator_update();

        pros::delay(kLoopDelayMs);
    }
}
