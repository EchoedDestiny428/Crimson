#include "main.h"
#include "display/dashboard.hpp"
#include "lemlib/api.hpp"
#include "pros/motors.h"
#include "vision/crimson.hpp"
#include "robotconfig.hpp"
#include "subsystems/elevator.hpp"
#include "subsystems/pivot.hpp"
#include <cmath>
#include <cstdint>

// --------------------------------------------------
// Hardware Configuration & LemLib Setup
// --------------------------------------------------
constexpr std::uint8_t kLoopDelayMs = 20;

// Anti-tipping over. Positive pitch is treated as a forward lean (TODO: check with the real robot); flip the sign to -1
// if a tip test drives the robot the wrong way.
constexpr double kForwardPitchSign = 1.0;
constexpr double kTipEnterDeg = 12.0; // start reversing past this forward lean
constexpr double kTipExitDeg = 6.0;   // keep reversing until the lean falls below this
constexpr int kAntiTipPower = -80;    // arcade backward. Negative throttle is reverse.

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

// True while the robot is leaned far enough forward that the drivetrain should drive backward. Stays true until the lean drops below kTipExitDeg
static bool correcting_forward_tip()
{
    static bool active = false;
    const double pitch = imu.get_pitch();
    if (std::isinf(pitch))
        return active;

    const double forward_lean = kForwardPitchSign * pitch;
    if (forward_lean >= kTipEnterDeg)
        active = true;
    else if (forward_lean <= kTipExitDeg)
        active = false;
    return active;
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

    subsystems::elevator_init();
    subsystems::pivot_init();
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
    pros::Task pivot_task(subsystems::pivot_loop, "Pivot");

    // --- Main thread: drive + camera + dashboard ---
    static float slewedThrottle = 0;
    while (true)
    {
        // LemLib Arcade Drive (split arcade: left stick Y for throttle, right stick X for turn)
        int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        if (correcting_forward_tip())
        {
            // Straight reverse, drive curve off, so the save is not slewed or turned.
            slewedThrottle = kAntiTipPower;
            chassis.arcade(kAntiTipPower, 0, true);
        }
        else
        {
            slewedThrottle = lemlib::slew(throttle, slewedThrottle, 40);
            chassis.arcade(slewedThrottle, turn);
        }

        crimson_cam.update();
        update_dashboard();

        // Update elevator control
        subsystems::elevator_update();

        pros::delay(kLoopDelayMs);
    }
}
