#include "robotconfig.hpp"


//I/O
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor intake(1, pros::MotorGears::blue);
crimson::Crimson crimson_cam(1);
pros::Imu imu(2);
display::Dashboard dashboard;

//Motor groups
pros::MotorGroup left_motors({1, 2, 20}, pros::MotorGears::blue);
pros::MotorGroup right_motors({4, 5, 19}, pros::MotorGears::blue);
pros::MotorGroup elevator({17, -18}, pros::MotorGears::blue); // Change to the elevator ports once we wire them up


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



