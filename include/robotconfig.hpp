#pragma once

#include "api.h"
#include "display/dashboard.hpp"
#include "lemlib/api.hpp"
#include "motion/chassis.hpp"
#include "telemetry/logger.hpp"
#include "vision/crimson.hpp"

extern pros::Controller controller;
extern pros::Imu imu;
extern crimson::Crimson crimson_cam;
extern display::Dashboard dashboard;

extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::MotorGroup extension_motors;
extern pros::Motor pivot_motor;
extern pros::Motor claw_motor;
extern telemetry::Logger logger;
extern pros::Rotation vertical_odom;
extern pros::Rotation horizontal_odom;

extern pros::adi::Pneumatics claw_piston;

extern crimson::Chassis chassis;
