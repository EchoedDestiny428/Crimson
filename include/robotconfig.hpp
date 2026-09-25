#pragma once

#include "api.h"
#include "display/dashboard.hpp"
#include "lemlib/api.hpp"
#include "vision/crimson.hpp"

extern pros::Controller controller;
extern pros::Imu imu;
extern crimson::Crimson crimson_cam;
extern display::Dashboard dashboard;

extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::MotorGroup elevator_motors;
extern pros::Motor intake_motor;
extern pros::Motor claw_motor;
extern pros::Motor pivot_motor;

extern pros::adi::Pneumatics intake_lift;

extern lemlib::Chassis chassis;
