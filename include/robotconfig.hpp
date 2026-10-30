#pragma once

#include "api.h"
#include "display/dashboard.hpp"
#include "lemlib/api.hpp"
#include "motion/chassis.hpp"

extern pros::Controller controller;
extern pros::Imu imu;
extern display::Dashboard dashboard;

extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::Rotation vertical_odom;
extern pros::Rotation horizontal_odom;

extern crimson::Chassis chassis;
