#pragma once

#include "display/dashboard.hpp"
#include "vision/crimson.hpp"
#include "lemlib/api.hpp"
#include "main.h"
#include "api.h"

// --------------------------------------------------
// I/O Devices
// --------------------------------------------------
extern pros::Controller controller;
extern pros::Motor intake;
extern crimson::Crimson crimson_cam;
extern pros::Imu imu;
extern display::Dashboard dashboard;

// --------------------------------------------------
// Motor Groups
// --------------------------------------------------
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::MotorGroup elevator;
extern pros::Motor claw;
extern pros::Motor pivot;

// --------------------------------------------------
// LemLib Configuration
// --------------------------------------------------
extern lemlib::Drivetrain drivetrain;
extern lemlib::ControllerSettings linearController;
extern lemlib::ControllerSettings angularController;
extern lemlib::OdomSensors sensors;
extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;
extern lemlib::Chassis chassis;
