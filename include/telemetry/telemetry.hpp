#pragma once

#include "lemlib/pose.hpp"
#include "telemetry/structs.hpp"
#include <source_location>

namespace telemetry {

void start();

void pose_reset(const lemlib::Pose& from, const lemlib::Pose& to, const std::source_location& where);
void motion_started(const structs::Motion& motion, const std::source_location& where);
void auton_started(const char* name);
void auton_finished();

}
