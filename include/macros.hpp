#pragma once

#include "subsystems/elevator.hpp"
#include "subsystems/pivot.hpp"

namespace macros {

void flip_out(double height = subsystems::elevator::kFlipOut,
              double pivot_motor_deg = subsystems::pivot::kFlippedMotorDeg);
void go_to(double height);
void home();
void dual_setup();
void dual_pickup();
void cancel();

void press(bool up, bool down, bool down_pressed);
void update();
void stop();

}
