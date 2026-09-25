#pragma once

#include "subsystems/elevator.hpp"

namespace macros {

void flip_out(double height = subsystems::elevator::kFlipOut);
void go_to(double height);
void home();
void dual_setup();
void dual_pickup();
void cancel();

void update();
void stop();

}
