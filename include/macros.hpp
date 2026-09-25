#pragma once

#include "subsystems/elevator.hpp"
#include <cstdint>

namespace macros {

void flip_out(double height = subsystems::elevator::kFlipOut);
void go_to(double height);
void home();
bool wait_until_done(std::uint32_t timeout_ms);

void dual_setup();
void dual_pickup();

void update();
void stop();

}
