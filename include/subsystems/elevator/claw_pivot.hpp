#pragma once

#include "subsystems/elevator/constants.hpp"
#include <cstdint>

namespace subsystems::elevator::claw_pivot {

void init();
void update();
void stop();
void set_pos(double motor_deg);
void move_to(double motor_deg);
bool is_settled();
bool wait_until_settled(std::uint32_t timeout_ms);
double get_pos();

}
