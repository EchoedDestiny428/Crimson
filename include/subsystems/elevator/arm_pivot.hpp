#pragma once

#include "subsystems/elevator/constants.hpp"
#include <cstdint>

namespace subsystems::elevator::arm_pivot {

void init();
void update();
void stop();
void set_pos(double motor_deg);
void move_to(double motor_deg);
void set_power(int power);
void set_down_slow_zone(bool enabled);
bool is_settled();
bool wait_until_settled(std::uint32_t timeout_ms);
double get_pos();
double target();

}
