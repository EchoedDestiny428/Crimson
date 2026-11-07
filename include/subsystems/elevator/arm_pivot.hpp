#pragma once

#include "subsystems/elevator/constants.hpp"
#include <cstdint>

namespace subsystems::elevator::arm_pivot {

void init();
void update();
void stop();
void set_pos(double motor_deg);
void move_to(double motor_deg);
void set_power(int power); // open loop, -127..127
void set_slow_zone(bool enabled); // cap the last 30 deg of PID moves at 35%
bool is_settled();
bool wait_until_settled(std::uint32_t timeout_ms);
double get_pos();
double target(); // motor deg

}
