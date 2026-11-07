#pragma once

#include "subsystems/elevator/constants.hpp"

namespace subsystems::elevator::arm_pivot {

void init();
void update();
void stop();
void move_to(double motor_deg);
void set_power(int power); // open loop, -127..127
void set_slow_zone(bool enabled); // cap the last 30 deg of PID moves at 35%
double get_pos();
double target(); // motor deg

}
