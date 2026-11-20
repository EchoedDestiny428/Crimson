#pragma once

#include "subsystems/elevator/constants.hpp"

namespace subsystems::elevator::arm_pivot {

void init();
void stop();
// Brake mode: coast while disabled, hold in auton and driver control. Turning hold on also
// retargets the PID to where the arm is now, so a coasted arm isn't yanked back on enable.
void set_hold(bool hold);
void move_to(double motor_deg);
// PID to motor_deg (the home angle), then push down at power 20 into the hardstop and hold
// once the arm stops moving. Any other arm command cancels it.
void home(double motor_deg);
void set_power(int power); // open loop, -127..127
void set_slow_zone(bool enabled); // cap the last 30 deg of PID moves at 35%
double get_pos();
double target(); // motor deg

}
