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
// once the arm stops moving. Any other arm command cancels it. The last 40% of the swing home
// runs at 40% speed.
void home(double motor_deg);
bool homing(); // a home() move is still on its way to the hardstop
void set_power(int power); // open loop, -127..127
void set_slow_zone(bool enabled); // cap the last 30 deg of PID moves at 35%
double get_pos();
double target(); // motor deg
bool is_settled(); // settled on its target, and not mid-home (autons)
// How hard swings speed up and slow down right now (deg/s^2): 3000 with the lift down, gentler
// the higher the lift (about 1100 at L7). See arm_pivot.cpp for the measurements behind it.
double swing_accel();
// Runtime tuning (the arm-tuning branch's tuner uses these; every value starts at its constant in arm_pivot.cpp):
// - set_swing_tuning: fade_deg, how far ahead of its setpoint the arm gets before the speed push is gone (0 = never
//   fades); brake, power per deg/s the arm is faster than its setpoint while ahead (0 = off). 5 / 0.3 to start.
// - set_swing_finish: swings out (move_to) slow to speed_fraction of full swing speed for their last `fraction` of
//   the move, like the swing home does (0 = off). 0.2 at 0.4 to start. Applies from the next move.
// - set_hold_power: the minimum push (out of 127) toward the target once a move has arrived and is more than 1 deg
//   off; it is what pulls the arm back after an overshoot. 45 to start (30 could barely lift a cup-pin back at L3).
void set_swing_tuning(double fade_deg, double brake);
double swing_fade();
double swing_brake();
void set_swing_finish(double fraction, double speed_fraction);
double swing_finish_fraction();
double swing_finish_speed_fraction();
void set_hold_power(double power);
double hold_power();

}
