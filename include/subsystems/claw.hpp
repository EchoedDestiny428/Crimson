#pragma once

namespace subsystems::claw {

void init();
void update();
void spin(int power);
void stop();
void automatic();
void set_pos(double motor_deg);

}
