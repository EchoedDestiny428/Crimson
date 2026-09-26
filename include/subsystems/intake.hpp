#pragma once

namespace subsystems::intake {

void update();
void stop();
void spin(int power);
void lift();
void drop();
void set_pos(double motor_deg);

}
