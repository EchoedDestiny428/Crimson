#pragma once

namespace subsystems::elevator::claw {

void init();
void update();
void stop();
void spin(int power);
void open();
void close();
void toggle();
bool is_open();
void set_pos(double motor_deg);

}
