#pragma once

#include "subsystems/elevator/constants.hpp"

namespace subsystems::elevator::extension {

void init();
void hold();
void set_target(double height);
void set_manual(int power);
double height();
double target();

}
