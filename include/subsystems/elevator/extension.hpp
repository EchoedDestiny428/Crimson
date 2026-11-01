#pragma once

#include "subsystems/elevator/constants.hpp"
#include <cstdint>

namespace subsystems::elevator::extension {

void init();
void update();
void stop();
void hold();
void set_pos(double height);
void set_target(double height);
void set_manual(int power);
double height();
double target();
bool is_settled();
bool wait_until_settled(std::uint32_t timeout_ms);

}
