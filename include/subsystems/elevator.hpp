#pragma once

#include <cstdint>

namespace subsystems::elevator {

void init();
void update();
void stop();
void set_target(double height);
void raise_by(double delta);
double target();
bool wait_until_settled(std::uint32_t timeout_ms);

}
