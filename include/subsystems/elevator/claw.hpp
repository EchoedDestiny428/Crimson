#pragma once

#include <cstdint>

namespace subsystems::elevator::claw {

void update();
void open();
void close();
void toggle();
bool is_open();
std::int32_t distance_mm();

}
