#pragma once

#include <cstdint>

namespace subsystems::elevator::claw {

enum class State { Empty, Primed, Grabbed };

void update();
void open();
void close();
void toggle();
bool is_open();
std::int32_t distance_mm();

void prime();
void grab();
void release();
State state();

}
