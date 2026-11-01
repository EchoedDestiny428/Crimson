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

// Open and wait for a piece (Primed), close on it (Grabbed), or let go (Empty).
void prime();
void grab();
void release();
State state();

}
