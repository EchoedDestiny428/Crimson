#pragma once

namespace autons {

// The 15 s match auton "default" from PointWeaver (its own drawn RedSnacky). "default" is a C++ keyword, hence the
// namespace name.
namespace match::default_match {

void setup();
void run();

}

namespace skills {

void setup();
void run();

}

void select();
void draw_selector();
void run_selected();

}
