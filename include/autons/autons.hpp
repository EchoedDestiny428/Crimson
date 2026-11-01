#pragma once

namespace autons {

namespace match::test {

void run();

}

namespace match::mech_test {

void draw();

}

void select();
void draw_selector();
void run_selected();
const char* selected_name();

}
