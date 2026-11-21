#pragma once

namespace autons {

namespace skills {

void setup();
void run();

}

// latency-testing branch only: camera latency and drivetrain limits in one logged run (src/autons/latency_test.cpp).
namespace latency_test {

void setup();
void run();
int phase(); // what the test is doing, for the log; 0 when it isn't running
// Sends the newest test recording on the SD card down the terminal, framed for tools/pull-logs.ps1.
void dump_latest();
// True while the test's results are on the brain screen (until it is touched); the screen task leaves it alone.
bool keep_results_on_screen();

}

void select();
void draw_selector();
void run_selected();

}
