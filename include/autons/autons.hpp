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

}

void select();
void draw_selector();
void run_selected();

}
