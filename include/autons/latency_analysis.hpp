#pragma once

// latency-testing branch: the latency test's maths, run on the brain after the test (same method as
// tools/latency_test.py). Plain C++ with no PROS calls, so it can also be compiled and checked on a laptop.

#include <cstdint>
#include <vector>

namespace autons::latency_analysis {

// One 10 ms row of the test recording.
struct Sample {
    std::uint32_t ms;
    double x_in;
    double y_in;
    double heading_deg;
    int tags;               // tags in view at the last camera read
    std::uint32_t read_ms;  // when the camera fields below were read
    double u_px;            // raw centre of the largest tag (NaN if none)
    double v_px;
    int phase;              // latency_test phase
    double left_rpm;        // drive motor speeds
    double right_rpm;
};

struct Latency {
    bool valid = false;
    int ms = 0;
    double sharpness = 0.0; // how much worse the fit is 25 ms either side; under ~1.1 means the curve is flat
    int reads = 0;
};

struct Results {
    bool completed = false; // reached the "done" phase
    bool aborted = false;
    int last_phase = 0;

    Latency turning;
    Latency driving;
    bool latency_valid = false;
    int latency_ms = 0;     // what to put in kCameraLatencyMs

    bool drive_valid = false;
    double top_forward = 0, top_reverse = 0;        // in/s
    double accel_forward = 0, accel_reverse = 0;    // in/s^2, peak over ~50 ms
    double brake_forward = 0, brake_reverse = 0;    // in/s^2
    double ms_to_90 = 0;                            // to 90% of top speed
    double motors_forward = 0;                      // in/s the drive motors claim at full speed

    bool turn_valid = false;
    double turn_right = 0, turn_left = 0;               // deg/s
    double turn_accel_right = 0, turn_accel_left = 0;   // deg/s^2
    double turn_brake_right = 0, turn_brake_left = 0;   // deg/s^2
    double turn_ms_to_90 = 0;
};

Results analyze(const std::vector<Sample>& samples);

}
