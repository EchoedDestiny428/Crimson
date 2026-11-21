#pragma once

#include "vision/tag_solver.hpp"
#include <cstdint>
#include <optional>

namespace vision::localizer {

enum class Status { Calibrating, NoImu, NoSensor, Searching, Ambiguous, Locked };

// What happened to the last picture read.
enum class Outcome {
    NoTags,       // nothing known in view
    NoFit,        // tags in view, but no pose explains them well enough
    Accepted,     // moved the estimate
    Ambiguous,    // two or more poses fit about as well, nothing changed
    Rejected,     // far from the estimate; five in a row in one place jump the estimate there
    Stale,        // same corners as last time although odometry moved: an old picture, skipped
    Turning,      // turning too fast for a sharp, well-timed picture, skipped
    NoHistory,    // taken before the last setPose, skipped
};

// Estimated robot position in inches and heading in degrees, in the frame described in field_constants.hpp.
struct FieldPose {
    double x_in;
    double y_in;
    double heading_deg;
};

// Largest tag in view, unsolved, for calibrating against a tape measure.
struct RawTag {
    int id;
    double u;
    double v;
    double width_px;
    double height_px;
};

struct Snapshot {
    FieldPose pose{};
    Status status = Status::Calibrating;
    int tags = 0;
    int goals = 0;
    int candidates = 0;
    double error_px = 0.0;
    double range_in = 0.0;
    std::optional<RawTag> raw;
    Outcome outcome = Outcome::NoTags;
    double correction_in = 0.0; // how far the last accepted picture moved the estimate
    std::uint32_t read_ms = 0;  // latency-testing: pros::millis() when these camera fields were read
};

void start();
Snapshot snapshot();

}
