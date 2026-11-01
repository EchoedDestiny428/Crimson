#pragma once

#include "vision/tag_solver.hpp"
#include <optional>

namespace vision::localizer {

enum class Status { Calibrating, NoImu, NoSensor, Searching, Ambiguous, Locked };

// Estimated camera position in inches and heading in degrees, in the frame described in field_constants.hpp.
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
};

void start();
Snapshot snapshot();

}
