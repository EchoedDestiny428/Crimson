#pragma once

#include "vision/tag_solver.hpp"
#include <optional>

namespace vision::localizer {

enum class Status { Calibrating, NoImu, NoSensor, Searching, Ambiguous, Locked };

// Largest tag in view, unsolved, for calibrating against a tape measure.
struct RawTag {
    int id;
    double u;
    double v;
    double width_px;
    double height_px;
};

struct Snapshot {
    Pose pose{};
    Status status = Status::Calibrating;
    int tags = 0;
    int goals = 0;
    int candidates = 0;
    double error_px = 0.0;
    double range_mm = 0.0;
    std::optional<RawTag> raw;
};

void start();
Snapshot snapshot();

}
