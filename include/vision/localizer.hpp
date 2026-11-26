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
    Stale,        // same corners as the last read: the same picture again, skipped (each picture counts once)
    Turning,      // turning around the time the picture was taken (over kStillMaxDegPerS), skipped
    NoHistory,    // taken before the last setPose, skipped
    TooFar,       // every answer was further than config::kCameraMaxRangeIn: too rough to correct with, skipped
    Moving,       // driving around the time the picture was taken (over kStillMaxInPerS), skipped
};

// Estimated robot position in inches and heading in degrees, in the frame described in field_constants.hpp.
struct FieldPose {
    double x_in;
    double y_in;
    double heading_deg;
};

// Largest tag in view, unsolved, for calibrating against a tape measure (and from logs, against odometry).
struct RawTag {
    int id;
    double u;
    double v;
    double width_px;
    double height_px;
    std::array<Pixel, 4> corners; // as order_corners() puts them: top left, top right, bottom right, bottom left
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
    std::uint32_t accepted = 0; // accepted pictures since boot; goes up by one per fix
    std::uint32_t read_ms = 0;  // pros::millis() when the sensor was last read (the picture is older by the latency)
    std::uint32_t reads = 0;    // sensor reads since boot; goes up by one per read, tags or not
};

void start();
Snapshot snapshot();

// ---- Resetting odometry from the camera when the robot stops ----

struct ResetParams {
    // How long the robot has to stand still before the camera is trusted. Pictures are config::kCameraLatencyMs old
    // when they arrive, so waiting a bit longer than that means every picture used was taken after the robot stopped.
    // 0 resets as soon as the camera agrees, trusting the latency correction instead.
    std::uint32_t still_ms = 350;
    // Accepted camera pictures needed after the robot stopped.
    int fixes = 2;
    // Give up after this long and leave odometry alone (no tag in view, still moving, camera unsure).
    std::uint32_t timeout_ms = 1000;
    // A bigger difference is more likely the camera picking the twin goal than odometry drifting: refuse it.
    double max_correction_in = 6.0;
    // Smaller differences are left alone; a reset restarts the camera's estimate, so tiny ones aren't worth it.
    double min_correction_in = 0.25;
};

enum class ResetOutcome {
    Applied,     // odometry moved to the camera's position (heading kept from the IMU)
    Agreed,      // camera and odometry already within min_correction_in
    TooFar,      // camera disagrees by more than max_correction_in; odometry left alone
    NoCamera,    // no locked camera fix after the robot stopped before the timeout
    StillMoving, // the robot never stood still for still_ms before the timeout
};

struct ResetResult {
    ResetOutcome outcome;
    double difference_in; // camera minus odometry distance when it decided (0 if it never got that far)
};

// For autons: call it where the robot stops with its back (the camera) toward a goal within about 2 ft. Waits for the
// current motion to finish, then for the robot to be still and the camera to agree, then moves odometry's x and y
// to the camera's. Blocks for still_ms or so when it works, up to timeout_ms when it can't.
ResetResult reset_odometry(const ResetParams& params = {});

}
