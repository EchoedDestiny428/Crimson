#pragma once

#include "autons/autons.hpp"
#include "robotconfig.hpp"
#include "telemetry/logger.hpp" // telemetry::route() and telemetry::mark(), which PointWeaver's generated code calls
#include "vision/localizer.hpp"

namespace autons {

// Where the robot stops with its back (the camera) toward a goal within about 2 ft:
//     reset_odometry();
// waits for it to settle, then moves odometry to the camera's position if they disagree by 0.25 to 6 in.
using vision::localizer::reset_odometry;

}
