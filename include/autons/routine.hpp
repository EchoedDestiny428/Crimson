#pragma once

#include "autons/autons.hpp"
#include "robotconfig.hpp"
#include "subsystems/elevator/claw.hpp"
#include "subsystems/elevator/macros.hpp"
#include "telemetry/logger.hpp" // telemetry::route() and telemetry::mark(), which PointWeaver's generated code calls
#include "vision/localizer.hpp"

namespace autons {

// The elevator's auton calls, short, for every auton (PointWeaver's generated code uses them as written):
//     claw::grab();                               // hold the preload (the claw boots open)
//     macros::home_async();                       // arm onto its hardstop (re-zeroes it)
//     macros::aim_lift(macros::stage(5, 0));      // L5 on an alliance goal: only the lift moves while driving
//     macros::swing_arm();                        // at the goal: swing, wait for lift and arm to settle
//     macros::drop();                             // open, wait for the piece to fall
//     macros::pickup();                           // close on a piece, wait for the grip
namespace macros = subsystems::elevator::macros;
namespace claw = subsystems::elevator::claw;

// Where the robot stops with its back (the camera) toward a goal within about 2 ft:
//     reset_odometry();
// waits for it to settle, then moves odometry to the camera's position if they disagree by 0.25 to 6 in.
using vision::localizer::reset_odometry;

}
