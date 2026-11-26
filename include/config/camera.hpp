#pragma once

#include <cstdint>

namespace config {

// How old a picture is by the time the code reads it. Each picture is solved with odometry from that moment, then
// moved forward by however far odometry has driven since.
// Measured 2026-11-21 on the robot (latency-testing branch): 254 ms overall; turning slow / medium / fast 227, 270,
// 276 and driving slow / fast 239, 231; an earlier run gave 250+ and 218. About 250 +- 25 ms. Keep it under the
// localizer's odometry history (1.28 s).
inline constexpr std::uint32_t kCameraLatencyMs = 250;

// Feed the camera into odometry (the pose every motion drives on), the way an FRC pose estimator feeds vision in: the
// localizer's filter keeps the best estimate (odometry plus camera fixes, weighted by how sure each is), and odometry
// is slid toward it continuously. x and y only; the IMU keeps heading.
// - At most kCameraFeedInPerS, so a motion in progress sees a slow drift, never a jump.
// - Only while the estimate is within kCameraFeedMaxIn of odometry; further than that is more likely the camera
//   locked onto the wrong (twin) goal than odometry drifting.
// - Gaps under kCameraFeedDeadbandIn are left alone.
// false: the camera only keeps its own estimate (the dashboard and log show it; reset_odometry() can still apply it).
inline constexpr bool kCameraFeedsOdometry = true;
inline constexpr double kCameraFeedInPerS = 3.0;
inline constexpr double kCameraFeedMaxIn = 6.0;
inline constexpr double kCameraFeedDeadbandIn = 0.05;

// Lens height above the field tiles. With one tag in view, range comes mostly from this number. Stay well clear of the
// tag heights (37.8, 101.8 and 178.0 mm): level with a tag, range becomes unusable.
// Measured 282.5 mm to the middle of the lens glass on the robot mount. The 2026-11-21 robot calibration (see
// kCameraPitchDeg) fits this height as measured, so the 19 mm the old level rig needed on top does not apply here.
inline constexpr double kCameraHeightMm = 282.5;

// kFocalYPx, kCenterYPx and the extra camera height are fitted together from the centre tag at 35 to 65 cm and the red
// goal tag at 47 to 66 cm, measured from the lens glass. kCenterXPx is the column the centre tag sits in when the rig is
// square to it at heading 180. kFocalXPx comes from sliding the rig 10 cm either side of the centre tag at 56 cm.
inline constexpr double kFocalXPx = 233.0;
inline constexpr double kFocalYPx = 219.0;
inline constexpr double kCenterXPx = 170.0;
// Image row the lens axis passes through. It was fitted with the camera level, so it also soaks up any small tilt the
// old mount had: 1 degree of tilt moves it by about 3.8 px. Leave it alone when changing kCameraPitchDeg.
inline constexpr double kCenterYPx = 115.3;

// How far the lens points below level, in degrees. 0 is level. Tune this (not kCenterYPx) against taped-out distances.
// On a pitched camera the horizon sits fy * tan(pitch) rows above kCenterYPx.
// Fitted 2026-11-21 on the robot: the turning centre 17, 24 and 31 in from the centre of a neutral goal (tag 4) and an
// alliance goal (tag 3), facing it. All six tag rows agree on 41.2 deg to within 0.9 deg, and the lens-to-tag ranges
// then come out within 0.3 in. The mount was estimated at 35 deg by eye, which read ranges up to 14 in long.
inline constexpr double kCameraPitchDeg = 41.2;

// Where the lens sits relative to the robot's turning centre, and which way it faces (clockwise from the robot's
// front; 180 looks out the back). The solver fits the lens, then steps back to the turning centre with these.
inline constexpr double kCameraForwardIn = -5.5; // ahead of the turning centre; negative is behind
inline constexpr double kCameraRightIn = 0.0;
inline constexpr double kCameraYawDeg = 180.0; // the camera is behind the centre, looking out the back

// Where the robot's turning centre sits at boot, and its heading, in the field frame (field_constants.hpp: (0, 0) is
// the corner on blue's right, so the field centre is (72, 72), and heading 0 faces red). main.cpp puts odometry here
// after calibrating, and the camera estimate starts from odometry. An auton's own setPose moves odometry later.
// Heading must be right: the IMU only knows how far it has turned since then. Position picks between the two goals that share a tag ID until a view
// settles it, and every such pair mirrors through the field centre, so a centre start can't tell them apart.
inline constexpr double kStartXIn = 72.0;
inline constexpr double kStartYIn = 72.0;
inline constexpr double kStartHeadingDeg = 0.0;

}
