#pragma once

namespace config {

// Lens height above the field tiles. With one tag in view, range comes mostly from this number. Stay well clear of the
// tag heights (37.8, 101.8 and 178.0 mm): level with a tag, range becomes unusable.
// Measured 282.5 mm to the middle of the lens glass on the robot mount. On the old level rig, the same measurement was
// 264 mm, but fitting the centre and red goal readings put the effective height about 19 mm higher (283.4 mm). If
// tuning kCameraPitchDeg can't get both a near and a far taped distance right, try adding that 19 mm here.
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

// How far the lens points below level, in degrees. 0 is level. Set it to the angle measured on the mount, then fine
// tune it (not kCenterYPx) until the range to a tag at a taped-out distance reads right. On a pitched camera the
// horizon sits fy * tan(pitch) rows above kCenterYPx.
inline constexpr double kCameraPitchDeg = 35.0; // approximate, measured on the robot mount

// Where the lens sits relative to the robot's turning centre, and which way it faces (clockwise from the robot's
// front; 180 looks out the back). The solver fits the lens, then steps back to the turning centre with these.
inline constexpr double kCameraForwardIn = -5.5; // ahead of the turning centre; negative is behind
inline constexpr double kCameraRightIn = 0.0;
inline constexpr double kCameraYawDeg = 180.0; // the camera is behind the centre, looking out the back

// Where the robot's turning centre sits at boot, and its heading, in the field frame (field_constants.hpp: (0, 0) is
// the corner on blue's right, so the field centre is (72, 72), and heading 0 faces red). Heading must be right: the IMU
// only knows how far it has turned since then. Position picks between the two goals that share a tag ID until a view
// settles it, and every such pair mirrors through the field centre, so a centre start can't tell them apart.
inline constexpr double kStartXIn = 72.0;
inline constexpr double kStartYIn = 72.0;
inline constexpr double kStartHeadingDeg = 0.0;

}
