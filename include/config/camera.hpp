#pragma once

namespace config {

// Lens height above the field tiles. With one tag in view, range comes mostly from this number, so measure it on the
// rig. Stay well clear of the tag heights (37.8, 101.8 and 178.0 mm): level with a tag, range becomes unusable.
// Measured 264 mm to the middle of the lens glass; fitting the centre and red goal readings puts the optical centre
// about 19 mm higher than that.
inline constexpr double kCameraHeightMm = 283.4;

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
inline constexpr double kCameraPitchDeg = 0.0;

// Where the rig sits at boot. Heading must be right: the IMU only knows how far it has turned since then. Position
// picks between the two goals that share a tag ID until a view settles it.
inline constexpr double kStartXIn = 96.0;
inline constexpr double kStartYIn = 78.0;
inline constexpr double kStartHeadingDeg = -180.0;

}
