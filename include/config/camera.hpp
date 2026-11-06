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
// Image row of the horizon. Any tilt in the mount shows up here: 1 degree moves it by about 3.8 px.
inline constexpr double kCenterYPx = 115.3;

}
