#pragma once

namespace config {

// Lens height above the field tiles. With one tag in view, range comes mostly from this number, so measure it on the
// rig. Stay well clear of the tag heights (37.8, 101.8 and 178.0 mm): level with a tag, range becomes unusable.
inline constexpr double kCameraHeightMm = 250.0;

// Starting values from github.com/Deen-Weible/pros_pose. Two hardware revisions of the sensor exist with different
// focal lengths, so check these against a tag at a taped distance.
inline constexpr double kFocalXPx = 212.3;
inline constexpr double kFocalYPx = 195.82;
inline constexpr double kCenterXPx = 160.0;
// Image row of the horizon. Any tilt in the mount shows up here: 1 degree moves it by about 3.4 px.
inline constexpr double kCenterYPx = 120.0;

// Where the rig sits at boot. Heading must be right: the IMU only knows how far it has turned since then. Position
// picks between the two goals that share a tag ID until a view settles it.
inline constexpr double kStartXIn = 48.0;
inline constexpr double kStartYIn = 96.0;
inline constexpr double kStartHeadingDeg = 0.0;

}
