#pragma once

#include <array>

namespace field {

// Same frame as FRC's blue origin. (0, 0) is the corner on blue's right as they face red. +X runs from the blue wall to
// the red wall, and +Y runs to blue's left. Heading 0 faces red as in FRC, but turns are clockwise to match LemLib, so
// heading 90 faces -Y. Distances here are millimetres. The goal layout has no mirror symmetry, so no axis can be
// flipped alone. Appendix A draws red on the left, so its corner coordinates convert with x' = size - x, y' = size - y.

struct Point2D {
    double x;
    double y;
};

enum class GoalType { Neutral, Red, Blue };

struct Goal {
    Point2D pos;
    GoalType type;
    int tag_id;
    double tag_height_mm;
};

inline constexpr double kMmPerInch = 25.4;
inline constexpr double kFieldSizeMm = 3566.4;
inline constexpr double kFieldSizeIn = kFieldSizeMm / kMmPerInch;
inline constexpr double kGoalPitchMm = 598.05;

// Height of the tag centre above the tiles.
inline constexpr double kCentreTagHeightMm = 178.0;
inline constexpr double kNeutralTagHeightMm = 101.8;
inline constexpr double kAllianceTagHeightMm = 37.8;

// Every goal carries the same tag on four plates, one facing each wall.
inline constexpr std::array kTagFacesDeg{0.0, 90.0, 180.0, 270.0};
inline constexpr double kTagPlateOffsetMm = 49.0;
// Side of the 5-cell inner square of the tag, which is the outline the sensor is expected to report.
inline constexpr double kTagSizeMm = 18.0;

inline constexpr Goal goal_at(double x_pitches, double y_pitches, GoalType type, int tag_id, double tag_height_mm) {
    const double centre = kFieldSizeMm / 2.0;
    return {{centre + x_pitches * kGoalPitchMm, centre + y_pitches * kGoalPitchMm}, type, tag_id, tag_height_mm};
}

inline constexpr std::array kGoals{
    goal_at(0, 0, GoalType::Neutral, 0, kCentreTagHeightMm),
    goal_at(2, -1, GoalType::Neutral, 1, kNeutralTagHeightMm),
    goal_at(-2, 1, GoalType::Neutral, 1, kNeutralTagHeightMm),
    goal_at(1, -2, GoalType::Neutral, 4, kNeutralTagHeightMm),
    goal_at(-1, 2, GoalType::Neutral, 4, kNeutralTagHeightMm),
    goal_at(2, 1, GoalType::Red, 2, kAllianceTagHeightMm),
    goal_at(1, 2, GoalType::Red, 3, kAllianceTagHeightMm),
    goal_at(-2, -1, GoalType::Blue, 2, kAllianceTagHeightMm),
    goal_at(-1, -2, GoalType::Blue, 3, kAllianceTagHeightMm),
};

inline constexpr bool in_field(const Point2D& point, double margin_mm = 0.0) {
    const double limit = kFieldSizeMm + margin_mm;
    return point.x >= -margin_mm && point.x <= limit && point.y >= -margin_mm && point.y <= limit;
}

}
