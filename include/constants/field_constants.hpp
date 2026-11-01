#pragma once

#include <array>
#include <cmath>

namespace field {

// Origin is the centre goal, in millimetres. +Y is the red alliance wall and -Y is blue. Facing red from the blue wall,
// +X is on your right. Heading is a compass angle: 0 faces red and 90 faces +X. The goal layout has no mirror symmetry,
// so X cannot be flipped alone. Appendix A draws red on the left, so its coordinates convert with x' = y, y' = -x.

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
    return {{x_pitches * kGoalPitchMm, y_pitches * kGoalPitchMm}, type, tag_id, tag_height_mm};
}

inline constexpr std::array kGoals{
    goal_at(0, 0, GoalType::Neutral, 0, kCentreTagHeightMm),
    goal_at(1, 2, GoalType::Neutral, 1, kNeutralTagHeightMm),
    goal_at(-1, -2, GoalType::Neutral, 1, kNeutralTagHeightMm),
    goal_at(2, 1, GoalType::Neutral, 4, kNeutralTagHeightMm),
    goal_at(-2, -1, GoalType::Neutral, 4, kNeutralTagHeightMm),
    goal_at(-1, 2, GoalType::Red, 2, kAllianceTagHeightMm),
    goal_at(-2, 1, GoalType::Red, 3, kAllianceTagHeightMm),
    goal_at(1, -2, GoalType::Blue, 2, kAllianceTagHeightMm),
    goal_at(2, -1, GoalType::Blue, 3, kAllianceTagHeightMm),
};

inline bool in_field(const Point2D& point, double margin_mm = 0.0) {
    const double limit = kFieldSizeMm / 2.0 + margin_mm;
    return std::abs(point.x) <= limit && std::abs(point.y) <= limit;
}

}
