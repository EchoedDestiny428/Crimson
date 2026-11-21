#pragma once

#include <array>

namespace field {

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
inline constexpr double kFieldSizeIn = 144.0;
inline constexpr double kFieldSizeMm = kFieldSizeIn * kMmPerInch;

inline constexpr double kCentreTagHeightMm = 178.0;
inline constexpr double kNeutralTagHeightMm = 101.8;
inline constexpr double kAllianceTagHeightMm = 37.8;

inline constexpr std::array kTagFacesDeg{0.0, 90.0, 180.0, 270.0};
inline constexpr double kTagPlateOffsetMm = 49.0;
inline constexpr double kTagSizeMm = 18.0; // unsure

inline constexpr Goal goal_at(double x_in, double y_in, GoalType type, int tag_id, double tag_height_mm) {
    return {{x_in * kMmPerInch, y_in * kMmPerInch}, type, tag_id, tag_height_mm};
}

inline constexpr std::array kGoals{
    goal_at(72, 72, GoalType::Neutral, 0, kCentreTagHeightMm),
    goal_at(120, 48, GoalType::Neutral, 1, kNeutralTagHeightMm),
    goal_at(24, 96, GoalType::Neutral, 1, kNeutralTagHeightMm),
    goal_at(96, 24, GoalType::Neutral, 4, kNeutralTagHeightMm),
    goal_at(48, 120, GoalType::Neutral, 4, kNeutralTagHeightMm),
    goal_at(120, 96, GoalType::Red, 2, kAllianceTagHeightMm),
    goal_at(96, 120, GoalType::Red, 3, kAllianceTagHeightMm),
    goal_at(24, 48, GoalType::Blue, 2, kAllianceTagHeightMm),
    goal_at(48, 24, GoalType::Blue, 3, kAllianceTagHeightMm),
};

inline constexpr bool in_field(const Point2D& point, double margin_mm = 0.0) {
    const double limit = kFieldSizeMm + margin_mm;
    return point.x >= -margin_mm && point.x <= limit && point.y >= -margin_mm && point.y <= limit;
}

}
