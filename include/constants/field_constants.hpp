#pragma once

#include <array>
#include <vector>

namespace field {

struct Point2D {
    double x;
    double y;
};

enum class GoalType { Neutral, Red, Blue };

struct Goal {
    Point2D pos;
    GoalType type;
    int tag;
};

struct GoalInfo {
    Point2D pos;
    double height;
};

inline constexpr double kFieldLength = 3566.4;
inline constexpr double kFieldWidth = 3566.4;
inline constexpr double kMmPerInch = 25.4;

inline constexpr double kCenterGoalTagHeight = 222.7;
inline constexpr double kNeutralGoalTagHeight = 146.5;
inline constexpr double kAllianceGoalTagHeight = 82.5;

inline constexpr Point2D from_center_mm(double x, double y) {
    return {x + kFieldLength / 2.0, y + kFieldWidth / 2.0};
}

inline constexpr std::array kGoals{
    Goal{from_center_mm(0, 0), GoalType::Neutral, 0},       Goal{from_center_mm(-1200, 600), GoalType::Neutral, 1},
    Goal{from_center_mm(1200, -600), GoalType::Neutral, 1}, Goal{from_center_mm(-600, 1200), GoalType::Neutral, 4},
    Goal{from_center_mm(600, -1200), GoalType::Neutral, 4}, Goal{from_center_mm(-1200, -600), GoalType::Red, 2},
    Goal{from_center_mm(-600, -1200), GoalType::Red, 3},    Goal{from_center_mm(1200, 600), GoalType::Blue, 2},
    Goal{from_center_mm(600, 1200), GoalType::Blue, 3},
};

inline constexpr bool in_field(const Point2D& point) {
    return point.x >= 0.0 && point.x <= kFieldLength && point.y >= 0.0 && point.y <= kFieldWidth;
}

inline constexpr Point2D to_lemlib_inches(const Point2D& field_mm) {
    return {(field_mm.x - kFieldLength / 2.0) / kMmPerInch, (field_mm.y - kFieldWidth / 2.0) / kMmPerInch};
}

inline constexpr double tag_height(const Goal& goal) {
    if (goal.type != GoalType::Neutral) {
        return kAllianceGoalTagHeight;
    }
    return goal.tag == 0 ? kCenterGoalTagHeight : kNeutralGoalTagHeight;
}

inline std::vector<GoalInfo> get_goal_locations_for_tag(int tag_id) {
    std::vector<GoalInfo> goals;
    for (const Goal& goal : kGoals) {
        if (goal.tag == tag_id) {
            goals.push_back({goal.pos, tag_height(goal)});
        }
    }
    return goals;
}

}
