#pragma once

#include <vector>

namespace field {

struct Point2D {
    double x;
    double y;
};

struct GoalInfo {
    Point2D pos;
    double height;
};

inline constexpr double kFieldLength = 3566.4;
inline constexpr double kFieldWidth = 3566.4;

inline constexpr double kTallGoalHeight = 222.7;
inline constexpr double kShortNeutralGoalHeight = 146.5;
inline constexpr double kAllianceGoalHeight = 82.5;

inline constexpr Point2D kCenterGoal{kFieldLength / 2.0, kFieldWidth / 2.0};

inline constexpr Point2D kNeutralGoalTopLeft{1185.1, 2381.3};
inline constexpr Point2D kNeutralGoalBottomLeft{1185.1, 1185.1};
inline constexpr Point2D kNeutralGoalTopRight{2381.3, 2381.3};
inline constexpr Point2D kNeutralGoalBottomRight{2381.3, 1185.1};

inline constexpr Point2D kRedGoalTop{587.1, 2979.3};
inline constexpr Point2D kRedGoalBottom{587.1, 587.1};
inline constexpr Point2D kBlueGoalTop{2979.3, 2979.3};
inline constexpr Point2D kBlueGoalBottom{2979.3, 587.1};

inline constexpr bool in_field(const Point2D& point) {
    return point.x >= 0.0 && point.x <= kFieldLength && point.y >= 0.0 && point.y <= kFieldWidth;
}

inline std::vector<GoalInfo> get_goal_locations_for_tag(int tag_id) {
    switch (tag_id) {
    case 0:
        return {{kCenterGoal, kTallGoalHeight}};
    case 1:
        return {{kRedGoalTop, kAllianceGoalHeight}, {kRedGoalBottom, kAllianceGoalHeight}};
    case 2:
        return {{kBlueGoalTop, kAllianceGoalHeight}, {kBlueGoalBottom, kAllianceGoalHeight}};
    case 3:
        return {{kNeutralGoalTopLeft, kShortNeutralGoalHeight}, {kNeutralGoalBottomRight, kShortNeutralGoalHeight}};
    case 4:
        return {{kNeutralGoalTopRight, kShortNeutralGoalHeight}, {kNeutralGoalBottomLeft, kShortNeutralGoalHeight}};
    default:
        return {};
    }
}

}
