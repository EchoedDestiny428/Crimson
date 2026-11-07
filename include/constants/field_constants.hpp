#pragma once

#include <array>
#include <cstddef>

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

inline constexpr std::size_t kGoalCount = kGoals.size();

enum class GoalId : std::size_t {
    Center,
    Neutral1Far,
    Neutral1Near,
    Neutral4Far,
    Neutral4Near,
    Red2,
    Red3,
    Blue2,
    Blue3,
};

inline constexpr const Goal& goal(GoalId id) {
    return kGoals[static_cast<std::size_t>(id)];
}

inline constexpr bool goal_is(GoalId id, double x_in, double y_in, GoalType type, int tag_id) {
    const Goal& g = goal(id);
    return g.pos.x == x_in * kMmPerInch && g.pos.y == y_in * kMmPerInch && g.type == type && g.tag_id == tag_id;
}

static_assert(kGoalCount == 9);
static_assert(goal_is(GoalId::Center, 72, 72, GoalType::Neutral, 0));
static_assert(goal_is(GoalId::Neutral1Far, 120, 48, GoalType::Neutral, 1));
static_assert(goal_is(GoalId::Neutral1Near, 24, 96, GoalType::Neutral, 1));
static_assert(goal_is(GoalId::Neutral4Far, 96, 24, GoalType::Neutral, 4));
static_assert(goal_is(GoalId::Neutral4Near, 48, 120, GoalType::Neutral, 4));
static_assert(goal_is(GoalId::Red2, 120, 96, GoalType::Red, 2));
static_assert(goal_is(GoalId::Red3, 96, 120, GoalType::Red, 3));
static_assert(goal_is(GoalId::Blue2, 24, 48, GoalType::Blue, 2));
static_assert(goal_is(GoalId::Blue3, 48, 24, GoalType::Blue, 3));

enum class Color { Yellow, Red, Blue };

inline constexpr int kNeutralStartingYellowPins = 1;

inline constexpr int starting_yellow_pins(const Goal& goal) {
    return goal.type == GoalType::Neutral ? kNeutralStartingYellowPins : 0;
}

enum class Wall : std::size_t { BlueWall, RedWall, LeftWall, RightWall };

struct ToggleBar {
    Point2D center_in;
    double ram_heading_deg;
};

inline constexpr std::array kToggleBars{
    ToggleBar{{0.0, kFieldSizeIn / 2.0}, 180.0},
    ToggleBar{{kFieldSizeIn, kFieldSizeIn / 2.0}, 0.0},
    ToggleBar{{kFieldSizeIn / 2.0, kFieldSizeIn}, 270.0},
    ToggleBar{{kFieldSizeIn / 2.0, 0.0}, 90.0},
};

inline constexpr const ToggleBar& toggle_bar(Wall wall) {
    return kToggleBars[static_cast<std::size_t>(wall)];
}

inline constexpr bool in_field(const Point2D& point, double margin_mm = 0.0) {
    const double limit = kFieldSizeMm + margin_mm;
    return point.x >= -margin_mm && point.x <= limit && point.y >= -margin_mm && point.y <= limit;
}

}
