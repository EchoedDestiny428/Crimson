#ifndef FIELD_CONSTANTS_HPP_
#define FIELD_CONSTANTS_HPP_

#include <vector>

namespace field {

// All dimensions are in millimeters (mm)
struct Point2D {
    double x;
    double y;
};

// Field Dimensions
constexpr double kFieldLength = 3566.4; // approx 140.41 inches
constexpr double kFieldWidth = 3566.4;
constexpr Point2D kCenter = { kFieldLength / 2.0, kFieldWidth / 2.0 }; // (1783.2, 1783.2)

// Goal Heights
constexpr double kTallGoalHeight = 222.7; // 8.7 inches
constexpr double kShortNeutralGoalHeight = 146.5; // 5.8 inches
constexpr double kAllianceGoalHeight = 82.5; // 3.25 inches

// Goal Locations
constexpr Point2D kCenterGoal = { 1783.2, 1783.2 };

// Neutral Short Goals (Quadrants)
constexpr Point2D kNeutralGoalTopLeft = { 1185.1, 2381.3 };
constexpr Point2D kNeutralGoalBottomLeft = { 1185.1, 1185.1 };
constexpr Point2D kNeutralGoalTopRight = { 2381.3, 2381.3 };
constexpr Point2D kNeutralGoalBottomRight = { 2381.3, 1185.1 };

// Alliance Short Goals (Corners)
constexpr Point2D kRedGoalTop = { 587.1, 2979.3 };
constexpr Point2D kRedGoalBottom = { 587.1, 587.1 };
constexpr Point2D kBlueGoalTop = { 2979.3, 2979.3 };
constexpr Point2D kBlueGoalBottom = { 2979.3, 587.1 };

// To handle ambiguous tags (multiple goals with the same ID), 
// we group known locations by their expected tag ID.
// Note: You may need to verify the exact ID mapping on the physical field.
struct GoalInfo {
    Point2D pos;
    double height;
};

inline std::vector<GoalInfo> get_goal_locations_for_tag(int tag_id) {
    switch (tag_id) {
        case 0:
            return {{kCenterGoal, kTallGoalHeight}};
        case 1:
            // Example mapping: Red Alliance Goals
            return {{kRedGoalTop, kAllianceGoalHeight}, {kRedGoalBottom, kAllianceGoalHeight}};
        case 2:
            // Example mapping: Blue Alliance Goals
            return {{kBlueGoalTop, kAllianceGoalHeight}, {kBlueGoalBottom, kAllianceGoalHeight}};
        case 3:
            // Example mapping: Left/Right Neutral Goals
            return {{kNeutralGoalTopLeft, kShortNeutralGoalHeight}, {kNeutralGoalBottomRight, kShortNeutralGoalHeight}};
        case 4:
            // Example mapping: Top/Bottom Neutral Goals
            return {{kNeutralGoalTopRight, kShortNeutralGoalHeight}, {kNeutralGoalBottomLeft, kShortNeutralGoalHeight}};
        default:
            return {};
    }
}

} // namespace field

#endif // FIELD_CONSTANTS_HPP_
