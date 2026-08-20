#ifndef FIELD_CONSTANTS_HPP_
#define FIELD_CONSTANTS_HPP_

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

// AprilTag IDs
// Center Goal
constexpr int kAprilTagCenter = 0;

// Alliance Goals
constexpr int kAprilTagRedTop = 1;
constexpr int kAprilTagRedBottom = 2;
constexpr int kAprilTagBlueTop = 2;
constexpr int kAprilTagBlueBottom = 1;

// Neutral Short Goals
constexpr int kAprilTagNeutralTopLeft = 4;
constexpr int kAprilTagNeutralBottomLeft = 3;
constexpr int kAprilTagNeutralTopRight = 3;
constexpr int kAprilTagNeutralBottomRight = 4;

// Scoring Objects Heights
constexpr double kPinHeight = 165.0; // 6.5 inches
constexpr double kCupHeight = 164.5; // 6.48 inches

} // namespace field

#endif // FIELD_CONSTANTS_HPP_
