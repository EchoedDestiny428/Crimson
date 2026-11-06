#pragma once

#include "lemlib/api.hpp"

namespace crimson {

struct DonutParams {
    lemlib::AngularDirection direction = lemlib::AngularDirection::AUTO;
    bool forwards = true;
    int fastSpeed = 100;
    int slowSpeed = 75;
    float earlyExitRange = 0;
};

// Goal center to robot center when the robot is pressed against a goal, in inches
// (LemLib units; / 2.54 converts cm). Front and back differ on the new robot.
inline constexpr float kGoalResetFrontIn = 15.0f / 2.54f; // front of the robot against the goal
inline constexpr float kGoalResetBackIn = 22.0f / 2.54f;  // back of the robot against the goal

struct GoalPoseParams {
    bool forwards = false;     // true: front against the goal; false: back against it
    float distance = 0.0f;     // 0 uses kGoalResetFrontIn or kGoalResetBackIn to match `forwards`
    float maxCorrection = 10;
};

// Every pose and target here is in the field frame from field_constants.hpp, where heading 0 faces +X. LemLib's
// heading 0 faces its own +Y, so poses are turned a quarter turn on the way in and out.
class Chassis : public lemlib::Chassis {
public:
    using lemlib::Chassis::Chassis;

    lemlib::Pose getPose(bool radians = false, bool standardPos = false);
    void setPose(float x, float y, float theta, bool radians = false);
    void setPose(lemlib::Pose pose, bool radians = false);
    void turnToPoint(float x, float y, int timeout, lemlib::TurnToPointParams params = {}, bool async = true);
    void swingToPoint(float x, float y, lemlib::DriveSide lockedSide, int timeout,
                      lemlib::SwingToPointParams params = {}, bool async = true);
    void moveToPose(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params = {},
                    bool async = true);
    void moveToPoint(float x, float y, int timeout, lemlib::MoveToPointParams params = {}, bool async = true);

    void donut(float theta, int timeout, DonutParams params = {}, bool async = true);
    bool setPoseFromGoal(float goalX, float goalY, GoalPoseParams params = {});
    // The robot's front (or back) is pressed against the goal at (goalX, goalY); keeps
    // the IMU heading and always applies the reset.
    bool resetFromGoalFront(float goalX, float goalY);
    bool resetFromGoalBack(float goalX, float goalY);
};

}
