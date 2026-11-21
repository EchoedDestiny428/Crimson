#pragma once

#include "lemlib/api.hpp"
#include <atomic>
#include <cstdint>

namespace crimson {

// At full turn one side drives +fastSpeed and the other -slowSpeed (out of 127). 127 / 30 is what every donut in the
// old autons used (partially-beneficial: snacky, teambad). PointWeaver's DEFAULTS (src/editor/types.ts) must match:
// it leaves speeds out of the code when they equal its defaults.
struct DonutParams {
    lemlib::AngularDirection direction = lemlib::AngularDirection::AUTO;
    bool forwards = true;
    int fastSpeed = 127;
    int slowSpeed = 30;
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

// Every pose in and out of this class is in the field frame of constants/field_constants.hpp, the one the camera and
// PointWeaver use: (0, 0) is the corner on blue's right, +X runs from the blue wall to the red wall, and heading 0
// faces red, turning clockwise. LemLib's own frame (heading 0 along its +Y) is kept inside: the overrides below turn
// field (x, y) into LemLib (-y, x) and back. Headings are the same number in both frames, so turnToHeading,
// swingToHeading and donut need no override.
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

    // Goes up by one every time setPose teleports odometry (auton start, goal resets), so anything following odometry
    // by its motion can tell a reset from real driving.
    std::uint32_t poseResets() const;

    void donut(float theta, int timeout, DonutParams params = {}, bool async = true);
    bool setPoseFromGoal(float goalX, float goalY, GoalPoseParams params = {});
    // The robot's front (or back) is pressed against the goal at (goalX, goalY); keeps
    // the IMU heading and always applies the reset.
    bool resetFromGoalFront(float goalX, float goalY);
    bool resetFromGoalBack(float goalX, float goalY);

private:
    std::atomic<std::uint32_t> pose_resets_{0};
};

}
