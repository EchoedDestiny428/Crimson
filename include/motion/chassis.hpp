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

inline constexpr float kGoalResetFrontIn = 15.0f / 2.54f;
inline constexpr float kGoalResetBackIn = 22.0f / 2.54f;

struct GoalPoseParams {
    bool forwards = false;
    float distance = 0.0f;
    float maxCorrection = 10;
};

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
    bool resetFromGoalFront(float goalX, float goalY);
    bool resetFromGoalBack(float goalX, float goalY);
};

}
