#pragma once

#include "lemlib/api.hpp"
#include "pros/rtos.hpp"
#include <source_location>

namespace crimson {

struct DonutParams {
    lemlib::AngularDirection direction = lemlib::AngularDirection::AUTO;
    bool forwards = true;
    int fastSpeed = 100;
    int slowSpeed = 75;
    float earlyExitRange = 0;
};

struct GoalPoseParams {
    bool forwards = false;
    float distance = 7;
    float maxCorrection = 10;
};

class Chassis : public lemlib::Chassis {
public:
    using lemlib::Chassis::Chassis;

    void setPose(float x, float y, float theta, bool radians = false,
                 std::source_location where = std::source_location::current());
    void setPose(lemlib::Pose pose, bool radians = false, std::source_location where = std::source_location::current());

    void turnToPoint(float x, float y, int timeout, lemlib::TurnToPointParams params = {}, bool async = true,
                     std::source_location where = std::source_location::current());
    void turnToHeading(float theta, int timeout, lemlib::TurnToHeadingParams params = {}, bool async = true,
                       std::source_location where = std::source_location::current());
    void swingToHeading(float theta, lemlib::DriveSide lockedSide, int timeout,
                        lemlib::SwingToHeadingParams params = {}, bool async = true,
                        std::source_location where = std::source_location::current());
    void swingToPoint(float x, float y, lemlib::DriveSide lockedSide, int timeout,
                      lemlib::SwingToPointParams params = {}, bool async = true,
                      std::source_location where = std::source_location::current());
    void moveToPose(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params = {}, bool async = true,
                    std::source_location where = std::source_location::current());
    void moveToPoint(float x, float y, int timeout, lemlib::MoveToPointParams params = {}, bool async = true,
                     std::source_location where = std::source_location::current());

    void donut(float theta, int timeout, DonutParams params = {}, bool async = true,
               std::source_location where = std::source_location::current());
    bool setPoseFromGoal(float goalX, float goalY, GoalPoseParams params = {},
                         std::source_location where = std::source_location::current());

    void armRawAnchor();
    lemlib::Pose getRawPose();

private:
    struct RawCorrection {
        double x = 0;
        double y = 0;
        double theta = 0;
    };

    template <typename Apply>
    void resetPose(Apply apply, const std::source_location& where);
    void runDonut(float theta, int timeout, DonutParams params, bool async);
    lemlib::Pose applyRaw(const lemlib::Pose& pose) const;
    void rebaseRaw(const lemlib::Pose& before, const lemlib::Pose& after);

    pros::Mutex rawMutex;
    RawCorrection raw;
    bool rawArmed = true;
};

}
