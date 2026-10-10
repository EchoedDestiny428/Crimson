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

struct GoalPoseParams {
    bool forwards = false;
    float distance = 7.5;
    float maxCorrection = 6;
};

class Chassis : public lemlib::Chassis {
public:
    using lemlib::Chassis::Chassis;

    void donut(float theta, int timeout, DonutParams params = {}, bool async = true);
    bool setPoseFromGoal(float goalX, float goalY, GoalPoseParams params = {});
};

}
