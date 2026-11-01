#pragma once

#include "lemlib/api.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include <atomic>
#include <cstdint>
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

// Lift-off detection. While the IMU's pitch or roll is more than tiltDeg away from
// where it read flat at startup, the tracking wheels may be off the floor, so their
// movement is not trusted: x/y are held at the pose from the tick before the tilt was
// seen, so only the tilted ticks are ignored. Heading still comes from the IMU, which
// measures rotation correctly while lifted. Trust returns on the first flat tick.
struct TiltGuardParams {
    float tiltDeg = 9.1f;
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

    // Starts the lift-off guard task; call after calibrate(). The current pitch and
    // roll are taken as flat.
    void startTiltGuard(pros::Imu& imu, TiltGuardParams params = {});
    // False while the robot is tipped, i.e. while x/y are being held.
    bool odomTrusted() const;
    // How many times the guard has started holding since startup.
    int tiltHoldCount() const;
    // The pose as if the guard had never held: getPose() plus every move it discarded
    // since the last setPose.
    lemlib::Pose getUnguardedPose();

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
    void holdPosition(float x, float y);

    pros::Mutex rawMutex;
    RawCorrection raw;
    bool rawArmed = true;
    std::atomic<bool> odomTrustedFlag{true};
    std::atomic<int> tiltHolds{0};
    float discardedX = 0; // guarded by rawMutex
    float discardedY = 0;
};

}
