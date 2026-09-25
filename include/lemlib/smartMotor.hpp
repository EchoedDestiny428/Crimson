#pragma once

#include "lemlib/pid.hpp"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <atomic>
#include <cstdint>

namespace lemlib {

class SmartMotor {
public:
    SmartMotor(pros::MotorGroup* actuator, pros::adi::Encoder* encoder, PID controller, float settleRange = 5.0f,
               float feedforward = 0.0f);
    SmartMotor(pros::MotorGroup* actuator, pros::Rotation* rotation, PID controller, float settleRange = 5.0f,
               float feedforward = 0.0f);
    SmartMotor(pros::MotorGroup* actuator, std::int32_t maxVelocity, float settleRange = 5.0f);

    void start();
    void reset();

    float getRotation() const;
    float getTarget() const;
    bool isSettled() const;

    void setTarget(float newTarget);
    void setManual(int power);
    void holdCurrent();
    bool waitUntilSettled(std::uint32_t timeout);
    void setLogging(bool enabled);

private:
    bool usesMotorEncoders() const;
    void controlLoop();
    void runPid(float goal, float position, bool newTarget);
    void log(float goal, float position) const;

    pros::MotorGroup* actuator;
    pros::adi::Encoder* encoder;
    pros::Rotation* rotation;
    PID controller;
    float settleRange;
    float feedforward;
    std::int32_t maxVelocity = 0;

    std::atomic<float> target{0.0f};
    std::atomic<bool> targetChanged{false};
    std::atomic<bool> manual{false};
    std::atomic<int> manualPower{0};
    std::atomic<bool> logging{false};
    pros::Task* task = nullptr;
};

}
