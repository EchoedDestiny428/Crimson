#pragma once

#include "lemlib/pid.hpp"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <atomic>
#include <cstdint>
#include <functional>

namespace lemlib {

class SmartMotor {
public:
    SmartMotor(pros::MotorGroup* actuator, pros::adi::Encoder* encoder, PID controller, float settleRange = 5.0f,
               float feedforward = 0.0f);
    SmartMotor(pros::MotorGroup* actuator, pros::Rotation* rotation, PID controller, float settleRange = 5.0f,
               float feedforward = 0.0f);
    SmartMotor(pros::MotorGroup* actuator, std::int32_t maxVelocity, float settleRange = 5.0f,
               float decelerationDistance = 0.0f);
    /** PID (runPid) on the motor group's own encoders, in motor degrees. */
    SmartMotor(pros::MotorGroup* actuator, PID controller, float settleRange = 5.0f, float feedforward = 0.0f);

    void start();
    void reset();
    void setPosition(float position);

    float getRotation() const;
    float getTarget() const;
    bool isSettled() const;

    void setTarget(float newTarget);
    void setDecelerationDistance(float distance);
    void setManual(int power);
    void holdCurrent();
    bool waitUntilSettled(std::uint32_t timeout);
    /**
     * Read position from the motors' raw encoder counts times degreesPerCount instead of
     * PROS degrees. Raw counts come from the encoder before the cartridge, so they don't
     * depend on the gearset setting (red is 1800 per output turn: 0.2 deg per count).
     * Call before reset().
     */
    void useRawCounts(float degreesPerCount);
    /** Cap on every output (manual, PID, profile), in motor power 0..127. Default 127. */
    void setMaxPower(float power);
    /** Cap on ramp-mode output going negative (e.g. a lift coming down), 0..127. Default 127. */
    void setMaxReversePower(float power);
    /**
     * PID mode: once a move is within range of its target, cap its output at power
     * (0..127), either direction. Range 0 (the default) turns it off.
     */
    void setSlowZone(float range, float power);
    /**
     * PID mode: keep running the PID inside settleRange and only brake once the error has
     * stayed inside it this long. 0 (the default) brakes as soon as it enters the range.
     */
    void setSettleTime(std::uint32_t ms);
    /** Extra PID output from the current position, e.g. gravity on an arm (kG * sin(angle)). */
    void setFeedforward(std::function<float(float position)> positionFeedforward);
    /** Enable or disable SmartMotor diagnostics through the configured info sink. */
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
    bool motorEncoderPid = false;
    std::function<float(float)> positionFeedforward;
    std::atomic<float> maxPower{127.0f};
    std::atomic<float> maxReversePower{127.0f};
    std::atomic<float> slowZoneRange{0.0f};
    std::atomic<float> slowZonePower{127.0f};
    std::atomic<std::uint32_t> settleTimeMs{0};
    std::uint32_t inRangeSince = 0; // control task only; 0 = not inside settleRange
    float degreesPerCount = 0.0f; // > 0: read raw counts (see useRawCounts)
    std::atomic<float> rawZero{0.0f};
    float averageRawCounts() const;
    std::atomic<float> decelerationDistance{0.0f};

    std::atomic<float> encoderOffset{0.0f};
    std::atomic<float> target{0.0f};
    std::atomic<bool> targetChanged{false};
    std::atomic<bool> manual{false};
    std::atomic<int> manualPower{0};
    std::atomic<bool> logging{false};
    pros::Task* task = nullptr;
};

}
