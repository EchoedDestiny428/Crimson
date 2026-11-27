#pragma once

#include "lemlib/pid.hpp"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <atomic>
#include <cstdint>
#include <functional>
#include <limits>

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
    /** Cap on ramp and PID output going negative (e.g. a lift coming down), 0..127. Default 127. */
    void setMaxReversePower(float power);
    /**
     * PID and ramp modes: once a move is within range of its target, cap its output at power
     * (0..127), either direction. Range 0 (the default) turns it off.
     */
    void setSlowZone(float range, float power);
    /**
     * PID mode: keep running the PID inside settleRange and only brake once the error has
     * stayed inside it this long. 0 (the default) brakes as soon as it enters the range.
     */
    void setSettleTime(std::uint32_t ms);
    /**
     * PID mode: also brake once the error has stayed inside this wider range (> settleRange)
     * for ms. Whichever of the two settle conditions is met first brakes; once braked it
     * holds until the error leaves the wider range. 0 range (the default) turns it off.
     */
    void setSettleHold(float range, std::uint32_t ms);
    /** Extra PID output from the current position, e.g. gravity on an arm (kG * sin(angle)). */
    void setFeedforward(std::function<float(float position)> positionFeedforward);
    /**
     * Last say on manual and PID output: called with the position and the output about to
     * be sent, returns what is sent (e.g. slowing down near a soft limit). Set before start().
     */
    void setOutputFilter(std::function<float(float position, float output)> filter);
    /**
     * PID mode: while outside settleRange and closer than `range` to the target (0: no
     * limit), never push toward the target with less than this power (0..127), so friction
     * can't stall it short. Default 0 (off).
     */
    void setMinPower(float power, float range = 0.0f);
    /**
     * PID mode: ease into and out of every move instead of handing the PID the whole distance at once. The PID chases
     * a setpoint that speeds up and slows down by at most maxAcceleration (position units / s^2), up to maxVelocity
     * (units / s; 0: no cap), plus kV x the setpoint's speed (power per unit / s) so it doesn't fall behind. The
     * setpoint never gets more than maxLag (0: no limit) ahead of the motor. Settling and the minimum power wait for
     * the setpoint to reach the target. maxAcceleration 0 (the default) turns it off.
     * aheadFade (units; 0, the default: off): the kV push fades out as the motor gets ahead of the setpoint, from full
     * when level or behind to none once aheadFade ahead, so something carrying the motor along (gravity on an arm)
     * isn't pushed on past the setpoint into an overshoot.
     * aheadBrake (power per unit / s; 0, the default: off): while the motor is ahead of the setpoint and moving faster
     * than it, push back by this much per unit / s of the difference, so it's braked back onto the setpoint instead of
     * carrying on past the target.
     */
    void setProfile(float maxAcceleration, float maxVelocity = 0.0f, float kV = 0.0f, float maxLag = 0.0f,
                    float aheadFade = 0.0f, float aheadBrake = 0.0f);
    /**
     * With setProfile: the last `fraction` (0..1) of every move runs at most `velocity` (units / s), slowing down to it
     * before that part starts, so the move eases in instead of arriving at speed. fraction 0 (the default) turns it off.
     */
    void setProfileFinish(float fraction, float velocity);
    /** Enable or disable SmartMotor diagnostics through the configured info sink. */
    void setLogging(bool enabled);

private:
    bool usesMotorEncoders() const;
    void controlLoop();
    void runPid(float goal, float position, bool newTarget);
    void advanceProfile(float goal, float position);
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
    std::function<float(float, float)> outputFilter;
    std::atomic<float> maxPower{127.0f};
    std::atomic<float> maxReversePower{127.0f};
    std::atomic<float> minPower{0.0f};
    std::atomic<float> minPowerRange{0.0f};
    std::atomic<float> minPowerNear{0.0f};
    std::atomic<float> minPowerNearRange{0.0f};
    std::atomic<float> slowZoneRange{0.0f};
    std::atomic<float> slowZonePower{127.0f};
    std::atomic<std::uint32_t> settleTimeMs{0};
    std::uint32_t inRangeSince = 0; // control task only; 0 = not inside settleRange
    std::atomic<float> holdSettleRange{0.0f};
    std::atomic<std::uint32_t> holdSettleTimeMs{0};
    std::uint32_t holdInRangeSince = 0; // control task only; 0 = not inside holdSettleRange
    float degreesPerCount = 0.0f; // > 0: read raw counts (see useRawCounts)
    std::atomic<float> rawZero{0.0f};
    float averageRawCounts() const;
    std::atomic<float> decelerationDistance{0.0f};
    std::atomic<float> profileAcceleration{0.0f};
    std::atomic<float> profileMaxVelocity{0.0f};
    std::atomic<float> profileKV{0.0f};
    std::atomic<float> profileMaxLag{0.0f};
    std::atomic<float> profileAheadFade{0.0f};
    std::atomic<float> profileAheadBrake{0.0f};
    std::atomic<float> profileFinishFraction{0.0f};
    std::atomic<float> profileFinishVelocity{0.0f};
    float profileSpan = 0.0f; // control task only: the current move's length, for setProfileFinish
    bool profileActive = false;   // control task only: the setpoint below is in use
    float profilePosition = 0.0f; // control task only
    float profileVelocity = 0.0f; // control task only, units / s
    float lastPidPosition = 0.0f; // control task only: the position runPid read last tick, for the measured speed
    bool lastPidPositionValid = false; // control task only: false after open-loop power (or a re-zero) and at start

    std::atomic<float> encoderOffset{0.0f};
    std::atomic<float> target{0.0f};
    // The target the PID last braked on as settled (either settle range), NaN while it isn't braked: isSettled()
    // counts it, so a move that brakes inside the wider settle-hold range still reads settled. Compared with the
    // current target, so a new setTarget() never reads the move before's settle.
    std::atomic<float> brakedAt{std::numeric_limits<float>::quiet_NaN()};
    std::atomic<bool> targetChanged{false};
    std::atomic<bool> manual{false};
    std::atomic<int> manualPower{0};
    std::atomic<bool> logging{false};
    pros::Task* task = nullptr;
};

}
