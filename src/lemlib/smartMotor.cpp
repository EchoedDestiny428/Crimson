#include "lemlib/smartMotor.hpp"
#include "lemlib/logger/logger.hpp"
#include "pros/error.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace lemlib {

namespace {

constexpr std::uint32_t kPeriodMs = 10;
constexpr std::uint32_t kLogEveryTicks = 10;
constexpr float kMaxPower = 127.0f;
constexpr float kMinimumProfilePower = 20.0f;
constexpr float kInvalid = std::numeric_limits<float>::quiet_NaN();

}

SmartMotor::SmartMotor(pros::MotorGroup* actuator, pros::adi::Encoder* encoder, PID controller, float settleRange,
                       float feedforward)
    : actuator(actuator),
      encoder(encoder),
      rotation(nullptr),
      controller(controller),
      settleRange(settleRange),
      feedforward(feedforward) {}

SmartMotor::SmartMotor(pros::MotorGroup* actuator, pros::Rotation* rotation, PID controller, float settleRange,
                       float feedforward)
    : actuator(actuator),
      encoder(nullptr),
      rotation(rotation),
      controller(controller),
      settleRange(settleRange),
      feedforward(feedforward) {}

SmartMotor::SmartMotor(pros::MotorGroup* actuator, std::int32_t maxVelocity, float settleRange,
                       float decelerationDistance)
    : actuator(actuator),
      encoder(nullptr),
      rotation(nullptr),
      controller(0, 0, 0),
      settleRange(settleRange),
      feedforward(0.0f),
      maxVelocity(maxVelocity),
      decelerationDistance(std::max(0.0f, decelerationDistance)) {}

SmartMotor::SmartMotor(pros::MotorGroup* actuator, PID controller, float settleRange, float feedforward)
    : actuator(actuator),
      encoder(nullptr),
      rotation(nullptr),
      controller(controller),
      settleRange(settleRange),
      feedforward(feedforward),
      motorEncoderPid(true) {}

bool SmartMotor::usesMotorEncoders() const {
    return encoder == nullptr && rotation == nullptr;
}

void SmartMotor::start() {
    if (task == nullptr) {
        task = new pros::Task([this] { controlLoop(); }, "SmartMotor");
    }
}

void SmartMotor::reset() {
    setPosition(0.0f);
}

void SmartMotor::setPosition(float position) {
    manualPower = 0;
    manual = true;
    if (encoder != nullptr) {
        encoder->reset();
        encoderOffset = position;
    } else if (rotation != nullptr) {
        rotation->set_position(static_cast<std::int32_t>(position));
    } else {
        // tare, then read as `position` from here (set_zero_position does not tare)
        actuator->tare_position_all();
        if (degreesPerCount > 0.0f) {
            rawZero = averageRawCounts();
        }
        encoderOffset = position;
    }
    setTarget(position);
}

float SmartMotor::getRotation() const {
    if (encoder != nullptr) {
        const std::int32_t value = encoder->get_value();
        return value == PROS_ERR ? kInvalid : static_cast<float>(value) + encoderOffset;
    }
    if (rotation != nullptr) {
        const std::int32_t value = rotation->get_position();
        return value == PROS_ERR ? kInvalid : static_cast<float>(value);
    }

    if (degreesPerCount > 0.0f) {
        const float raw = averageRawCounts();
        return std::isfinite(raw) ? (raw - rawZero) * degreesPerCount + encoderOffset : kInvalid;
    }

    float sum = 0.0f;
    int count = 0;
    for (const double position : actuator->get_position_all()) {
        if (std::isfinite(position)) {
            sum += static_cast<float>(position);
            ++count;
        }
    }
    return count > 0 ? sum / static_cast<float>(count) + encoderOffset : kInvalid;
}

float SmartMotor::getTarget() const {
    return target;
}

// Within settleRange now, or the PID has braked on this target (it also brakes inside setSettleHold's wider range,
// which this alone would never call settled).
bool SmartMotor::isSettled() const {
    if (manual) {
        return false;
    }
    const float goal = target;
    return brakedAt.load() == goal || std::fabs(goal - getRotation()) < settleRange;
}

void SmartMotor::setTarget(float newTarget) {
    if (!std::isfinite(newTarget)) {
        return;
    }
    target = newTarget;
    targetChanged = true;
    manual = false;
}

void SmartMotor::setDecelerationDistance(float distance) {
    decelerationDistance = std::max(0.0f, distance);
}

void SmartMotor::setManual(int power) {
    manualPower = power;
    manual = true;
    brakedAt = std::numeric_limits<float>::quiet_NaN();
}

void SmartMotor::holdCurrent() {
    const float position = getRotation();
    if (std::isfinite(position)) {
        setTarget(position);
    } else {
        manual = false;
    }
}

bool SmartMotor::waitUntilSettled(std::uint32_t timeout) {
    const std::uint32_t start = pros::millis();
    while (!isSettled()) {
        if (pros::millis() - start >= timeout) {
            return false;
        }
        pros::delay(kPeriodMs);
    }
    return true;
}

void SmartMotor::setMaxPower(float power) {
    maxPower = std::clamp(power, 0.0f, kMaxPower);
}

void SmartMotor::setMaxReversePower(float power) {
    maxReversePower = std::clamp(power, 0.0f, kMaxPower);
}

void SmartMotor::setSlowZone(float range, float power) {
    slowZoneRange = std::max(0.0f, range);
    slowZonePower = std::clamp(power, 0.0f, kMaxPower);
}

void SmartMotor::useRawCounts(float countsToDegrees) {
    degreesPerCount = countsToDegrees;
}

float SmartMotor::averageRawCounts() const {
    std::uint32_t timestamp = 0;
    double sum = 0.0;
    int count = 0;
    for (const std::int32_t raw : actuator->get_raw_position_all(&timestamp)) {
        if (raw != PROS_ERR) {
            sum += raw;
            ++count;
        }
    }
    return count > 0 ? static_cast<float>(sum / count) : kInvalid;
}

void SmartMotor::setSettleTime(std::uint32_t ms) {
    settleTimeMs = ms;
}

void SmartMotor::setSettleHold(float range, std::uint32_t ms) {
    holdSettleRange = std::max(0.0f, range);
    holdSettleTimeMs = ms;
}

void SmartMotor::setMinPower(float power, float range) {
    minPower = std::clamp(power, 0.0f, kMaxPower);
    minPowerRange = std::max(0.0f, range);
}

void SmartMotor::setProfile(float maxAcceleration, float maxVelocity, float kV, float maxLag, float aheadFade,
                            float aheadBrake) {
    profileMaxVelocity = std::max(0.0f, maxVelocity);
    profileKV = kV;
    profileMaxLag = std::max(0.0f, maxLag);
    profileAheadFade = std::max(0.0f, aheadFade);
    profileAheadBrake = std::max(0.0f, aheadBrake);
    profileAcceleration = std::max(0.0f, maxAcceleration);
}

void SmartMotor::setProfileFinish(float fraction, float velocity) {
    profileFinishVelocity = std::max(0.0f, velocity);
    profileFinishFraction = std::clamp(fraction, 0.0f, 1.0f);
}

void SmartMotor::setOutputFilter(std::function<float(float position, float output)> filter) {
    outputFilter = std::move(filter);
}

void SmartMotor::setFeedforward(std::function<float(float position)> feedforwardFn) {
    positionFeedforward = std::move(feedforwardFn);
}

void SmartMotor::setLogging(bool enabled) {
    logging = enabled;
}

void SmartMotor::controlLoop() {
    std::uint32_t now = pros::millis();
    std::uint32_t tick = 0;
    bool positionCommanded = false;

    while (true) {
        const float goal = target;
        const float position = getRotation();
        const bool newTarget = targetChanged.exchange(false);
        if (manual) {
            positionCommanded = false;
            profileActive = false; // the next PID move starts its profile from wherever this leaves the motor
            lastPidPositionValid = false; // (setPosition's re-zero also goes through here: no speed across the jump)
            const float cap = maxPower;
            float output = std::clamp(static_cast<float>(manualPower.load()), -cap, cap);
            if (outputFilter) {
                output = outputFilter(position, output);
            }
            actuator->move(static_cast<std::int32_t>(output));
        } else if (usesMotorEncoders() && !motorEncoderPid) {
            const float distance = decelerationDistance;
            if (distance > settleRange) {
                const float error = goal - position;
                if (!std::isfinite(error) || std::fabs(error) < settleRange) {
                    actuator->brake();
                    positionCommanded = true;
                } else {
                    const float top = error < 0.0f ? std::min(kMaxPower, maxReversePower.load()) : kMaxPower;
                    const float power = std::max(std::min(kMinimumProfilePower, top),
                                                 std::clamp(std::fabs(error) / distance, 0.0f, 1.0f) * top);
                    float output = std::copysign(power, error);
                    if (std::fabs(error) < slowZoneRange) {
                        const float slow = slowZonePower;
                        output = std::clamp(output, -slow, slow);
                    }
                    if (outputFilter) {
                        output = outputFilter(position, output);
                    }
                    actuator->move(static_cast<std::int32_t>(output));
                    positionCommanded = true;
                }
            } else if (newTarget || !positionCommanded) {
                actuator->move_absolute(goal - encoderOffset, maxVelocity);
                positionCommanded = true;
            }
        } else {
            runPid(goal, position, newTarget);
        }

        if (logging && ++tick % kLogEveryTicks == 0) {
            log(goal, position);
        }

        pros::Task::delay_until(&now, kPeriodMs);
    }
}

// One tick of the setProfile setpoint: toward the goal, speeding up or slowing down by at most profileAcceleration,
// never faster than it can still stop at the goal (sqrt(2 x accel x distance left)) or than profileMaxVelocity.
void SmartMotor::advanceProfile(float goal, float position) {
    constexpr float kDt = static_cast<float>(kPeriodMs) / 1000.0f;
    const float acceleration = profileAcceleration;
    const float remaining = goal - profilePosition;
    const float direction = remaining >= 0.0f ? 1.0f : -1.0f;
    const float toward = profileVelocity * direction; // speed toward the goal; negative is moving away from it
    float wanted = std::sqrt(2.0f * acceleration * std::fabs(remaining));
    const float cap = profileMaxVelocity;
    if (cap > 0.0f) {
        wanted = std::min(wanted, cap);
    }
    // setProfileFinish: inside the move's last stretch, at most the finish speed; before it, no faster than it can
    // still slow down to that speed by the time it gets there.
    const float finishFraction = profileFinishFraction;
    const float finishVelocity = profileFinishVelocity;
    if (finishFraction > 0.0f && finishVelocity > 0.0f) {
        const float beyond = std::fabs(remaining) - finishFraction * profileSpan;
        wanted = std::min(wanted, beyond <= 0.0f ? finishVelocity
                                                 : std::sqrt(finishVelocity * finishVelocity +
                                                             2.0f * acceleration * beyond));
    }
    // Slowing down may brake harder than the limit, just enough to still stop at the goal: setProfile can lower the
    // limit mid-move (the arm's follows the lift height), and the setpoint must not arrive at speed.
    float brake = acceleration;
    if (toward > 0.0f && std::fabs(remaining) > 0.0f) {
        brake = std::max(brake, toward * toward / (2.0f * std::fabs(remaining)));
    }
    const float next = toward + std::clamp(wanted - toward, -brake * kDt, acceleration * kDt);
    if (next >= 0.0f && next * kDt >= std::fabs(remaining)) {
        profilePosition = goal; // gets there this tick
        profileVelocity = 0.0f;
    } else {
        profilePosition += next * kDt * direction;
        profileVelocity = next * direction;
    }
    const float lag = profileMaxLag;
    if (lag > 0.0f) {
        profilePosition = std::clamp(profilePosition, position - lag, position + lag);
    }
}

void SmartMotor::runPid(float goal, float position, bool newTarget) {
    if (newTarget) {
        controller.reset();
        inRangeSince = 0;
        holdInRangeSince = 0;
    }

    if (!std::isfinite(goal - position)) {
        controller.reset();
        profileActive = false;
        lastPidPositionValid = false;
        brakedAt = std::numeric_limits<float>::quiet_NaN();
        actuator->brake();
        return;
    }
    // The motor's speed from this tick's and last tick's positions (units / s), for setProfile's aheadBrake.
    const bool haveSpeed = lastPidPositionValid;
    const float measuredSpeed = haveSpeed ? (position - lastPidPosition) * 1000.0f / static_cast<float>(kPeriodMs) : 0.0f;
    lastPidPosition = position;
    lastPidPositionValid = true;

    // With setProfile the PID chases a setpoint easing toward the goal; without it the setpoint is the goal. A move
    // that starts from rest starts its setpoint where the motor is; a new target mid-move carries on from the setpoint.
    float setpoint = goal;
    float setpointVelocity = 0.0f;
    if (profileAcceleration > 0.0f) {
        if (!profileActive || (newTarget && profileVelocity == 0.0f)) {
            profilePosition = position;
            profileVelocity = 0.0f;
        }
        if (newTarget || !profileActive) {
            profileSpan = std::fabs(goal - profilePosition);
        }
        profileActive = true;
        advanceProfile(goal, position);
        setpoint = profilePosition;
        setpointVelocity = profileVelocity;
    } else {
        profileActive = false;
    }
    const bool arrived = setpoint == goal && setpointVelocity == 0.0f; // always true without a profile
    const float error = setpoint - position;

    // Inside settleRange: keep running the PID until it has stayed there settleTimeMs,
    // then brake. Leaving the range (or sagging out after braking) restarts the timer.
    // A second, wider range (setSettleHold) brakes too, after its own (usually longer) time.
    // Each timer runs while the error stays inside its range; leaving it restarts it.
    // With a profile, neither starts until the setpoint has reached the goal (then error is the distance to it).
    const std::uint32_t now = pros::millis();
    const std::uint32_t stamp = now == 0 ? 1 : now;
    bool settled = false;
    if (arrived && std::fabs(error) < settleRange) {
        if (inRangeSince == 0) {
            inRangeSince = stamp;
        }
        settled = now - inRangeSince >= settleTimeMs;
    } else {
        inRangeSince = 0;
    }
    const float holdRange = holdSettleRange;
    if (arrived && holdRange > 0.0f && std::fabs(error) < holdRange) {
        if (holdInRangeSince == 0) {
            holdInRangeSince = stamp;
        }
        settled = settled || now - holdInRangeSince >= holdSettleTimeMs;
    } else {
        holdInRangeSince = 0;
    }
    if (settled) {
        controller.reset();
        brakedAt = goal;
        actuator->brake();
        return;
    }
    brakedAt = std::numeric_limits<float>::quiet_NaN(); // driving again (or not there yet)

    // The profile's speed push (kV), faded out as the motor gets ahead of the setpoint (setProfile's aheadFade). Without
    // the fade, an arm that gravity carries past its setpoint gets the full push on top (the setpoint, held within
    // maxLag, comes along with it), and overshoots: 12-25 deg on every swing in run 37.
    float speedPush = profileKV * setpointVelocity;
    const float fade = profileAheadFade;
    if (fade > 0.0f && setpointVelocity != 0.0f) {
        const float ahead = setpointVelocity > 0.0f ? position - setpoint : setpoint - position;
        if (ahead > 0.0f) {
            speedPush *= std::max(0.0f, 1.0f - ahead / fade);
        }
    }
    float output = controller.update(error) + feedforward + speedPush;
    // setProfile's aheadBrake: ahead of the setpoint and faster than it, push back in proportion to how much faster.
    // The PID alone can't: the setpoint is held within maxLag of the motor, so its error (and braking) stays small.
    // Added after the output filter below, so a cap on pushing toward a limit (the arm's last 15 deg before 315) can't
    // swallow it: it only ever pushes against the motion.
    float aheadBrake = 0.0f;
    const float brakeGain = profileAheadBrake;
    if (brakeGain > 0.0f && haveSpeed && setpointVelocity != 0.0f) {
        const float direction = setpointVelocity > 0.0f ? 1.0f : -1.0f;
        const float ahead = (position - setpoint) * direction;
        const float faster = (measuredSpeed - setpointVelocity) * direction;
        if (ahead > 0.0f && faster > 0.0f) {
            aheadBrake = -direction * brakeGain * faster;
        }
    }
    if (positionFeedforward) {
        output += positionFeedforward(position);
    }
    // Minimum power: outside settleRange, a push toward the target is raised to at least
    // minPower so friction can't stall it short. A push away from the target (the D term
    // braking) is left alone. With a profile, only once the setpoint has reached the goal.
    const float floorPower = minPower;
    const float floorRange = minPowerRange;
    if (arrived && floorPower > 0.0f && std::fabs(error) >= settleRange &&
        (floorRange <= 0.0f || std::fabs(error) < floorRange) && output * error >= 0.0f &&
        std::fabs(output) < floorPower) {
        output = std::copysign(floorPower, error);
    }
    const float goalError = goal - position;
    if (std::fabs(goalError) < slowZoneRange) {
        const float slow = slowZonePower;
        output = std::clamp(output, -slow, slow);
    }
    const float cap = maxPower;
    output = std::clamp(output, -std::min(cap, maxReversePower.load()), cap);
    if (outputFilter) {
        output = outputFilter(position, output);
    }
    if (aheadBrake != 0.0f) {
        output = std::clamp(output + aheadBrake, -std::min(cap, maxReversePower.load()), cap);
    }
    actuator->move(static_cast<std::int32_t>(output));
}

void SmartMotor::log(float goal, float position) const {
    const std::vector<std::int32_t> voltages = actuator->get_voltage_all();
    double total = 0.0;
    for (const std::int32_t voltage : voltages) {
        total += voltage;
    }
    const double average = voltages.empty() ? 0.0 : total / static_cast<double>(voltages.size());
    infoSink()->info("SmartMotor | Target: {:.1f} | Current: {:.1f} | Voltage: {:.0f}mV", goal, position, average);
}

}
