#include "lemlib/smartMotor.hpp"
#include "lemlib/logger/logger.hpp"
#include "pros/error.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace lemlib {

namespace {

constexpr std::uint32_t kPeriodMs = 10;
constexpr std::uint32_t kLogEveryTicks = 10;
constexpr float kMaxPower = 127.0f;
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

SmartMotor::SmartMotor(pros::MotorGroup* actuator, PID controller, float settleRange, float feedforward)
    : actuator(actuator),
      encoder(nullptr),
      rotation(nullptr),
      controller(controller),
      settleRange(settleRange),
      feedforward(feedforward) {}

void SmartMotor::start() {
    if (task == nullptr) {
        task = new pros::Task([this] { controlLoop(); }, "SmartMotor");
    }
}

void SmartMotor::reset() {
    if (encoder != nullptr) {
        encoder->reset();
    } else if (rotation != nullptr) {
        rotation->reset_position();
    } else {
        actuator->tare_position_all();
    }
    setTarget(0.0f);
}

float SmartMotor::getRotation() const {
    if (encoder != nullptr) {
        const std::int32_t value = encoder->get_value();
        return value == PROS_ERR ? kInvalid : static_cast<float>(value);
    }
    if (rotation != nullptr) {
        const std::int32_t value = rotation->get_position();
        return value == PROS_ERR ? kInvalid : static_cast<float>(value);
    }

    float sum = 0.0f;
    int count = 0;
    for (const double position : actuator->get_position_all()) {
        if (std::isfinite(position)) {
            sum += static_cast<float>(position);
            ++count;
        }
    }
    return count > 0 ? sum / static_cast<float>(count) : kInvalid;
}

float SmartMotor::getTarget() const {
    return target;
}

bool SmartMotor::isSettled() const {
    return !manual && std::fabs(target - getRotation()) < settleRange;
}

void SmartMotor::setTarget(float newTarget) {
    if (!std::isfinite(newTarget)) {
        return;
    }
    target = newTarget;
    targetChanged = true;
    manual = false;
}

void SmartMotor::setManual(int power) {
    manualPower = power;
    manual = true;
}

void SmartMotor::holdCurrent() {
    setTarget(getRotation());
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

void SmartMotor::setLogging(bool enabled) {
    logging = enabled;
}

void SmartMotor::controlLoop() {
    std::uint32_t now = pros::millis();
    std::uint32_t tick = 0;

    while (true) {
        const float goal = target;
        const float position = getRotation();
        const float error = goal - position;

        if (targetChanged.exchange(false)) {
            controller.reset();
        }

        if (manual) {
            actuator->move(manualPower);
        } else if (!std::isfinite(error) || std::fabs(error) < settleRange) {
            controller.reset();
            actuator->brake();
        } else {
            const float output = controller.update(error) + feedforward;
            actuator->move(static_cast<std::int32_t>(std::clamp(output, -kMaxPower, kMaxPower)));
        }

        if (logging && ++tick % kLogEveryTicks == 0) {
            log(goal, position);
        }

        pros::Task::delay_until(&now, kPeriodMs);
    }
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
