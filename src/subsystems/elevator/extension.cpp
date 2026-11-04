#include "subsystems/elevator/extension.hpp"
#include "config/controls.hpp"
#include "lemlib/smartMotor.hpp"
#include "robotconfig.hpp"
#include <cmath>

namespace subsystems::elevator::extension {

namespace {

constexpr int kManualPower = 127;
constexpr std::int32_t kMaxVelocity = constants::kExtensionMaxRpm;
constexpr float kSettleRange = 5.0f;
constexpr float kDecelerationDistance = 300.0f; // slows over the last 5 cm of a move

lemlib::SmartMotor motor(&extension_motors, kMaxVelocity, kSettleRange, kDecelerationDistance);
bool manual_active = false;

}

void init() {
    extension_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    motor.reset();
    motor.setMaxReversePower(constants::kExtensionDownPower); // snaps and home coming down at 35%
    motor.start();
}

void update() {
    const bool up = controller.get_digital(controls::kExtensionUp);
    const bool down = controller.get_digital(controls::kExtensionDown);

    if (up || down) {
        manual_active = true;
        set_manual(up ? kManualPower : -kManualPower);
    } else if (manual_active) {
        manual_active = false;
        hold();
    }
}

void stop() {
    manual_active = false;
    hold();
}

void hold() {
    const double current = height();
    if (std::isfinite(current)) {
        set_target(current);
    } else {
        motor.holdCurrent();
    }
}

void set_pos(double height) {
    motor.setPosition(static_cast<float>(height));
}

void set_target(double height) {
    motor.setTarget(static_cast<float>(height));
}

// No software limits on purpose: if the encoder skips, the driver still has full manual control.
void set_manual(int power) {
    motor.setManual(power);
}

double height() {
    return motor.getRotation();
}

double target() {
    return motor.getTarget();
}

bool is_settled() {
    return motor.isSettled();
}

bool wait_until_settled(std::uint32_t timeout_ms) {
    return motor.waitUntilSettled(timeout_ms);
}

}
