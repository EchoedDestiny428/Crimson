#include "subsystems/elevator/extension.hpp"
#include "lemlib/smartMotor.hpp"
#include "robotconfig.hpp"
#include <cmath>

namespace subsystems::elevator::extension {

namespace {

// SmartMotor PID on the motors' encoders, in motor degrees (33 cm per 1100 deg). Every
// move's distance is known up front (hard setpoints), so the PID runs the whole way.
// Error is in motor deg: kP 3 is full power until ~42 deg (1.3 cm) out. D damps the stop.
// Coming down it is capped at kExtensionDownPower (70%).
constexpr float kP = 3.0f;
constexpr float kI = 0.0f;
constexpr float kD = 4.0f;
constexpr float kSettleRange = 20.0f;          // motor deg (~0.6 cm)
constexpr std::uint32_t kSettleTimeMs = 100;   // brakes (HOLD) after this long inside kSettleRange

lemlib::SmartMotor motor(&extension_motors, lemlib::PID(kP, kI, kD), kSettleRange);

}

void init() {
    extension_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    motor.setSettleTime(kSettleTimeMs);
    motor.reset();
    motor.setMaxReversePower(constants::kExtensionDownPower); // coming down at 70% at most
    motor.start();
}

void hold() {
    const double current = height();
    if (std::isfinite(current)) {
        set_target(current);
    } else {
        motor.holdCurrent();
    }
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

bool settled() {
    return motor.isSettled();
}

}
