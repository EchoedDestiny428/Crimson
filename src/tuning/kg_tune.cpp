#include "tuning/kg_tune.hpp"
#include "robotconfig.hpp"
#include "subsystems/elevator/constants.hpp"
#include <cmath>
#include <cstdint>

namespace tuning::kg {

namespace {

namespace constants = subsystems::elevator::constants;

constexpr double kStep = 0.5;
constexpr double kMaxOutput = 127.0;
constexpr double kMillivoltsPerPower = 12000.0 / 127.0;
constexpr double kDegreesPerCount = 360.0 / 1800.0; // red, raw encoder counts
constexpr std::uint32_t kPrintMs = 60;              // the controller drops text sent faster

double output = 0.0;
double raw_zero = 0.0;
std::uint32_t last_print = 0;
int line = 0;

double raw_counts() {
    std::uint32_t timestamp = 0;
    double sum = 0.0;
    int count = 0;
    for (const std::int32_t raw : arm_pivot_motor.get_raw_position_all(&timestamp)) {
        if (raw != PROS_ERR) {
            sum += raw;
            ++count;
        }
    }
    return count > 0 ? sum / count : NAN;
}

double arm_deg() {
    return (raw_counts() - raw_zero) * kDegreesPerCount + constants::kArmStartDeg;
}

}

void init() {
    arm_pivot_motor.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    arm_pivot_motor.move_voltage(0);
    raw_zero = raw_counts(); // the arm starts at constants::kArmStartDeg
}

void update() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        output = std::fmin(output + kStep, kMaxOutput);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        output = std::fmax(output - kStep, -kMaxOutput);
    }
    const bool on = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    arm_pivot_motor.move_voltage(on ? static_cast<std::int32_t>(std::lround(output * kMillivoltsPerPower)) : 0);

    const std::uint32_t now = pros::millis();
    if (now - last_print < kPrintMs) {
        return;
    }
    last_print = now;
    const double angle = arm_deg();
    const double s = std::sin(angle * 3.14159265358979323846 / 180.0);
    if (line == 0) {
        controller.print(0, 0, "out %5.1f %s     ", output, on ? "ON " : "off");
    } else if (line == 1) {
        controller.print(1, 0, "arm %6.1f deg   ", angle);
    } else if (std::fabs(s) > 0.2) {
        controller.print(2, 0, "kG = %6.1f      ", output / s);
    } else {
        controller.print(2, 0, "kG = (near vert)");
    }
    line = (line + 1) % 3;
}

}
