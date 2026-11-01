#include "subsystems/imu_peaks.hpp"
#include "robotconfig.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace subsystems::imu_peaks {

namespace {

constexpr std::uint32_t kSampleMs = 10;
constexpr std::uint32_t kPrintMs = 60; // the controller drops text sent faster than ~50 ms
constexpr auto kResetButton = pros::E_CONTROLLER_DIGITAL_A;

struct Peaks {
    double pitch = 0;
    double roll = 0;
};

// PROS returns PROS_ERR_F (infinity) if the IMU is unplugged.
void raise(double& peak, double value) {
    if (std::isfinite(value)) {
        peak = std::max(peak, std::fabs(value));
    }
}

void run() {
    const double flat_pitch = imu.get_pitch();
    const double flat_roll = imu.get_roll();

    Peaks peaks;
    int holds_at_reset = 0;
    std::uint32_t now = pros::millis();
    std::uint32_t last_print = 0;
    int line = 0;

    while (true) {
        if (controller.get_digital_new_press(kResetButton)) {
            peaks = {};
            holds_at_reset = chassis.tiltHoldCount();
        }

        raise(peaks.pitch, imu.get_pitch() - flat_pitch);
        raise(peaks.roll, imu.get_roll() - flat_roll);

        if (now - last_print >= kPrintMs) {
            last_print = now;
            if (line == 0) {
                const lemlib::Pose zach = chassis.getUnguardedPose();
                controller.print(0, 0, "Z%6.1f%6.1f%4.0f ", zach.x, zach.y,
                                 std::fmod(std::fmod(zach.theta, 360.0f) + 360.0f, 360.0f));
            } else if (line == 1) {
                const lemlib::Pose ours = chassis.getPose();
                controller.print(1, 0, "G%6.1f%6.1f #%-3d", ours.x, ours.y,
                                 chassis.tiltHoldCount() - holds_at_reset);
            } else {
                controller.print(2, 0, "P%5.1f R%5.1f   ", peaks.pitch, peaks.roll);
            }
            line = (line + 1) % 3;
        }

        pros::Task::delay_until(&now, kSampleMs);
    }
}

}

void start() {
    pros::Task task(run, "IMU peaks");
}

}
