#include "macros.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include "subsystems/pivot.hpp"
#include <algorithm>
#include <atomic>
#include <cstdint>

namespace macros {

namespace elevator = subsystems::elevator;
namespace pivot = subsystems::pivot;

namespace {

enum class Mode { Stowed, Deployed };

constexpr int kManualPower = 127;
constexpr double kTopArrivalTolerance = 50.0;

constexpr std::uint32_t kElevatorTimeoutMs = 2000;
constexpr std::uint32_t kPivotTimeoutMs = 1000;
constexpr std::uint32_t kDualPickupDwellMs = 200;
constexpr std::uint32_t kPollMs = 10;

std::atomic<Mode> mode{Mode::Stowed};
std::atomic<bool> tilted{false};
std::atomic<std::uint32_t> generation{0};
bool manual_active = false;

class Run {
public:
    Run() : generation_(generation) {}

    bool active() const {
        return generation == generation_;
    }

    template <typename Done>
    bool wait_for(Done done, std::uint32_t timeout_ms) const {
        const std::uint32_t start = pros::millis();
        while (active() && !done() && pros::millis() - start < timeout_ms) {
            pros::delay(kPollMs);
        }
        return active();
    }

    bool pause(std::uint32_t ms) const {
        return wait_for([] { return false; }, ms);
    }

private:
    std::uint32_t generation_;
};

void tilt_if_at_top() {
    if (!tilted && elevator::height() >= elevator::top_stage() - kTopArrivalTolerance) {
        tilted = true;
        pivot::move_to(pivot::kTopMotorDeg);
    }
}

void untilt_pivot() {
    if (tilted) {
        tilted = false;
        pivot::move_to(pivot::kFlippedMotorDeg);
    }
}

void start_flip_out(double height, double pivot_motor_deg) {
    mode = Mode::Deployed;
    tilted = false;
    pivot::move_to(pivot_motor_deg);
    elevator::set_target_below_home(std::max(height, elevator::kFlipOutMin));
}

void start_home() {
    mode = Mode::Stowed;
    tilted = false;
    pivot::move_to(pivot::kHomeMotorDeg);
    elevator::set_target(elevator::kHome);
}

void finish_motion(const Run& run, bool auto_tilt = true) {
    if (!run.wait_for(elevator::is_settled, kElevatorTimeoutMs)) {
        return;
    }
    if (auto_tilt && mode == Mode::Deployed) {
        tilt_if_at_top();
    }
    run.wait_for(pivot::is_settled, kPivotTimeoutMs);
}

}

void flip_out(double height, double pivot_motor_deg) {
    const Run run;
    start_flip_out(height, pivot_motor_deg);
    finish_motion(run, pivot_motor_deg == pivot::kFlippedMotorDeg);
}

void go_to(double height) {
    if (height <= elevator::kHome) {
        home();
        return;
    }
    if (mode == Mode::Stowed) {
        flip_out(height);
        return;
    }

    const Run run;
    if (height < elevator::top_stage()) {
        untilt_pivot();
    }
    elevator::set_target(height);
    finish_motion(run);
}

void home() {
    const Run run;
    start_home();
    finish_motion(run);
}

void dual_setup() {
    const Run run;
    mode = Mode::Deployed;
    tilted = false;
    elevator::set_target_below_home(elevator::kDualSetup);
    if (!run.wait_for(elevator::is_settled, kElevatorTimeoutMs)) {
        return;
    }
    pivot::move_to(pivot::kDualSetupDeg);
    run.wait_for(pivot::is_settled, kPivotTimeoutMs);
}

void dual_pickup() {
    const Run run;
    pivot::move_to(pivot::kDualPickupDeg);
    if (run.wait_for(pivot::is_settled, kPivotTimeoutMs) && run.pause(kDualPickupDwellMs)) {
        pivot::move_to(pivot::kFlippedMotorDeg);
        run.wait_for(pivot::is_settled, kPivotTimeoutMs);
    }
}

void cancel() {
    ++generation;
}

void update() {
    const bool up = controller.get_digital(controls::kElevatorUp);
    const bool down = controller.get_digital(controls::kElevatorDown);
    const bool down_pressed = controller.get_digital_new_press(controls::kElevatorDown);

    if (mode == Mode::Stowed) {
        if (up) {
            start_flip_out(elevator::kFlipOut, pivot::kFlippedMotorDeg);
        } else if (down_pressed) {
            start_home();
        }
        return;
    }

    if (down && elevator::height() < elevator::kFlipOut) {
        manual_active = false;
        start_home();
        return;
    }

    if (up || down) {
        if (down) {
            untilt_pivot();
        }
        manual_active = true;
        elevator::set_manual(up ? kManualPower : -kManualPower);
    } else if (manual_active) {
        manual_active = false;
        elevator::snap_to_stage();
    }

    if (!down) {
        tilt_if_at_top();
    }
}

void stop() {
    manual_active = false;
    elevator::hold();
}

}
