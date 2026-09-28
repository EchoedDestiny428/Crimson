#include "macros.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include "subsystems/pivot.hpp"
#include "util/system_check.hpp"
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

constexpr std::uint32_t kElevatorTimeoutMs = 650;
constexpr std::uint32_t kPivotTimeoutMs = 500;
constexpr std::uint32_t kDualPivotEndTimeoutMs = 500;
constexpr std::uint32_t kDualPickupDwellMs = 250;
constexpr std::uint32_t kPollMs = 10;

std::atomic<Mode> mode{Mode::Stowed};
std::atomic<bool> tilted{false};
std::atomic<bool> offset_on{true};
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
    ++generation;
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
    start_home();
    const Run run;
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
        run.wait_for(pivot::is_settled, kDualPivotEndTimeoutMs);
    }
}

void dual_pickup_height(double height) {
    const Run run;
    pivot::move_to(pivot::kDualPickupDeg);
    if (run.wait_for(pivot::is_settled, kPivotTimeoutMs) && run.pause(kDualPickupDwellMs)) {
        start_flip_out(height, pivot::kFlippedMotorDeg);
        finish_motion(run);
    }
}

void cancel() {
    ++generation;
}
void outtake_for_score(bool pressed) {
    static bool busy = false;
    if (!pressed || busy || mode != Mode::Deployed) {
        return;
    }
    busy = true;
    pros::Task([] {
        const Run run;
        pivot::move_to(pivot::kFlippedMotorDeg);
        if (!run.wait_for(pivot::is_settled, 100)) {
            busy = false;
            return;
        }
        if (!run.pause(250)) {
            busy = false;
            return;
        }
        pivot::move_to(pivot::kScoredMotorDeg);
        elevator::set_target(elevator::height() + elevator::kOffset);
        if (!run.pause(2000)) {
            busy = false;
            return;
        }
        pivot::move_to(pivot::kFlippedMotorDeg);
        elevator::snap_to_stage();
        busy = false;
    });
}


void press(bool up, bool down, bool down_pressed, bool offset_button) {
    if (offset_button) {
        offset_on = !offset_on;
        if (mode == Mode::Deployed) {
            elevator::set_target(elevator::target() + (offset_on ? elevator::kOffset : -elevator::kOffset));
        }
    }
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
        elevator::snap_to_stage(offset_on);
    }

    if (!down) {
        tilt_if_at_top();
    }
}

void update() {
    if (util::system_check::running()) {
        return;
    }
    press(controller.get_digital(controls::kElevatorUp), controller.get_digital(controls::kElevatorDown),
          controller.get_digital_new_press(controls::kElevatorDown), controller.get_digital_new_press(controls::kElevatorOffset));
    outtake_for_score(controller.get_digital(controls::kIntakeOut));
    if (controller.get_digital_new_press(controls::kHome)) {
        manual_active = false;
        start_home();
    }
}

void stop() {
    manual_active = false;
    elevator::hold();
}

}
