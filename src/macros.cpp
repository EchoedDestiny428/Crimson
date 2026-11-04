#include "macros.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include "subsystems/claw.hpp"
#include "subsystems/elevator/constants.hpp"
#include "subsystems/elevator/extension.hpp"
#include "subsystems/pivot.hpp"
#include "subsystems/toggle.hpp"
#include <atomic>
#include <array>
#include <cstddef>
#include <cstdint>

namespace macros {

namespace {

constexpr std::uint32_t kPollMs = 10;
constexpr std::uint32_t kPivotTimeoutMs = 3000;       // on timeout the rest of the command is skipped, so a stuck bar does not open the claw.
constexpr std::uint32_t kElevatorTimeoutMs = 2000;
constexpr std::uint32_t kClawTimeoutMs = subsystems::claw::kTravelMs + 200; // piston travel, plus a little.

constexpr std::array kScoreAngles{subsystems::pivot::kHighDeg, subsystems::pivot::kMidDeg,
                                  subsystems::pivot::kLowDeg}; // Right cycles this. It does not move.

std::atomic<std::uint32_t> generation{0};
std::atomic<std::size_t> score_index{0};

class Run {
public:
    static Run begin() {
        return Run{++generation};
    }

    bool active() const {
        return generation.load() == generation_;
    }

    template <typename Done>
    bool wait(Done done, std::uint32_t timeout_ms) const {
        const std::uint32_t start = pros::millis();
        while (active() && !done() && !pros::competition::is_disabled()) {
            if (pros::millis() - start >= timeout_ms) {
                return false;
            }
            pros::delay(kPollMs);
        }
        return active() && !pros::competition::is_disabled();
    }

private:
    explicit Run(std::uint32_t generation) : generation_(generation) {}

    std::uint32_t generation_;
};

bool bar_to(const Run& run, double motor_deg) {
    if (!run.active()) {
        return false;
    }
    subsystems::pivot::move_to(motor_deg);
    return run.wait(subsystems::pivot::is_settled, kPivotTimeoutMs);
}

void do_pickup(const Run& run) { // the string stays where the driver left it. Raise the carriage with L1 before this.
    if (!bar_to(run, subsystems::pivot::kPickupDeg)) {
        return;
    }
    subsystems::claw::close();
    if (!run.wait(subsystems::claw::is_settled, kClawTimeoutMs)) {
        return;
    }
    bar_to(run, subsystems::pivot::kCarryDeg);
}

void do_score(const Run& run, double angle) { // the string stays where the driver left it.
    if (!bar_to(run, angle)) {
        return;
    }
    subsystems::claw::open();
    if (!run.wait(subsystems::claw::is_settled, kClawTimeoutMs)) {
        return;
    }
    bar_to(run, subsystems::pivot::kCarryDeg);
}

void do_stow(const Run& run) {
    if (!bar_to(run, subsystems::pivot::kCarryDeg)) {
        return;
    }
    if (!run.active()) {
        return;
    }
    subsystems::elevator::extension::set_target(
        subsystems::elevator::constants::extension_cm_to_motor_deg(subsystems::elevator::constants::kExtensionHomeCm)); // stow is the only command that moves the string. Home is still 0 cm.
    run.wait(subsystems::elevator::extension::is_settled, kElevatorTimeoutMs);
}

}

void cancel() {
    ++generation;
}

void stop() {
    cancel();
    subsystems::elevator::extension::hold();
    subsystems::pivot::stop();
    subsystems::toggle::stop();
    subsystems::claw::stop();
}

void pickup() {
    do_pickup(Run::begin());
}

void score() {
    do_score(Run::begin(), kScoreAngles[score_index.load()]);
}

void stow() {
    do_stow(Run::begin());
}

void update() {
    static bool manual = false;
    const bool chain =
        controller.get_digital(controls::kElevatorUp) || controller.get_digital(controls::kElevatorDown);
    const bool bar = controller.get_digital(controls::kPivotOut) || controller.get_digital(controls::kPivotIn);
    const bool grip = controller.get_digital(controls::kClawClose) || controller.get_digital(controls::kClawOpen);
    const bool wheels = controller.get_digital(controls::kToggleIn);
    if (chain || bar || grip || wheels) {
        if (!manual) {
            cancel();
            manual = true;
        }
        return;
    }
    manual = false;

    if (controller.get_digital_new_press(controls::kNextHeight)) {
        score_index = (score_index.load() + 1) % kScoreAngles.size();
    }
    if (controller.get_digital_new_press(controls::kPickup)) {
        const Run run = Run::begin();
        pros::Task([run] { do_pickup(run); }, "Pickup");
    } else if (controller.get_digital_new_press(controls::kScore)) {
        const Run run = Run::begin();
        const double angle = kScoreAngles[score_index.load()];
        pros::Task([run, angle] { do_score(run, angle); }, "Score");
    } else if (controller.get_digital_new_press(controls::kStow)) {
        const Run run = Run::begin();
        pros::Task([run] { do_stow(run); }, "Stow");
    }
}

}
