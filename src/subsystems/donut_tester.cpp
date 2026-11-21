#include "subsystems/donut_tester.hpp"
#include "robotconfig.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace subsystems::donut_tester {

namespace {

constexpr int kStep = 5;
constexpr std::uint32_t kRepeatDelayMs = 400; // hold a speed button this long and it starts repeating
constexpr std::uint32_t kRepeatEveryMs = 120;
constexpr std::uint32_t kPrintMs = 60;        // the controller drops text sent faster than ~50 ms

const crimson::DonutParams kDefaults{};
int outer = kDefaults.fastSpeed;
int inner = kDefaults.slowSpeed;

// A button that steps once on press, then repeats while held.
class Repeat {
public:
    explicit Repeat(pros::controller_digital_e_t button) : button_(button) {}

    bool fired(std::uint32_t now) {
        const bool down = controller.get_digital(button_);
        if (!down) {
            held_ = false;
            return false;
        }
        if (!held_) {
            held_ = true;
            pressed_ = now;
            last_ = now;
            return true;
        }
        if (now - pressed_ >= kRepeatDelayMs && now - last_ >= kRepeatEveryMs) {
            last_ = now;
            return true;
        }
        return false;
    }

private:
    pros::controller_digital_e_t button_;
    bool held_ = false;
    std::uint32_t pressed_ = 0;
    std::uint32_t last_ = 0;
};

Repeat outer_up(pros::E_CONTROLLER_DIGITAL_X);
Repeat outer_down(pros::E_CONTROLLER_DIGITAL_B);
Repeat inner_up(pros::E_CONTROLLER_DIGITAL_UP);
Repeat inner_down(pros::E_CONTROLLER_DIGITAL_DOWN);

std::uint32_t last_print = 0;
int line = 0;
int left_cmd = 0;
int right_cmd = 0;
const char* running = "";

// Same split as Chassis::donut at full turn, backwards (back into the goal): push = -(outer - inner) / 2,
// spin = (outer + inner) / 2. Clockwise: left = push + spin = inner, right = push - spin = -outer.
void sides(bool clockwise, int& left, int& right) {
    left = clockwise ? inner : -outer;
    right = clockwise ? -outer : inner;
}

void print(std::uint32_t now) {
    if (now - last_print < kPrintMs) {
        return;
    }
    last_print = now;
    if (line == 0) {
        controller.print(0, 0, "OUT %4d  IN %4d   ", outer, inner);
    } else if (line == 1) {
        const double push = (outer - inner) / 2.0;
        const double spin = (outer + inner) / 2.0;
        controller.print(1, 0, "push %3.0f spin %3.0f  ", push, spin);
    } else if (running[0] != '\0') {
        controller.print(2, 0, "%s L%4d R%4d   ", running, left_cmd, right_cmd);
    } else {
        int left = 0;
        int right = 0;
        sides(true, left, right);
        controller.print(2, 0, "R1: L%4d R%4d   ", left, right);
    }
    line = (line + 1) % 3;
}

}

bool update() {
    const std::uint32_t now = pros::millis();
    if (outer_up.fired(now)) {
        outer = std::min(outer + kStep, 127);
    }
    if (outer_down.fired(now)) {
        outer = std::max(outer - kStep, 0);
    }
    if (inner_up.fired(now)) {
        inner = std::min(inner + kStep, 127);
    }
    if (inner_down.fired(now)) {
        inner = std::max(inner - kStep, -127);
    }

    const bool cw = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    const bool ccw = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    const bool driving = cw != ccw;
    if (driving) {
        sides(cw, left_cmd, right_cmd);
        left_motors.move(left_cmd);
        right_motors.move(right_cmd);
        running = cw ? "CW " : "CCW";
    } else {
        left_cmd = 0;
        right_cmd = 0;
        running = "";
    }
    print(now);
    return driving;
}

}
