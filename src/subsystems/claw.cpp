#include "subsystems/claw.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include <atomic>
#include <cstdint>

namespace subsystems::claw {

namespace {

std::atomic<bool> closed{false};
std::atomic<bool> moving{false};
std::atomic<std::uint32_t> since{0};

void command(bool next) {
    claw_piston.set_value(next ? kClosedLevel : !kClosedLevel);
    since = pros::millis();
    moving = true;
    closed = next;
}

}

void init() {
    open();                                         // boot open. Call close() here if the match starts gripped.
}

void update() {
    if (controller.get_digital_new_press(controls::kClawClose)) {
        close();
    } else if (controller.get_digital_new_press(controls::kClawOpen)) {
        open();
    }
}

void stop() {}

void close() {
    command(true);
}

void open() {
    command(false);
}

bool is_closed() {
    return closed.load();
}

bool is_settled() {
    if (!moving.load()) {
        return true;
    }
    if (pros::millis() - since.load() >= kTravelMs) {
        moving = false;
        return true;
    }
    return false;
}

bool wait_until_settled(std::uint32_t timeout_ms) {
    const std::uint32_t start = pros::millis();
    while (!is_settled()) {
        if (pros::millis() - start >= timeout_ms) {
            return false;
        }
        pros::delay(10);
    }
    return true;
}

}
