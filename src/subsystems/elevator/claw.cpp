#include "subsystems/elevator/claw.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include <atomic>

namespace subsystems::elevator::claw {

namespace {

std::atomic<State> current_state{State::Empty};

}

void update() {
    if (controller.get_digital_new_press(controls::kClawPiston)) {
        toggle();
    }
    if (controller.get_digital_new_press(controls::kClawExtend)) {
        release();
    }
    if (controller.get_digital_new_press(controls::kClawRetract)) {
        grab();
    }
}

void open() {
    claw_piston.extend();
}

void close() {
    claw_piston.retract();
}

void toggle() {
    claw_piston.toggle();
    current_state = is_open() ? State::Empty : State::Grabbed;
}

bool is_open() {
    return claw_piston.is_extended();
}

std::int32_t distance_mm() {
    return claw_distance.get_distance();
}

void prime() {
    open();
    current_state = State::Primed;
}

void grab() {
    close();
    current_state = State::Grabbed;
}

void release() {
    open();
    current_state = State::Empty;
}

State state() {
    return current_state;
}

}
