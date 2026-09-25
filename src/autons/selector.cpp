#include "autons/autons.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace autons {

namespace {

struct Routine {
    const char* name;
    void (*run)();
};

constexpr std::array kRoutines{
    Routine{"Test", match::test},
    Routine{"SkillsV1", skills::v1},
    Routine{"None", nullptr},
};

constexpr std::uint32_t kPollMs = 20;

std::size_t selected = 0;

void show_selected() {
    controller.print(2, 0, "Auton: %-8s", kRoutines[selected].name);
}

}

void select() {
    show_selected();
    while (pros::competition::is_disabled()) {
        if (controller.get_digital_new_press(controls::kAutonNext)) {
            selected = (selected + 1) % kRoutines.size();
            show_selected();
        }
        if (controller.get_digital_new_press(controls::kAutonPrevious)) {
            selected = (selected + kRoutines.size() - 1) % kRoutines.size();
            show_selected();
        }
        pros::delay(kPollMs);
    }
}

void run_selected() {
    if (kRoutines[selected].run != nullptr) {
        kRoutines[selected].run();
    }
}

}
