#include "autons/autons.hpp"
#include "config/controls.hpp"
#include "robotconfig.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace autons {

namespace {

constexpr std::string_view kDefaultRoutine = "Snacky";

struct Routine {
    const char* name;
    void (*setup)();
    void (*run)();
};

constexpr std::array kRoutines{
    Routine{"Test", match::test::setup, match::test::run},
    Routine{"Snacky", match::snacky::setup, match::snacky::run},
    Routine{"SkillsV1", skills::v1::setup, skills::v1::run},
    Routine{"None", nullptr, nullptr},
};

constexpr std::size_t index_of(std::string_view name) {
    for (std::size_t i = 0; i < kRoutines.size(); ++i) {
        if (kRoutines[i].name == name) {
            return i;
        }
    }
    return kRoutines.size();
}

constexpr std::size_t kDefaultIndex = index_of(kDefaultRoutine);
static_assert(kDefaultIndex < kRoutines.size(), "kDefaultRoutine must match a name in kRoutines");

constexpr std::uint32_t kPollMs = 20;

std::size_t selected = kDefaultIndex;
bool prepared = false;
bool auton_ran = false;

void show_selected() {
    controller.print(2, 0, "Auton: %-8s", kRoutines[selected].name);
}

void prepare() {
    if (kRoutines[selected].setup != nullptr) {
        kRoutines[selected].setup();
    }
    prepared = true;
}

void change_selection(std::size_t index) {
    selected = index;
    show_selected();
    prepare();
}

}

void select() {
    show_selected();
    if (!prepared && !auton_ran) {
        prepare();
    }

    while (pros::competition::is_disabled()) {
        if (controller.get_digital_new_press(controls::kAutonNext)) {
            change_selection((selected + 1) % kRoutines.size());
        }
        if (controller.get_digital_new_press(controls::kAutonPrevious)) {
            change_selection((selected + kRoutines.size() - 1) % kRoutines.size());
        }
        pros::delay(kPollMs);
    }
}

void run_selected() {
    prepare();
    prepared = false;
    auton_ran = true;

    if (kRoutines[selected].run != nullptr) {
        kRoutines[selected].run();
    }
}

}
