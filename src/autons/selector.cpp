#include "autons/autons.hpp"
#include "pros/screen.hpp"
#include "robotconfig.hpp"
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace autons {

namespace {

constexpr std::string_view kDefaultRoutine = "Cali";

struct Routine {
    const char* name;
    void (*setup)();
    void (*run)();
};

constexpr std::array kRoutines{
    Routine{"Test", match::test::setup, match::test::run},
    Routine{"Snacky", match::snacky::setup, match::snacky::run},
    Routine{"TeamBad", match::teambad::setup, match::teambad::run},


    Routine{"SkillsV1", skills::v1::setup, skills::v1::run},
    Routine{"Cali", cali::setup, cali::run},
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

constexpr auto kNames = [] {
    std::array<const char*, kRoutines.size()> names{};
    for (std::size_t i = 0; i < kRoutines.size(); ++i) {
        names[i] = kRoutines[i].name;
    }
    return names;
}();

constexpr std::uint32_t kPollMs = 20;

std::atomic<std::size_t> selected{kDefaultIndex};
bool prepared = false;
bool auton_ran = false;

void prepare() {
    if (kRoutines[selected].setup != nullptr) {
        kRoutines[selected].setup();
    }
    prepared = true;
}

void change_selection(std::size_t index) {
    selected = index;
    prepare();
}

}

void select() {
    if (!prepared && !auton_ran) {
        prepare();
    }

    std::int32_t presses = pros::screen::touch_status().press_count;
    while (pros::competition::is_disabled()) {
        const auto touch = pros::screen::touch_status();
        if (touch.press_count != presses) {
            presses = touch.press_count;
            const auto tapped = display::Dashboard::auton_at(touch.x, touch.y, kRoutines.size());
            if (tapped && *tapped != selected) {
                change_selection(*tapped);
            }
        }
        pros::delay(kPollMs);
    }
}

void draw_selector() {
    const lemlib::Pose pose = chassis.getPose();
    dashboard.show_autons(kNames, selected, {pose.x, pose.y, pose.theta});
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
