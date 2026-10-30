#include "autons/autons.hpp"
#include "pros/screen.hpp"
#include "robotconfig.hpp"
#include "telemetry/telemetry.hpp"
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace autons {

namespace {

constexpr std::string_view kDefaultRoutine = "Test";

struct Routine {
    const char* name;
    void (*run)();
};

constexpr std::array kRoutines{
    Routine{"Test", match::test::run},
    Routine{"None", nullptr},
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

}

void select() {
    std::int32_t presses = pros::screen::touch_status().press_count;
    while (pros::competition::is_disabled()) {
        const auto touch = pros::screen::touch_status();
        if (touch.press_count != presses) {
            presses = touch.press_count;
            const auto tapped = display::Dashboard::auton_at(touch.x, touch.y, kRoutines.size());
            if (tapped) {
                selected = *tapped;
            }
        }
        pros::delay(kPollMs);
    }
}

void draw_selector() {
    dashboard.show_autons(kNames, selected, chassis.getPose());
}

void run_selected() {
    const Routine& routine = kRoutines[selected];
    chassis.armRawAnchor();
    telemetry::auton_started(routine.name);
    if (routine.run != nullptr) {
        routine.run();
    }
    telemetry::auton_finished();
}

const char* selected_name() {
    return kRoutines[selected].name;
}

}
