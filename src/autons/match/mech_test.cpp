#include "autons/autons.hpp"
#include "pros/screen.hpp"
#include "robotconfig.hpp"
#include "subsystems/claw.hpp"
#include "subsystems/elevator/constants.hpp"
#include "subsystems/elevator/extension.hpp"
#include "subsystems/pivot.hpp"
#include "subsystems/toggle.hpp"
#include <cmath>
#include <cstdio>

namespace autons::match::mech_test {

namespace {

void value(char* buf, std::size_t size, double reading) {
    if (std::isfinite(reading)) {
        std::snprintf(buf, size, "%6.0f", reading);
    } else {
        std::snprintf(buf, size, "%s", "  none");
    }
}

}

void draw() {
    char chain[8];
    char chain_cm[8];
    char bar[8];
    char flex[8];
    const double chain_deg = subsystems::elevator::extension::height();
    value(chain, sizeof chain, chain_deg);
    value(chain_cm, sizeof chain_cm, subsystems::elevator::constants::extension_motor_deg_to_cm(chain_deg));
    value(bar, sizeof bar, subsystems::pivot::get_pos());
    value(flex, sizeof flex, toggle_motor.get_actual_velocity());

    const char* piston = !subsystems::claw::is_settled() ? "moving"
                         : subsystems::claw::is_closed() ? "closed"
                                                         : "open";

    pros::screen::set_pen(0x1E1E24);
    pros::screen::fill_rect(0, 0, 480, 240);
    pros::screen::set_pen(0xF8F9FA);
    pros::screen::print(pros::E_TEXT_MEDIUM, 12, 6, "MECH  you drive");
    pros::screen::print(pros::E_TEXT_SMALL, 12, 36, "string %s deg  %s cm   + raises", chain, chain_cm);
    pros::screen::print(pros::E_TEXT_SMALL, 12, 52, "bar    %s deg    + swings the claw down", bar);
    pros::screen::print(pros::E_TEXT_SMALL, 12, 68, "piston %-6s       R1 closes, R2 opens", piston);
    pros::screen::print(pros::E_TEXT_SMALL, 12, 84, "flex   %s rpm    + pulls the piece in", flex);
    pros::screen::print(pros::E_TEXT_SMALL, 12, 112, "L1/L2 string     Up/Down bar");
    pros::screen::print(pros::E_TEXT_SMALL, 12, 128, "Left hold flex in");
    pros::screen::print(pros::E_TEXT_SMALL, 12, 144, "X pickup   Y score   B stow");
    pros::screen::print(pros::E_TEXT_SMALL, 12, 168, "X and Y should end with the bar near 0");
    pros::screen::print(pros::E_TEXT_SMALL, 12, 184, "and the string where you left it");
    pros::screen::print(pros::E_TEXT_SMALL, 12, 200, "B should end with the bar near 0 and the string at 0");
}

}
