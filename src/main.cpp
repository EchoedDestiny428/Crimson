#include "main.h"
#include <array>
#include <cstdint>

// Motor tester: pick any ports on the brain screen and run every selected
// motor at the same power, forward or reverse.

namespace {

constexpr int kPorts = 21;
constexpr int kCols = 7;
constexpr int kSpeedStep = 10;
constexpr std::uint32_t kLoopMs = 20;
constexpr std::uint32_t kDetectEveryMs = 500;

constexpr std::uint32_t kBlack = 0x00000000;
constexpr std::uint32_t kWhite = 0x00FFFFFF;
constexpr std::uint32_t kGray = 0x00555555;
constexpr std::uint32_t kDarkGray = 0x00303030;
constexpr std::uint32_t kGreen = 0x0000A651;
constexpr std::uint32_t kRed = 0x00C0392B;
constexpr std::uint32_t kBlue = 0x002E6FB5;
constexpr std::uint32_t kAmber = 0x00D68910;

struct Box {
    std::int16_t x0, y0, x1, y1;
    bool contains(std::int16_t x, std::int16_t y) const {
        return x >= x0 && x <= x1 && y >= y0 && y <= y1;
    }
};

constexpr Box kMinus{8, 162, 64, 226};
constexpr Box kSpeedBox{70, 162, 160, 226};
constexpr Box kPlus{166, 162, 222, 226};
constexpr Box kDir{230, 162, 316, 226};
constexpr Box kRun{324, 162, 410, 226};
constexpr Box kClear{418, 162, 472, 226};

std::array<bool, kPorts> selected{};
std::array<bool, kPorts> has_motor{};
std::array<bool, kPorts> commanded{};
int speed_pct = 50;
bool forward = true;
bool running = false;

Box port_box(int index) {
    const int col = index % kCols;
    const int row = index / kCols;
    const auto x0 = static_cast<std::int16_t>(8 + col * 66);
    const auto y0 = static_cast<std::int16_t>(28 + row * 44);
    return {x0, y0, static_cast<std::int16_t>(x0 + 60), static_cast<std::int16_t>(y0 + 38)};
}

void button(const Box& b, std::uint32_t fill, const char* label) {
    pros::screen::set_pen(fill);
    pros::screen::fill_rect(b.x0, b.y0, b.x1, b.y1);
    pros::screen::set_pen(kWhite);
    pros::screen::set_eraser(fill);
    pros::screen::print(pros::E_TEXT_MEDIUM, b.x0 + 8, (b.y0 + b.y1) / 2 - 8, "%s", label);
}

int selected_count() {
    int n = 0;
    for (bool s : selected) {
        n += s ? 1 : 0;
    }
    return n;
}

void draw() {
    pros::screen::set_eraser(kBlack);
    pros::screen::erase();

    pros::screen::set_pen(kWhite);
    pros::screen::print(pros::E_TEXT_SMALL, 8, 6, "MOTOR TESTER   %d selected   %d%% %s   %s", selected_count(), speed_pct,
                        forward ? "FWD" : "REV", running ? "RUNNING" : "stopped");

    for (int i = 0; i < kPorts; ++i) {
        const Box b = port_box(i);
        const std::uint32_t fill = selected[i] ? kGreen : has_motor[i] ? kDarkGray : kBlack;
        pros::screen::set_pen(fill);
        pros::screen::fill_rect(b.x0, b.y0, b.x1, b.y1);
        pros::screen::set_pen(has_motor[i] ? kWhite : kGray);
        pros::screen::draw_rect(b.x0, b.y0, b.x1, b.y1);
        pros::screen::set_eraser(fill);
        pros::screen::print(pros::E_TEXT_MEDIUM, b.x0 + 6, b.y0 + 10, "%d%s", i + 1, has_motor[i] ? " M" : "");
    }

    button(kMinus, kGray, " -");
    pros::screen::set_pen(kWhite);
    pros::screen::draw_rect(kSpeedBox.x0, kSpeedBox.y0, kSpeedBox.x1, kSpeedBox.y1);
    pros::screen::set_eraser(kBlack);
    pros::screen::print(pros::E_TEXT_MEDIUM, kSpeedBox.x0 + 18, kSpeedBox.y0 + 24, "%3d%%", speed_pct);
    button(kPlus, kGray, " +");
    button(kDir, forward ? kBlue : kAmber, forward ? "FWD" : "REV");
    button(kRun, running ? kRed : kGreen, running ? "STOP" : "RUN");
    button(kClear, kGray, "CLR");
}

void detect_motors() {
    for (int i = 0; i < kPorts; ++i) {
        has_motor[i] = pros::Device::get_plugged_type(static_cast<std::uint8_t>(i + 1)) == pros::DeviceType::motor;
    }
}

bool handle_touch(std::int16_t x, std::int16_t y) {
    for (int i = 0; i < kPorts; ++i) {
        if (port_box(i).contains(x, y)) {
            selected[i] = !selected[i];
            return true;
        }
    }
    if (kMinus.contains(x, y)) {
        speed_pct = speed_pct - kSpeedStep < 0 ? 0 : speed_pct - kSpeedStep;
    } else if (kPlus.contains(x, y)) {
        speed_pct = speed_pct + kSpeedStep > 100 ? 100 : speed_pct + kSpeedStep;
    } else if (kDir.contains(x, y)) {
        forward = !forward;
    } else if (kRun.contains(x, y)) {
        running = !running;
    } else if (kClear.contains(x, y)) {
        selected.fill(false);
    } else {
        return false;
    }
    return true;
}

void apply_motors() {
    const int power = (forward ? 1 : -1) * speed_pct * 127 / 100;
    for (int i = 0; i < kPorts; ++i) {
        const auto port = static_cast<std::int8_t>(i + 1);
        if (running && selected[i]) {
            pros::c::motor_move(port, power);
            commanded[i] = true;
        } else if (commanded[i]) {
            pros::c::motor_move(port, 0);
            commanded[i] = false;
        }
    }
}

void run_tester() {
    detect_motors();
    draw();
    std::int32_t presses = pros::screen::touch_status().press_count;
    std::uint32_t last_detect = pros::millis();

    while (true) {
        bool changed = false;
        const auto touch = pros::screen::touch_status();
        if (touch.press_count != presses) {
            presses = touch.press_count;
            changed = handle_touch(touch.x, touch.y);
        }
        if (pros::millis() - last_detect >= kDetectEveryMs) {
            last_detect = pros::millis();
            const auto before = has_motor;
            detect_motors();
            changed = changed || before != has_motor;
        }
        apply_motors();
        if (changed) {
            draw();
        }
        pros::delay(kLoopMs);
    }
}

}

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    run_tester();
}
