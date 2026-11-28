#include "subsystems/arm_tune.hpp"
#include "config/controls.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "robotconfig.hpp"
#include "subsystems/elevator/arm_pivot.hpp"
#include "subsystems/elevator/claw.hpp"
#include "subsystems/elevator/constants.hpp"
#include "subsystems/elevator/extension.hpp"
#include "subsystems/elevator/macros.hpp"
#include "telemetry/logger.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace subsystems::arm_tune {

namespace {

namespace arm = elevator::arm_pivot;
namespace claw = elevator::claw;
namespace constants = elevator::constants;
namespace extension = elevator::extension;
namespace macros = elevator::macros;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// The sweep. Run 42 (fade x brake) showed those can't stop the overshoot: past the top the arm and cup-pin run ahead of
// the motor into the arm's slack, where its encoder can't see them. Run 48 (end-of-swing slowdowns) found the last 15%
// at 40% speed fixes L1, L2 and L2N (0-3.5 deg, released sooner too), but L3 still overshot 8-15 deg and often stayed
// 7-14 deg past: the hold floor (kMinPower 30, -2.8 V) can barely lift an arm and cup-pin back up against gravity
// there. So this sweep tries stronger floors with those slowdowns, gentlest floor first, L3 twice per setting (it
// varied a lot). The fade and brake stay at whatever they're set to (5 / 0.3).
struct Combo {
    double finish;
    double speed;
    double hold;
};
constexpr std::array<Combo, 6> kCombos{{
    {0.15, 0.4, 30.0},
    {0.20, 0.4, 30.0},
    {0.15, 0.4, 45.0},
    {0.20, 0.4, 45.0},
    {0.15, 0.4, 60.0},
    {0.20, 0.4, 60.0},
}};
constexpr int kSwings = 4;
constexpr std::array<int, 5> kSweepSwings{0, 1, 2, 2, 3}; // L1, L2, L3, L3 again, L2N
constexpr std::size_t kMaxRows = kCombos.size() * kSweepSwings.size();
constexpr const char* kSwingNames[kSwings] = {"L1", "L2", "L3", "L2N"};
// Manual: L2 cycles the finish speed through these.
constexpr std::array<double, 4> kFinishSpeeds{0.3, 0.4, 0.5, 0.6};

// Measuring a swing (see arm_tune.hpp).
constexpr double kWindowDeg = 4.0;    // arrived(), as the auton has it
constexpr double kStillRpm = 10.0;    // arrived_still(), as the auton has it
constexpr std::uint32_t kAfterReadyMs = 600;
constexpr std::uint32_t kSwingMaxMs = 2500;
constexpr std::size_t kMaxSamples = kSwingMaxMs / 10 + 10;
constexpr std::size_t kLeanSamples = 20; // the lean: where the robot's tilt settles, over the swing's last 200 ms
constexpr double kTiltStopDeg = 10.0;    // a swing that tips the robot this far (from where it started) ends the sweep
// The pick: every swing ready, ending within kEndOkDeg, rocking at most kRockOkDeg past its lean.
constexpr double kEndOkDeg = 2.0;
constexpr double kRockOkDeg = 3.0;

// Between swings: home with the lift at home_lift_cm(): the carry height while the claw holds a cup-pin, where an auton
// pickup leaves it too (it lifts to carry height after the grab), so every swing starts from there. Then wait for the
// robot and arm to be still.
constexpr double kLiftNearCm = 1.0;
constexpr std::uint32_t kHomeMaxMs = 3000;
constexpr double kStillDps = 3.0;
constexpr double kStillArmDeg = 2.0;
constexpr std::uint32_t kStillMs = 300;
constexpr std::uint32_t kStillMaxMs = 3000;

constexpr int kAbortStick = 30;
constexpr std::uint32_t kPollMs = 10;
constexpr std::uint32_t kPrintMs = 60; // the controller drops text sent faster than ~50 ms

struct Result {
    double over = 0.0;
    double to4 = kNaN;
    double ready = kNaN;
    double end = kNaN;
    double lean = 0.0; // where the robot's tilt settled (deg from where it started): the weight moved
    double rock = 0.0; // the furthest it swung past that lean (deg): the momentum
    bool aborted = false;
    bool tipped = false;
};

struct Row {
    double fade;
    double brake;
    double finish;
    double finish_speed;
    double hold;
    int swing;
    Result result;
};

int selected = 2; // L3 first: the one overshooting most
bool out = false; // manual: at the swing's target (true) or home
std::uint32_t carry_at_ms = 0; // A closed the claw: when to lift to the carry height (0: nothing waiting)
int swing_count = 0;
bool have_last = false;
std::array<Row, kMaxRows> rows{};
std::size_t row_count = 0;
std::array<float, kMaxSamples> sample_pitch{};
std::array<float, kMaxSamples> sample_roll{};

// ---- The controller (three lines, one sent per kPrintMs) ----

std::array<char[20], 3> lines{};
std::array<char[20], 3> sent{};
std::uint32_t printed_at = 0;
int next_line = 0;

void set_line(int line, const char* text) {
    std::snprintf(lines[line], sizeof(lines[line]), "%-18.18s", text);
}

void flush() {
    const std::uint32_t now = pros::millis();
    if (now - printed_at < kPrintMs) {
        return;
    }
    for (int i = 0; i < 3; ++i) {
        const int line = (next_line + i) % 3;
        if (std::strncmp(lines[line], sent[line], sizeof(lines[line])) != 0) {
            controller.print(line, 0, "%s", lines[line]);
            std::memcpy(sent[line], lines[line], sizeof(lines[line]));
            printed_at = now;
            next_line = (line + 1) % 3;
            return;
        }
    }
}

// e.g. "e15@40 h45 L3 C": the last 15% of the swing at 40% speed, hold floor 45/127, swing L3, claw closed.
void show_settings() {
    char text[32];
    std::snprintf(text, sizeof(text), "e%02.0f@%02.0f h%.0f %s %s", arm::swing_finish_fraction() * 100.0,
                  arm::swing_finish_speed_fraction() * 100.0, arm::hold_power(), kSwingNames[selected],
                  claw::is_closed() ? "C" : "O");
    set_line(0, text);
}

void show_result(const Result& r) {
    char text[32];
    if (r.aborted) {
        std::snprintf(text, sizeof(text), "stopped");
    } else {
        std::snprintf(text, sizeof(text), "ov%.1f r%.2f k%.1f", r.over, r.ready, r.rock);
    }
    set_line(1, text);
}

void status(const char* text) {
    set_line(2, text);
}

// ---- Measuring ----

bool stick_moved() {
    return std::abs(controller.get_analog(controls::kThrottle)) > kAbortStick ||
           std::abs(controller.get_analog(controls::kTurn)) > kAbortStick;
}

void stop_drive() {
    left_motors.brake();
    right_motors.brake();
}

// Polls until done() or timeout_ms. False only if a stick stopped it (a timeout moves on).
template <typename Done> bool wait_for(Done done, std::uint32_t timeout_ms) {
    const std::uint32_t start = pros::millis();
    while (!done() && pros::millis() - start < timeout_ms) {
        if (stick_moved()) {
            return false;
        }
        flush();
        pros::delay(kPollMs);
    }
    return true;
}

bool lift_near(double cm) {
    return std::fabs(macros::pose_now().lift_cm - cm) < kLiftNearCm;
}

// The robot (gyro) and arm (on its target) have been still for kStillMs.
bool wait_still() {
    std::uint32_t still_since = 0;
    return wait_for(
        [&still_since] {
            const pros::imu_gyro_s_t rate = imu.get_gyro_rate();
            const auto calm = [](double dps) { return !std::isfinite(dps) || std::fabs(dps) < kStillDps; };
            const bool still = calm(rate.x) && calm(rate.y) && calm(rate.z) &&
                               std::fabs(arm::get_pos() - arm::target()) < kStillArmDeg;
            const std::uint32_t now = pros::millis();
            if (!still) {
                still_since = 0;
                return false;
            }
            if (still_since == 0) {
                still_since = now == 0 ? 1 : now;
            }
            return now - still_since >= kStillMs;
        },
        kStillMaxMs);
}

bool go_home() {
    stop_drive();
    status("homing");
    macros::home_async();
    if (!wait_for([] { return !arm::homing() && lift_near(macros::target_now().lift_cm); }, kHomeMaxMs)) {
        return false;
    }
    return wait_still();
}

// The Default auton's swings at their real lifts. For L1 (1.25 cm) and L3 (0 cm), aim() itself holds the lift at the
// carry height until the arm has swung clear of the floor (60 deg), then lowers it during the swing, as in the auton.
macros::Pose pose_for(int swing) {
    macros::Pose pose;
    switch (swing) {
    case 0:
        pose = macros::stage(1); // the Default auton's preload: L1, lift 1.75 cm lower
        pose.lift_cm -= 1.75;
        break;
    case 1:
        pose = macros::stage(2, 0);
        break;
    case 2:
        pose = macros::stage(3, 0);
        break;
    default:
        pose = macros::stage(2, 1); // L2 on neutral goal 1a: lift 27 cm
        break;
    }
    return pose;
}

// One swing out, the way the auton does it (aim(): lift and arm together, from home at carry height).
Result swing_out(int swing) {
    stop_drive();
    telemetry::mark(9000 + ++swing_count);
    Result r;
    const double start_arm = macros::pose_now().arm_deg;
    const double pitch0 = imu.get_pitch();
    const double roll0 = imu.get_roll();
    macros::aim(pose_for(swing));
    const double target = macros::target_now().arm_deg; // with the slack adjustment
    const double dir = target >= start_arm ? 1.0 : -1.0;
    const std::uint32_t start = pros::millis();
    std::uint32_t ready_at = 0;
    std::size_t samples = 0;
    while (true) {
        pros::delay(kPollMs);
        const std::uint32_t now = pros::millis();
        const double elapsed = (now - start) / 1000.0;
        const double error = macros::pose_now().arm_deg - target;
        const double rpm = arm_pivot_motor.get_actual_velocity();
        r.over = std::max(r.over, error * dir);
        if (std::fabs(error) < kWindowDeg) {
            if (std::isnan(r.to4)) {
                r.to4 = elapsed;
            }
            if (std::isnan(r.ready) && (!std::isfinite(rpm) || std::fabs(rpm) < kStillRpm)) {
                r.ready = elapsed;
                ready_at = now;
            }
        }
        const double pitch = imu.get_pitch() - pitch0;
        const double roll = imu.get_roll() - roll0;
        if (std::isfinite(pitch) && std::isfinite(roll) && samples < kMaxSamples) {
            sample_pitch[samples] = static_cast<float>(pitch);
            sample_roll[samples] = static_cast<float>(roll);
            ++samples;
            if (std::hypot(pitch, roll) > kTiltStopDeg) {
                r.tipped = true;
                break;
            }
        }
        if (stick_moved()) {
            r.aborted = true;
            break;
        }
        if ((ready_at != 0 && now - ready_at >= kAfterReadyMs) || now - start >= kSwingMaxMs) {
            break;
        }
        flush();
    }
    r.end = std::fabs(macros::pose_now().arm_deg - target);
    // Lean: the tilt it settled at (the last kLeanSamples); rock: the furthest it swung past that.
    if (samples > 0) {
        const std::size_t from = samples > kLeanSamples ? samples - kLeanSamples : 0;
        double lean_pitch = 0.0;
        double lean_roll = 0.0;
        for (std::size_t i = from; i < samples; ++i) {
            lean_pitch += sample_pitch[i];
            lean_roll += sample_roll[i];
        }
        lean_pitch /= static_cast<double>(samples - from);
        lean_roll /= static_cast<double>(samples - from);
        r.lean = std::hypot(lean_pitch, lean_roll);
        for (std::size_t i = 0; i < samples; ++i) {
            r.rock = std::max(r.rock, std::hypot(sample_pitch[i] - lean_pitch, sample_roll[i] - lean_roll));
        }
    }
    return r;
}

// ---- Results file ----

constexpr const char* kResultsPath = "/usd/arm_tune3.csv";

void save_rows(const Row* first, std::size_t count, const char* kind) {
    if (pros::usd::is_installed() != 1 || count == 0) {
        return;
    }
    bool fresh = true;
    if (std::FILE* existing = std::fopen(kResultsPath, "r")) {
        fresh = false;
        std::fclose(existing);
    }
    std::FILE* file = std::fopen(kResultsPath, "a");
    if (file == nullptr) {
        return;
    }
    if (fresh) {
        std::fprintf(file, "boot_ms,kind,fade_deg,brake,finish_frac,finish_speed_frac,hold_power,swing,overshoot_deg,to4_s,"
                           "ready_s,end_deg,lean_deg,rock_deg,stopped\n");
    }
    const unsigned long boot_ms = pros::millis();
    for (std::size_t i = 0; i < count; ++i) {
        const Row& row = first[i];
        const Result& r = row.result;
        std::fprintf(file, "%lu,%s,%.1f,%.2f,%.2f,%.2f,%.0f,%s,%.2f,%.3f,%.3f,%.2f,%.2f,%.2f,%d\n", boot_ms, kind, row.fade,
                     row.brake, row.finish, row.finish_speed, row.hold, kSwingNames[row.swing], r.over, r.to4, r.ready,
                     r.end, r.lean, r.rock, r.aborted || r.tipped ? 1 : 0);
    }
    std::fclose(file);
}

// ---- The sweep ----

bool usable(const Result& r) {
    return !r.aborted && !r.tipped && std::isfinite(r.ready) && r.end <= kEndOkDeg && r.rock <= kRockOkDeg;
}

void sweep() {
    const double finish_before = arm::swing_finish_fraction();
    const double speed_before = arm::swing_finish_speed_fraction();
    const double hold_before = arm::hold_power();
    row_count = 0;
    bool stopped = false;
    for (const Combo& combo : kCombos) {
        for (const int swing : kSweepSwings) {
            arm::set_swing_finish(combo.finish, combo.speed);
            arm::set_hold_power(combo.hold);
            char text[32];
            std::snprintf(text, sizeof(text), "e%02.0f@%02.0f h%.0f %s", combo.finish * 100.0, combo.speed * 100.0,
                          combo.hold, kSwingNames[swing]);
            set_line(0, text);
            if (!go_home()) {
                stopped = true;
                break;
            }
            std::snprintf(text, sizeof(text), "sweep %u/%u", static_cast<unsigned>(row_count + 1),
                          static_cast<unsigned>(kMaxRows));
            status(text);
            const Result r = swing_out(swing);
            rows[row_count++] = {arm::swing_fade(), arm::swing_brake(), combo.finish, combo.speed, combo.hold, swing, r};
            show_result(r);
            if (r.aborted || r.tipped) {
                stopped = true;
                status(r.tipped ? "tipped: stopped" : "stopped");
                break;
            }
        }
        if (stopped) {
            break;
        }
    }
    go_home();
    save_rows(rows.data(), row_count, "sweep");

    // The pick: the setting whose swings (L3 both times) are all usable, with the shortest total time to release.
    double best_sum = std::numeric_limits<double>::infinity();
    const Combo* best = nullptr;
    std::array<double, kSwings> best_over{};
    for (const Combo& combo : kCombos) {
        double sum = 0.0;
        std::size_t good = 0;
        std::array<double, kSwings> over{};
        for (std::size_t i = 0; i < row_count; ++i) {
            const Row& row = rows[i];
            if (row.finish == combo.finish && row.finish_speed == combo.speed && row.hold == combo.hold &&
                usable(row.result)) {
                sum += row.result.ready;
                over[row.swing] = std::max(over[row.swing], row.result.over);
                ++good;
            }
        }
        if (good == kSweepSwings.size() && sum < best_sum) {
            best_sum = sum;
            best = &combo;
            best_over = over;
        }
    }
    char text[32];
    if (best != nullptr) {
        arm::set_swing_finish(best->finish, best->speed);
        arm::set_hold_power(best->hold);
        std::snprintf(text, sizeof(text), "best e%02.0f@%02.0f h%.0f", best->finish * 100.0, best->speed * 100.0,
                      best->hold);
        status(text);
        std::snprintf(text, sizeof(text), "ov%.0f %.0f %.0f %.0f %.1fs", best_over[0], best_over[1], best_over[2],
                      best_over[3], best_sum);
        set_line(1, text);
        std::printf("ARM_TUNE best finish %.2f at %.2f, hold %.0f, total ready %.2f s\n", best->finish, best->speed,
                    best->hold, best_sum);
    } else {
        arm::set_swing_finish(finish_before, speed_before);
        arm::set_hold_power(hold_before);
        status(stopped ? "stopped: no pick" : "none usable");
    }
    show_settings();
}

std::size_t finish_speed_index() {
    const double now = arm::swing_finish_speed_fraction();
    for (std::size_t i = 0; i < kFinishSpeeds.size(); ++i) {
        if (std::fabs(kFinishSpeeds[i] - now) < 0.01) {
            return i;
        }
    }
    return kFinishSpeeds.size() - 1;
}

}

void update() {
    // A: the claw. Closing it at home lifts to the carry height kCarryDelayMs later (the claw grips first), so the
    // cup-pin comes off the ground; opening it at home brings the lift back down.
    if (controller.get_digital_new_press(controls::kClawPiston)) {
        claw::toggle();
        carry_at_ms = 0;
        if (!out) {
            if (claw::is_closed()) {
                const std::uint32_t at = pros::millis() + static_cast<std::uint32_t>(constants::kCarryDelayMs);
                carry_at_ms = at == 0 ? 1 : at;
                status("gripping");
            } else {
                extension::set_target(constants::extension_cm_to_motor_deg(0.0));
            }
        }
    }
    if (carry_at_ms != 0 && static_cast<std::int32_t>(pros::millis() - carry_at_ms) >= 0) {
        carry_at_ms = 0;
        if (claw::is_closed() && !out && macros::target_now().lift_cm < constants::kCarryLiftCm) {
            extension::set_target(constants::extension_cm_to_motor_deg(constants::kCarryLiftCm));
            status("ready");
        }
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        selected = std::min(selected + 1, kSwings - 1);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        selected = std::max(selected - 1, 0);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
        arm::set_swing_tuning(arm::swing_fade(), std::round((arm::swing_brake() + 0.1) * 10.0) / 10.0);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
        arm::set_swing_tuning(arm::swing_fade(), std::max(0.0, std::round((arm::swing_brake() - 0.1) * 10.0) / 10.0));
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        arm::set_hold_power(arm::hold_power() + 5.0);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        arm::set_hold_power(std::max(0.0, arm::hold_power() - 5.0));
    }
    // RIGHT / Y: how much of the swing is slowed (+/- 5%); L2: the slowed speed (30-60%, cycling).
    const auto set_finish = [](double fraction) {
        const double speed = arm::swing_finish_speed_fraction() > 0.0 ? arm::swing_finish_speed_fraction() : 0.4;
        arm::set_swing_finish(std::clamp(std::round(fraction * 20.0) / 20.0, 0.0, 0.6), speed);
    };
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        set_finish(arm::swing_finish_fraction() + 0.05);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        set_finish(arm::swing_finish_fraction() - 0.05);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        arm::set_swing_finish(arm::swing_finish_fraction(),
                              kFinishSpeeds[(finish_speed_index() + 1) % kFinishSpeeds.size()]);
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        if (!out) {
            status("swinging");
            const Result r = swing_out(selected);
            have_last = true;
            show_result(r);
            const Row row{arm::swing_fade(),
                          arm::swing_brake(),
                          arm::swing_finish_fraction(),
                          arm::swing_finish_speed_fraction(),
                          arm::hold_power(),
                          selected,
                          r};
            save_rows(&row, 1, "manual");
            out = !r.aborted;
            status(r.tipped ? "tipped" : "L1: home");
        } else {
            out = !go_home();
            status(out ? "stopped" : "ready");
        }
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        out = false;
        sweep();
    }
    show_settings();
    if (!have_last && row_count == 0) {
        set_line(1, "LEFT sweep, L1 one");
    }
    if (lines[2][0] == '\0') {
        status("ready");
    }
    flush();
}

}
