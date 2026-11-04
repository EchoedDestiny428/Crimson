#include "display/controller_hud.hpp"
#include "robotconfig.hpp"
#include "subsystems/elevator/macros.hpp"

namespace display::controller_hud {

namespace {

constexpr std::uint32_t kPrintMs = 60; // the controller drops text sent faster than ~50 ms
std::uint32_t last_print = 0;
int line = 0;

}

// Line 1: lift cm and arm deg. Line 2: piece height, the stage a release snaps to, and
// the goal. Line 3: the setpoints being driven to (stage, lift cm, arm deg).
void update() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) &&
        controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        logger.dump();
    }

    const std::uint32_t now = pros::millis();
    if (now - last_print < kPrintMs) {
        return;
    }
    last_print = now;
    namespace macros = subsystems::elevator::macros;
    if (line == 0) {
        const macros::Pose pose = macros::pose_now();
        controller.print(0, 0, "L%5.1f A%6.1f   ", pose.lift_cm, pose.arm_deg);
    } else if (line == 1) {
        static constexpr char kGoal[] = {'A', 'N', 'C'};
        controller.print(1, 0, "tip%5.1f S%d %c   ", macros::tip_now(), macros::snap_stage_now(),
                         kGoal[macros::goal_tier_now()]);
    } else {
        const macros::Pose target = macros::target_now();
        controller.print(2, 0, "T%d L%4.1f A%5.1f  ", macros::target_stage(), target.lift_cm, target.arm_deg);
    }
    line = (line + 1) % 3;
}

}
