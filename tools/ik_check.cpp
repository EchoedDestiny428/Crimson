#include "subsystems/elevator/ik.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

int failures = 0;

void expect(bool ok, const char* what) {
    if (!ok) {
        std::cerr << "fail: " << what << '\n';
        ++failures;
    }
}

}

int main() {
    using namespace subsystems::elevator;
    using ik::Motion;
    using ik::Pair;
    using ik::Solution;

    const double design_x = ik::release_x_cm(constants::kArmScoreLowDeg);
    expect(std::fabs(ik::release_x_cm(constants::kArmScoreHighDeg) - design_x) < 1e-9, "230 and 310 share x");
    expect(design_x < -15.0 && design_x > -15.4, "design reach is 15.2 cm behind the pivot");
    expect(std::fabs(ik::x_from_score_range_cm(0.0) - design_x) < 1e-9, "0 cm reading is the rammed pose");
    expect(std::fabs(ik::x_from_score_range_cm(5.0) - (design_x - 5.0)) < 1e-9, "5 cm short reaches 5 cm further");

    const auto from_floor = [&](double z) { return ik::choose(ik::solve(design_x, z), 0.0, 35.0); };
    const auto check_stage = [&](double z, double arm, double lift, const char* name) {
        const Solution got = from_floor(z);
        expect(got.valid, name);
        expect(std::fabs(got.arm_deg - arm) < 0.05, name);
        expect(std::fabs(got.lift_cm - lift) < 0.1, name);
        expect(std::fabs(ik::release_x_cm(got.arm_deg) - design_x) < 1e-6, name);
        expect(std::fabs(got.z_cm - z) < 1e-6, name);
    };
    expect(!from_floor(17.5).valid, "L1 is below the lift and is refused");
    check_stage(36.0, 310.0, 15.0, "L2");
    check_stage(53.0, 310.0, 32.0, "L3");
    check_stage(70.0, 230.0, 11.8, "L4 from the floor takes the high arm");
    check_stage(87.0, 230.0, 28.8, "L5");
    check_stage(104.0, 230.0, 45.8, "L6");
    check_stage(121.0, 230.0, 62.8, "L7");

    const Pair at_design = ik::solve(design_x, 70.0);
    expect(at_design.high.valid && at_design.low.valid, "L4 height is reachable on both arms");
    expect(std::fabs(at_design.high.arm_deg + at_design.low.arm_deg - 540.0) < 0.05,
           "the two arms are 540 minus each other");
    expect(std::fabs(at_design.low.arm_deg - 310.0) < 0.05 && std::fabs(at_design.high.arm_deg - 230.0) < 0.05,
           "design pair is 310 and 230");
    const Solution stay = ik::choose(at_design, 32.0, 310.0);
    expect(stay.valid && std::fabs(stay.arm_deg - 310.0) < 0.05 && std::fabs(stay.lift_cm - 49.0) < 0.15,
           "from L3, L4 height stays on the low arm: less lift travel");

    const double further = design_x - 5.0;
    const Solution reached = ik::choose(ik::solve(further, 70.0), 0.0, 35.0);
    expect(reached.valid, "5 cm further still reaches L4");
    expect(std::fabs(ik::release_x_cm(reached.arm_deg) - further) < 1e-6, "further L4 hits x");
    expect(std::fabs(reached.z_cm - 70.0) < 1e-6, "further L4 hits z");
    expect(reached.arm_deg > 230.0 && reached.arm_deg < 270.0, "further reach swings the high arm toward 270");
    expect(!ik::solve(design_x - 20.0, 70.0).high.valid && !ik::solve(design_x - 20.0, 70.0).low.valid,
           "past the arm's reach has no pose");
    expect(!ik::solve(design_x, 200.0).high.valid && !ik::solve(design_x, 200.0).low.valid,
           "above the lift max has no pose");

    // Close enough that the low arm would pass the crossbar: only the high arm remains.
    const double close_x = -7.5;
    const Pair close = ik::solve(close_x, 80.0);
    expect(!close.low.valid && close.high.valid, "inside the crossbar, only the high arm");
    expect(std::fabs(ik::release_x_cm(close.high.arm_deg) - close_x) < 1e-6, "close pose hits x");

    const Motion from_intake = ik::plan(0.0, 35.0, design_x, 36.0);
    expect(from_intake.valid && from_intake.count == 2, "intake to L2 is raise, then swing");
    expect(std::fabs(from_intake.steps[0].lift_cm - from_intake.goal.lift_cm) < 1e-9 &&
               std::fabs(from_intake.steps[0].arm_deg - 35.0) < 1e-9,
           "intake to L2 raises the lift before the arm moves");
    expect(std::fabs(from_intake.steps[1].arm_deg - 310.0) < 0.05, "intake to L2 then sets the reach");

    expect(!ik::plan(0.0, 35.0, design_x, 17.5).valid, "L1 plans nothing");

    const Motion stay_low = ik::plan(32.0, 310.0, design_x, 70.0);
    expect(stay_low.valid && stay_low.count == 1, "from L3 the lift just rises");
    expect(std::fabs(stay_low.steps[0].arm_deg - 310.0) < 0.05 && std::fabs(stay_low.steps[0].lift_cm - 49.0) < 0.15,
           "from L3 the arm stays at 310");

    const Motion high_from_intake = ik::plan(0.0, 35.0, design_x, 70.0);
    expect(high_from_intake.count == 3, "intake to L4 is raise, swing above the pin, drop");
    expect(high_from_intake.steps[1].lift_cm > high_from_intake.goal.lift_cm + 4.0, "L4 swing is above the pin");
    expect(std::fabs(high_from_intake.steps[2].lift_cm - high_from_intake.goal.lift_cm) < 1e-9 &&
               std::fabs(high_from_intake.steps[2].arm_deg - 230.0) < 0.05,
           "L4 drops onto the pin last");

    const Motion hold = ik::plan(stay_low.goal.lift_cm, stay_low.goal.arm_deg, design_x, 70.0);
    expect(hold.valid && hold.count == 1, "already there is a single hold step");

    if (failures != 0) {
        std::cerr << failures << " failed\n";
        return EXIT_FAILURE;
    }
    std::cout << "ik ok\n";
    return EXIT_SUCCESS;
}
