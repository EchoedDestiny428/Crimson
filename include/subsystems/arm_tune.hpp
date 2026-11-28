#pragma once

// arm-tuning branch only: tunes how the arm arrives at the end of a swing out, swinging it the way the Default auton
// does, robot still, a cup-pin in the claw. The first sweep (fade x brake, run 42) found neither stops the overshoot:
// past the top the arm and pin run ahead of the motor into the arm's slack, where the motor's encoder can't see them.
// So it slows the end of each swing instead (arm_pivot::set_swing_finish), like the swing home already does. The last
// 15% at 40% fixed L1, L2 and L2N (run 48); L3 then overshot and stayed past because the hold floor is too weak to lift
// an arm and cup-pin back, so this sweep also tries stronger hold floors (arm_pivot::set_hold_power).
//
// Driver control on this branch is the drive plus this tuner (the stage macros, side toggle and HUD are off):
//   A        claw open / close (load a cup-pin). Closing it at home lifts to the carry height (10 cm) 0.5 s later,
//            so the cup-pin is off the ground; opening it there brings the lift back down
//   UP/DOWN  pick the swing: L1, L2, L3, L2N (L2 on the neutral goal, lift 27 cm). For L1 and L3 (lift under the
//            10 cm carry height) aim() holds 10 cm until the arm is past 60 deg, so the cup-pin never touches the floor
//   L1       one swing out to it with the current settings, measured; L1 again swings home
//   RIGHT/Y  slow the last +/- 5% more / less of the swing     L2  the slowed speed: 30, 40, 50, 60% (cycles)
//   R1/R2    brake +/- 0.1        X/B  hold floor +/- 5 (out of 127)
//   LEFT     the automatic sweep: the last 15 or 20% at 40% speed, each with a hold floor of 30, 45 and 60; L1, L2, L3
//            twice and L2N each time, from home every swing; picks the best and leaves it set
//   a stick  stops a swing or the sweep
// Controller top line, e.g. "e15@40 h45 L3 C": the last 15% at 40% speed, hold floor 45, swing L3, claw closed.
//
// Each swing starts from home with the lift at carry height (where an auton pickup leaves a pin) and measures, from
// aim() (lift and arm together, as the auton does) to 0.6 s after it can release:
//   ov   overshoot past the target (deg)
//   r    time until within 4 deg and under 10 rpm: when the auton's arrived_still() lets the claw open (s)
//   k    rock: how far the robot tipped past where it settled (deg, IMU). Its lean (the settled tilt from moving the
//        weight) is logged too but doesn't count against it
//   (logged: time to first within 4 deg; how far off it ended, over 2 deg meaning short of or past the 2 deg hold)
// The sweep's pick: every swing (both L3s) ready with end <= 2 deg and rock <= 3 deg, then the shortest total ready time.
// Results go to /usd/arm_tune3.csv (one row per swing) and the controller; each swing also sets a telemetry::mark
// (9000 + swing number) in the run's normal log, whose arm_deg / arm_target_deg columns trace it.
namespace subsystems::arm_tune {

// Call every driver loop. Blocks while a swing or the sweep runs.
void update();

}
