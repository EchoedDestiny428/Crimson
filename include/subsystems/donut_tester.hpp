#pragma once

// donut-testing branch only: tune the donut's outer / inner speeds by hand.
//
// Back the robot against a goal, then:
//   X / B      outer speed up / down (fastSpeed, 0..127)
//   UP / DOWN  inner speed up / down (slowSpeed, -127..127; positive = inside wheels go backward)
//   hold R1    donut clockwise, hold L1 counterclockwise, back pushed into the goal (forwards = false)
// Hold a speed button to repeat. The controller screen shows the speeds, the push/spin they make, and what each side
// of the drive is being sent. Starts from DonutParams' defaults (include/motion/chassis.hpp).
namespace subsystems::donut_tester {

// Call every driver loop. Returns true while it is driving the motors (L1 or R1 held), so the normal drive
// should not run that loop.
bool update();

}
