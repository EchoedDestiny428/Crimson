#pragma once

// kG tuning: the arm gets no output (coasting) unless A is held, when it gets a fixed
// output in kG units (0-127 power, sent as an exact voltage). Up/Down change it by 0.5.
// Move the arm by hand and raise the output until it stops falling; the controller shows
// the kG that implies at the current angle (output / sin(arm angle from straight down)).
namespace tuning::kg {

void init();   // instead of arm_pivot::init(): the arm's SmartMotor never runs
void update(); // driver task

}
