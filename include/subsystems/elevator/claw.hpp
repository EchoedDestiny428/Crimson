#pragma once

namespace subsystems::elevator::claw {

void update(); // A and X toggle the claw piston
void toggle();
void open();  // hard-set open (outtake), not a toggle
void close(); // hard-set closed
void grab();  // close on a piece

}
