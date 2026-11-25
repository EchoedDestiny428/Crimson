#pragma once

namespace subsystems::elevator::claw {

void update(); // L2 and X toggle the claw piston; clamping lifts the cup off the floor
void toggle();
void open();  // hard-set open (outtake), not a toggle
void close(); // hard-set closed
void grab();  // close on a piece

// Autons.
void prime();   // open, waiting for a piece
void release(); // open to drop the piece

}
