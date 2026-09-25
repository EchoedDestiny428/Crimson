#include "autons/routine.hpp"

namespace autons::match::snacky {

void setup() {
    intake::lift();
}

void run() {
    chassis.setPose(7.8, -63.4, 180);

    parallel({
        [] { macros::flip_out(500); },
        [] {
            chassis.moveToPoint(7.8, -58, 1000, {.forwards = false, .minSpeed = 127, .earlyExitRange = 1.5});
            chassis.moveToPoint(24, -48, 1000, {.forwards = false});
            chassis.waitUntilDone();
        },
    });

    intake::spin(-127);

}

}
