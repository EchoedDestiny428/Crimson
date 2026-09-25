#include "autons/routine.hpp"

namespace autons::match::test {

void setup() {}

void run() {
    chassis.setPose(0, 0, 0);

    parallel({
        macros::dual_setup,
        [] {
            chassis.moveToPoint(0, -12, 2000, {.forwards = false, .maxSpeed = 50});
            chassis.waitUntilDone();
        },
    });

    macros::dual_pickup();
}

}
