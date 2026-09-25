#include "autons/routine.hpp"

namespace autons::match {

void test() {
    chassis.setPose(0, 0, 0);

    macros::dual_setup();

    chassis.moveToPoint(0, -12, 2000, {.forwards = false});
    chassis.waitUntilDone();

    macros::dual_pickup();
}

}
