#include "autons/autons.hpp"
#include "robotconfig.hpp"

namespace autons::match::test {

void run() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -12, 2000, {.forwards = false, .maxSpeed = 50});
    chassis.waitUntilDone();
}

}
