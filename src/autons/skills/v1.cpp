#include "autons/routine.hpp"

namespace autons::skills::v1 {

void setup() {}

void run() {
    chassis.setPose(20.25, 8, 90);
    parallel({
        [] {
            macros::flip_out(10, 950);
        },
        [] {
            chassis.moveToPoint(20.25, 24, 500, {.forwards = false, .minSpeed = 127});
            chassis.turnToPoint(48, 24, 400, {.forwards = false});
        },
    });
}

}
