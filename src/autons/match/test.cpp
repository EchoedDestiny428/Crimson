#include "autons/autons.hpp"
#include "robotconfig.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace autons::match::test {

namespace {

struct Leg {
    float heading;
    float x;
    float y;
};

constexpr float kCornerIn = 24;
constexpr int kDriveSpeed = 60;
constexpr int kTurnSpeed = 50;
constexpr int kDriveTimeoutMs = 3500;
constexpr int kTurnTimeoutMs = 1500;
constexpr std::uint32_t kSettleMs = 500;

constexpr std::array kForwardSquare{
    Leg{0, -kCornerIn, kCornerIn},
    Leg{90, kCornerIn, kCornerIn},
    Leg{180, kCornerIn, -kCornerIn},
    Leg{270, -kCornerIn, -kCornerIn},
};

constexpr std::array kBackwardSquare{
    Leg{270, kCornerIn, -kCornerIn},
    Leg{180, kCornerIn, kCornerIn},
    Leg{90, -kCornerIn, kCornerIn},
    Leg{0, -kCornerIn, -kCornerIn},
};

void settle() {
    chassis.waitUntilDone();
    pros::delay(kSettleMs);
}

void turn_to(float heading) {
    chassis.turnToHeading(heading, kTurnTimeoutMs, {.maxSpeed = kTurnSpeed});
    settle();
}

template <std::size_t Count>
void drive(const std::array<Leg, Count>& legs, bool forwards) {
    for (const Leg& leg : legs) {
        turn_to(leg.heading);
        chassis.moveToPoint(leg.x, leg.y, kDriveTimeoutMs, {.forwards = forwards, .maxSpeed = kDriveSpeed});
        settle();
    }
    turn_to(0);
}

}

void run() {
    chassis.setPose(-kCornerIn, -kCornerIn, 0);
    pros::delay(kSettleMs);

    drive(kForwardSquare, true);
    drive(kBackwardSquare, false);
}

}
