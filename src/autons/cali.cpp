#include "autons/routine.hpp"
#include <cmath>
#include <cstdio>

namespace autons::cali {

namespace {

// Must match kTrackingWheelDiameterIn in robotconfig.cpp.
constexpr double kTrackingWheelDiameterIn = 2.0;
constexpr double kTurnDeg = 90.0;
constexpr double kPi = 3.14159265358979323846;

double horizontal_wheel_in() {
    return horizontal_odom.get_position() / 36000.0 * kPi * kTrackingWheelDiameterIn;
}

}

void setup() {}

// Turns 90 degrees in place and works out how far the horizontal tracking wheel
// is from the turning center. The wheel rolls along an arc, so travel = radius * angle;
// for exactly 90 degrees that arc is a quarter circumference.
void run() {
    chassis.setPose(0, 0, 0);
    pros::delay(200);
    const double start_in = horizontal_wheel_in();
    const double start_deg = imu.get_rotation();

    chassis.turnToHeading(kTurnDeg, 3000);
    chassis.waitUntilDone();
    pros::delay(500); // let the robot come to a stop before reading

    const double travel_in = horizontal_wheel_in() - start_in;
    const double turned_deg = imu.get_rotation() - start_deg;
    const double radius_90 = travel_in / (kPi / 2.0);
    const double radius_imu = std::fabs(turned_deg) > 1.0 ? travel_in / (turned_deg * kPi / 180.0) : NAN;

    std::printf("CALI travel=%.3f in turned=%.2f deg r90=%.3f in rIMU=%.3f in\n", travel_in, turned_deg, radius_90,
                radius_imu);

    // Keep reprinting so the numbers stay on the controller after the turn.
    while (true) {
        controller.print(0, 0, "r90  %6.2f in", radius_90);
        pros::delay(60);
        controller.print(1, 0, "rIMU %6.2f in", radius_imu);
        pros::delay(60);
        controller.print(2, 0, "s%.2f t%.1f  ", travel_in, turned_deg);
        pros::delay(500);
    }
}

}
