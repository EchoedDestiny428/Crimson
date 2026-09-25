#include "subsystems/vision.hpp"
#include "robotconfig.hpp"
#include <cstdint>

namespace subsystems::vision {

namespace {

constexpr double kCameraHeightMm = 100.0;
constexpr double kCameraPitchDeg = 0.0;
constexpr std::uint32_t kCameraPeriodMs = 20;
constexpr std::uint32_t kDashboardEveryTicks = 5;

void run() {
    std::uint32_t now = pros::millis();
    std::uint32_t tick = 0;

    while (true) {
        crimson_cam.update();

        if (++tick % kDashboardEveryTicks == 0) {
            const lemlib::Pose odom = chassis.getPose();
            const auto vision = crimson_cam.estimate_global_pose(odom.theta);
            dashboard.update({odom.x, odom.y, odom.theta}, vision, crimson_cam.tag_count(),
                             crimson_cam.primary_tag_id());
        }

        pros::Task::delay_until(&now, kCameraPeriodMs);
    }
}

}

void init() {
    crimson_cam.initialize();
    crimson_cam.set_camera_mount_metrics(kCameraHeightMm, kCameraPitchDeg);
    dashboard.initialize();
}

void start() {
    pros::Task task(run, "Vision");
}

}
