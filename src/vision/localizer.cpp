#include "vision/localizer.hpp"
#include "config/camera.hpp"
#include "config/ports.hpp"
#include "constants/field_constants.hpp"
#include "pros/ai_vision.hpp"
#include "pros/error.h"
#include "pros/rtos.hpp"
#include "robotconfig.hpp"
#include "vision/pose_filter.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>

namespace vision::localizer {

namespace {

constexpr std::uint32_t kPeriodMs = 33;
constexpr std::uint32_t kLockHoldMs = 500;

constexpr Camera kCamera{config::kCameraHeightMm, config::kFocalXPx, config::kFocalYPx, config::kCenterXPx,
                         config::kCenterYPx};

pros::Mutex shared_mutex;
Snapshot shared;

FieldPose to_field_pose(const Pose& pose) {
    return {pose.x_mm / field::kMmPerInch, pose.y_mm / field::kMmPerInch, pose.heading_deg};
}

void publish(const Snapshot& next) {
    const std::lock_guard lock(shared_mutex);
    shared = next;
}

bool configure(pros::AIVision& sensor) {
    using pros::AivisionModeType;
    return sensor.enable_detection_types(AivisionModeType::tags) != PROS_ERR &&
           sensor.disable_detection_types(AivisionModeType::colors, AivisionModeType::objects) != PROS_ERR &&
           sensor.set_tag_family(pros::AivisionTagFamily::tag_21H7, true) != PROS_ERR;
}

std::optional<std::size_t> read_tags(pros::AIVision& sensor, std::array<Detection, kMaxDetections>& tags) {
    const std::int32_t objects = sensor.get_object_count();
    if (objects == PROS_ERR) {
        return std::nullopt;
    }

    std::size_t count = 0;
    for (std::int32_t i = 0; i < objects && count < tags.size(); ++i) {
        const pros::AIVision::Object object = sensor.get_object(static_cast<std::uint32_t>(i));
        if (!pros::AIVision::is_type(object, pros::AivisionDetectType::tag)) {
            continue;
        }
        const auto& tag = object.object.tag;
        tags[count++] = {object.id,
                         {{{static_cast<double>(tag.x0), static_cast<double>(tag.y0)},
                           {static_cast<double>(tag.x1), static_cast<double>(tag.y1)},
                           {static_cast<double>(tag.x2), static_cast<double>(tag.y2)},
                           {static_cast<double>(tag.x3), static_cast<double>(tag.y3)}}}};
    }
    return count;
}

std::optional<RawTag> largest_tag(std::span<const Detection> tags) {
    std::optional<RawTag> largest;
    for (const Detection& tag : tags) {
        const auto [top_left, top_right, bottom_right, bottom_left] = order_corners(tag.corners);
        const RawTag raw{tag.id,
                         (top_left.u + top_right.u + bottom_right.u + bottom_left.u) / 4.0,
                         (top_left.v + top_right.v + bottom_right.v + bottom_left.v) / 4.0,
                         ((top_right.u - top_left.u) + (bottom_right.u - bottom_left.u)) / 2.0,
                         ((bottom_left.v - top_left.v) + (bottom_right.v - top_right.v)) / 2.0};
        if (!largest || raw.height_px > largest->height_px) {
            largest = raw;
        }
    }
    return largest;
}

void run() {
    pros::AIVision sensor(ports::kVision);

    const lemlib::Pose boot = chassis.getPose();
    const Pose start{boot.x * field::kMmPerInch, boot.y * field::kMmPerInch, boot.theta};

    Snapshot starting;
    starting.pose = to_field_pose(start);
    publish(starting);

    PoseFilter filter;
    filter.reset(start);

    std::array<Detection, kMaxDetections> tags{};
    bool configured = false;
    double last_heading = boot.theta;
    std::optional<std::uint32_t> last_fix_ms;
    std::uint32_t last_ms = pros::millis();
    std::uint32_t wake_ms = last_ms;

    while (true) {
        const std::uint32_t now_ms = pros::millis();
        const double heading = chassis.getPose().theta;
        const bool imu_ok = std::isfinite(heading);
        const double turned = imu_ok && std::isfinite(last_heading) ? heading - last_heading : 0.0;
        last_heading = heading;
        filter.predict(static_cast<double>(now_ms - last_ms) / 1000.0, turned);
        last_ms = now_ms;

        configured = configured || configure(sensor);
        const std::optional<std::size_t> count = configured ? read_tags(sensor, tags) : std::nullopt;
        configured = count.has_value();

        Snapshot next;
        if (count) {
            const auto seen = std::span<const Detection>(tags).first(*count);
            const Solution solution = solve(seen, filter.pose().heading_deg, kCamera);
            const Fix fix = filter.update(solution);
            if (fix == Fix::Accepted) {
                last_fix_ms = now_ms;
            }

            const bool recent = last_fix_ms && now_ms - *last_fix_ms <= kLockHoldMs;
            next.status = fix == Fix::Ambiguous ? Status::Ambiguous : recent ? Status::Locked : Status::Searching;
            next.tags = static_cast<int>(*count);
            next.candidates = static_cast<int>(solution.count);
            next.raw = largest_tag(seen);
            if (solution.count > 0) {
                next.goals = solution.candidates[0].goals;
                next.error_px = solution.candidates[0].error_px;
                next.range_in = solution.candidates[0].range_mm / field::kMmPerInch;
            }
        } else {
            next.status = Status::NoSensor;
        }
        if (!imu_ok) {
            next.status = Status::NoImu;
        }
        next.pose = to_field_pose(filter.pose());
        publish(next);

        pros::Task::delay_until(&wake_ms, kPeriodMs);
    }
}

}

void start() {
    pros::Task task(run, "Localizer");
}

Snapshot snapshot() {
    const std::lock_guard lock(shared_mutex);
    return shared;
}

}
