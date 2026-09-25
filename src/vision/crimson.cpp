#include "vision/crimson.hpp"
#include "constants/field_constants.hpp"
#include "pros/error.h"
#include "pros/rtos.hpp"
#include "util/angle.hpp"
#include <cmath>

namespace crimson {

using util::deg_to_rad;
using util::wrap_degrees;

namespace {

constexpr double kMinTriangulationSin = 0.01;
constexpr double kMinPitchTan = 0.01;

Pose2D to_lemlib_pose(const field::Point2D& robot_mm, double heading_deg) {
    const field::Point2D robot_in = field::to_lemlib_inches(robot_mm);
    return {robot_in.x, robot_in.y, wrap_degrees(heading_deg)};
}

}

Crimson::Crimson(std::uint8_t port) : sensor_(port) {}

void Crimson::initialize() {
    sensor_.enable_detection_types(pros::AivisionModeType::tags);
}

void Crimson::update() {
    tags_.clear();

    const std::int32_t count = sensor_.get_object_count();
    if (count > 0 && count != PROS_ERR) {
        for (const auto& object : sensor_.get_all_objects()) {
            if (pros::AIVision::is_type(object, pros::AivisionDetectType::tag)) {
                tags_.push_back(object);
            }
        }
    }

    last_update_ms_ = pros::millis();
}

void Crimson::set_camera_mount_metrics(double height_mm, double pitch_deg) {
    camera_height_mm_ = height_mm;
    camera_pitch_deg_ = pitch_deg;
}

bool Crimson::has_target() const {
    return !tags_.empty();
}

int Crimson::tag_count() const {
    return static_cast<int>(tags_.size());
}

int Crimson::primary_tag_id() const {
    return tags_.empty() ? -1 : tags_.front().id;
}

double Crimson::get_tx() const {
    return tags_.empty() ? 0.0 : calc_tx(tag_center(tags_.front()).x);
}

double Crimson::get_ty() const {
    return tags_.empty() ? 0.0 : calc_ty(tag_center(tags_.front()).y);
}

std::uint32_t Crimson::get_data_age_ms() const {
    return last_update_ms_ == 0 ? 0 : pros::millis() - last_update_ms_;
}

Crimson::Pixel Crimson::tag_center(const pros::AIVision::Object& tag) {
    const auto& corners = tag.object.tag;
    return {(corners.x0 + corners.x1 + corners.x2 + corners.x3) / 4.0,
            (corners.y0 + corners.y1 + corners.y2 + corners.y3) / 4.0};
}

double Crimson::calc_tx(double px) {
    return (px - kFrameWidth / 2.0) * (kFovHorizontalDeg / kFrameWidth);
}

double Crimson::calc_ty(double py) {
    return (kFrameHeight / 2.0 - py) * (kFovVerticalDeg / kFrameHeight);
}

std::optional<Pose2D> Crimson::estimate_global_pose(double heading_deg) const {
    if (tags_.empty()) {
        return std::nullopt;
    }
    if (tags_.size() >= 2) {
        if (const auto pose = triangulate(tags_[0], tags_[1], heading_deg)) {
            return pose;
        }
    }
    return from_single_tag(tags_.front(), heading_deg);
}

std::optional<Pose2D> Crimson::triangulate(const pros::AIVision::Object& first, const pros::AIVision::Object& second,
                                           double heading_deg) {
    const double bearing1 = deg_to_rad(heading_deg + calc_tx(tag_center(first).x));
    const double bearing2 = deg_to_rad(heading_deg + calc_tx(tag_center(second).x));
    const double det = std::sin(bearing1 - bearing2);
    if (std::abs(det) < kMinTriangulationSin) {
        return std::nullopt;
    }

    for (const auto& goal1 : field::get_goal_locations_for_tag(first.id)) {
        for (const auto& goal2 : field::get_goal_locations_for_tag(second.id)) {
            const double dx = goal2.pos.x - goal1.pos.x;
            const double dy = goal2.pos.y - goal1.pos.y;
            const double dist1 = (dy * std::sin(bearing2) - dx * std::cos(bearing2)) / det;
            const double dist2 = (dy * std::sin(bearing1) - dx * std::cos(bearing1)) / det;
            const field::Point2D robot{goal1.pos.x - dist1 * std::sin(bearing1),
                                       goal1.pos.y - dist1 * std::cos(bearing1)};

            if (dist1 > 0.0 && dist2 > 0.0 && field::in_field(robot)) {
                return to_lemlib_pose(robot, heading_deg);
            }
        }
    }
    return std::nullopt;
}

std::optional<Pose2D> Crimson::from_single_tag(const pros::AIVision::Object& tag, double heading_deg) const {
    const Pixel center = tag_center(tag);
    const double pitch_tan = std::tan(deg_to_rad(camera_pitch_deg_ + calc_ty(center.y)));
    if (std::abs(pitch_tan) < kMinPitchTan) {
        return std::nullopt;
    }

    const double bearing = deg_to_rad(heading_deg + calc_tx(center.x));

    for (const auto& goal : field::get_goal_locations_for_tag(tag.id)) {
        const double distance = (goal.height - camera_height_mm_) / pitch_tan;
        if (!std::isfinite(distance) || distance <= 0.0) {
            continue;
        }

        const field::Point2D robot{goal.pos.x - distance * std::sin(bearing),
                                   goal.pos.y - distance * std::cos(bearing)};
        if (field::in_field(robot)) {
            return to_lemlib_pose(robot, heading_deg);
        }
    }
    return std::nullopt;
}

}
