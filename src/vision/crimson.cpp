#include "vision/crimson.hpp"
#include "constants/field_constants.hpp"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace crimson {

static inline double deg2rad(double deg) { return deg * M_PI / 180.0; }
static inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }

Crimson::Crimson(std::uint8_t port)
    : sensor_(port), target_{}, target_valid_(false), last_update_ms_(0) {
}

void Crimson::initialize() {
    sensor_.enable_detection_types(pros::AivisionModeType::tags);
}

void Crimson::set_camera_mount_metrics(double height_mm, double pitch_deg) {
    camera_height_mm_ = height_mm;
    camera_pitch_deg_ = pitch_deg;
}

void Crimson::update() {
    tags_visible_.clear();
    target_valid_ = false;

    int obj_count = sensor_.get_object_count();
    if (obj_count > 0) {
        auto objects = sensor_.get_all_objects();
        for (const auto& obj : objects) {
            if (pros::AIVision::is_type(obj, pros::AivisionDetectType::tag)) {
                tags_visible_.push_back(obj);
            }
        }
    }

    if (!tags_visible_.empty()) {
        primary_target_ = tags_visible_.front();
        target_valid_ = true;
    }

    last_update_ms_ = pros::millis();
}

void Crimson::get_tag_center(const pros::AIVision::Object& tag, double& px, double& py) const {
    px = (tag.object.tag.x0 + tag.object.tag.x1 + tag.object.tag.x2 + tag.object.tag.x3) / 4.0;
    py = (tag.object.tag.y0 + tag.object.tag.y1 + tag.object.tag.y2 + tag.object.tag.y3) / 4.0;
}

double Crimson::calc_tx(double px) const {
    return (px - kCenterX) * (kFovHorizontalDeg / kFrameWidth);
}

double Crimson::calc_ty(double py) const {
    return (kCenterY - py) * (kFovVerticalDeg / kFrameHeight);
}

bool Crimson::has_target() const { return target_valid_; }
int Crimson::get_tv() const { return target_valid_ ? 1 : 0; }

double Crimson::get_tx() const {
    if (!target_valid_) return 0.0;
    double px, py;
    get_tag_center(primary_target_, px, py);
    return calc_tx(px);
}

double Crimson::get_ty() const {
    if (!target_valid_) return 0.0;
    double px, py;
    get_tag_center(primary_target_, px, py);
    return calc_ty(py);
}

double Crimson::get_ta() const { return 0.0; }
double Crimson::get_ts() const { return 0.0; }

std::uint32_t Crimson::get_data_age_ms() const {
    if (last_update_ms_ == 0) return 0;
    return pros::millis() - last_update_ms_;
}

pros::AIVision::Object Crimson::get_raw_target() const { return primary_target_; }
pros::AIVision &Crimson::sensor() { return sensor_; }

// Note: To truly resolve ambiguous tags, we pick the one that gives the most reasonable distance.
// For now, we assume the first valid mapping is correct, but ideally you pass in the last known robot pose to disambiguate.
std::optional<Pose2D> Crimson::estimate_global_pose(double imu_heading_deg) {
    if (tags_visible_.empty()) return std::nullopt;

    double heading_rad = deg2rad(imu_heading_deg);

    // --- TWO TAG TRIANGULATION ---
    if (tags_visible_.size() >= 2) {
        auto tag1 = tags_visible_[0];
        auto tag2 = tags_visible_[1];
        
        auto locs1 = field::get_goal_locations_for_tag(tag1.id);
        auto locs2 = field::get_goal_locations_for_tag(tag2.id);

        if (!locs1.empty() && !locs2.empty()) {
            double px1, py1, px2, py2;
            get_tag_center(tag1, px1, py1);
            get_tag_center(tag2, px2, py2);

            double tx1 = calc_tx(px1);
            double tx2 = calc_tx(px2);

            double gamma1 = heading_rad + deg2rad(tx1);
            double gamma2 = heading_rad + deg2rad(tx2);

            // We pick the first possible location for each tag for simplicity.
            // A more advanced version would test all combinations of ambiguous tags and pick the one that yields a realistic robot position.
            field::Point2D p1 = locs1[0].pos;
            field::Point2D p2 = locs2[0].pos;

            double D = std::sin(gamma1 - gamma2);
            if (std::abs(D) > 0.01) { // Ensure lines aren't parallel
                double d1 = ((p2.x - p1.x) * std::sin(gamma2) - (p2.y - p1.y) * std::cos(gamma2)) / D;
                double rx = p1.x - d1 * std::cos(gamma1);
                double ry = p1.y - d1 * std::sin(gamma1);
                return Pose2D{rx, ry, imu_heading_deg};
            }
        }
    }

    // --- SINGLE TAG FALLBACK ---
    auto tag = tags_visible_.front();
    auto locs = field::get_goal_locations_for_tag(tag.id);
    if (!locs.empty()) {
        double px, py;
        get_tag_center(tag, px, py);
        double ty = calc_ty(py);
        double tx = calc_tx(px);

        double total_pitch_rad = deg2rad(camera_pitch_deg_ + ty);
        if (std::tan(total_pitch_rad) <= 0.01) return std::nullopt; // Protect against div by zero or negative pitch

        double delta_h = locs[0].height - camera_height_mm_;
        double distance_horizontal = delta_h / std::tan(total_pitch_rad);

        double robot_x = locs[0].pos.x - distance_horizontal * std::cos(heading_rad + deg2rad(tx));
        double robot_y = locs[0].pos.y - distance_horizontal * std::sin(heading_rad + deg2rad(tx));

        return Pose2D{robot_x, robot_y, imu_heading_deg};
    }

    return std::nullopt;
}

}  // namespace crimson
