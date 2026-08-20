#ifndef CRIMSON_HPP_
#define CRIMSON_HPP_

#include "main.h"
#include "pros/ai_vision.hpp"
#include <optional>
#include <vector>

namespace crimson {

struct Pose2D {
    double x;
    double y;
    double theta; // global heading in degrees
};

class Crimson {
public:
    explicit Crimson(std::uint8_t port = 1);

    void initialize();
    void update();

    // Global Pose Estimation
    void set_camera_mount_metrics(double height_mm, double pitch_deg);
    std::optional<Pose2D> estimate_global_pose(double imu_heading_deg);

    bool has_target() const;
    int get_tv() const;
    double get_tx() const;
    double get_ty() const;
    
    // Legacy support methods (might return 0 for tags since tags use corner points)
    double get_ta() const;
    double get_ts() const;
    std::uint32_t get_data_age_ms() const;

    pros::AIVision::Object get_raw_target() const;
    pros::AIVision &sensor();

private:
    static constexpr double kFrameWidth = 320.0;
    static constexpr double kFrameHeight = 240.0;
    static constexpr double kCenterX = kFrameWidth / 2.0;
    static constexpr double kCenterY = kFrameHeight / 2.0;

    // AI Vision FOV (approximate, adjust if necessary)
    static constexpr double kFovHorizontalDeg = 74.0; 
    static constexpr double kFovVerticalDeg = 55.5;

    pros::AIVision sensor_;
    pros::AIVision::Object primary_target_;
    bool target_valid_;
    std::uint32_t last_update_ms_;

    std::vector<pros::AIVision::Object> tags_visible_;

    double camera_height_mm_ = 100.0;
    double camera_pitch_deg_ = 0.0;

    // Helper to compute tag center in pixels
    void get_tag_center(const pros::AIVision::Object& tag, double& px, double& py) const;
    
    // Helper to get tx, ty from pixel coordinates
    double calc_tx(double px) const;
    double calc_ty(double py) const;
};

}  // namespace crimson

#endif
