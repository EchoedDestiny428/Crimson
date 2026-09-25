#pragma once

#include "pros/ai_vision.hpp"
#include <cstdint>
#include <optional>
#include <vector>

namespace crimson {

struct Pose2D {
    double x;
    double y;
    double theta;
};

class Crimson {
public:
    explicit Crimson(std::uint8_t port);

    void initialize();
    void update();
    void set_camera_mount_metrics(double height_mm, double pitch_deg);
    std::optional<Pose2D> estimate_global_pose(double heading_deg) const;

    bool has_target() const;
    int tag_count() const;
    int primary_tag_id() const;
    double get_tx() const;
    double get_ty() const;
    std::uint32_t get_data_age_ms() const;

private:
    struct Pixel {
        double x;
        double y;
    };

    static constexpr double kFrameWidth = 320.0;
    static constexpr double kFrameHeight = 240.0;
    static constexpr double kFovHorizontalDeg = 74.0;
    static constexpr double kFovVerticalDeg = 55.5;

    static Pixel tag_center(const pros::AIVision::Object& tag);
    static double calc_tx(double px);
    static double calc_ty(double py);
    static std::optional<Pose2D> triangulate(const pros::AIVision::Object& first, const pros::AIVision::Object& second,
                                             double heading_deg);
    std::optional<Pose2D> from_single_tag(const pros::AIVision::Object& tag, double heading_deg) const;

    pros::AIVision sensor_;
    std::vector<pros::AIVision::Object> tags_;
    std::uint32_t last_update_ms_ = 0;
    double camera_height_mm_ = 100.0;
    double camera_pitch_deg_ = 0.0;
};

}
