#pragma once

#include "vision/crimson.hpp"
#include <cstdint>
#include <optional>

namespace display {

class Dashboard {
public:
    void initialize();
    void show_status(const char* text);
    void update(const crimson::Pose2D& odom, const std::optional<crimson::Pose2D>& vision, int tag_count,
                int primary_tag_id);

private:
    static constexpr int kMapSize = 195;
    static constexpr int kMapOffsetX = 15;
    static constexpr int kMapOffsetY = 38;

    static int to_screen_x(double x_in);
    static int to_screen_y(double y_in);

    void draw_status(const char* text, std::uint32_t color);
    void draw_field();
    void draw_pose(const crimson::Pose2D& pose, std::uint32_t color, bool filled);
    void draw_legend(int y, std::uint32_t color, bool filled, const char* label);
    void print_stats(const crimson::Pose2D& odom, const std::optional<crimson::Pose2D>& vision, int tag_count,
                     int primary_tag_id);
};

}
