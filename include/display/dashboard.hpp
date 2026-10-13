#pragma once

#include "vision/crimson.hpp"
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace display {

class Dashboard {
public:
    void initialize();
    void show_status(const char* text);
    void update(const crimson::Pose2D& odom, const std::optional<crimson::Pose2D>& vision, int tag_count,
                int primary_tag_id);
    void show_autons(std::span<const char* const> names, std::size_t selected, const crimson::Pose2D& odom);

    static std::optional<std::size_t> auton_at(int x, int y, std::size_t count);

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

    bool showing_autons = false;
    std::size_t shown_auton = 0;
    char shown_pose[48] {};
};

}
