#pragma once

#include "lemlib/pose.hpp"
#include "vision/localizer.hpp"
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace display {

class Dashboard {
public:
    void initialize();
    void show_status(const char* text);
    void update(const lemlib::Pose& odom, const vision::localizer::Snapshot& camera);
    void show_autons(std::span<const char* const> names, std::size_t selected, const lemlib::Pose& odom);

    static std::optional<std::size_t> auton_at(int x, int y, std::size_t count);

private:
    static constexpr int kMapSize = 180;
    static constexpr int kMapOffsetX = 34;
    static constexpr int kMapOffsetY = 35;

    static int to_screen_x(double x_in);
    static int to_screen_y(double y_in);

    void draw_status(const char* text, std::uint32_t color);
    void draw_field();
    void draw_tick_labels();
    void draw_pose(const vision::localizer::FieldPose& pose, std::uint32_t color);
    void draw_legend(int y, const char* label, std::uint32_t color);
    void print_stats(const vision::localizer::FieldPose& odom, const vision::localizer::Snapshot& camera,
                     std::uint32_t camera_color);

    bool showing_autons = false;
    std::size_t shown_auton = 0;
    char shown_pose[48]{};
};

}
