#pragma once

#include "vision/localizer.hpp"
#include <cstdint>

namespace display {

class Dashboard {
public:
    void initialize();
    void update(const vision::localizer::Snapshot& snapshot);

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
    void print_stats(const vision::localizer::Snapshot& snapshot, std::uint32_t pose_color);
};

}
