#pragma once

#include "vision/localizer.hpp"
#include <cstdint>

namespace display {

class Dashboard {
public:
    void initialize();
    void update(const vision::localizer::Snapshot& snapshot);

private:
    static constexpr int kMapSize = 195;
    static constexpr int kMapOffsetX = 15;
    static constexpr int kMapOffsetY = 38;

    static int to_screen_x(double x_mm);
    static int to_screen_y(double y_mm);

    void draw_status(const char* text, std::uint32_t color);
    void draw_field();
    void draw_pose(const vision::Pose& pose, std::uint32_t color);
    void draw_legend(int y, const char* label, std::uint32_t color);
    void print_stats(const vision::localizer::Snapshot& snapshot, std::uint32_t pose_color);
};

}
