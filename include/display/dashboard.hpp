#pragma once

#include "vision/crimson.hpp"
#include <cstdint>
#include <optional>

namespace display {

class Dashboard {
public:
    void initialize();
    void show_status(const char* text);
    void update(const std::optional<crimson::Pose2D>& pose, int tag_count, int primary_tag_id);

private:
    static constexpr int kMapSize = 195;
    static constexpr int kMapOffsetX = 15;
    static constexpr int kMapOffsetY = 38;

    static int field_to_screen_x(double field_x);
    static int field_to_screen_y(double field_y);

    void draw_status(const char* text, std::uint32_t color);
    void draw_field();
    void draw_robot(const crimson::Pose2D& pose);
    void print_stats(const std::optional<crimson::Pose2D>& pose, int tag_count, int primary_tag_id);

    std::optional<crimson::Pose2D> last_pose_;
};

}
