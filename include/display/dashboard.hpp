#ifndef DASHBOARD_HPP_
#define DASHBOARD_HPP_

#include "vision/crimson.hpp"

namespace display {

class Dashboard {
public:
    Dashboard();

    void initialize();
    void update(const std::optional<crimson::Pose2D>& pose, int tv, int tag_id);

private:
    void draw_field();
    void draw_robot(const crimson::Pose2D& pose);
    void print_stats(const std::optional<crimson::Pose2D>& pose, int tv, int tag_id);

    int field_to_screen_x(double field_x) const;
    int field_to_screen_y(double field_y) const;

    static constexpr int kMapSize = 195; // pixels
    static constexpr int kMapOffsetX = 15;
    static constexpr int kMapOffsetY = 38;

    std::optional<crimson::Pose2D> last_pose_;
};

} // namespace display

#endif
