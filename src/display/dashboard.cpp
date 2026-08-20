#include "display/dashboard.hpp"
#include "constants/field_constants.hpp"
#include "pros/screen.hpp"
#include <cmath>
#include <cstdio>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Modern UI Color Palette
constexpr uint32_t BG_COLOR        = 0x1E1E24; // Dark charcoal
constexpr uint32_t HEADER_BG       = 0x15151A; // Darker header
constexpr uint32_t FIELD_BG        = 0x252830; // Soft field gray
constexpr uint32_t FIELD_LINE      = 0x4B5363; // Soft line color
constexpr uint32_t ROBOT_COLOR     = 0x00F0B5; // Neon mint
constexpr uint32_t ROBOT_GLOW      = 0x117760; // Darker mint for shadow
constexpr uint32_t ROBOT_HEADING   = 0xFFFFFF; // White heading
constexpr uint32_t GOAL_RED        = 0xFF4D4D; // Vibrant Red
constexpr uint32_t GOAL_BLUE       = 0x3B82F6; // Vibrant Blue
constexpr uint32_t GOAL_NEUTRAL    = 0xFBBF24; // Gold
constexpr uint32_t TEXT_WHITE      = 0xF8F9FA; // Off-white
constexpr uint32_t TEXT_ACCENT     = 0xA5B4FC; // Light purple-blue for headings

namespace display {

Dashboard::Dashboard() {}

void Dashboard::initialize() {
    pros::screen::set_pen(BG_COLOR);
    pros::screen::set_eraser(BG_COLOR);
    pros::screen::erase();

    // Draw Header Top Bar
    pros::screen::set_pen(HEADER_BG);
    pros::screen::fill_rect(0, 0, 480, 30);
    
    pros::screen::set_pen(TEXT_WHITE);
    pros::screen::print(pros::E_TEXT_MEDIUM, 15, 6, "CRIMSON | POSE ESTIMATION");

    draw_field();
}

int Dashboard::field_to_screen_x(double field_x) const {
    return kMapOffsetX + static_cast<int>((field_x / field::kFieldLength) * kMapSize);
}

int Dashboard::field_to_screen_y(double field_y) const {
    return kMapOffsetY + kMapSize - static_cast<int>((field_y / field::kFieldWidth) * kMapSize);
}

void Dashboard::draw_field() {
    // Fill Field Area
    pros::screen::set_pen(FIELD_BG);
    pros::screen::fill_rect(kMapOffsetX, kMapOffsetY, kMapOffsetX + kMapSize, kMapOffsetY + kMapSize);

    // Outline Field
    pros::screen::set_pen(FIELD_LINE);
    pros::screen::draw_rect(kMapOffsetX, kMapOffsetY, kMapOffsetX + kMapSize, kMapOffsetY + kMapSize);

    // Draw midfield cross to give visual structure
    pros::screen::draw_line(field_to_screen_x(field::kNeutralGoalTopLeft.x), field_to_screen_y(field::kNeutralGoalTopLeft.y),
                            field_to_screen_x(field::kNeutralGoalBottomRight.x), field_to_screen_y(field::kNeutralGoalBottomRight.y));
    pros::screen::draw_line(field_to_screen_x(field::kNeutralGoalBottomLeft.x), field_to_screen_y(field::kNeutralGoalBottomLeft.y),
                            field_to_screen_x(field::kNeutralGoalTopRight.x), field_to_screen_y(field::kNeutralGoalTopRight.y));

    // Plot Goals
    auto draw_goal = [&](const field::Point2D& p, uint32_t color) {
        pros::screen::set_pen(color);
        pros::screen::fill_circle(field_to_screen_x(p.x), field_to_screen_y(p.y), 5);
    };

    draw_goal(field::kCenterGoal, GOAL_NEUTRAL);
    draw_goal(field::kNeutralGoalTopLeft, GOAL_NEUTRAL);
    draw_goal(field::kNeutralGoalBottomLeft, GOAL_NEUTRAL);
    draw_goal(field::kNeutralGoalTopRight, GOAL_NEUTRAL);
    draw_goal(field::kNeutralGoalBottomRight, GOAL_NEUTRAL);

    draw_goal(field::kRedGoalTop, GOAL_RED);
    draw_goal(field::kRedGoalBottom, GOAL_RED);
    
    draw_goal(field::kBlueGoalTop, GOAL_BLUE);
    draw_goal(field::kBlueGoalBottom, GOAL_BLUE);
}

void Dashboard::draw_robot(const crimson::Pose2D& pose) {
    int rx = field_to_screen_x(pose.x);
    int ry = field_to_screen_y(pose.y);

    // Draw outer glow/shadow layer
    pros::screen::set_pen(ROBOT_GLOW);
    pros::screen::fill_circle(rx, ry, 8);

    // Draw solid inner core
    pros::screen::set_pen(ROBOT_COLOR);
    pros::screen::fill_circle(rx, ry, 5);

    // Draw pointing heading line
    double heading_rad = pose.theta * M_PI / 180.0;
    int lx = rx + static_cast<int>(14 * std::cos(heading_rad));
    int ly = ry - static_cast<int>(14 * std::sin(heading_rad));

    pros::screen::set_pen(ROBOT_HEADING);
    pros::screen::draw_line(rx, ry, lx, ly);
}

void Dashboard::print_stats(const std::optional<crimson::Pose2D>& pose, int tv, int tag_id) {
    int text_x = kMapOffsetX + kMapSize + 25; // padding
    
    // Clear right side data panel
    pros::screen::set_pen(BG_COLOR);
    pros::screen::fill_rect(text_x, 30, 480, 240);

    // Status Indicator in Header
    pros::screen::set_pen(HEADER_BG);
    pros::screen::fill_rect(320, 0, 480, 30); // clear top right
    
    if (pose.has_value()) {
        pros::screen::set_pen(ROBOT_COLOR);
        pros::screen::print(pros::E_TEXT_MEDIUM, 340, 6, "STATUS: OK");
    } else {
        pros::screen::set_pen(GOAL_RED);
        pros::screen::print(pros::E_TEXT_MEDIUM, 340, 6, "STATUS: LOST");
    }

    // --- Coordinates Panel ---
    pros::screen::set_pen(TEXT_ACCENT);
    pros::screen::print(pros::E_TEXT_MEDIUM, text_x, 45, "Global Coordinates");
    
    pros::screen::set_pen(TEXT_WHITE);
    char buf[64];
    if (pose.has_value()) {
        std::snprintf(buf, sizeof(buf), "X:  %5.0f mm", pose->x);
        pros::screen::print(pros::E_TEXT_LARGE, text_x, 70, buf);
        
        std::snprintf(buf, sizeof(buf), "Y:  %5.0f mm", pose->y);
        pros::screen::print(pros::E_TEXT_LARGE, text_x, 105, buf);
        
        std::snprintf(buf, sizeof(buf), "H:  %5.1f deg", pose->theta);
        pros::screen::print(pros::E_TEXT_LARGE, text_x, 140, buf);
    } else {
        pros::screen::print(pros::E_TEXT_LARGE, text_x, 70, "X:  ---");
        pros::screen::print(pros::E_TEXT_LARGE, text_x, 105, "Y:  ---");
        pros::screen::print(pros::E_TEXT_LARGE, text_x, 140, "H:  ---");
    }

    // --- Vision Details Panel ---
    pros::screen::set_pen(TEXT_ACCENT);
    pros::screen::print(pros::E_TEXT_MEDIUM, text_x, 185, "Sensor Diagnostics");

    pros::screen::set_pen(TEXT_WHITE);
    if (tv) {
        std::snprintf(buf, sizeof(buf), "Tags: %d | Primary ID: %d", tv, tag_id);
        pros::screen::print(pros::E_TEXT_MEDIUM, text_x, 210, buf);
    } else {
        pros::screen::print(pros::E_TEXT_MEDIUM, text_x, 210, "Tags: 0 | Primary ID: -");
    }
}

void Dashboard::update(const std::optional<crimson::Pose2D>& pose, int tv, int tag_id) {
    // Wipe the old robot trail
    if (last_pose_.has_value()) {
        int old_rx = field_to_screen_x(last_pose_->x);
        int old_ry = field_to_screen_y(last_pose_->y);
        pros::screen::set_pen(FIELD_BG);
        pros::screen::fill_circle(old_rx, old_ry, 16); 
    }
    
    // Always refresh the map underneath to ensure the lines/goals remain crisp
    draw_field();

    if (pose.has_value()) {
        draw_robot(*pose);
    }

    last_pose_ = pose;
    print_stats(pose, tv, tag_id);
}

} // namespace display
