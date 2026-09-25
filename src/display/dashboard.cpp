#include "display/dashboard.hpp"
#include "constants/field_constants.hpp"
#include "pros/screen.hpp"
#include "util/angle.hpp"
#include <algorithm>
#include <cmath>

extern "C" const char* const _PROS_COMPILE_TIMESTAMP;

namespace display {

namespace {

constexpr std::uint32_t kBackground = 0x1E1E24;
constexpr std::uint32_t kHeaderBackground = 0x15151A;
constexpr std::uint32_t kFieldBackground = 0x252830;
constexpr std::uint32_t kFieldLine = 0x4B5363;
constexpr std::uint32_t kOdomColor = 0x00F0B5;
constexpr std::uint32_t kVisionColor = 0xE879F9;
constexpr std::uint32_t kGoalRed = 0xFF4D4D;
constexpr std::uint32_t kGoalBlue = 0x3B82F6;
constexpr std::uint32_t kGoalNeutral = 0xFBBF24;
constexpr std::uint32_t kTextWhite = 0xF8F9FA;
constexpr std::uint32_t kTextAccent = 0xA5B4FC;
constexpr std::uint32_t kTextDim = 0x9CA3AF;

constexpr int kScreenWidth = 480;
constexpr int kScreenHeight = 240;
constexpr int kHeaderHeight = 30;
constexpr int kStatusPanelX = 320;
constexpr int kStatusTextX = 340;
constexpr int kPanelX = 235;
constexpr int kLegendTextX = kPanelX + 20;
constexpr int kLegendBoxOffsetY = 8;
constexpr int kGoalRadius = 5;
constexpr int kBoxHalf = 6;
constexpr int kHeadingLength = 14;

void draw_box(int center_x, int center_y, std::uint32_t color, bool filled) {
    pros::screen::set_pen(color);
    if (filled) {
        pros::screen::fill_rect(center_x - kBoxHalf, center_y - kBoxHalf, center_x + kBoxHalf, center_y + kBoxHalf);
    } else {
        pros::screen::draw_rect(center_x - kBoxHalf, center_y - kBoxHalf, center_x + kBoxHalf, center_y + kBoxHalf);
        pros::screen::draw_rect(center_x - kBoxHalf + 1, center_y - kBoxHalf + 1, center_x + kBoxHalf - 1,
                                center_y + kBoxHalf - 1);
    }
}

}

void Dashboard::initialize() {
    pros::screen::set_pen(kBackground);
    pros::screen::set_eraser(kBackground);
    pros::screen::erase();

    pros::screen::set_pen(kHeaderBackground);
    pros::screen::fill_rect(0, 0, kScreenWidth, kHeaderHeight);
    pros::screen::set_pen(kTextWhite);
    pros::screen::print(pros::E_TEXT_MEDIUM, 15, 6, "CRIMSON | POSE ESTIMATION");

    draw_field();
}

void Dashboard::show_status(const char* text) {
    draw_status(text, kTextAccent);
}

void Dashboard::update(const crimson::Pose2D& odom, const std::optional<crimson::Pose2D>& vision, int tag_count,
                       int primary_tag_id) {
    draw_field();
    draw_pose(odom, kOdomColor, true);
    if (vision) {
        draw_pose(*vision, kVisionColor, false);
    }
    print_stats(odom, vision, tag_count, primary_tag_id);
}

int Dashboard::to_screen_x(double x_in) {
    const double fraction = std::clamp(x_in * field::kMmPerInch / field::kFieldLength + 0.5, 0.0, 1.0);
    return kMapOffsetX + static_cast<int>(fraction * kMapSize);
}

int Dashboard::to_screen_y(double y_in) {
    const double fraction = std::clamp(y_in * field::kMmPerInch / field::kFieldWidth + 0.5, 0.0, 1.0);
    return kMapOffsetY + kMapSize - static_cast<int>(fraction * kMapSize);
}

void Dashboard::draw_status(const char* text, std::uint32_t color) {
    pros::screen::set_pen(kHeaderBackground);
    pros::screen::fill_rect(kStatusPanelX, 0, kScreenWidth, kHeaderHeight);
    pros::screen::set_pen(color);
    pros::screen::print(pros::E_TEXT_MEDIUM, kStatusTextX, 6, "%s", text);
}

void Dashboard::draw_field() {
    pros::screen::set_pen(kFieldBackground);
    pros::screen::fill_rect(kMapOffsetX, kMapOffsetY, kMapOffsetX + kMapSize, kMapOffsetY + kMapSize);

    pros::screen::set_pen(kFieldLine);
    pros::screen::draw_rect(kMapOffsetX, kMapOffsetY, kMapOffsetX + kMapSize, kMapOffsetY + kMapSize);

    const auto draw_line = [](const field::Point2D& from_mm, const field::Point2D& to_mm) {
        const field::Point2D from = field::to_lemlib_inches(from_mm);
        const field::Point2D to = field::to_lemlib_inches(to_mm);
        pros::screen::draw_line(to_screen_x(from.x), to_screen_y(from.y), to_screen_x(to.x), to_screen_y(to.y));
    };
    draw_line(field::kNeutralGoalTopLeft, field::kNeutralGoalBottomRight);
    draw_line(field::kNeutralGoalBottomLeft, field::kNeutralGoalTopRight);

    const auto draw_goal = [](const field::Point2D& goal_mm, std::uint32_t color) {
        const field::Point2D goal = field::to_lemlib_inches(goal_mm);
        pros::screen::set_pen(color);
        pros::screen::fill_circle(to_screen_x(goal.x), to_screen_y(goal.y), kGoalRadius);
    };
    draw_goal(field::kCenterGoal, kGoalNeutral);
    draw_goal(field::kNeutralGoalTopLeft, kGoalNeutral);
    draw_goal(field::kNeutralGoalBottomLeft, kGoalNeutral);
    draw_goal(field::kNeutralGoalTopRight, kGoalNeutral);
    draw_goal(field::kNeutralGoalBottomRight, kGoalNeutral);
    draw_goal(field::kRedGoalTop, kGoalRed);
    draw_goal(field::kRedGoalBottom, kGoalRed);
    draw_goal(field::kBlueGoalTop, kGoalBlue);
    draw_goal(field::kBlueGoalBottom, kGoalBlue);
}

void Dashboard::draw_pose(const crimson::Pose2D& pose, std::uint32_t color, bool filled) {
    const int x = std::clamp(to_screen_x(pose.x), kMapOffsetX + kBoxHalf, kMapOffsetX + kMapSize - kBoxHalf);
    const int y = std::clamp(to_screen_y(pose.y), kMapOffsetY + kBoxHalf, kMapOffsetY + kMapSize - kBoxHalf);
    draw_box(x, y, color, filled);

    const double heading = util::deg_to_rad(pose.theta);
    const int tip_x = std::clamp(x + static_cast<int>(std::lround(kHeadingLength * std::sin(heading))), kMapOffsetX,
                                 kMapOffsetX + kMapSize);
    const int tip_y = std::clamp(y - static_cast<int>(std::lround(kHeadingLength * std::cos(heading))), kMapOffsetY,
                                 kMapOffsetY + kMapSize);
    pros::screen::draw_line(x, y, tip_x, tip_y);
}

void Dashboard::draw_legend(int y, std::uint32_t color, bool filled, const char* label) {
    draw_box(kPanelX + kBoxHalf, y + kLegendBoxOffsetY, color, filled);
    pros::screen::print(pros::E_TEXT_MEDIUM, kLegendTextX, y, "%s", label);
}

void Dashboard::print_stats(const crimson::Pose2D& odom, const std::optional<crimson::Pose2D>& vision, int tag_count,
                            int primary_tag_id) {
    pros::screen::set_pen(kBackground);
    pros::screen::fill_rect(kPanelX, kHeaderHeight + 1, kScreenWidth, kScreenHeight);

    if (vision) {
        draw_status("VISION: OK", kVisionColor);
    } else {
        draw_status("VISION: LOST", kGoalRed);
    }

    draw_legend(40, kOdomColor, true, "ODOMETRY");
    pros::screen::set_pen(kTextWhite);
    pros::screen::print(pros::E_TEXT_MEDIUM, kPanelX, 62, "X %6.1f  Y %6.1f in", odom.x, odom.y);
    pros::screen::print(pros::E_TEXT_MEDIUM, kPanelX, 84, "H %6.1f deg", util::wrap_degrees(odom.theta));

    draw_legend(114, kVisionColor, false, "VISION");
    pros::screen::set_pen(kTextWhite);
    if (vision) {
        pros::screen::print(pros::E_TEXT_MEDIUM, kPanelX, 136, "X %6.1f  Y %6.1f in", vision->x, vision->y);
    } else {
        pros::screen::print(pros::E_TEXT_MEDIUM, kPanelX, 136, "X    ---  Y    ---");
    }
    if (tag_count > 0) {
        pros::screen::print(pros::E_TEXT_MEDIUM, kPanelX, 158, "Tags %d  ID %d", tag_count, primary_tag_id);
    } else {
        pros::screen::print(pros::E_TEXT_MEDIUM, kPanelX, 158, "Tags 0  ID -");
    }

    pros::screen::set_pen(kTextAccent);
    if (vision) {
        pros::screen::print(pros::E_TEXT_MEDIUM, kPanelX, 190, "Offset %5.1f in",
                            std::hypot(vision->x - odom.x, vision->y - odom.y));
    } else {
        pros::screen::print(pros::E_TEXT_MEDIUM, kPanelX, 190, "Offset ---");
    }

    pros::screen::set_pen(kTextDim);
    pros::screen::print(pros::E_TEXT_SMALL, kPanelX, 220, "Build %s", _PROS_COMPILE_TIMESTAMP);
}

}
