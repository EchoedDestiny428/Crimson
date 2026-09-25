#include "display/dashboard.hpp"
#include "constants/field_constants.hpp"
#include "pros/screen.hpp"
#include <cmath>
#include <numbers>

namespace display {

namespace {

constexpr std::uint32_t kBackground = 0x1E1E24;
constexpr std::uint32_t kHeaderBackground = 0x15151A;
constexpr std::uint32_t kFieldBackground = 0x252830;
constexpr std::uint32_t kFieldLine = 0x4B5363;
constexpr std::uint32_t kRobotColor = 0x00F0B5;
constexpr std::uint32_t kRobotGlow = 0x117760;
constexpr std::uint32_t kRobotHeading = 0xFFFFFF;
constexpr std::uint32_t kGoalRed = 0xFF4D4D;
constexpr std::uint32_t kGoalBlue = 0x3B82F6;
constexpr std::uint32_t kGoalNeutral = 0xFBBF24;
constexpr std::uint32_t kTextWhite = 0xF8F9FA;
constexpr std::uint32_t kTextAccent = 0xA5B4FC;

constexpr int kScreenWidth = 480;
constexpr int kScreenHeight = 240;
constexpr int kHeaderHeight = 30;
constexpr int kStatusPanelX = 320;
constexpr int kStatusTextX = 340;
constexpr int kGoalRadius = 5;
constexpr int kRobotRadius = 5;
constexpr int kRobotGlowRadius = 8;
constexpr int kHeadingLength = 14;
constexpr int kTrailEraseRadius = 16;

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

void Dashboard::update(const std::optional<crimson::Pose2D>& pose, int tag_count, int primary_tag_id) {
    if (last_pose_) {
        pros::screen::set_pen(kBackground);
        pros::screen::fill_circle(field_to_screen_x(last_pose_->x), field_to_screen_y(last_pose_->y),
                                  kTrailEraseRadius);
    }

    draw_field();
    if (pose) {
        draw_robot(*pose);
    }
    last_pose_ = pose;

    print_stats(pose, tag_count, primary_tag_id);
}

int Dashboard::field_to_screen_x(double field_x) {
    return kMapOffsetX + static_cast<int>(field_x / field::kFieldLength * kMapSize);
}

int Dashboard::field_to_screen_y(double field_y) {
    return kMapOffsetY + kMapSize - static_cast<int>(field_y / field::kFieldWidth * kMapSize);
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

    const auto draw_line = [](const field::Point2D& from, const field::Point2D& to) {
        pros::screen::draw_line(field_to_screen_x(from.x), field_to_screen_y(from.y), field_to_screen_x(to.x),
                                field_to_screen_y(to.y));
    };
    draw_line(field::kNeutralGoalTopLeft, field::kNeutralGoalBottomRight);
    draw_line(field::kNeutralGoalBottomLeft, field::kNeutralGoalTopRight);

    const auto draw_goal = [](const field::Point2D& goal, std::uint32_t color) {
        pros::screen::set_pen(color);
        pros::screen::fill_circle(field_to_screen_x(goal.x), field_to_screen_y(goal.y), kGoalRadius);
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

void Dashboard::draw_robot(const crimson::Pose2D& pose) {
    const int x = field_to_screen_x(pose.x);
    const int y = field_to_screen_y(pose.y);

    pros::screen::set_pen(kRobotGlow);
    pros::screen::fill_circle(x, y, kRobotGlowRadius);
    pros::screen::set_pen(kRobotColor);
    pros::screen::fill_circle(x, y, kRobotRadius);

    const double heading = pose.theta * std::numbers::pi / 180.0;
    const int tip_x = x + static_cast<int>(kHeadingLength * std::sin(heading));
    const int tip_y = y - static_cast<int>(kHeadingLength * std::cos(heading));
    pros::screen::set_pen(kRobotHeading);
    pros::screen::draw_line(x, y, tip_x, tip_y);
}

void Dashboard::print_stats(const std::optional<crimson::Pose2D>& pose, int tag_count, int primary_tag_id) {
    const int text_x = kMapOffsetX + kMapSize + 25;

    pros::screen::set_pen(kBackground);
    pros::screen::fill_rect(text_x, kHeaderHeight, kScreenWidth, kScreenHeight);

    if (pose) {
        draw_status("STATUS: OK", kRobotColor);
    } else {
        draw_status("STATUS: LOST", kGoalRed);
    }

    pros::screen::set_pen(kTextAccent);
    pros::screen::print(pros::E_TEXT_MEDIUM, text_x, 45, "Global Coordinates");

    pros::screen::set_pen(kTextWhite);
    if (pose) {
        pros::screen::print(pros::E_TEXT_LARGE, text_x, 70, "X:  %5.0f mm", pose->x);
        pros::screen::print(pros::E_TEXT_LARGE, text_x, 105, "Y:  %5.0f mm", pose->y);
        pros::screen::print(pros::E_TEXT_LARGE, text_x, 140, "H:  %5.1f deg", pose->theta);
    } else {
        pros::screen::print(pros::E_TEXT_LARGE, text_x, 70, "X:  ---");
        pros::screen::print(pros::E_TEXT_LARGE, text_x, 105, "Y:  ---");
        pros::screen::print(pros::E_TEXT_LARGE, text_x, 140, "H:  ---");
    }

    pros::screen::set_pen(kTextAccent);
    pros::screen::print(pros::E_TEXT_MEDIUM, text_x, 185, "Sensor Diagnostics");

    pros::screen::set_pen(kTextWhite);
    if (tag_count > 0) {
        pros::screen::print(pros::E_TEXT_MEDIUM, text_x, 210, "Tags: %d | Primary ID: %d", tag_count, primary_tag_id);
    } else {
        pros::screen::print(pros::E_TEXT_MEDIUM, text_x, 210, "Tags: 0 | Primary ID: -");
    }
}

}
