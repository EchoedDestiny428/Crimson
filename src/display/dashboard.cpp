#include "display/dashboard.hpp"
#include "constants/field_constants.hpp"
#include "pros/screen.hpp"
#include "util/angle.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

extern "C" const char* const _PROS_COMPILE_TIMESTAMP;

namespace display {

namespace {

using vision::localizer::FieldPose;
using vision::localizer::Snapshot;
using vision::localizer::Status;

constexpr std::uint32_t kBackground = 0x1E1E24;
constexpr std::uint32_t kHeaderBackground = 0x15151A;
constexpr std::uint32_t kFieldBackground = 0x252830;
constexpr std::uint32_t kFieldLine = 0x4B5363;
constexpr std::uint32_t kPoseColor = 0x00F0B5;
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
constexpr int kTitleX = 15;
constexpr int kTitleY = 6;
constexpr int kStationGap = 2;
constexpr int kStationThickness = 3;
constexpr int kTickSpacingIn = 24;
constexpr int kTickLength = 4;
constexpr int kTickLabelGap = 3;
constexpr int kSmallCharWidth = 7;
constexpr int kSmallCharHalfHeight = 6;

constexpr int kLegendY = 40;
constexpr int kPositionY = 62;
constexpr int kHeadingY = 84;
constexpr int kCountsY = 112;
constexpr int kFitY = 134;
constexpr int kRawSizeY = 162;
constexpr int kRawCentreY = 184;
constexpr int kBuildY = 220;

void draw_header(const char* title) {
    pros::screen::set_pen(kHeaderBackground);
    pros::screen::fill_rect(0, 0, kScreenWidth, kHeaderHeight);
    pros::screen::set_pen(kTextWhite);
    pros::screen::print(pros::E_TEXT_MEDIUM, kTitleX, kTitleY, "%s", title);
}

void draw_box(int center_x, int center_y, std::uint32_t color) {
    pros::screen::set_pen(color);
    pros::screen::fill_rect(center_x - kBoxHalf, center_y - kBoxHalf, center_x + kBoxHalf, center_y + kBoxHalf);
}

std::uint32_t goal_color(field::GoalType type) {
    switch (type) {
    case field::GoalType::Red:
        return kGoalRed;
    case field::GoalType::Blue:
        return kGoalBlue;
    default:
        return kGoalNeutral;
    }
}

std::uint32_t status_color(Status status) {
    switch (status) {
    case Status::Locked:
        return kPoseColor;
    case Status::Ambiguous:
        return kGoalNeutral;
    case Status::NoImu:
    case Status::NoSensor:
        return kGoalRed;
    default:
        return kTextAccent;
    }
}

const char* status_text(Status status) {
    switch (status) {
    case Status::Locked:
        return "LOCKED";
    case Status::Searching:
        return "SEARCHING";
    case Status::NoSensor:
        return "NO SENSOR";
    case Status::NoImu:
        return "NO IMU";
    default:
        return "CALIBRATING";
    }
}

}

void Dashboard::initialize() {
    pros::screen::set_pen(kBackground);
    pros::screen::set_eraser(kBackground);
    pros::screen::erase();

    draw_header("CRIMSON | POSE ESTIMATION");
    draw_tick_labels();
    draw_field();
}

void Dashboard::update(const Snapshot& snapshot) {
    const std::uint32_t pose_color = snapshot.status == Status::Locked ? kPoseColor : kTextDim;

    if (snapshot.status == Status::Ambiguous) {
        char text[16];
        std::snprintf(text, sizeof(text), "%d CANDIDATES", snapshot.candidates);
        draw_status(text, status_color(snapshot.status));
    } else {
        draw_status(status_text(snapshot.status), status_color(snapshot.status));
    }

    draw_field();
    draw_pose(snapshot.pose, pose_color);
    print_stats(snapshot, pose_color);
}

int Dashboard::to_screen_x(double x_in) {
    const double fraction = std::clamp(x_in / field::kFieldSizeIn, 0.0, 1.0);
    return kMapOffsetX + static_cast<int>(fraction * kMapSize);
}

int Dashboard::to_screen_y(double y_in) {
    const double fraction = std::clamp(y_in / field::kFieldSizeIn, 0.0, 1.0);
    return kMapOffsetY + kMapSize - static_cast<int>(fraction * kMapSize);
}

void Dashboard::draw_status(const char* text, std::uint32_t color) {
    pros::screen::set_pen(kHeaderBackground);
    pros::screen::fill_rect(kStatusPanelX, 0, kScreenWidth, kHeaderHeight);
    pros::screen::set_pen(color);
    pros::screen::print(pros::E_TEXT_MEDIUM, kStatusTextX, kTitleY, "%s", text);
}

void Dashboard::draw_field() {
    pros::screen::set_pen(kFieldBackground);
    pros::screen::fill_rect(kMapOffsetX, kMapOffsetY, kMapOffsetX + kMapSize, kMapOffsetY + kMapSize);

    pros::screen::set_pen(kFieldLine);
    pros::screen::draw_rect(kMapOffsetX, kMapOffsetY, kMapOffsetX + kMapSize, kMapOffsetY + kMapSize);

    const int map_bottom = kMapOffsetY + kMapSize;
    pros::screen::set_pen(kTextDim);
    for (int inches = kTickSpacingIn; inches < field::kFieldSizeIn; inches += kTickSpacingIn) {
        const int x = to_screen_x(inches);
        const int y = to_screen_y(inches);
        pros::screen::draw_line(x, map_bottom, x, map_bottom - kTickLength);
        pros::screen::draw_line(kMapOffsetX, y, kMapOffsetX + kTickLength, y);
    }

    const int blue_station_x = kMapOffsetX - kStationGap;
    const int red_station_x = kMapOffsetX + kMapSize + kStationGap;

    pros::screen::set_pen(kGoalBlue);
    pros::screen::draw_rect(blue_station_x - kStationThickness, kMapOffsetY, blue_station_x, map_bottom);
    pros::screen::set_pen(kGoalRed);
    pros::screen::draw_rect(red_station_x, kMapOffsetY, red_station_x + kStationThickness, map_bottom);

    for (const field::Goal& goal : field::kGoals) {
        pros::screen::set_pen(goal_color(goal.type));
        pros::screen::fill_circle(to_screen_x(goal.pos.x / field::kMmPerInch),
                                  to_screen_y(goal.pos.y / field::kMmPerInch), kGoalRadius);
    }
}

void Dashboard::draw_tick_labels() {
    const int y_labels_right = kMapOffsetX - kStationGap - kStationThickness - kTickLabelGap;
    const int x_labels_top = kMapOffsetY + kMapSize + kTickLabelGap;

    pros::screen::set_pen(kTextDim);
    for (int inches = 0; inches < field::kFieldSizeIn; inches += kTickSpacingIn) {
        char label[8];
        const int width = std::snprintf(label, sizeof(label), "%d", inches) * kSmallCharWidth;
        pros::screen::print(pros::E_TEXT_SMALL, to_screen_x(inches) - width / 2, x_labels_top, "%s", label);
        pros::screen::print(pros::E_TEXT_SMALL, y_labels_right - width, to_screen_y(inches) - kSmallCharHalfHeight,
                            "%s", label);
    }

    pros::screen::set_pen(kTextAccent);
    pros::screen::print(pros::E_TEXT_SMALL, kMapOffsetX + kMapSize - kSmallCharWidth, x_labels_top, "X");
    pros::screen::print(pros::E_TEXT_SMALL, y_labels_right - kSmallCharWidth, kMapOffsetY, "Y");
}

void Dashboard::draw_pose(const FieldPose& pose, std::uint32_t color) {
    const int x = std::clamp(to_screen_x(pose.x_in), kMapOffsetX + kBoxHalf, kMapOffsetX + kMapSize - kBoxHalf);
    const int y = std::clamp(to_screen_y(pose.y_in), kMapOffsetY + kBoxHalf, kMapOffsetY + kMapSize - kBoxHalf);
    draw_box(x, y, color);

    // Screen rows grow downwards, so a clockwise heading turns the line from +X towards the bottom of the map.
    const double heading = util::deg_to_rad(pose.heading_deg);
    const int tip_x = std::clamp(x + static_cast<int>(std::lround(kHeadingLength * std::cos(heading))), kMapOffsetX,
                                 kMapOffsetX + kMapSize);
    const int tip_y = std::clamp(y + static_cast<int>(std::lround(kHeadingLength * std::sin(heading))), kMapOffsetY,
                                 kMapOffsetY + kMapSize);
    pros::screen::draw_line(x, y, tip_x, tip_y);
}

void Dashboard::draw_legend(int y, const char* label, std::uint32_t color) {
    draw_box(kPanelX + kBoxHalf, y + kLegendBoxOffsetY, color);
    pros::screen::print(pros::E_TEXT_MEDIUM, kLegendTextX, y, "%s", label);
}

void Dashboard::print_stats(const Snapshot& snapshot, std::uint32_t pose_color) {
    pros::screen::set_pen(kBackground);
    pros::screen::fill_rect(kPanelX, kHeaderHeight + 1, kScreenWidth, kScreenHeight);

    draw_legend(kLegendY, "CAMERA POSE", pose_color);
    pros::screen::set_pen(kTextWhite);
    pros::screen::print(pros::E_TEXT_MEDIUM, kPanelX, kPositionY, "X %6.1f  Y %6.1f in", snapshot.pose.x_in,
                        snapshot.pose.y_in);
    pros::screen::print(pros::E_TEXT_MEDIUM, kPanelX, kHeadingY, "H %6.1f deg", snapshot.pose.heading_deg);

    pros::screen::set_pen(kTextAccent);
    pros::screen::print(pros::E_TEXT_MEDIUM, kPanelX, kCountsY, "TAGS %d  GOALS %d  FITS %d", snapshot.tags,
                        snapshot.goals, snapshot.candidates);
    if (snapshot.candidates > 0) {
        pros::screen::print(pros::E_TEXT_MEDIUM, kPanelX, kFitY, "ERR %.1f px  R %.1f in", snapshot.error_px,
                            snapshot.range_in);
    }

    pros::screen::set_pen(kTextDim);
    if (snapshot.raw) {
        pros::screen::print(pros::E_TEXT_MEDIUM, kPanelX, kRawSizeY, "RAW ID %d  W %.0f  H %.0f", snapshot.raw->id,
                            snapshot.raw->width_px, snapshot.raw->height_px);
        pros::screen::print(pros::E_TEXT_MEDIUM, kPanelX, kRawCentreY, "CENTRE U %.0f  V %.0f", snapshot.raw->u,
                            snapshot.raw->v);
    }
    pros::screen::print(pros::E_TEXT_SMALL, kPanelX, kBuildY, "Build %s", _PROS_COMPILE_TIMESTAMP);
}

}
