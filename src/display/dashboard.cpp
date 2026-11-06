#include "display/dashboard.hpp"
#include "constants/field_constants.hpp"
#include "pros/screen.hpp"
#include "util/angle.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

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
constexpr std::uint32_t kOdomColor = 0x00F0B5;
constexpr std::uint32_t kCameraColor = 0xE879F9;
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
constexpr int kMediumTextHalfHeight = 8;
constexpr int kStationGap = 2;
constexpr int kStationThickness = 3;
constexpr int kTickSpacingIn = 24;
constexpr int kTickLength = 4;
constexpr int kTickLabelGap = 3;
constexpr int kSmallCharWidth = 7;
constexpr int kSmallCharHalfHeight = 6;

constexpr int kOdomLegendY = 36;
constexpr int kOdomPoseY = 56;
constexpr int kCameraLegendY = 82;
constexpr int kCameraPoseY = 102;
constexpr int kCountsY = 126;
constexpr int kFitY = 148;
constexpr int kRawSizeY = 172;
constexpr int kRawCentreY = 194;
constexpr int kBuildY = 222;

constexpr int kAutonColumns = 3;
constexpr int kAutonMargin = 12;
constexpr int kAutonGap = 10;
constexpr int kAutonTop = kHeaderHeight + kAutonMargin;
constexpr int kAutonTextPadding = 14;
constexpr int kAutonFooterHeight = 36;
constexpr int kAutonBottom = kScreenHeight - kAutonFooterHeight;
constexpr int kPoseTextY = kScreenHeight - 26;

struct Rect {
    int x0;
    int y0;
    int x1;
    int y1;
};

Rect auton_button(std::size_t index, std::size_t count) {
    const int rows = static_cast<int>((count + kAutonColumns - 1) / kAutonColumns);
    const int width = (kScreenWidth - 2 * kAutonMargin - (kAutonColumns - 1) * kAutonGap) / kAutonColumns;
    const int height = (kAutonBottom - kAutonTop - (rows - 1) * kAutonGap) / rows;
    const int x = kAutonMargin + static_cast<int>(index % kAutonColumns) * (width + kAutonGap);
    const int y = kAutonTop + static_cast<int>(index / kAutonColumns) * (height + kAutonGap);
    return {x, y, x + width, y + height};
}

void draw_header(const char* title) {
    pros::screen::set_pen(kHeaderBackground);
    pros::screen::fill_rect(0, 0, kScreenWidth, kHeaderHeight);
    pros::screen::set_pen(kTextWhite);
    pros::screen::print(pros::E_TEXT_MEDIUM, kTitleX, kTitleY, "%s", title);
}

void draw_auton_button(const Rect& rect, const char* name, bool selected) {
    const std::uint32_t fill = selected ? kOdomColor : kFieldBackground;
    pros::screen::set_pen(fill);
    pros::screen::fill_rect(rect.x0, rect.y0, rect.x1, rect.y1);
    pros::screen::set_pen(selected ? kOdomColor : kFieldLine);
    pros::screen::draw_rect(rect.x0, rect.y0, rect.x1, rect.y1);

    pros::screen::set_eraser(fill);
    pros::screen::set_pen(selected ? kHeaderBackground : kTextWhite);
    pros::screen::print(pros::E_TEXT_MEDIUM, rect.x0 + kAutonTextPadding,
                        (rect.y0 + rect.y1) / 2 - kMediumTextHalfHeight, "%s", name);
    pros::screen::set_eraser(kBackground);
}

void draw_box(int center_x, int center_y, std::uint32_t color) {
    pros::screen::set_pen(color);
    pros::screen::fill_rect(center_x - kBoxHalf, center_y - kBoxHalf, center_x + kBoxHalf, center_y + kBoxHalf);
}

bool pose_valid(const lemlib::Pose& pose) {
    return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.theta);
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
        return kCameraColor;
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

void Dashboard::show_status(const char* text) {
    draw_status(text, kTextAccent);
}

void Dashboard::update(const lemlib::Pose& odom, const Snapshot& camera) {
    if (showing_autons) {
        showing_autons = false;
        initialize();
    }

    const std::uint32_t camera_color = camera.status == Status::Locked ? kCameraColor : kTextDim;
    if (camera.status == Status::Ambiguous) {
        char text[16];
        std::snprintf(text, sizeof(text), "%d CANDIDATES", camera.candidates);
        draw_status(text, status_color(camera.status));
    } else {
        draw_status(status_text(camera.status), status_color(camera.status));
    }

    const FieldPose robot{odom.x, odom.y, odom.theta};
    draw_field();
    draw_pose(camera.pose, camera_color);
    draw_pose(robot, kOdomColor);
    print_stats(robot, camera, camera_color);
}

void Dashboard::show_autons(std::span<const char* const> names, std::size_t selected, const lemlib::Pose& odom) {
    const bool first = !showing_autons;
    if (first) {
        pros::screen::set_eraser(kBackground);
        pros::screen::erase();
        draw_header("CRIMSON | AUTON");
    }
    if (first || selected != shown_auton) {
        for (std::size_t i = 0; i < names.size(); ++i) {
            draw_auton_button(auton_button(i, names.size()), names[i], i == selected);
        }
    }

    char pose[sizeof(shown_pose)];
    std::snprintf(pose, sizeof(pose), "X %6.1f  Y %6.1f  H %6.1f", odom.x, odom.y, util::wrap_degrees(odom.theta));
    if (first || std::strcmp(pose, shown_pose) != 0) {
        pros::screen::set_pen(kBackground);
        pros::screen::fill_rect(0, kAutonBottom + 1, kScreenWidth, kScreenHeight);
        pros::screen::set_pen(pose_valid(odom) ? kOdomColor : kGoalRed);
        pros::screen::print(pros::E_TEXT_MEDIUM, kAutonMargin, kPoseTextY, "%s", pose);
        std::strcpy(shown_pose, pose);
    }

    showing_autons = true;
    shown_auton = selected;
}

std::optional<std::size_t> Dashboard::auton_at(int x, int y, std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) {
        const Rect rect = auton_button(i, count);
        if (x >= rect.x0 && x <= rect.x1 && y >= rect.y0 && y <= rect.y1) {
            return i;
        }
    }
    return std::nullopt;
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

void Dashboard::print_stats(const FieldPose& odom, const Snapshot& camera, std::uint32_t camera_color) {
    pros::screen::set_pen(kBackground);
    pros::screen::fill_rect(kPanelX, kHeaderHeight + 1, kScreenWidth, kScreenHeight);

    draw_legend(kOdomLegendY, "ODOMETRY", kOdomColor);
    pros::screen::set_pen(kTextWhite);
    pros::screen::print(pros::E_TEXT_MEDIUM, kPanelX, kOdomPoseY, "X %5.1f Y %5.1f H %5.1f", odom.x_in, odom.y_in,
                        util::wrap_degrees(odom.heading_deg));

    draw_legend(kCameraLegendY, "CAMERA", camera_color);
    pros::screen::set_pen(kTextWhite);
    pros::screen::print(pros::E_TEXT_MEDIUM, kPanelX, kCameraPoseY, "X %5.1f Y %5.1f H %5.1f", camera.pose.x_in,
                        camera.pose.y_in, camera.pose.heading_deg);

    pros::screen::set_pen(kTextAccent);
    pros::screen::print(pros::E_TEXT_MEDIUM, kPanelX, kCountsY, "TAGS %d  GOALS %d  FITS %d", camera.tags,
                        camera.goals, camera.candidates);
    if (camera.candidates > 0) {
        pros::screen::print(pros::E_TEXT_MEDIUM, kPanelX, kFitY, "ERR %.1f px  R %.1f in", camera.error_px,
                            camera.range_in);
    }

    pros::screen::set_pen(kTextDim);
    if (camera.raw) {
        pros::screen::print(pros::E_TEXT_MEDIUM, kPanelX, kRawSizeY, "RAW ID %d  W %.0f  H %.0f", camera.raw->id,
                            camera.raw->width_px, camera.raw->height_px);
        pros::screen::print(pros::E_TEXT_MEDIUM, kPanelX, kRawCentreY, "CENTRE U %.0f  V %.0f", camera.raw->u,
                            camera.raw->v);
    }
    pros::screen::print(pros::E_TEXT_SMALL, kPanelX, kBuildY, "Build %s", _PROS_COMPILE_TIMESTAMP);
}

}
