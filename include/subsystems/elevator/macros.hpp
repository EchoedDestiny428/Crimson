#pragma once

#include "subsystems/elevator/constants.hpp"
#include <cstdint>
#include <optional>

namespace subsystems::elevator::macros {

struct Pose {
    double lift_cm;
    double arm_deg;
};

bool go_to_pose(Pose pose);
void aim(Pose pose);
bool settled();
bool drop();

inline constexpr double kArmMinFromDownDeg = 25.0;
inline constexpr double kArmFrontMinDeg = kArmMinFromDownDeg;
inline constexpr double kArmFrontMaxDeg = 180.0;
inline constexpr double kArmBehindMinDeg = 180.0;
inline constexpr double kArmBehindMaxDeg = 360.0 - kArmMinFromDownDeg;

bool solve(double tip_cm, Pose& out, double arm_min_deg = kArmFrontMinDeg, double arm_max_deg = kArmBehindMaxDeg);

bool move_to(double tip_cm);

bool flip_out(double lift_cm = constants::kExtensionFlipOutCm, double arm_deg = constants::kArmFlippedDeg);
bool go_to_lift(double lift_cm);
bool go_to_stage(int stage);
bool home();
bool dual_setup();
bool dual_pickup();
bool dual_pickup_height(double lift_cm);

bool pickup(std::uint32_t timeout_ms = 5000);

bool score(constants::Goal goal, int level);

struct Approach {
    float x;
    float y;
    bool forwards;
};

double travel_seconds(const Pose& to);
std::optional<Approach> approach(const Pose& pose, float goal_x_in, float goal_y_in);
bool reach(const Pose& pose, float goal_x_in, float goal_y_in, std::uint32_t timeout_ms);

bool score(constants::Goal goal, int level, float goal_x_in, float goal_y_in, std::uint32_t timeout_ms = 3000);
bool score_front(constants::Goal goal, int level, float goal_x_in, float goal_y_in, std::uint32_t timeout_ms = 3000);
bool score_behind(constants::Goal goal, int level, float goal_x_in, float goal_y_in, std::uint32_t timeout_ms = 3000);

double claw_offset_in(const Pose& pose);

void cancel();

inline constexpr constants::Goal kStageGoal = constants::Goal::Alliance;
inline constexpr int kTopStage = static_cast<int>(constants::kAllianceReleaseCm.size());
bool go_to_level(int stage);

void update();
void stop();

Pose pose_now();
double tip_now();
int snap_stage_now();
int goal_tier_now();
Pose target_now();
int target_stage();

}
