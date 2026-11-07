#pragma once

#include "constants/field_constants.hpp"
#include "motion/chassis.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <optional>

namespace crimson {

enum class Side { Auto, Front, Behind };

inline constexpr field::Color kOurColor = field::Color::Blue;
inline constexpr Side kScoreSide = Side::Behind;
inline constexpr std::size_t kMaxPins = 8; // unsure

inline constexpr double kDriveInPerSec = 40.0; // placeholder
inline constexpr double kTurnDegPerSec = 360.0; // placeholder
inline constexpr double kDonutDegPerSec = 180.0; // placeholder
inline constexpr double kTimeoutScale = 1.5; // placeholder
inline constexpr int kTimeoutPadMs = 300; // placeholder

inline constexpr DonutParams kDonutParams{.fastSpeed = 100, .slowSpeed = 75}; // placeholder

inline constexpr double kPickupTipCm = 10.0; // placeholder
inline constexpr std::uint32_t kPickupTimeoutMs = 5000; // placeholder

inline constexpr bool kRamForwards = true; // placeholder
inline constexpr double kToggleApproachIn = 18.0; // placeholder
inline constexpr int kRamPushMs = 700; // placeholder
inline constexpr float kRamMinSpeed = 80.0f; // placeholder

inline constexpr float kResetMaxCorrectionIn = 10.0f; // placeholder

class Chaos {
public:
    Chaos();

    void reset();
    int pins(field::GoalId goal) const;
    std::optional<field::Color> top(field::GoalId goal) const;
    void setPins(field::GoalId goal, std::initializer_list<field::Color> colors);
    double stackTopCm(field::GoalId goal) const;
    double releaseCm(field::GoalId goal) const;
    field::Color toggleColor(field::Wall wall) const;
    void setToggle(field::Wall wall, field::Color color);
    std::optional<field::Color> holding() const;

    void start(float x, float y, float heading);
    void grab(field::Color color = kOurColor);
    void release();
    bool pickup(field::Color color = kOurColor, std::uint32_t timeoutMs = kPickupTimeoutMs);
    bool home();
    bool moveTo(field::GoalId goal, Side side = kScoreSide);
    bool score(field::GoalId goal, Side side = kScoreSide);
    bool donutWhileScoring(field::GoalId goal, Side side = kScoreSide, DonutParams params = kDonutParams);
    void toggle(field::Wall wall, bool forwards = kRamForwards);
    bool resetFrom(field::GoalId goal, Side side = Side::Auto);

private:
    struct Stack {
        std::array<field::Color, kMaxPins> colors{};
        std::size_t count = 0;
    };

    bool full(field::GoalId goal) const;
    void place(field::GoalId goal);
    void ram(field::Wall wall, bool forwards);

    std::array<Stack, field::kGoalCount> stacks{};
    std::array<field::Color, field::kToggleBars.size()> toggles{};
    std::optional<field::Color> held;
};

}

extern crimson::Chaos chaos;
