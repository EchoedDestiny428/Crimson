#pragma once

#include <array>

namespace field {

struct Point2D {
    double x;
    double y;
};

enum class GoalType { Neutral, Red, Blue };

struct Goal {
    Point2D pos;
    GoalType type;
};

inline constexpr double kMmPerInch = 25.4;
inline constexpr double kTileInches = 24.0;
inline constexpr double kTileMm = kTileInches * kMmPerInch;
inline constexpr double kFieldTiles = 6.0;
inline constexpr double kFieldLength = kFieldTiles * kTileMm;
inline constexpr double kFieldWidth = kFieldTiles * kTileMm;

inline constexpr Point2D from_center_tiles(double x, double y) {
    return {x * kTileMm + kFieldLength / 2.0, y * kTileMm + kFieldWidth / 2.0};
}

inline constexpr std::array kGoals{
    Goal{from_center_tiles(0, 0), GoalType::Neutral},  Goal{from_center_tiles(-2, 1), GoalType::Neutral},
    Goal{from_center_tiles(2, -1), GoalType::Neutral}, Goal{from_center_tiles(-1, 2), GoalType::Neutral},
    Goal{from_center_tiles(1, -2), GoalType::Neutral}, Goal{from_center_tiles(-2, -1), GoalType::Red},
    Goal{from_center_tiles(-1, -2), GoalType::Red},    Goal{from_center_tiles(2, 1), GoalType::Blue},
    Goal{from_center_tiles(1, 2), GoalType::Blue},
};

inline constexpr Point2D to_lemlib_inches(const Point2D& field_mm) {
    return {(field_mm.x - kFieldLength / 2.0) / kMmPerInch, (field_mm.y - kFieldWidth / 2.0) / kMmPerInch};
}

}
