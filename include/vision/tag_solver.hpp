#pragma once

#include <array>
#include <cstddef>
#include <span>

namespace vision {

struct Pixel {
    double u;
    double v;
};

struct Detection {
    int id;
    std::array<Pixel, 4> corners;
};

struct Pose {
    double x_mm;
    double y_mm;
    double heading_deg;
};

struct Camera {
    double height_mm;
    double fx;
    double fy;
    double cx;
    double cy;
};

struct Candidate {
    Pose pose;
    double error_px;
    double range_mm;
    int goals;
    bool yaw_solved;
};

inline constexpr std::size_t kMaxDetections = 8;
inline constexpr std::size_t kMaxCandidates = 4;

struct Solution {
    std::array<Candidate, kMaxCandidates> candidates{};
    std::size_t count = 0;
};

std::array<Pixel, 4> order_corners(std::array<Pixel, 4> corners);

Solution solve(std::span<const Detection> detections, double heading_deg, const Camera& camera);

}
