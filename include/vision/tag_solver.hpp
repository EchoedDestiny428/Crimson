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
    // True when pose.heading_deg is a measurement of heading worth feeding back, not just a by-product of the fit.
    bool yaw_solved;
};

inline constexpr std::size_t kMaxDetections = 8;
inline constexpr std::size_t kMaxCandidates = 4;

// Best fit first. Tag IDs repeat across the field, so one view can fit more than one pose.
struct Solution {
    std::array<Candidate, kMaxCandidates> candidates{};
    std::size_t count = 0;
};

// Returns the corners as top left, top right, bottom right, bottom left, in whatever order they arrive.
std::array<Pixel, 4> order_corners(std::array<Pixel, 4> corners);

// Finds every field pose that explains the detections, given the current heading. Heading is treated as known unless
// two or more goals are in view and the view leaves no doubt about which plate each tag sits on.
Solution solve(std::span<const Detection> detections, double heading_deg, const Camera& camera);

}
