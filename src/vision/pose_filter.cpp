#include "vision/pose_filter.hpp"
#include "util/angle.hpp"
#include <cmath>
#include <limits>

namespace vision {

namespace {

constexpr double kStartSigmaMm = 300.0;
constexpr double kStartHeadingSigmaDeg = 3.0;
constexpr double kPositionDriftMmPerRootS = 150.0;
constexpr double kHeadingDriftDegPerRootS = 0.1;

constexpr double kBaseSigmaMm = 20.0;
constexpr double kSigmaPerRange = 0.05;
constexpr double kManyGoalsSigmaScale = 0.5;
constexpr double kYawSigmaDeg = 1.0;

constexpr double kGateSigmas = 4.0;
constexpr int kRelockFrames = 5;
constexpr double kRelockRadiusMm = 200.0;

constexpr double square(double value) {
    return value * value;
}

double position_variance(const Candidate& candidate) {
    const double sigma = kBaseSigmaMm + kSigmaPerRange * candidate.range_mm;
    return square(candidate.goals >= 2 ? sigma * kManyGoalsSigmaScale : sigma);
}

}

void Kalman1D::reset(double value, double variance) {
    value_ = value;
    variance_ = variance;
}

void Kalman1D::predict(double control, double process_variance) {
    value_ += control;
    variance_ += process_variance;
}

void Kalman1D::correct(double innovation, double measurement_variance) {
    const double gain = variance_ / (variance_ + measurement_variance);
    value_ += gain * innovation;
    variance_ *= 1.0 - gain;
}

void PoseFilter::reset(const Pose& start) {
    x_.reset(start.x_mm, square(kStartSigmaMm));
    y_.reset(start.y_mm, square(kStartSigmaMm));
    heading_.reset(start.heading_deg, square(kStartHeadingSigmaDeg));
    pending_frames_ = 0;
}

void PoseFilter::predict(double dt_s, double imu_delta_deg) {
    const double position_noise = square(kPositionDriftMmPerRootS) * dt_s;
    x_.predict(0.0, position_noise);
    y_.predict(0.0, position_noise);
    heading_.predict(imu_delta_deg, square(kHeadingDriftDegPerRootS) * dt_s);
}

Fix PoseFilter::update(const Solution& solution) {
    if (solution.count == 0) {
        return Fix::None;
    }

    const Candidate* nearest = nullptr;
    double nearest_distance = std::numeric_limits<double>::infinity();
    std::size_t inside_gate = 0;

    for (std::size_t i = 0; i < solution.count; ++i) {
        const Candidate& candidate = solution.candidates[i];
        const double variance = position_variance(candidate);
        const double distance = square(candidate.pose.x_mm - x_.value()) / (x_.variance() + variance) +
                                square(candidate.pose.y_mm - y_.value()) / (y_.variance() + variance);
        inside_gate += distance <= square(kGateSigmas) ? 1 : 0;
        if (distance < nearest_distance) {
            nearest_distance = distance;
            nearest = &candidate;
        }
    }

    if (inside_gate == 1) {
        apply(*nearest);
        pending_frames_ = 0;
        return Fix::Accepted;
    }
    if (solution.count > 1) {
        return Fix::Ambiguous;
    }

    const Candidate& only = solution.candidates[0];
    const bool repeat = pending_frames_ > 0 && std::hypot(only.pose.x_mm - pending_.pose.x_mm,
                                                          only.pose.y_mm - pending_.pose.y_mm) < kRelockRadiusMm;
    pending_ = only;
    pending_frames_ = repeat ? pending_frames_ + 1 : 1;
    if (pending_frames_ < kRelockFrames) {
        return Fix::Rejected;
    }

    const double variance = position_variance(only);
    x_.reset(only.pose.x_mm, variance);
    y_.reset(only.pose.y_mm, variance);
    pending_frames_ = 0;
    return Fix::Accepted;
}

Pose PoseFilter::pose() const {
    return {x_.value(), y_.value(), util::wrap_degrees(heading_.value())};
}

void PoseFilter::apply(const Candidate& candidate) {
    const double variance = position_variance(candidate);
    x_.correct(candidate.pose.x_mm - x_.value(), variance);
    y_.correct(candidate.pose.y_mm - y_.value(), variance);
    if (candidate.yaw_solved) {
        const double innovation = util::wrap_signed_degrees(candidate.pose.heading_deg - heading_.value());
        heading_.correct(innovation, square(kYawSigmaDeg));
    }
}

}
