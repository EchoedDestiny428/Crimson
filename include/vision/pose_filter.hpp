#pragma once

#include "vision/tag_solver.hpp"

namespace vision {

class Kalman1D {
public:
    void reset(double value, double variance);
    void predict(double control, double process_variance);
    void correct(double innovation, double measurement_variance);

    double value() const { return value_; }
    double variance() const { return variance_; }

private:
    double value_ = 0.0;
    double variance_ = 0.0;
};

enum class Fix { None, Accepted, Ambiguous, Rejected };

class PoseFilter {
public:
    void reset(const Pose& start);
    void predict(double dt_s, double imu_delta_deg);
    Fix update(const Solution& solution);

    Pose pose() const;

private:
    void apply(const Candidate& candidate);

    Kalman1D x_;
    Kalman1D y_;
    Kalman1D heading_;
    Candidate pending_{};
    int pending_frames_ = 0;
};

}
