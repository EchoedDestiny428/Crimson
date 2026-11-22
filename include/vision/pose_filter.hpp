#pragma once

#include "vision/tag_solver.hpp"

namespace vision {

class Kalman1D {
public:
    void reset(double value, double variance);
    // The control input is the feedforward: how far the state is known to have moved since the last step.
    void predict(double control, double process_variance);
    void correct(double innovation, double measurement_variance);

    double value() const { return value_; }
    double variance() const { return variance_; }

private:
    double value_ = 0.0;
    double variance_ = 0.0;
};

enum class Fix { None, Accepted, Ambiguous, Rejected };

// X, Y and heading each run a 1D filter. Odometry is the feedforward for all three: each step moves the estimate by
// however far odometry moved, and uncertainty grows with the distance driven (odometry drifts as it goes), plus a
// little with time. The camera then only has to correct odometry's drift.
class PoseFilter {
public:
    // sigma_mm: how far off `start` may be. Boot uses a wide default; a setPose the robot trusts can pass less.
    void reset(const Pose& start, double sigma_mm = 300.0);
    void predict(double dx_mm, double dy_mm, double dheading_deg, double dt_s);
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
