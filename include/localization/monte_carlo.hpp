#pragma once

#include "lemlib/pose.hpp"
#include "pros/distance.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace localization {

// One forward-facing distance sensor. Offsets are in the robot frame:
// x is right, y is forward, yaw is clockwise from the robot heading.
struct DistanceSensorMount {
    double x_in = 0.0;
    double y_in = 0.0;
    double yaw_deg = 0.0;
};

struct MotionNoise {
    double forward_in = 0.4;
    double sideways_in = 0.2;
    double heading_deg = 1.5;
};

// Particle filter over field pose. LemLib coordinates: inches, degrees,
// heading 0 faces +Y and increases clockwise. This does not write the chassis pose.
class MonteCarlo {
public:
    static constexpr std::size_t kParticleCount = 150;
    static constexpr double kFieldHalfIn = 72.0;
    static constexpr double kRangeSigmaIn = 1.5;
    static constexpr double kResampleThreshold = 0.5;

    MonteCarlo(pros::Distance& distance, DistanceSensorMount mount, MotionNoise noise = {});

    void seed(const lemlib::Pose& pose, double position_sigma_in = 2.0, double heading_sigma_deg = 5.0);
    void predict(const lemlib::Pose& odom_pose);
    void update();
    lemlib::Pose estimate() const;

    double effective_sample_size() const;

private:
    struct Particle {
        double x = 0.0;
        double y = 0.0;
        double theta_deg = 0.0;
        double weight = 1.0;
    };

    double expected_range_in(const Particle& particle) const;
    void normalize();
    void resample();
    double random_normal(double mean, double stddev);
    double random_uniform();

    pros::Distance& distance_;
    DistanceSensorMount mount_;
    MotionNoise noise_;
    std::array<Particle, kParticleCount> particles_{};
    lemlib::Pose last_odom_{0.0f, 0.0f, 0.0f};
    bool seeded_ = false;
    std::uint32_t rng_state_ = 0xC0FFEE;
};

}
