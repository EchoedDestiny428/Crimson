#include "localization/monte_carlo.hpp"
#include "util/angle.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace localization {

namespace {

constexpr double kMmPerInch = 25.4;
constexpr std::int32_t kNoReturnMm = 9999;

double lerp_angle_component(double degrees) {
    return util::deg_to_rad(degrees);
}

}

MonteCarlo::MonteCarlo(pros::Distance& distance, DistanceSensorMount mount, MotionNoise noise)
    : distance_(distance), mount_(mount), noise_(noise) {}

void MonteCarlo::seed(const lemlib::Pose& pose, double position_sigma_in, double heading_sigma_deg) {
    last_odom_ = pose;
    seeded_ = true;
    const double uniform = 1.0 / static_cast<double>(kParticleCount);
    for (Particle& particle : particles_) {
        particle.x = random_normal(pose.x, position_sigma_in);
        particle.y = random_normal(pose.y, position_sigma_in);
        particle.theta_deg = util::wrap_degrees(random_normal(pose.theta, heading_sigma_deg));
        particle.weight = uniform;
    }
}

void MonteCarlo::predict(const lemlib::Pose& odom_pose) {
    if (!seeded_) {
        seed(odom_pose);
        return;
    }

    const double prev_rad = lerp_angle_component(last_odom_.theta);
    const double dx = odom_pose.x - last_odom_.x;
    const double dy = odom_pose.y - last_odom_.y;
    const double dtheta = util::wrap_degrees(odom_pose.theta - last_odom_.theta + 180.0) - 180.0;
    const double forward = dx * std::sin(prev_rad) + dy * std::cos(prev_rad);
    const double right = dx * std::cos(prev_rad) - dy * std::sin(prev_rad);
    last_odom_ = odom_pose;

    for (Particle& particle : particles_) {
        const double noisy_forward = forward + random_normal(0.0, noise_.forward_in);
        const double noisy_right = right + random_normal(0.0, noise_.sideways_in);
        const double noisy_dtheta = dtheta + random_normal(0.0, noise_.heading_deg);
        const double theta = lerp_angle_component(particle.theta_deg);
        particle.x += noisy_forward * std::sin(theta) + noisy_right * std::cos(theta);
        particle.y += noisy_forward * std::cos(theta) - noisy_right * std::sin(theta);
        particle.theta_deg = util::wrap_degrees(particle.theta_deg + noisy_dtheta);
    }
}

void MonteCarlo::update() {
    if (!seeded_) {
        return;
    }

    const std::int32_t reading_mm = distance_.get();
    if (reading_mm < 0 || reading_mm >= kNoReturnMm) {
        return;
    }
    const double measured_in = static_cast<double>(reading_mm) / kMmPerInch;

    for (Particle& particle : particles_) {
        if (std::abs(particle.x) > kFieldHalfIn || std::abs(particle.y) > kFieldHalfIn) {
            particle.weight = 0.0;
            continue;
        }
        const double expected_in = expected_range_in(particle);
        const double error = measured_in - expected_in;
        const double likelihood = std::exp(-0.5 * error * error / (kRangeSigmaIn * kRangeSigmaIn));
        particle.weight *= likelihood;
    }

    normalize();
    if (effective_sample_size() < kResampleThreshold * static_cast<double>(kParticleCount)) {
        resample();
    }
}

lemlib::Pose MonteCarlo::estimate() const {
    double x = 0.0;
    double y = 0.0;
    double sin_sum = 0.0;
    double cos_sum = 0.0;
    for (const Particle& particle : particles_) {
        x += particle.weight * particle.x;
        y += particle.weight * particle.y;
        const double theta = lerp_angle_component(particle.theta_deg);
        sin_sum += particle.weight * std::sin(theta);
        cos_sum += particle.weight * std::cos(theta);
    }
    const double theta_deg = util::wrap_degrees(std::atan2(sin_sum, cos_sum) * 180.0 / std::numbers::pi);
    return {static_cast<float>(x), static_cast<float>(y), static_cast<float>(theta_deg)};
}

double MonteCarlo::effective_sample_size() const {
    double sum_sq = 0.0;
    for (const Particle& particle : particles_) {
        sum_sq += particle.weight * particle.weight;
    }
    return sum_sq > 0.0 ? 1.0 / sum_sq : 0.0;
}

double MonteCarlo::expected_range_in(const Particle& particle) const {
    const double heading = lerp_angle_component(particle.theta_deg + mount_.yaw_deg);
    const double robot = lerp_angle_component(particle.theta_deg);
    const double origin_x = particle.x + mount_.x_in * std::cos(robot) + mount_.y_in * std::sin(robot);
    const double origin_y = particle.y - mount_.x_in * std::sin(robot) + mount_.y_in * std::cos(robot);
    const double dir_x = std::sin(heading);
    const double dir_y = std::cos(heading);

    double best = std::numeric_limits<double>::infinity();
    const auto consider = [&](double t, double hit_x, double hit_y) {
        if (t <= 0.0) {
            return;
        }
        if (std::abs(hit_x) > kFieldHalfIn + 1e-6 || std::abs(hit_y) > kFieldHalfIn + 1e-6) {
            return;
        }
        best = std::min(best, t);
    };

    if (std::abs(dir_x) > 1e-6) {
        const double t_right = (kFieldHalfIn - origin_x) / dir_x;
        consider(t_right, kFieldHalfIn, origin_y + t_right * dir_y);
        const double t_left = (-kFieldHalfIn - origin_x) / dir_x;
        consider(t_left, -kFieldHalfIn, origin_y + t_left * dir_y);
    }
    if (std::abs(dir_y) > 1e-6) {
        const double t_top = (kFieldHalfIn - origin_y) / dir_y;
        consider(t_top, origin_x + t_top * dir_x, kFieldHalfIn);
        const double t_bottom = (-kFieldHalfIn - origin_y) / dir_y;
        consider(t_bottom, origin_x + t_bottom * dir_x, -kFieldHalfIn);
    }
    return best;
}

void MonteCarlo::normalize() {
    double sum = 0.0;
    for (const Particle& particle : particles_) {
        sum += particle.weight;
    }
    if (sum <= 0.0) {
        seed(last_odom_);
        return;
    }
    for (Particle& particle : particles_) {
        particle.weight /= sum;
    }
}

void MonteCarlo::resample() {
    std::array<Particle, kParticleCount> next{};
    const double step = 1.0 / static_cast<double>(kParticleCount);
    double cumulative = particles_.front().weight;
    double cursor = random_uniform() * step;
    std::size_t source = 0;
    for (std::size_t i = 0; i < kParticleCount; ++i) {
        while (cursor > cumulative && source + 1 < kParticleCount) {
            ++source;
            cumulative += particles_[source].weight;
        }
        next[i] = particles_[source];
        next[i].weight = step;
        cursor += step;
    }
    particles_ = next;
}

double MonteCarlo::random_uniform() {
    rng_state_ = rng_state_ * 1664525u + 1013904223u;
    return static_cast<double>(rng_state_) / static_cast<double>(std::numeric_limits<std::uint32_t>::max());
}

double MonteCarlo::random_normal(double mean, double stddev) {
    const double u1 = std::max(random_uniform(), 1e-9);
    const double u2 = random_uniform();
    const double z = std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * std::numbers::pi * u2);
    return mean + stddev * z;
}

}
