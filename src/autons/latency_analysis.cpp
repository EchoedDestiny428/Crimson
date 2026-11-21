#include "autons/latency_analysis.hpp"
#include "config/camera.hpp"
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>

// Same method as tools/latency_test.py; keep the two in step.
namespace autons::latency_analysis {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr int kMaxLatencyMs = 250;
constexpr int kSharpnessStepMs = 25;
constexpr std::size_t kMinReads = 10;
constexpr int kSmoothSamples = 5;
// src/robotconfig.cpp: blue motors, wheels at 360 rpm, 3.25 in omnis.
constexpr double kMotorRpm = 600.0;
constexpr double kWheelRpm = 360.0;
constexpr double kWheelIn = 3.25;

double rad(double deg) {
    return deg * kPi / 180.0;
}

double deg(double rad) {
    return rad * 180.0 / kPi;
}

bool in(int phase, std::initializer_list<int> phases) {
    return std::find(phases.begin(), phases.end(), phase) != phases.end();
}

// Linear interpolation like numpy.interp: clamps outside the samples.
class Series {
public:
    Series(const std::vector<double>& t, const std::vector<double>& v) : t_(t), v_(v) {}

    double at(double time) const {
        if (t_.empty()) {
            return std::nan("");
        }
        if (time <= t_.front()) {
            return v_.front();
        }
        if (time >= t_.back()) {
            return v_.back();
        }
        const auto upper = std::upper_bound(t_.begin(), t_.end(), time);
        const std::size_t i = static_cast<std::size_t>(upper - t_.begin());
        const double f = (time - t_[i - 1]) / (t_[i] - t_[i - 1]);
        return v_[i - 1] + f * (v_[i] - v_[i - 1]);
    }

private:
    const std::vector<double>& t_;
    const std::vector<double>& v_;
};

struct Read {
    double ms;
    double response;
    int phase;
};

// RMS of the residual of a straight-line fit of y on x.
double fit_rms(const std::vector<double>& x, const std::vector<double>& y) {
    const std::size_t n = x.size();
    if (n < 4) {
        return kInf;
    }
    const auto [lo, hi] = std::minmax_element(x.begin(), x.end());
    if (*hi - *lo < 1e-6) {
        return kInf;
    }
    double mx = 0, my = 0;
    for (std::size_t i = 0; i < n; ++i) {
        mx += x[i];
        my += y[i];
    }
    mx /= n;
    my /= n;
    double sxx = 0, sxy = 0, syy = 0;
    for (std::size_t i = 0; i < n; ++i) {
        sxx += (x[i] - mx) * (x[i] - mx);
        sxy += (x[i] - mx) * (y[i] - my);
        syy += (y[i] - my) * (y[i] - my);
    }
    return std::sqrt(std::max(syy - sxy * sxy / sxx, 0.0) / n);
}

// For each delay, how well the camera value at read time fits odometry that much earlier.
Latency scan(const std::vector<Read>& reads, const Series& odom) {
    Latency out;
    out.reads = static_cast<int>(reads.size());
    if (reads.size() < kMinReads) {
        return out;
    }
    std::vector<double> curve(kMaxLatencyMs + 1);
    std::vector<double> x(reads.size());
    std::vector<double> y(reads.size());
    for (std::size_t i = 0; i < reads.size(); ++i) {
        y[i] = reads[i].response;
    }
    for (int lat = 0; lat <= kMaxLatencyMs; ++lat) {
        for (std::size_t i = 0; i < reads.size(); ++i) {
            x[i] = odom.at(reads[i].ms - lat);
        }
        curve[lat] = fit_rms(x, y);
    }
    const int best = static_cast<int>(std::min_element(curve.begin(), curve.end()) - curve.begin());
    if (!std::isfinite(curve[best])) {
        return out;
    }
    double side = kInf;
    for (const int i : {best - kSharpnessStepMs, best + kSharpnessStepMs}) {
        if (i >= 0 && i <= kMaxLatencyMs) {
            side = std::min(side, curve[i]);
        }
    }
    out.valid = true;
    out.ms = best;
    out.sharpness = curve[best] > 0 ? side / curve[best] : kInf;
    return out;
}

// Turns the tilted pixel into a level ray: bearing right of the lens axis, and depression below level (degrees).
void level_ray(double u, double v, double& bearing, double& depression) {
    const double p = rad(config::kCameraPitchDeg);
    const double right = (u - config::kCenterXPx) / config::kFocalXPx;
    const double down = (v - config::kCenterYPx) / config::kFocalYPx;
    const double forward = std::cos(p) - down * std::sin(p);
    const double drop = std::sin(p) + down * std::cos(p);
    bearing = deg(std::atan2(right, forward));
    depression = deg(std::atan2(drop, std::hypot(forward, right)));
}

// numpy.gradient: second-order central differences inside, first order at the ends, uneven spacing allowed.
std::vector<double> gradient(const std::vector<double>& f, const std::vector<double>& t) {
    const std::size_t n = f.size();
    std::vector<double> g(n, 0.0);
    if (n < 2) {
        return g;
    }
    g[0] = (f[1] - f[0]) / (t[1] - t[0]);
    g[n - 1] = (f[n - 1] - f[n - 2]) / (t[n - 1] - t[n - 2]);
    for (std::size_t i = 1; i + 1 < n; ++i) {
        const double hd = t[i] - t[i - 1];
        const double hs = t[i + 1] - t[i];
        g[i] = (-hs / (hd * (hd + hs))) * f[i - 1] + ((hs - hd) / (hd * hs)) * f[i] + (hd / (hs * (hd + hs))) * f[i + 1];
    }
    return g;
}

// Centred moving average with the ends held, like the Python script.
std::vector<double> smooth(const std::vector<double>& v) {
    const int n = static_cast<int>(v.size());
    if (n < kSmoothSamples) {
        return v;
    }
    std::vector<double> out(n);
    for (int i = 0; i < n; ++i) {
        double sum = 0;
        for (int k = -kSmoothSamples / 2; k <= kSmoothSamples / 2; ++k) {
            sum += v[std::clamp(i + k, 0, n - 1)];
        }
        out[i] = sum / kSmoothSamples;
    }
    return out;
}

}

Results analyze(const std::vector<Sample>& samples) {
    Results r;
    if (samples.empty()) {
        return r;
    }
    const std::size_t n = samples.size();
    std::vector<double> t(n), x(n), y(n), heading(n);
    std::vector<int> phase(n);
    for (std::size_t i = 0; i < n; ++i) {
        t[i] = samples[i].ms;
        x[i] = samples[i].x_in;
        y[i] = samples[i].y_in;
        heading[i] = samples[i].heading_deg;
        phase[i] = samples[i].phase;
        r.completed = r.completed || phase[i] == 17;
        r.aborted = r.aborted || phase[i] == -1;
        if (phase[i] > 0) {
            r.last_phase = std::max(r.last_phase, phase[i]);
        }
    }

    // ---- Latency ----
    const double h0 = rad(heading[0]);
    std::vector<double> along(n);
    for (std::size_t i = 0; i < n; ++i) {
        along[i] = (x[i] - x[0]) * std::cos(h0) - (y[i] - y[0]) * std::sin(h0);
    }
    const Series heading_at(t, heading);
    const Series along_at(t, along);

    std::vector<Read> turning;
    std::vector<Read> driving;
    double last_read = -1.0;
    for (const Sample& s : samples) {
        if (!std::isfinite(s.u_px) || !std::isfinite(s.v_px) || s.tags <= 0 || s.read_ms == last_read) {
            continue;
        }
        last_read = s.read_ms;
        double bearing = 0, depression = 0;
        level_ray(s.u_px, s.v_px, bearing, depression);
        if (in(s.phase, {2, 3, 4})) {
            turning.push_back({static_cast<double>(s.read_ms), bearing, s.phase});
        } else if (in(s.phase, {5, 6})) {
            const double clipped = std::clamp(depression, 1.0, 89.0);
            driving.push_back({static_cast<double>(s.read_ms), 1.0 / std::tan(rad(clipped)), s.phase});
        }
    }
    r.turning = scan(turning, heading_at);
    r.driving = scan(driving, along_at);

    double weighted = 0, weights = 0;
    for (const Latency* l : {&r.turning, &r.driving}) {
        if (l->valid) {
            const double w = std::max(l->sharpness - 1.0, 0.01);
            weighted += l->ms * w;
            weights += w;
        }
    }
    if (weights > 0) {
        r.latency_valid = true;
        r.latency_ms = static_cast<int>(std::lround(weighted / weights));
    }

    // ---- Limits ----
    const auto first8 = std::find(phase.begin(), phase.end(), 8);
    if (first8 == phase.end()) {
        return r;
    }
    const std::size_t b = static_cast<std::size_t>(first8 - phase.begin());
    std::vector<double> ts(n), burst_along(n);
    const double hb = rad(heading[b]);
    for (std::size_t i = 0; i < n; ++i) {
        ts[i] = t[i] / 1000.0;
        burst_along[i] = (x[i] - x[b]) * std::cos(hb) - (y[i] - y[b]) * std::sin(hb);
    }
    const std::vector<double> speed = smooth(gradient(burst_along, ts));
    const std::vector<double> accel = smooth(gradient(speed, ts));
    const std::vector<double> turn = smooth(gradient(heading, ts));
    const std::vector<double> turn_accel = smooth(gradient(turn, ts));

    const auto peak = [&](const std::vector<double>& v, std::initializer_list<int> phases, bool highest) {
        double best = highest ? -kInf : kInf;
        for (std::size_t i = 0; i < n; ++i) {
            if (in(phase[i], phases)) {
                best = highest ? std::max(best, v[i]) : std::min(best, v[i]);
            }
        }
        return std::isfinite(best) ? best : std::nan("");
    };
    const auto time_to = [&](const std::vector<double>& v, int ph, double frac) {
        double top = 0;
        std::size_t start = n;
        for (std::size_t i = 0; i < n; ++i) {
            if (phase[i] == ph) {
                start = std::min(start, i);
                top = std::max(top, std::fabs(v[i]));
            }
        }
        for (std::size_t i = start; i < n; ++i) {
            if (phase[i] == ph && std::fabs(v[i]) >= frac * top) {
                return (ts[i] - ts[start]) * 1000.0;
            }
        }
        return std::nan("");
    };

    r.drive_valid = true;
    r.top_forward = peak(speed, {8, 9}, true);
    r.top_reverse = -peak(speed, {10, 11}, false);
    r.accel_forward = peak(accel, {8}, true);
    r.accel_reverse = -peak(accel, {10}, false);
    r.brake_forward = -peak(accel, {9}, false);
    r.brake_reverse = peak(accel, {11}, true);
    r.ms_to_90 = time_to(speed, 8, 0.9);
    std::vector<double> wheels(n);
    for (std::size_t i = 0; i < n; ++i) {
        wheels[i] = (samples[i].left_rpm + samples[i].right_rpm) / 2.0 * (kWheelRpm / kMotorRpm) * kPi * kWheelIn /
                    60.0;
    }
    r.motors_forward = peak(wheels, {8, 9}, true);

    if (std::find(phase.begin(), phase.end(), 12) != phase.end()) {
        r.turn_valid = true;
        r.turn_right = peak(turn, {12, 13}, true);
        r.turn_left = -peak(turn, {14, 15}, false);
        r.turn_accel_right = peak(turn_accel, {12}, true);
        r.turn_accel_left = -peak(turn_accel, {14}, false);
        r.turn_brake_right = -peak(turn_accel, {13}, false);
        r.turn_brake_left = peak(turn_accel, {15}, true);
        r.turn_ms_to_90 = time_to(turn, 12, 0.9);
    }
    return r;
}

}
