#pragma once

#include <cmath>
#include <numbers>

namespace util {

inline double deg_to_rad(double deg) {
    return deg * std::numbers::pi / 180.0;
}

inline double rad_to_deg(double rad) {
    return rad * 180.0 / std::numbers::pi;
}

inline double wrap_degrees(double deg) {
    const double wrapped = std::fmod(deg, 360.0);
    return wrapped < 0.0 ? wrapped + 360.0 : wrapped;
}

inline double wrap_signed_degrees(double deg) {
    const double wrapped = wrap_degrees(deg);
    return wrapped > 180.0 ? wrapped - 360.0 : wrapped;
}

}
