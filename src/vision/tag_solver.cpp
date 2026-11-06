#include "vision/tag_solver.hpp"
#include "constants/field_constants.hpp"
#include "util/angle.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace vision {

namespace {

using util::deg_to_rad;
using util::rad_to_deg;
using util::wrap_signed_degrees;

constexpr std::size_t kCornersPerTag = 4;
constexpr double kResidualsPerTag = 2.0 * kCornersPerTag;

constexpr double kMinTagHeightPx = 2.0;
constexpr double kMaxObliqueDeg = 75.0;
constexpr double kMatchRadiusPx = 100.0;
constexpr double kMaxErrorPx = 3.0;
constexpr double kMaxYawCorrectionDeg = 20.0;
constexpr double kRivalMarginPx = 0.3;
constexpr double kMaxYawSpreadDeg = 1.0;
constexpr double kMinDepthMm = 50.0;
constexpr double kFieldMarginMm = 75.0;
constexpr double kGoalKeepoutMm = 100.0;
constexpr double kMergeRadiusMm = 150.0;

constexpr int kRefineSteps = 10;
constexpr int kDampingAttempts = 8;
constexpr double kStartDamping = 1e-3;
constexpr double kMinDamping = 1e-7;
constexpr double kMinPivot = 1e-12;

using Corners = std::array<Pixel, kCornersPerTag>;
using Vector3 = std::array<double, 3>;
using Matrix3 = std::array<Vector3, 3>;

struct Point3 {
    double x;
    double y;
    double z;
};

using WorldCorners = std::array<Point3, kCornersPerTag>;

struct Tag {
    int id;
    Corners corners;
};

struct Placement {
    std::size_t goal;
    double face_deg;
};

struct View {
    WorldCorners world;
    Corners pixels;
};

struct Fit {
    Pose pose;
    double cost;
};

struct Normal {
    Matrix3 a{};
    Vector3 b{};
};

Pixel centre(const Corners& corners) {
    Pixel sum{0.0, 0.0};
    for (const Pixel& corner : corners) {
        sum.u += corner.u;
        sum.v += corner.v;
    }
    return {sum.u / kCornersPerTag, sum.v / kCornersPerTag};
}

double pixel_height(const Corners& ordered) {
    return ((ordered[3].v - ordered[0].v) + (ordered[2].v - ordered[1].v)) / 2.0;
}

bool known_id(int id) {
    return std::ranges::any_of(field::kGoals, [id](const field::Goal& goal) { return goal.tag_id == id; });
}

// Corners match order_corners as seen by a camera facing the plate, so right is measured from that side.
WorldCorners tag_corners(const Placement& placement) {
    const field::Goal& goal = field::kGoals[placement.goal];
    const double face = deg_to_rad(placement.face_deg);
    const double normal_x = std::cos(face);
    const double normal_y = -std::sin(face);
    const double centre_x = goal.pos.x + field::kTagPlateOffsetMm * normal_x;
    const double centre_y = goal.pos.y + field::kTagPlateOffsetMm * normal_y;
    const double half = field::kTagSizeMm / 2.0;
    const double right_x = -normal_y * half;
    const double right_y = normal_x * half;
    const double top = goal.tag_height_mm + half;
    const double bottom = goal.tag_height_mm - half;
    return {{{centre_x - right_x, centre_y - right_y, top},
             {centre_x + right_x, centre_y + right_y, top},
             {centre_x + right_x, centre_y + right_y, bottom},
             {centre_x - right_x, centre_y - right_y, bottom}}};
}

Point3 tag_centre(const WorldCorners& world) {
    return {(world[0].x + world[2].x) / 2.0, (world[0].y + world[2].y) / 2.0, (world[0].z + world[2].z) / 2.0};
}

std::optional<Pixel> project(const Pose& pose, const Point3& point, const Camera& camera) {
    const double yaw = deg_to_rad(pose.heading_deg);
    const double dx = point.x - pose.x_mm;
    const double dy = point.y - pose.y_mm;
    const double depth = dx * std::cos(yaw) - dy * std::sin(yaw);
    if (depth < kMinDepthMm) {
        return std::nullopt;
    }
    const double right = -dx * std::sin(yaw) - dy * std::cos(yaw);
    return Pixel{camera.cx + camera.fx * right / depth, camera.cy + camera.fy * (camera.height_mm - point.z) / depth};
}

double reprojection_cost(const Pose& pose, std::span<const View> views, const Camera& camera) {
    double cost = 0.0;
    for (const View& view : views) {
        for (std::size_t i = 0; i < kCornersPerTag; ++i) {
            const std::optional<Pixel> predicted = project(pose, view.world[i], camera);
            if (!predicted) {
                return std::numeric_limits<double>::infinity();
            }
            const double du = predicted->u - view.pixels[i].u;
            const double dv = predicted->v - view.pixels[i].v;
            cost += du * du + dv * dv;
        }
    }
    return cost;
}

// With heading fixed, the projection is linear in the camera position once the depth is multiplied out.
std::optional<Pose> linear_seed(std::span<const View> views, double heading_deg, const Camera& camera) {
    const double yaw = deg_to_rad(heading_deg);
    const double s = std::sin(yaw);
    const double c = std::cos(yaw);

    double aa = 0.0;
    double ab = 0.0;
    double bb = 0.0;
    double ad = 0.0;
    double bd = 0.0;
    const auto add_row = [&](double a, double b, double d) {
        aa += a * a;
        ab += a * b;
        bb += b * b;
        ad += a * d;
        bd += b * d;
    };

    for (const View& view : views) {
        for (std::size_t i = 0; i < kCornersPerTag; ++i) {
            const Point3& point = view.world[i];
            const double bearing = (view.pixels[i].u - camera.cx) / camera.fx;
            const double drop = (view.pixels[i].v - camera.cy) / camera.fy;
            const double row_x = s + bearing * c;
            const double row_y = c - bearing * s;
            add_row(row_x, row_y, point.x * row_x + point.y * row_y);
            add_row(drop * c, -drop * s, drop * (point.x * c - point.y * s) - (camera.height_mm - point.z));
        }
    }

    const double determinant = aa * bb - ab * ab;
    if (std::abs(determinant) < kMinPivot) {
        return std::nullopt;
    }
    return Pose{(ad * bb - bd * ab) / determinant, (aa * bd - ab * ad) / determinant, heading_deg};
}

Normal normal_equations(const Pose& pose, std::span<const View> views, const Camera& camera) {
    const double yaw = deg_to_rad(pose.heading_deg);
    const double s = std::sin(yaw);
    const double c = std::cos(yaw);

    Normal normal;
    for (const View& view : views) {
        for (std::size_t i = 0; i < kCornersPerTag; ++i) {
            const Point3& point = view.world[i];
            const double dx = point.x - pose.x_mm;
            const double dy = point.y - pose.y_mm;
            const double depth = dx * c - dy * s;
            if (depth < kMinDepthMm) {
                continue;
            }
            const double right = -dx * s - dy * c;
            const double drop = camera.height_mm - point.z;
            const double per_depth_squared = 1.0 / (depth * depth);

            const Vector3 depth_slope{-c, s, right};
            const Vector3 right_slope{s, c, -depth};
            Vector3 u_slope{};
            Vector3 v_slope{};
            for (std::size_t k = 0; k < 3; ++k) {
                u_slope[k] = camera.fx * (right_slope[k] * depth - right * depth_slope[k]) * per_depth_squared;
                v_slope[k] = -camera.fy * drop * depth_slope[k] * per_depth_squared;
            }

            const double u_error = camera.cx + camera.fx * right / depth - view.pixels[i].u;
            const double v_error = camera.cy + camera.fy * drop / depth - view.pixels[i].v;
            for (std::size_t j = 0; j < 3; ++j) {
                for (std::size_t k = 0; k < 3; ++k) {
                    normal.a[j][k] += u_slope[j] * u_slope[k] + v_slope[j] * v_slope[k];
                }
                normal.b[j] -= u_slope[j] * u_error + v_slope[j] * v_error;
            }
        }
    }
    return normal;
}

std::optional<Vector3> solve_linear(Matrix3 a, Vector3 b, std::size_t size) {
    for (std::size_t column = 0; column < size; ++column) {
        std::size_t pivot = column;
        for (std::size_t row = column + 1; row < size; ++row) {
            if (std::abs(a[row][column]) > std::abs(a[pivot][column])) {
                pivot = row;
            }
        }
        if (std::abs(a[pivot][column]) < kMinPivot) {
            return std::nullopt;
        }
        std::swap(a[column], a[pivot]);
        std::swap(b[column], b[pivot]);
        for (std::size_t row = 0; row < size; ++row) {
            if (row == column) {
                continue;
            }
            const double scale = a[row][column] / a[column][column];
            for (std::size_t k = column; k < size; ++k) {
                a[row][k] -= scale * a[column][k];
            }
            b[row] -= scale * b[column];
        }
    }

    Vector3 solution{};
    for (std::size_t i = 0; i < size; ++i) {
        solution[i] = b[i] / a[i][i];
    }
    return solution;
}

Fit refine(const Pose& start, std::span<const View> views, const Camera& camera, bool free_yaw) {
    const std::size_t unknowns = free_yaw ? 3 : 2;
    Fit fit{start, reprojection_cost(start, views, camera)};
    double damping = kStartDamping;

    for (int step = 0; step < kRefineSteps; ++step) {
        const Normal normal = normal_equations(fit.pose, views, camera);
        bool improved = false;

        for (int attempt = 0; attempt < kDampingAttempts && !improved; ++attempt) {
            Matrix3 damped = normal.a;
            for (std::size_t i = 0; i < unknowns; ++i) {
                damped[i][i] *= 1.0 + damping;
            }

            const std::optional<Vector3> delta = solve_linear(damped, normal.b, unknowns);
            if (delta) {
                const Pose next{fit.pose.x_mm + (*delta)[0], fit.pose.y_mm + (*delta)[1],
                                fit.pose.heading_deg + (free_yaw ? rad_to_deg((*delta)[2]) : 0.0)};
                const double cost = reprojection_cost(next, views, camera);
                if (cost < fit.cost) {
                    fit = {next, cost};
                    damping = std::max(damping / 3.0, kMinDamping);
                    improved = true;
                    continue;
                }
            }
            damping *= 10.0;
        }

        if (!improved) {
            break;
        }
    }
    return fit;
}

bool can_see_face(const Tag& tag, double heading_deg, double face_deg, const Camera& camera) {
    const double bearing = std::atan2(centre(tag.corners).u - camera.cx, camera.fx);
    const double look = deg_to_rad(heading_deg) + bearing;
    return -std::cos(look - deg_to_rad(face_deg)) >= std::cos(deg_to_rad(kMaxObliqueDeg));
}

bool legal(const Pose& pose) {
    if (!field::in_field({pose.x_mm, pose.y_mm}, kFieldMarginMm)) {
        return false;
    }
    return std::ranges::none_of(field::kGoals, [&pose](const field::Goal& goal) {
        return std::hypot(pose.x_mm - goal.pos.x, pose.y_mm - goal.pos.y) < kGoalKeepoutMm;
    });
}

std::optional<Placement> match_tag(const Tag& tag, const Pose& pose, std::span<const Placement> taken,
                                   const Camera& camera) {
    const Pixel seen = centre(tag.corners);
    std::optional<Placement> best;
    double best_distance = kMatchRadiusPx;

    for (std::size_t goal = 0; goal < field::kGoals.size(); ++goal) {
        if (field::kGoals[goal].tag_id != tag.id) {
            continue;
        }
        for (const double face : field::kTagFacesDeg) {
            const bool used = std::ranges::any_of(
                taken, [&](const Placement& other) { return other.goal == goal && other.face_deg == face; });
            if (used || !can_see_face(tag, pose.heading_deg, face, camera)) {
                continue;
            }
            const Placement placement{goal, face};
            const std::optional<Pixel> predicted = project(pose, tag_centre(tag_corners(placement)), camera);
            if (!predicted) {
                continue;
            }
            const double distance = std::hypot(predicted->u - seen.u, predicted->v - seen.v);
            if (distance < best_distance) {
                best_distance = distance;
                best = placement;
            }
        }
    }
    return best;
}

int count_goals(std::span<const Placement> placements) {
    int goals = 0;
    for (std::size_t i = 0; i < placements.size(); ++i) {
        const auto earlier = placements.first(i);
        const bool repeat = std::ranges::any_of(
            earlier, [&](const Placement& other) { return other.goal == placements[i].goal; });
        goals += repeat ? 0 : 1;
    }
    return goals;
}

double fit_error(double cost, std::size_t tags) {
    return std::sqrt(cost / (kResidualsPerTag * static_cast<double>(tags)));
}

// Heading error moves where a goal's plates are predicted, but not which of two neighbouring plates is further left.
void uncross_plates(std::span<const Tag> tags, std::span<Placement> placements, const Pose& pose,
                    const Camera& camera) {
    for (std::size_t i = 1; i < tags.size(); ++i) {
        for (std::size_t j = i + 1; j < tags.size(); ++j) {
            if (placements[i].goal != placements[j].goal) {
                continue;
            }
            const std::optional<Pixel> first = project(pose, tag_centre(tag_corners(placements[i])), camera);
            const std::optional<Pixel> second = project(pose, tag_centre(tag_corners(placements[j])), camera);
            const double seen_gap = centre(tags[i].corners).u - centre(tags[j].corners).u;
            if (first && second && (first->u - second->u) * seen_gap < 0.0) {
                std::swap(placements[i], placements[j]);
            }
        }
    }
}

// Assumes the largest tag sits on the given face, then checks whether the rest of the view agrees.
std::optional<Candidate> try_placement(std::span<const Tag> tags, const Placement& first, double heading_deg,
                                       const Camera& camera) {
    std::array<Placement, kMaxDetections> placements{};
    std::array<View, kMaxDetections> views{};
    placements[0] = first;
    views[0] = {tag_corners(first), tags[0].corners};

    const std::optional<Pose> seed = linear_seed(std::span(views).first(1), heading_deg, camera);
    if (!seed) {
        return std::nullopt;
    }
    Fit fit = refine(*seed, std::span(views).first(1), camera, false);

    for (std::size_t i = 1; i < tags.size(); ++i) {
        const std::optional<Placement> match =
            match_tag(tags[i], fit.pose, std::span(placements).first(i), camera);
        if (!match) {
            return std::nullopt;
        }
        placements[i] = *match;
    }

    const auto all_views = std::span(views).first(tags.size());
    const auto all_placements = std::span(placements).first(tags.size());
    uncross_plates(tags, all_placements, fit.pose, camera);
    for (std::size_t i = 1; i < tags.size(); ++i) {
        views[i] = {tag_corners(placements[i]), tags[i].corners};
    }

    const int goals = count_goals(all_placements);
    if (tags.size() > 1) {
        fit = refine(fit.pose, all_views, camera, false);
    }

    bool yaw_solved = goals >= 2;
    if (yaw_solved) {
        const Pose fixed_pose = fit.pose;
        fit = refine(fixed_pose, all_views, camera, true);

        // A far tag seen corner-on can sit on either of two plates 70 mm apart, which still fits and tilts the solved
        // yaw by several degrees. Try each tag's other plates, and do not trust yaw if a rival fits about as well.
        for (std::size_t i = 1; i < tags.size(); ++i) {
            for (const double face : field::kTagFacesDeg) {
                const Placement other{placements[i].goal, face};
                const bool used = std::ranges::any_of(all_placements, [&](const Placement& placement) {
                    return placement.goal == other.goal && placement.face_deg == face;
                });
                if (used || !can_see_face(tags[i], heading_deg, face, camera)) {
                    continue;
                }

                const View kept = views[i];
                views[i] = {tag_corners(other), tags[i].corners};
                const Fit rival = refine(fixed_pose, all_views, camera, true);

                const double error_gap = fit_error(rival.cost, tags.size()) - fit_error(fit.cost, tags.size());
                const double yaw_gap = wrap_signed_degrees(rival.pose.heading_deg - fit.pose.heading_deg);
                if (std::abs(error_gap) < kRivalMarginPx && std::abs(yaw_gap) > kMaxYawSpreadDeg) {
                    yaw_solved = false;
                }

                if (rival.cost < fit.cost) {
                    fit = rival;
                    placements[i] = other;
                } else {
                    views[i] = kept;
                }
            }
        }

        if (std::abs(wrap_signed_degrees(fit.pose.heading_deg - heading_deg)) > kMaxYawCorrectionDeg) {
            return std::nullopt;
        }
    }

    const double error_px = fit_error(fit.cost, tags.size());
    if (error_px > kMaxErrorPx || !legal(fit.pose)) {
        return std::nullopt;
    }

    const Point3 tag = tag_centre(views[0].world);
    return Candidate{fit.pose, error_px, std::hypot(tag.x - fit.pose.x_mm, tag.y - fit.pose.y_mm), goals, yaw_solved};
}

void add_candidate(Solution& solution, const Candidate& candidate) {
    const auto begin = solution.candidates.begin();
    const auto end = begin + static_cast<std::ptrdiff_t>(solution.count);

    const auto twin = std::find_if(begin, end, [&candidate](const Candidate& other) {
        return std::hypot(other.pose.x_mm - candidate.pose.x_mm, other.pose.y_mm - candidate.pose.y_mm) <
               kMergeRadiusMm;
    });
    if (twin != end) {
        const bool rivals = std::abs(candidate.error_px - twin->error_px) < kRivalMarginPx &&
                            std::abs(wrap_signed_degrees(candidate.pose.heading_deg - twin->pose.heading_deg)) >
                                kMaxYawSpreadDeg;
        if (candidate.error_px < twin->error_px) {
            *twin = candidate;
        }
        twin->yaw_solved = twin->yaw_solved && !rivals;
    } else if (solution.count < kMaxCandidates) {
        solution.candidates[solution.count++] = candidate;
    } else if (candidate.error_px < solution.candidates.back().error_px) {
        solution.candidates.back() = candidate;
    }

    std::sort(begin, begin + static_cast<std::ptrdiff_t>(solution.count),
              [](const Candidate& a, const Candidate& b) { return a.error_px < b.error_px; });
}

}

std::array<Pixel, 4> order_corners(std::array<Pixel, 4> corners) {
    const Pixel middle = centre(corners);
    // Image rows grow downwards, so rising angle walks clockwise around the tag.
    std::ranges::sort(corners, {}, [&middle](const Pixel& corner) {
        return std::atan2(corner.v - middle.v, corner.u - middle.u);
    });

    // The tag is upright and the camera is flat, so the true starting corner is the one that gives the tallest sides.
    std::size_t top_left = 0;
    double tallest = -std::numeric_limits<double>::infinity();
    for (std::size_t start = 0; start < corners.size(); ++start) {
        Corners rotated = corners;
        std::rotate(rotated.begin(), rotated.begin() + static_cast<std::ptrdiff_t>(start), rotated.end());
        if (pixel_height(rotated) > tallest) {
            tallest = pixel_height(rotated);
            top_left = start;
        }
    }
    std::rotate(corners.begin(), corners.begin() + static_cast<std::ptrdiff_t>(top_left), corners.end());
    return corners;
}

Solution solve(std::span<const Detection> detections, double heading_deg, const Camera& camera) {
    Solution solution;

    std::array<Tag, kMaxDetections> tags{};
    std::size_t count = 0;
    for (const Detection& detection : detections) {
        if (count == tags.size()) {
            break;
        }
        const Corners ordered = order_corners(detection.corners);
        if (known_id(detection.id) && pixel_height(ordered) >= kMinTagHeightPx) {
            tags[count++] = {detection.id, ordered};
        }
    }
    if (count == 0) {
        return solution;
    }

    const auto usable = std::span(tags).first(count);
    std::ranges::sort(usable, [](const Tag& a, const Tag& b) {
        return pixel_height(a.corners) > pixel_height(b.corners);
    });

    const Tag& largest = usable.front();
    for (std::size_t goal = 0; goal < field::kGoals.size(); ++goal) {
        if (field::kGoals[goal].tag_id != largest.id) {
            continue;
        }
        for (const double face : field::kTagFacesDeg) {
            if (!can_see_face(largest, heading_deg, face, camera)) {
                continue;
            }
            const std::optional<Candidate> candidate = try_placement(usable, {goal, face}, heading_deg, camera);
            if (candidate) {
                add_candidate(solution, *candidate);
            }
        }
    }
    return solution;
}

}
