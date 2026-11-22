#include "vision/localizer.hpp"
#include "config/camera.hpp"
#include "config/ports.hpp"
#include "constants/field_constants.hpp"
#include "pros/ai_vision.hpp"
#include "pros/error.h"
#include "pros/rtos.hpp"
#include "robotconfig.hpp"
#include "vision/pose_filter.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <mutex>

namespace vision::localizer {

namespace {

// Odometry is followed every tick; the camera is read every kCameraEveryTicks ticks (30 ms).
constexpr std::uint32_t kTickMs = 10;
constexpr std::uint32_t kCameraEveryTicks = 3;
constexpr std::uint32_t kLockHoldMs = 500;
// How far off the estimate may be at boot, and after a setPose the code made on purpose.
constexpr double kBootSigmaMm = 300.0;
constexpr double kResetSigmaMm = 100.0;
// Turning faster than this blurs the picture, and it is where an error in kCameraLatencyMs costs the most.
constexpr double kMaxTurnRateDegPerS = 120.0;
// The same corners as the last read although odometry moved this much: the sensor handed back an old picture.
constexpr double kStaleMoveMm = 5.0;
constexpr double kStaleTurnDeg = 0.5;
constexpr std::size_t kHistorySize = 128; // 1.28 s of odometry at kTickMs

constexpr Camera kCamera{config::kCameraHeightMm,
                         config::kFocalXPx,
                         config::kFocalYPx,
                         config::kCenterXPx,
                         config::kCenterYPx,
                         config::kCameraPitchDeg,
                         config::kCameraForwardIn * field::kMmPerInch,
                         config::kCameraRightIn * field::kMmPerInch,
                         config::kCameraYawDeg};

pros::Mutex shared_mutex;
Snapshot shared;

FieldPose to_field_pose(const Pose& pose) {
    return {pose.x_mm / field::kMmPerInch, pose.y_mm / field::kMmPerInch, pose.heading_deg};
}

void publish(const Snapshot& next) {
    const std::lock_guard lock(shared_mutex);
    shared = next;
}

bool configure(pros::AIVision& sensor) {
    using pros::AivisionModeType;
    return sensor.enable_detection_types(AivisionModeType::tags) != PROS_ERR &&
           sensor.disable_detection_types(AivisionModeType::colors, AivisionModeType::objects) != PROS_ERR &&
           sensor.set_tag_family(pros::AivisionTagFamily::tag_21H7, true) != PROS_ERR;
}

// Returns the number of tags read, or nothing if the sensor is not answering.
std::optional<std::size_t> read_tags(pros::AIVision& sensor, std::array<Detection, kMaxDetections>& tags) {
    const std::int32_t objects = sensor.get_object_count();
    if (objects == PROS_ERR) {
        return std::nullopt;
    }

    std::size_t count = 0;
    for (std::int32_t i = 0; i < objects && count < tags.size(); ++i) {
        const pros::AIVision::Object object = sensor.get_object(static_cast<std::uint32_t>(i));
        if (!pros::AIVision::is_type(object, pros::AivisionDetectType::tag)) {
            continue;
        }
        const auto& tag = object.object.tag;
        tags[count++] = {object.id,
                         {{{static_cast<double>(tag.x0), static_cast<double>(tag.y0)},
                           {static_cast<double>(tag.x1), static_cast<double>(tag.y1)},
                           {static_cast<double>(tag.x2), static_cast<double>(tag.y2)},
                           {static_cast<double>(tag.x3), static_cast<double>(tag.y3)}}}};
    }
    return count;
}

std::optional<RawTag> largest_tag(std::span<const Detection> tags) {
    std::optional<RawTag> largest;
    for (const Detection& tag : tags) {
        const auto [top_left, top_right, bottom_right, bottom_left] = order_corners(tag.corners);
        const RawTag raw{tag.id,
                         (top_left.u + top_right.u + bottom_right.u + bottom_left.u) / 4.0,
                         (top_left.v + top_right.v + bottom_right.v + bottom_left.v) / 4.0,
                         ((top_right.u - top_left.u) + (bottom_right.u - bottom_left.u)) / 2.0,
                         ((bottom_left.v - top_left.v) + (bottom_right.v - top_right.v)) / 2.0};
        if (!largest || raw.height_px > largest->height_px) {
            largest = raw;
        }
    }
    return largest;
}

// Odometry owns the IMU (chassis.calibrate() in main.cpp), so heading comes from the chassis instead of a second IMU
// here. The camera does not move the odometry pose; it keeps its own estimate next to it.
Pose chassis_pose() {
    const lemlib::Pose pose = chassis.getPose();
    return {pose.x * field::kMmPerInch, pose.y * field::kMmPerInch, pose.theta};
}

bool finite(const Pose& pose) {
    return std::isfinite(pose.x_mm) && std::isfinite(pose.y_mm) && std::isfinite(pose.heading_deg);
}

// Odometry poses since the last setPose, so a picture can be matched to where the robot was when it was taken.
class History {
public:
    void clear() {
        start_ = 0;
        count_ = 0;
    }

    void add(std::uint32_t ms, const Pose& pose) {
        entries_[(start_ + count_) % kHistorySize] = {ms, pose};
        if (count_ < kHistorySize) {
            ++count_;
        } else {
            start_ = (start_ + 1) % kHistorySize;
        }
    }

    // Odometry at `ms`, straight-line between samples. Nothing if `ms` is older than everything kept.
    std::optional<Pose> at(std::uint32_t ms) const {
        if (count_ == 0 || ms < get(0).ms) {
            return std::nullopt;
        }
        for (std::size_t i = count_ - 1; i > 0; --i) {
            const Stamped& before = get(i - 1);
            const Stamped& after = get(i);
            if (ms >= after.ms) {
                return after.pose;
            }
            if (ms >= before.ms) {
                const double f = static_cast<double>(ms - before.ms) / static_cast<double>(after.ms - before.ms);
                return Pose{before.pose.x_mm + f * (after.pose.x_mm - before.pose.x_mm),
                            before.pose.y_mm + f * (after.pose.y_mm - before.pose.y_mm),
                            before.pose.heading_deg + f * (after.pose.heading_deg - before.pose.heading_deg)};
            }
        }
        return get(0).pose;
    }

private:
    struct Stamped {
        std::uint32_t ms;
        Pose pose;
    };

    const Stamped& get(std::size_t i) const {
        return entries_[(start_ + i) % kHistorySize];
    }

    std::array<Stamped, kHistorySize> entries_{};
    std::size_t start_ = 0;
    std::size_t count_ = 0;
};

bool same_view(std::span<const Detection> a, std::span<const Detection> b) {
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (a[i].id != b[i].id) {
            return false;
        }
        for (std::size_t k = 0; k < a[i].corners.size(); ++k) {
            if (a[i].corners[k].u != b[i].corners[k].u || a[i].corners[k].v != b[i].corners[k].v) {
                return false;
            }
        }
    }
    return true;
}

// Moves each candidate from where the robot was when the picture was taken to where odometry says it is now.
void move_forward(Solution& solution, const Pose& then, const Pose& now) {
    for (std::size_t i = 0; i < solution.count; ++i) {
        Pose& pose = solution.candidates[i].pose;
        pose.x_mm += now.x_mm - then.x_mm;
        pose.y_mm += now.y_mm - then.y_mm;
        pose.heading_deg += now.heading_deg - then.heading_deg;
    }
}

void run() {
    pros::AIVision sensor(ports::kVision);

    // main.cpp has already put odometry at config::kStart*, so that is where the estimate starts too.
    Pose odom = chassis_pose();
    std::uint32_t resets = chassis.poseResets();
    PoseFilter filter;
    filter.reset(odom, kBootSigmaMm);

    Snapshot published;
    published.pose = to_field_pose(odom);
    publish(published);

    History history;
    std::array<Detection, kMaxDetections> tags{};
    std::array<Detection, kMaxDetections> last_tags{};
    std::size_t last_count = 0;
    Pose last_read_odom = odom;
    std::uint32_t last_read_ms = pros::millis();
    bool configured = false;
    std::optional<std::uint32_t> last_fix_ms;
    std::uint32_t accepted = 0;
    std::uint32_t tick = 0;
    std::uint32_t last_ms = pros::millis();
    std::uint32_t wake_ms = last_ms;

    while (true) {
        const std::uint32_t now_ms = pros::millis();
        const Pose now = chassis_pose();
        const bool odom_ok = finite(now);
        const double dt_s = static_cast<double>(now_ms - last_ms) / 1000.0;
        last_ms = now_ms;

        // Follow odometry. A setPose is a teleport, not driving: start the estimate over at the new pose.
        const std::uint32_t now_resets = chassis.poseResets();
        if (now_resets != resets && odom_ok) {
            resets = now_resets;
            filter.reset(now, kResetSigmaMm);
            history.clear();
            last_fix_ms.reset();
            last_count = 0;
            last_read_odom = now;
        } else if (odom_ok && finite(odom)) {
            filter.predict(now.x_mm - odom.x_mm, now.y_mm - odom.y_mm, now.heading_deg - odom.heading_deg, dt_s);
        } else {
            filter.predict(0.0, 0.0, 0.0, dt_s);
        }
        if (odom_ok) {
            odom = now;
            history.add(now_ms, now);
        }

        if (++tick % kCameraEveryTicks == 0) {
            configured = configured || configure(sensor);
            const std::optional<std::size_t> count = configured ? read_tags(sensor, tags) : std::nullopt;
            configured = count.has_value();

            Snapshot next;
            next.correction_in = published.correction_in;
            if (count) {
                const auto seen = std::span<const Detection>(tags).first(*count);
                const bool repeat = same_view(seen, std::span<const Detection>(last_tags).first(last_count));
                const bool moved = std::hypot(odom.x_mm - last_read_odom.x_mm, odom.y_mm - last_read_odom.y_mm) >
                                       kStaleMoveMm ||
                                   std::fabs(odom.heading_deg - last_read_odom.heading_deg) > kStaleTurnDeg;
                const double read_gap_s = std::max(static_cast<double>(now_ms - last_read_ms) / 1000.0, 1e-3);
                const double turn_rate = std::fabs(odom.heading_deg - last_read_odom.heading_deg) / read_gap_s;
                const std::optional<Pose> then = history.at(now_ms - config::kCameraLatencyMs);

                Solution solution;
                Fix fix = Fix::None;
                if (*count == 0) {
                    next.outcome = Outcome::NoTags;
                } else if (repeat && moved) {
                    next.outcome = Outcome::Stale;
                } else if (turn_rate > kMaxTurnRateDegPerS) {
                    next.outcome = Outcome::Turning;
                } else if (!then) {
                    next.outcome = Outcome::NoHistory;
                } else {
                    // Solve with the heading the robot had when the picture was taken, then bring the answer up to now.
                    const Pose before = filter.pose();
                    const double heading_then = before.heading_deg - (odom.heading_deg - then->heading_deg);
                    solution = solve(seen, heading_then, kCamera);
                    move_forward(solution, *then, odom);
                    fix = filter.update(solution);
                    if (fix == Fix::Accepted) {
                        last_fix_ms = now_ms;
                        ++accepted;
                        const Pose after = filter.pose();
                        next.correction_in =
                            std::hypot(after.x_mm - before.x_mm, after.y_mm - before.y_mm) / field::kMmPerInch;
                    }
                    next.outcome = fix == Fix::Accepted    ? Outcome::Accepted
                                   : fix == Fix::Ambiguous ? Outcome::Ambiguous
                                   : fix == Fix::Rejected  ? Outcome::Rejected
                                                           : Outcome::NoFit;
                }

                const bool recent = last_fix_ms && now_ms - *last_fix_ms <= kLockHoldMs;
                next.status = fix == Fix::Ambiguous ? Status::Ambiguous : recent ? Status::Locked : Status::Searching;
                next.tags = static_cast<int>(*count);
                next.candidates = static_cast<int>(solution.count);
                next.raw = largest_tag(seen);
                if (solution.count > 0) {
                    next.goals = solution.candidates[0].goals;
                    next.error_px = solution.candidates[0].error_px;
                    next.range_in = solution.candidates[0].range_mm / field::kMmPerInch;
                }

                std::copy(seen.begin(), seen.end(), last_tags.begin());
                last_count = seen.size();
                last_read_odom = odom;
                last_read_ms = now_ms;
            } else {
                next.status = Status::NoSensor;
            }
            if (!odom_ok) {
                next.status = Status::NoImu;
            }
            next.accepted = accepted;
            published = next;
        }

        // The pose moves with odometry every tick; the camera fields change on reads.
        published.pose = to_field_pose(filter.pose());
        publish(published);

        pros::Task::delay_until(&wake_ms, kTickMs);
    }
}

}

void start() {
    pros::Task task(run, "Localizer");
}

Snapshot snapshot() {
    const std::lock_guard lock(shared_mutex);
    return shared;
}

namespace {

constexpr std::uint32_t kResetPollMs = 10;
// Odometry moving more than this from where the robot stopped restarts the still clock.
constexpr double kStillDriftIn = 0.25;
constexpr double kStillTurnDeg = 1.0;

const char* reset_name(ResetOutcome outcome) {
    switch (outcome) {
    case ResetOutcome::Applied:
        return "applied";
    case ResetOutcome::Agreed:
        return "agreed";
    case ResetOutcome::TooFar:
        return "too_far";
    case ResetOutcome::NoCamera:
        return "no_camera";
    case ResetOutcome::StillMoving:
        return "still_moving";
    }
    return "?";
}

ResetResult report(ResetOutcome outcome, double difference_in) {
    std::printf("ODOM_RESET,%s,%.2f\n", reset_name(outcome), difference_in);
    return {outcome, difference_in};
}

}

ResetResult reset_odometry(const ResetParams& params) {
    chassis.waitUntilDone();
    const std::uint32_t start = pros::millis();
    lemlib::Pose anchor = chassis.getPose();
    std::uint32_t still_since = start;
    std::uint32_t fixes_at_still = snapshot().accepted;
    bool was_still = false;

    while (pros::millis() - start <= params.timeout_ms) {
        const std::uint32_t now = pros::millis();
        const lemlib::Pose odom = chassis.getPose();
        if (std::hypot(odom.x - anchor.x, odom.y - anchor.y) > kStillDriftIn ||
            std::fabs(odom.theta - anchor.theta) > kStillTurnDeg) {
            anchor = odom;
            still_since = now;
            fixes_at_still = snapshot().accepted;
        }

        const Snapshot camera = snapshot();
        const bool still = now - still_since >= params.still_ms;
        was_still = was_still || still;
        const bool fresh = static_cast<int>(camera.accepted - fixes_at_still) >= params.fixes;
        if (still && fresh && camera.status == Status::Locked) {
            const double difference = std::hypot(camera.pose.x_in - odom.x, camera.pose.y_in - odom.y);
            if (difference > params.max_correction_in) {
                return report(ResetOutcome::TooFar, difference);
            }
            if (difference < params.min_correction_in) {
                return report(ResetOutcome::Agreed, difference);
            }
            // Keep the IMU heading: the camera only measures heading when it sees two goals at once.
            chassis.setPose(static_cast<float>(camera.pose.x_in), static_cast<float>(camera.pose.y_in), odom.theta);
            return report(ResetOutcome::Applied, difference);
        }
        pros::delay(kResetPollMs);
    }
    return report(was_still ? ResetOutcome::NoCamera : ResetOutcome::StillMoving, 0.0);
}

}
