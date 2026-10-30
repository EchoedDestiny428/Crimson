#include "robotconfig.hpp"
#include "telemetry/channels.hpp"
#include "telemetry/keys.hpp"
#include "telemetry/samplers.hpp"

namespace telemetry::vision_log {

void fast(std::uint64_t timestamp) {
    static BooleanChannel connected(keys::kVisionConnected);
    static const std::uint32_t tag_count = logger.entry(keys::kVisionTagCount, types::kInt64);
    static const std::uint32_t primary_tag = logger.entry(keys::kVisionPrimaryTag, types::kInt64);
    static const std::uint32_t tx = logger.entry(keys::kVisionTx, types::kDouble);
    static const std::uint32_t ty = logger.entry(keys::kVisionTy, types::kDouble);
    static const std::uint32_t pose = logger.entry(keys::kVisionPose, structs::Pose::kType);

    const bool online = crimson_cam.connected();
    connected.record(online, timestamp);
    if (!online) {
        return;
    }

    logger.record_int64(tag_count, crimson_cam.tag_count(), timestamp);
    logger.record_int64(primary_tag, crimson_cam.primary_tag_id(), timestamp);
    logger.record_double(tx, crimson_cam.get_tx(), timestamp);
    logger.record_double(ty, crimson_cam.get_ty(), timestamp);

    const double heading = chassis.getPose().theta;
    if (const auto estimate = crimson_cam.estimate_global_pose(heading)) {
        logger.record_struct(pose, structs::Pose{estimate->x, estimate->y, heading}.pack(), timestamp);
    }
}

}
