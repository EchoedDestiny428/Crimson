#include "macros.hpp"
#include "robotconfig.hpp"
#include "subsystems/claw.hpp"
#include "subsystems/elevator.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/pivot.hpp"
#include "telemetry/channels.hpp"
#include "telemetry/keys.hpp"
#include "telemetry/samplers.hpp"

namespace telemetry::mechanism_log {

namespace {

namespace claw = subsystems::claw;
namespace elevator = subsystems::elevator;
namespace intake = subsystems::intake;
namespace pivot = subsystems::pivot;

const std::array<MotorChannel, 5>& motors() {
    static const std::array<MotorChannel, 5> channels{
        MotorChannel(elevator_motors, 0, keys::kMotorElevatorA, keys::kMotorElevatorAHealth),
        MotorChannel(elevator_motors, 1, keys::kMotorElevatorB, keys::kMotorElevatorBHealth),
        MotorChannel(intake_motor, 0, keys::kMotorIntake, keys::kMotorIntakeHealth),
        MotorChannel(claw_motor, 0, keys::kMotorClaw, keys::kMotorClawHealth),
        MotorChannel(pivot_motor, 0, keys::kMotorPivot, keys::kMotorPivotHealth),
    };
    return channels;
}

}

void fast(std::uint64_t timestamp) {
    static const std::uint32_t elevator_id = logger.entry(keys::kMechanismElevator, structs::Mechanism::kType);
    static const std::uint32_t pivot_id = logger.entry(keys::kMechanismPivot, structs::Mechanism::kType);
    static const std::uint32_t manual_id = logger.entry(keys::kMechanismElevatorManual, types::kDouble);
    static const std::uint32_t claw_id = logger.entry(keys::kMechanismClawPower, types::kDouble);
    static StringChannel macro_mode(keys::kMechanismMacroMode);
    static BooleanChannel tilted(keys::kMechanismTilted);
    static BooleanChannel offset_on(keys::kMechanismOffsetOn);
    static BooleanChannel intake_lift_state(keys::kPneumaticIntakeLift);
    static BooleanChannel pull_toggle_state(keys::kPneumaticPullToggle);

    for (const MotorChannel& motor : motors()) {
        motor.record(timestamp);
    }
    const structs::Mechanism elevator_state{to_double(elevator::target()), to_double(elevator::height()),
                                            elevator::is_settled()};
    const structs::Mechanism pivot_state{to_double(pivot_motor.get_target_position()), to_double(pivot::get_pos()),
                                         pivot::is_settled()};
    logger.record_struct(elevator_id, elevator_state.pack(), timestamp);
    logger.record_struct(pivot_id, pivot_state.pack(), timestamp);
    logger.record_double(manual_id, elevator::manual_power(), timestamp);
    logger.record_double(claw_id, claw::commanded_power(), timestamp);
    macro_mode.record(macros::mode_name(), timestamp);
    tilted.record(macros::tilted(), timestamp);
    offset_on.record(macros::offset_on(), timestamp);
    intake_lift_state.record(intake::lifted(), timestamp);
    pull_toggle_state.record(pull_toggle.is_extended(), timestamp);
}

void slow(std::uint64_t timestamp) {
    for (const MotorChannel& motor : motors()) {
        motor.record_health(timestamp);
    }
}

}
