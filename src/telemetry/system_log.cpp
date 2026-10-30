#include "autons/autons.hpp"
#include "pros/misc.hpp"
#include "robotconfig.hpp"
#include "telemetry/channels.hpp"
#include "telemetry/keys.hpp"
#include "telemetry/samplers.hpp"
#include <algorithm>

namespace telemetry::system_log {

namespace {

constexpr int kButtonCount = pros::E_CONTROLLER_DIGITAL_A - pros::E_CONTROLLER_DIGITAL_L1 + 1;
constexpr std::int32_t kStickLimit = 127;

std::int8_t stick(pros::controller_analog_e_t channel) {
    return static_cast<std::int8_t>(std::clamp(controller.get_analog(channel), -kStickLimit, kStickLimit));
}

structs::Controller read_controller() {
    std::uint16_t buttons = 0;
    for (int i = 0; i < kButtonCount; ++i) {
        const auto button = static_cast<pros::controller_digital_e_t>(pros::E_CONTROLLER_DIGITAL_L1 + i);
        if (controller.get_digital(button) == 1) {
            buttons = static_cast<std::uint16_t>(buttons | (1u << i));
        }
    }
    return {
        stick(pros::E_CONTROLLER_ANALOG_LEFT_X),
        stick(pros::E_CONTROLLER_ANALOG_LEFT_Y),
        stick(pros::E_CONTROLLER_ANALOG_RIGHT_X),
        stick(pros::E_CONTROLLER_ANALOG_RIGHT_Y),
        buttons,
    };
}

structs::Battery read_battery() {
    return {
        to_float(pros::battery::get_voltage(), 0.001f),
        to_float(pros::battery::get_current(), 0.001f),
        to_float(pros::battery::get_capacity()),
        to_float(pros::battery::get_temperature()),
    };
}

}

void every_tick(std::uint64_t timestamp) {
    static BooleanChannel enabled(keys::kDriverStationEnabled);
    static BooleanChannel autonomous(keys::kDriverStationAutonomous);
    static BooleanChannel connected(keys::kDriverStationConnected);
    static BooleanChannel field_control(keys::kDriverStationFieldControl);
    static StringChannel selected(keys::kAutonSelected);

    enabled.record(!pros::competition::is_disabled(), timestamp);
    autonomous.record(pros::competition::is_autonomous(), timestamp);
    connected.record(pros::competition::is_connected(), timestamp);
    field_control.record(pros::competition::is_field_control(), timestamp);
    selected.record(autons::selected_name(), timestamp);
}

void fast(std::uint64_t timestamp) {
    static StructChannel<structs::Controller> master(keys::kControllerMaster);
    static BooleanChannel controller_connected(keys::kControllerConnected);

    controller_connected.record(controller.is_connected() == 1, timestamp);
    master.record(read_controller(), timestamp);
}

void slow(std::uint64_t timestamp) {
    static const std::uint32_t battery = logger.entry(keys::kSystemBattery, structs::Battery::kType);
    static const std::uint32_t controller_battery = logger.entry(keys::kControllerBattery, types::kInt64);
    static const std::uint32_t dropped = logger.entry(keys::kLoggerDropped, types::kInt64);
    static const std::uint32_t bytes = logger.entry(keys::kLoggerBytes, types::kInt64);
    static const std::uint32_t overruns = logger.entry(keys::kLoggerLoopOverruns, types::kInt64);

    logger.record_struct(battery, read_battery().pack(), timestamp);
    logger.record_int64(controller_battery, controller.get_battery_capacity(), timestamp);
    logger.record_int64(dropped, static_cast<std::int64_t>(logger.dropped()), timestamp);
    logger.record_int64(bytes, static_cast<std::int64_t>(logger.bytes_written()), timestamp);
    logger.record_int64(overruns, loop_overruns(), timestamp);
}

}
