#pragma once

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace telemetry::structs {

static_assert(std::endian::native == std::endian::little);

template <typename... Fields>
constexpr std::array<std::uint8_t, (sizeof(Fields) + ...)> pack_fields(const Fields&... fields) {
    static_assert((std::is_arithmetic_v<Fields> && ...));
    static_assert((!std::is_same_v<Fields, bool> && ...));
    std::array<std::uint8_t, (sizeof(Fields) + ...)> bytes{};
    std::size_t offset = 0;
    const auto put = [&bytes, &offset]<typename Field>(const Field& field) {
        for (const std::uint8_t byte : std::bit_cast<std::array<std::uint8_t, sizeof(Field)>>(field)) {
            bytes[offset++] = byte;
        }
    };
    (put(fields), ...);
    return bytes;
}

constexpr std::uint8_t flag(bool value) {
    return value ? 1 : 0;
}

struct Pose {
    static constexpr const char* kType = "struct:Pose";
    static constexpr const char* kSchemaKey = "/.schema/struct:Pose";
    static constexpr const char* kSchema = "double x;double y;double theta";
    static constexpr std::size_t kSize = 24;

    double x;
    double y;
    double theta;

    constexpr std::array<std::uint8_t, kSize> pack() const {
        return pack_fields(x, y, theta);
    }
};

struct Motor {
    static constexpr const char* kType = "struct:Motor";
    static constexpr const char* kSchemaKey = "/.schema/struct:Motor";
    static constexpr const char* kSchema =
        "float position;float velocity;float voltage;float current;float power;float torque";
    static constexpr std::size_t kSize = 24;

    float position;
    float velocity;
    float voltage;
    float current;
    float power;
    float torque;

    constexpr std::array<std::uint8_t, kSize> pack() const {
        return pack_fields(position, velocity, voltage, current, power, torque);
    }
};

struct MotorHealth {
    static constexpr const char* kType = "struct:MotorHealth";
    static constexpr const char* kSchemaKey = "/.schema/struct:MotorHealth";
    static constexpr const char* kSchema =
        "float temperature;float efficiency;uint32 faults;uint32 flags;bool overTemp;bool overCurrent";
    static constexpr std::size_t kSize = 18;

    float temperature;
    float efficiency;
    std::uint32_t faults;
    std::uint32_t flags;
    bool overTemp;
    bool overCurrent;

    constexpr std::array<std::uint8_t, kSize> pack() const {
        return pack_fields(temperature, efficiency, faults, flags, flag(overTemp), flag(overCurrent));
    }
};

struct Imu {
    static constexpr const char* kType = "struct:Imu";
    static constexpr const char* kSchemaKey = "/.schema/struct:Imu";
    static constexpr const char* kSchema =
        "double heading;double rotation;float pitch;float roll;float yaw;float gyroX;float gyroY;float gyroZ;"
        "float accelX;float accelY;float accelZ;uint8 status";
    static constexpr std::size_t kSize = 53;

    double heading;
    double rotation;
    float pitch;
    float roll;
    float yaw;
    float gyroX;
    float gyroY;
    float gyroZ;
    float accelX;
    float accelY;
    float accelZ;
    std::uint8_t status;

    constexpr std::array<std::uint8_t, kSize> pack() const {
        return pack_fields(heading, rotation, pitch, roll, yaw, gyroX, gyroY, gyroZ, accelX, accelY, accelZ, status);
    }
};

struct Wheel {
    static constexpr const char* kType = "struct:Wheel";
    static constexpr const char* kSchemaKey = "/.schema/struct:Wheel";
    static constexpr const char* kSchema = "int32 position;int32 velocity;int32 angle";
    static constexpr std::size_t kSize = 12;

    std::int32_t position;
    std::int32_t velocity;
    std::int32_t angle;

    constexpr std::array<std::uint8_t, kSize> pack() const {
        return pack_fields(position, velocity, angle);
    }
};

struct Battery {
    static constexpr const char* kType = "struct:Battery";
    static constexpr const char* kSchemaKey = "/.schema/struct:Battery";
    static constexpr const char* kSchema = "float voltage;float current;float capacity;float temperature";
    static constexpr std::size_t kSize = 16;

    float voltage;
    float current;
    float capacity;
    float temperature;

    constexpr std::array<std::uint8_t, kSize> pack() const {
        return pack_fields(voltage, current, capacity, temperature);
    }
};

struct Controller {
    static constexpr const char* kType = "struct:Controller";
    static constexpr const char* kSchemaKey = "/.schema/struct:Controller";
    static constexpr const char* kSchema = "int8 leftX;int8 leftY;int8 rightX;int8 rightY;uint16 buttons";
    static constexpr std::size_t kSize = 6;

    std::int8_t leftX;
    std::int8_t leftY;
    std::int8_t rightX;
    std::int8_t rightY;
    std::uint16_t buttons;

    constexpr std::array<std::uint8_t, kSize> pack() const {
        return pack_fields(leftX, leftY, rightX, rightY, buttons);
    }
};

enum class MotionKind : std::uint8_t {
    MoveToPoint = 1,
    MoveToPose = 2,
    TurnToPoint = 3,
    TurnToHeading = 4,
    SwingToPoint = 5,
    SwingToHeading = 6,
    Donut = 7,
};

struct Motion {
    static constexpr const char* kType = "struct:Motion";
    static constexpr const char* kSchemaKey = "/.schema/struct:Motion";
    static constexpr const char* kSchema = "uint8 kind;double x;double y;double theta;int32 timeout;float maxSpeed;"
                                           "float minSpeed;float earlyExit;bool forwards;bool async";
    static constexpr std::size_t kSize = 43;

    MotionKind kind;
    double x;
    double y;
    double theta;
    std::int32_t timeout;
    float maxSpeed;
    float minSpeed;
    float earlyExit;
    bool forwards;
    bool async;

    constexpr std::array<std::uint8_t, kSize> pack() const {
        return pack_fields(static_cast<std::uint8_t>(kind), x, y, theta, timeout, maxSpeed, minSpeed, earlyExit,
                           flag(forwards), flag(async));
    }
};

}
