#pragma once

#include "pros/abstract_motor.hpp"
#include "pros/rtos.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace telemetry {

class Logger {
public:
    using FieldCallback = double (*)(const void* context);

    explicit Logger(const char* directory = "/usd", std::uint32_t period = 20);

    bool attach(pros::AbstractMotor* motor);
    bool register_field(const char* name, FieldCallback callback, const void* context = nullptr);
    void start();
    void dump();

private:
    static constexpr std::size_t kMaxPorts = 21;
    static constexpr std::size_t kMaxFields = 64;

    struct Field {
        const char* name;
        FieldCallback callback;
        const void* context;
    };

    struct MotorPort {
        pros::AbstractMotor* motor;
        std::uint8_t index;
        std::int8_t port;
    };

    void run();
    bool reserveSession();
    bool openFile(std::FILE*& file);
    bool writeMetadata(std::FILE* file);
    bool writeHeader(std::FILE* file);
    bool writeSample(std::FILE* file);

    const char* directory;
    std::uint32_t period;
    std::uint32_t sessionId = 0;
    char filePath[64] {};
    std::array<MotorPort, kMaxPorts> motorPorts {};
    std::size_t motorPortCount = 0;
    std::array<Field, kMaxFields> fields {};
    std::size_t fieldCount = 0;
    pros::Task* task = nullptr;
    pros::Mutex fileMutex;
};

}
