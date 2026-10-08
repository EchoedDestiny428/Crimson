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
    explicit Logger(const char* directory = "/usd", std::uint32_t period = 20);

    bool attach(pros::AbstractMotor* motor);
    void start();
    void dump();

private:
    static constexpr std::size_t kMaxMotors = 21;

    void run();
    bool openFile(std::FILE*& file);
    bool writeMetadata(std::FILE* file);
    bool writeHeader(std::FILE* file);
    bool writeSample(std::FILE* file);

    const char* directory;
    std::uint32_t period;
    char filePath[64] {};
    std::array<pros::AbstractMotor*, kMaxMotors> motors {};
    std::size_t motorCount = 0;
    pros::Task* task = nullptr;
    pros::Mutex fileMutex;
};

}
