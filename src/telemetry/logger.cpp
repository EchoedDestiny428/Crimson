#include "telemetry/logger.hpp"
#include "lemlib/logger/logger.hpp"
#include "pros/error.h"
#include "pros/misc.hpp"
#include <cmath>
#include <cstring>
#include <cstdio>

namespace telemetry {

Logger::Logger(const char* directory, std::uint32_t period) : directory(directory), period(period) {}

bool Logger::attach(pros::AbstractMotor* motor) {
    if (motor == nullptr) {
        return false;
    }
    bool attached = false;
    const std::vector<std::int8_t> ports = motor->get_port_all();
    for (std::size_t index = 0; index < ports.size(); ++index) {
        if (motorPortCount >= motorPorts.size()) {
            break;
        }
        const std::int8_t port = ports[index];
        bool duplicate = false;
        for (std::size_t i = 0; i < motorPortCount; ++i) {
            const int existingPort = motorPorts[i].port < 0 ? -motorPorts[i].port : motorPorts[i].port;
            const int currentPort = port < 0 ? -port : port;
            if (existingPort == currentPort) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            motorPorts[motorPortCount++] = {motor, static_cast<std::uint8_t>(index), port};
            attached = true;
        }
    }
    return attached;
}

bool Logger::register_field(const char* name, FieldCallback callback, const void* context) {
    if (name == nullptr || callback == nullptr || fieldCount >= fields.size()) {
        return false;
    }
    for (std::size_t i = 0; i < fieldCount; ++i) {
        if (std::strcmp(fields[i].name, name) == 0) {
            return false;
        }
    }
    fields[fieldCount++] = {name, callback, context};
    return true;
}

void Logger::start() {
    if (task == nullptr) {
        task = new pros::Task([this] { run(); }, "Telemetry");
    }
}

void Logger::dump() {
    if (pros::usd::is_installed() == 0) {
        std::printf("LOG_DUMP_ERROR,sd_not_installed\n");
        return;
    }

    char fileList[4096] {};
    if (pros::usd::list_files("/", fileList, sizeof(fileList)) < 0) {
        std::printf("LOG_DUMP_ERROR,list_files_failed\n");
        return;
    }

    fileMutex.take();
    char* fileName = fileList;
    while (*fileName != '\0') {
        char* lineEnd = std::strchr(fileName, '\n');
        if (lineEnd != nullptr) {
            *lineEnd = '\0';
        }

        if (std::strncmp(fileName, "telemetry_", 10) == 0 &&
            std::strstr(fileName, ".csv") != nullptr) {
            char path[96] {};
            std::snprintf(path, sizeof(path), "/usd/%s", fileName);
            std::FILE* file = std::fopen(path, "r");
            if (file != nullptr) {
                std::printf("LOG_DUMP_BEGIN,%s\n", fileName);
                char line[256];
                while (std::fgets(line, sizeof(line), file) != nullptr) {
                    std::printf("%s", line);
                    pros::delay(2);
                }
                std::fclose(file);
                std::printf("LOG_DUMP_END,%s\n", fileName);
            }
        }

        if (lineEnd == nullptr) {
            break;
        }
        fileName = lineEnd + 1;
    }
    fileMutex.give();
    std::printf("LOG_DUMP_COMPLETE\n");
}

void Logger::run() {
    if (pros::usd::is_installed() == 0) {
        lemlib::infoSink()->warn("Telemetry: SD card is not installed");
        return;
    }

    if (!reserveSession()) {
        lemlib::infoSink()->warn("Telemetry: unable to reserve a session number");
        return;
    }

    std::FILE* file = nullptr;
    if (!openFile(file)) {
        lemlib::infoSink()->warn("Telemetry: unable to open {}", filePath);
        return;
    }
    if (!writeMetadata(file) || !writeHeader(file)) {
        std::fclose(file);
        lemlib::infoSink()->warn("Telemetry: unable to write metadata to {}", filePath);
        return;
    }

    std::uint32_t now = pros::millis();
    while (true) {
        fileMutex.take();
        if (!writeSample(file)) {
            fileMutex.give();
            lemlib::infoSink()->warn("Telemetry: unable to write to {}", filePath);
            break;
        }
        std::fflush(file);
        fileMutex.give();
        pros::Task::delay_until(&now, period);
    }
    std::fclose(file);
}

bool Logger::openFile(std::FILE*& file) {
    for (std::uint32_t suffix = 0; suffix < 1000000; ++suffix) {
        const int length = suffix == 0
                               ? std::snprintf(filePath, sizeof(filePath), "%s/telemetry_%06lu.csv", directory,
                                               static_cast<unsigned long>(sessionId))
                               : std::snprintf(filePath, sizeof(filePath), "%s/telemetry_%06lu_%u.csv", directory,
                                               static_cast<unsigned long>(sessionId), suffix);
        if (length < 0 || static_cast<std::size_t>(length) >= sizeof(filePath)) {
            return false;
        }

        std::FILE* existing = std::fopen(filePath, "r");
        if (existing != nullptr) {
            std::fclose(existing);
            continue;
        }

        file = std::fopen(filePath, "w");
        return file != nullptr;
    }
    return false;
}

bool Logger::reserveSession() {
    char counterPath[64] {};
    const int pathLength = std::snprintf(counterPath, sizeof(counterPath), "%s/telemetry_session.txt", directory);
    if (pathLength < 0 || static_cast<std::size_t>(pathLength) >= sizeof(counterPath)) {
        return false;
    }

    std::uint32_t previous = 0;
    std::FILE* counter = std::fopen(counterPath, "r");
    if (counter != nullptr) {
        const int result = std::fscanf(counter, "%u", &previous);
        std::fclose(counter);
        if (result != 1) {
            return false;
        }
    }

    sessionId = previous + 1;
    counter = std::fopen(counterPath, "w");
    if (counter == nullptr) {
        return false;
    }
    const bool written = std::fprintf(counter, "%u\n", sessionId) >= 0;
    std::fclose(counter);
    return written;
}

bool Logger::writeMetadata(std::FILE* file) {
    const bool controllerConnected = pros::competition::is_connected();
    const bool fieldControl = pros::competition::is_field_control();
    const bool competitionSwitch = pros::competition::is_competition_switch();
    const bool autonomous = pros::competition::is_autonomous();
    const bool disabled = pros::competition::is_disabled();
    const bool inMatch = fieldControl || competitionSwitch;

    return std::fprintf(file, "# session_id,%u\n"
                              "# start_time_ms,%lu\n"
                              "# competition_controller_connected,%d\n"
                              "# field_control,%d\n"
                              "# competition_switch,%d\n"
                              "# autonomous,%d\n"
                              "# disabled,%d\n"
                              "# in_match,%d\n",
                        sessionId, static_cast<unsigned long>(pros::millis()), controllerConnected, fieldControl,
                        competitionSwitch, autonomous, disabled, inMatch) >= 0;
}

bool Logger::writeHeader(std::FILE* file) {
    if (std::fprintf(file, "time_ms,competition_controller_connected,field_control,competition_switch,"
                           "autonomous,disabled,in_match") < 0) {
        return false;
    }
    for (std::size_t i = 0; i < motorPortCount; ++i) {
        const int port = motorPorts[i].port < 0 ? -motorPorts[i].port : motorPorts[i].port;
        if (std::fprintf(file,
                         ",port_%d_position,port_%d_velocity,port_%d_voltage_mV,port_%d_temperature_C"
                         ",port_%d_current_mA,port_%d_target_position,port_%d_target_velocity",
                         port, port, port, port, port, port, port) < 0) {
            return false;
        }
    }
    for (std::size_t i = 0; i < fieldCount; ++i) {
        if (std::fprintf(file, ",%s", fields[i].name) < 0) {
            return false;
        }
    }
    return std::fputc('\n', file) != EOF;
}

bool Logger::writeSample(std::FILE* file) {
    const bool controllerConnected = pros::competition::is_connected();
    const bool fieldControl = pros::competition::is_field_control();
    const bool competitionSwitch = pros::competition::is_competition_switch();
    const bool autonomous = pros::competition::is_autonomous();
    const bool disabled = pros::competition::is_disabled();
    const bool inMatch = fieldControl || competitionSwitch;

    if (std::fprintf(file, "%lu,%d,%d,%d,%d,%d,%d", static_cast<unsigned long>(pros::millis()), controllerConnected,
                     fieldControl, competitionSwitch, autonomous, disabled, inMatch) < 0) {
        return false;
    }
    for (std::size_t i = 0; i < motorPortCount; ++i) {
        const MotorPort& motorPort = motorPorts[i];
        pros::AbstractMotor* motor = motorPort.motor;
        const std::uint8_t index = motorPort.index;
        const auto writeDouble = [file](double value) {
            return std::isfinite(value) ? std::fprintf(file, "%.3f", value) : std::fprintf(file, "nan");
        };
        const auto writeInteger = [file](std::int32_t value) {
            return value == PROS_ERR ? std::fprintf(file, "nan") : std::fprintf(file, "%d", value);
        };

        if (std::fputc(',', file) == EOF || writeDouble(motor->get_position(index)) < 0 ||
            std::fputc(',', file) == EOF || writeDouble(motor->get_actual_velocity(index)) < 0 ||
            std::fputc(',', file) == EOF || writeInteger(motor->get_voltage(index)) < 0 ||
            std::fputc(',', file) == EOF || writeDouble(motor->get_temperature(index)) < 0 ||
            std::fputc(',', file) == EOF || writeInteger(motor->get_current_draw(index)) < 0 ||
            std::fputc(',', file) == EOF || writeDouble(motor->get_target_position(index)) < 0 ||
            std::fputc(',', file) == EOF || writeInteger(motor->get_target_velocity(index)) < 0) {
            return false;
        }
    }
    for (std::size_t i = 0; i < fieldCount; ++i) {
        if (std::fprintf(file, ",%.6f", fields[i].callback(fields[i].context)) < 0) {
            return false;
        }
    }
    return std::fputc('\n', file) != EOF;
}

}
