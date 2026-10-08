#include "telemetry/logger.hpp"
#include "lemlib/logger/logger.hpp"
#include "pros/misc.hpp"
#include <cstring>
#include <cstdio>

namespace telemetry {

Logger::Logger(const char* directory, std::uint32_t period) : directory(directory), period(period) {}

bool Logger::attach(pros::AbstractMotor* motor) {
    if (motor == nullptr || motorCount >= motors.size()) {
        return false;
    }
    for (std::size_t i = 0; i < motorCount; ++i) {
        if (motors[i] == motor) {
            return false;
        }
    }
    motors[motorCount++] = motor;
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
    const int length =
        std::snprintf(filePath, sizeof(filePath), "%s/telemetry_%lu.csv", directory,
                      static_cast<unsigned long>(pros::millis()));
    if (length < 0 || static_cast<std::size_t>(length) >= sizeof(filePath)) {
        return false;
    }
    file = std::fopen(filePath, "w");
    return file != nullptr;
}

bool Logger::writeMetadata(std::FILE* file) {
    const bool controllerConnected = pros::competition::is_connected();
    const bool fieldControl = pros::competition::is_field_control();
    const bool competitionSwitch = pros::competition::is_competition_switch();
    const bool autonomous = pros::competition::is_autonomous();
    const bool disabled = pros::competition::is_disabled();
    const bool inMatch = fieldControl || competitionSwitch;

    return std::fprintf(file, "# start_time_ms,%lu\n"
                              "# competition_controller_connected,%d\n"
                              "# field_control,%d\n"
                              "# competition_switch,%d\n"
                              "# autonomous,%d\n"
                              "# disabled,%d\n"
                              "# in_match,%d\n",
                        static_cast<unsigned long>(pros::millis()), controllerConnected, fieldControl,
                        competitionSwitch, autonomous, disabled, inMatch) >= 0;
}

bool Logger::writeHeader(std::FILE* file) {
    if (std::fprintf(file, "time_ms,competition_controller_connected,field_control,competition_switch,"
                           "autonomous,disabled,in_match") < 0) {
        return false;
    }
    for (std::size_t i = 0; i < motorCount; ++i) {
        if (std::fprintf(file, ",motor%zu_position,motor%zu_velocity,motor%zu_voltage_mV,motor%zu_temperature_C", i,
                         i, i, i) < 0) {
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
    for (std::size_t i = 0; i < motorCount; ++i) {
        pros::AbstractMotor* motor = motors[i];
        if (std::fprintf(file, ",%.3f,%.3f,%d,%.3f", motor->get_position(), motor->get_actual_velocity(),
                         motor->get_voltage(), motor->get_temperature()) < 0) {
            return false;
        }
    }
    return std::fputc('\n', file) != EOF;
}

}
