#pragma once

#include "pros/abstract_motor.hpp"
#include "pros/rtos.hpp"
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>

namespace telemetry {

// PointWeaver hooks, called from the auton code it generates (src/autons/skills.cpp gets them through
// autons/routine.hpp). Both only store a value for the logger's next row and return, so they cost nothing on the
// motion path.
//
// route(): once, at the top of an auton's run(): which exported route this code is (PointWeaver's auton name and
// export time). Goes into the run's CSV as "# route,<name>" (in the metadata above the header row when it arrives
// before the file is opened, which is the usual case, else as a comment line before the next row).
void route(const char* name);
// mark(): right after every chassis motion call, wait and reset. The latest id and the pros::millis() it was set at go
// on every row as pw_mark and pw_mark_ms (-1 before the first mark of an auton). PointWeaver maps the id back to its
// code; the robot doesn't use it.
void mark(int id);

// CSV logger. A sampler task takes a row every `period` ms (every 100 ms while disabled; every 2 * period while the
// SD card is behind) into a RAM ring; a lowest-priority writer task formats the rows and writes them in big batches,
// flushing about every 0.5 s so a power-off leaves a readable file. Neither ever blocks the caller.
//
// One file per run: begin_run() at the start of autonomous (named after the auton) and of driver control ("driver")
// starts /usd/telemetry_<NNNN>_<name>.csv, NNNN from the counter in /usd/telemetry_session.txt. Nothing is logged
// before the first run starts.
class Logger {
public:
    using FieldCallback = double (*)(const void* context);

    explicit Logger(const char* directory = "/usd", std::uint32_t period = 10);

    // Columns: call these before start().
    bool attach(pros::AbstractMotor* motor);
    bool register_field(const char* name, FieldCallback callback, const void* context = nullptr);
    void register_robot_fields();
    void start();

    // A new file from the next row on. autonomous: also restarts auton_ms and clears the PointWeaver mark.
    void begin_run(const char* name, bool autonomous);
    void route(const char* name);
    // Writes "# key,value" into the current run's log just before its next row, for things that aren't numbers. Also
    // printed to the terminal as LOG_NOTE,key,value.
    void note(const char* key, const char* value);
    void dump();

private:
    static constexpr std::size_t kMaxPorts = 21;
    static constexpr std::size_t kMaxFields = 64;
    static constexpr std::size_t kMaxNotes = 8; // waiting to be written; more than this are dropped
    static constexpr std::size_t kMaxRuns = 4;  // run infos kept, for runs started faster than the writer catches up
    static constexpr std::size_t kNameLength = 32;
    static constexpr std::size_t kRouteLength = 96;

    struct Note {
        std::uint32_t run;
        char key[32];
        char value[64];
    };

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

    struct Run {
        std::uint32_t id = 0;
        bool autonomous = false;
        std::uint32_t start_ms = 0;
        char name[kNameLength] {};
    };

    void sampleLoop();
    void writeLoop();
    void takeSample(std::uint32_t run, std::uint32_t now);
    std::size_t columnCount() const;
    bool reserveSession(std::uint32_t& id);
    std::FILE* openRun(std::uint32_t run, std::uint32_t& routeWritten);
    bool writeHeader(std::FILE* file);
    std::size_t formatRow(char* out, const double* values) const;
    std::size_t formatNotes(char* out, std::size_t room, std::uint32_t run);
    std::size_t formatRoute(char* out, std::size_t room, std::uint32_t run, std::uint32_t& routeWritten);

    const char* directory;
    std::uint32_t period;
    std::array<MotorPort, kMaxPorts> motorPorts {};
    std::size_t motorPortCount = 0;
    std::array<Field, kMaxFields> fields {};
    std::size_t fieldCount = 0;
    bool robotFields = false; // register_robot_fields() ran: refresh its cached pose / vision snapshot each row
    bool started = false;
    std::unique_ptr<std::int8_t[]> columnDecimals; // per column; 0 writes an integer

    // Ring of rows (sampler -> writer): slot i holds its run id and columnCount() values.
    std::size_t columns = 0;
    std::size_t capacity = 0;
    std::unique_ptr<std::uint32_t[]> slotRun;
    std::unique_ptr<double[]> slotValues;
    std::atomic<std::size_t> head {0}; // next slot the sampler fills
    std::atomic<std::size_t> tail {0}; // next slot the writer reads
    std::atomic<std::uint32_t> dropped {0};

    std::array<Run, kMaxRuns> runs {};
    std::atomic<std::uint32_t> currentRun {0}; // 0: no run yet, nothing logged
    std::atomic<std::uint32_t> autonStartMs {0};
    std::atomic<bool> autonStarted {false};
    pros::Mutex runsMutex;

    char routeName[kRouteLength] {};
    std::uint32_t routeRun = 0;   // the run route() was called in
    std::uint32_t routeSeq = 0;   // goes up with every route() call
    pros::Mutex routeMutex;

    std::array<Note, kMaxNotes> notes {};
    std::size_t noteCount = 0;
    pros::Mutex notesMutex;

    pros::Mutex fileMutex; // the SD card: the writer and dump() take turns
};

}
