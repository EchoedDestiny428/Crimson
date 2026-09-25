#include "util/parallel.hpp"
#include "pros/rtos.hpp"
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace util {

namespace {

constexpr std::uint32_t kPollMs = 5;

}

void parallel(std::vector<std::function<void()>> tasks) {
    const auto remaining = std::make_shared<std::atomic<std::size_t>>(tasks.size());

    for (auto& task : tasks) {
        pros::Task worker(
            [remaining, task = std::move(task)] {
                task();
                --*remaining;
            },
            "Parallel");
    }

    while (*remaining > 0) {
        pros::delay(kPollMs);
    }
}

}
