#include "display/controller_hud.hpp"
#include "robotconfig.hpp"
#include "subsystems/elevator.hpp"
#include <cstdint>

namespace display::controller_hud {

namespace {

constexpr std::uint32_t kPrintEveryTicks = 5;

std::uint32_t tick = 0;

}

void update() {
    if (++tick % kPrintEveryTicks != 0) {
        return;
    }
    controller.print(0, 0, "P:%5.0f E:%5.0f", pivot_motor.get_position(), subsystems::elevator::height());
}

}
    