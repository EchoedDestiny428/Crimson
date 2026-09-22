#pragma once

#include "lemlib/smartMotor.hpp"

/**
 * @brief Elevator subsystem for height control using SmartMotor PID
 */
namespace subsystems {

/**
 * @brief Initialize the elevator subsystem
 */
void elevator_init();

/**
 * @brief Move elevator to a target height with bounds checking
 * @param target Target height in encoder units
 * @param timeout Maximum time to reach target in milliseconds
 */
void elev_goto(double target, float timeout = 2000);

/**
 * @brief Update elevator control during driver control
 * Handles preset buttons and manual control
 */
void elevator_update();

} // namespace subsystems
