#pragma once

/**
 * @brief Elevator subsystem for height control with PID feedback
 */
namespace subsystems {

/**
 * @brief Initialize the elevator subsystem
 */
void elevator_init();

/**
 * @brief Move elevator to a target height with bounds checking
 * @param target Target height in encoder units
 */
void elev_goto(double target);

/**
 * @brief PID control loop for holding elevator at setpoint
 * @param pos Current elevator position
 */
void elev_hold_pid(double pos);

/**
 * @brief Update elevator control during driver control
 * @param pos Current elevator position
 */
void elevator_update(double pos);

} // namespace subsystems
