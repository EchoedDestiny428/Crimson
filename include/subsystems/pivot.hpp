#pragma once

/**
 * @brief Pivot subsystem: manual hold control and the A-button macro
 */
namespace subsystems {

/**
 * @brief Set the pivot brake mode used outside of a macro
 */
void pivot_init();

/**
 * @brief Driver-control task for the pivot.
 *
 * Hold X to run forward, hold B to run reverse. Tap A to turn +90 degrees
 * and raise the elevator. The loop exits when the robot leaves driver control.
 */
void pivot_loop();

} // namespace subsystems
