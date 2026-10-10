# 4253Z Code 6-7

Robot code built on top of PROS and LemLib, with project-specific control,
diagnostics, and mechanism abstractions layered on top of the standard
libraries.

## Key additions

### SmartMotor extensions

`lemlib::SmartMotor` was extended to support:

- ADI encoder, V5 Rotation Sensor, and integrated motor encoder feedback.
- A shared position/target interface for elevator-style mechanisms.
- Motor-group position averaging for grouped actuators.
- Synchronous and asynchronous PID moves with configurable timeout and
  acceptable-position range.
- Background diagnostics for commanded output, measured motor voltage, target,
  and current position.

The elevator subsystem builds on this abstraction instead of placing motor
control logic directly in the competition callbacks.

### Structured logging

The project uses LemLib's sink-based logging architecture for separate classes
of output:

- **Info logs** for human-readable debugging messages.
- **Telemetry logs** for machine-readable values intended for analysis.
- Configurable log levels and sinks so diagnostic output can be routed without
  coupling control code to a specific display or terminal.

SmartMotor diagnostics are emitted from a background PROS task so logging does
not block the control loop.

### Mechanism-level control

The subsystem layer adds reusable control around the raw PROS devices:

- Elevator homing, staged target positions, manual control, and brake/hold
  behavior.
- Pivot presets for home, flipped, scored, top, and dual-pickup positions.
- Intake, claw, and pivot driver-control tasks.
- Macro routines that coordinate elevator and pivot motion.
- Shared controller and mechanism helpers instead of duplicating button logic.

### Motion and driver-safety behavior

The robot code adds behavior beyond the basic LemLib drivetrain:

- Slewed throttle input for smoother driver control.
- Anti-tipping recovery based on inertial pitch with separate enter and exit
  thresholds.
- Mechanism tasks that stop when driver control ends, preventing duplicate
  tasks from accumulating across competition-state transitions.
- Brake-mode configuration for mechanisms that must hold position.

### Vision and dashboard integration

The Crimson vision wrapper is integrated with the dashboard and chassis
workflow. The robot can use camera target data and global-pose estimates
alongside the inertial sensor while the dashboard reports current vision and
robot state.

## Project structure

- `src/main.cpp` — PROS lifecycle, hardware setup, driver control, and
  competition callbacks.
- `src/lemlib/` — project extensions to LemLib, including `SmartMotor`.
- `src/subsystems/` — elevator and pivot mechanism behavior.
- `include/lemlib/logger/` — logging sinks and message routing.
- `src/vision/` — Crimson camera integration.
- `include/constants/` — field and robot constants.

## Design principles

- Keep hardware access behind subsystem interfaces.
- Keep control loops independent from logging and display I/O.
- Use background tasks for periodic diagnostics.
- Prefer explicit targets and named mechanism states over scattered motor
  commands.
- Stop mechanism tasks when their competition mode is no longer active.

## Building

Build the project with the PROS CLI:

```powershell
pros make
```

The generated PROS binary can then be downloaded to the V5 Brain using the
normal PROS workflow.
