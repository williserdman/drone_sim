# Root Internal Interface

## Purpose

Define relationships among the repository's immediate child modules. Top-level children are initially separate processes, so they communicate through their external interfaces rather than language imports.

## Sibling modules

| Producer | Consumer | Mechanism | Data |
| --- | --- | --- | --- |
| Companion | ArduPilot SITL | MAVLink | Mission and flight commands |
| ArduPilot SITL | Companion | MAVLink | Telemetry, modes, acknowledgements |
| ArduPilot SITL | Gazebo | ArduPilot-Gazebo adapter | Actuator outputs |
| Gazebo | ArduPilot SITL | ArduPilot-Gazebo adapter | Simulated sensors and dynamics |
| Gazebo | Companion | ROS 2 image transport | Camera frames |
| Gazebo | Scorekeeper | ROS 2 | Ground truth |
| Electromagnet | Gazebo | ROS 2 | Physical-effect requests |
| Electromagnet | Scorekeeper | ROS 2 | Scenario events |
| Gazebo | Simulation-aware modules | ROS 2 `/clock` | Simulation time |

## Allowed dependencies

Nested same-process siblings may import one another only through the interface documented by the module they consume.

## Forbidden dependencies

- Companion-to-Gazebo control
- Electromagnet-to-ArduPilot or direct aircraft-state mutation
- Scorekeeper-to-control or physics mutation
- Imports of another module's private implementation files

## Run identity

Every run has a unique `run_id`. Run-scoped messages preserve it, and receivers ignore stale run IDs while emitting diagnostics.

## Timing rules

Simulation time schedules simulated behavior. Wall time is limited to infrastructure health checks, profiling, and host-performance diagnostics.

