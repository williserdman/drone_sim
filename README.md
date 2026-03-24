## Purpose

This repository contains core code for MAV mission control and sensor interfaces used for precision VTOL research and prototyping.

## Project layout

Top-level (under `src/`):

- `common_types.py` — shared type definitions and aliases.
- `drone/` — high-level drone mission code and control logic.
    - `misson.py` — mission entrypoint and high-level mission orchestration.
    - `control/drone_control.py` — drone control algorithms and commands.
    - `control/mission_info.py` — mission metadata and state structures.
    - `sensors/camera/_camera_manager.py` — internal camera manager implementation.
    - `sensors/camera/camera.py` — camera interface abstractions.
    - `sensors/lidar/lidar.py` — lidar sensor wrapper and helpers.
    - `utils/position_smoother.py` — helper for smoothing position estimates.
