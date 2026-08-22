# Companion Plan

## Responsibility

Run mission, vision, and autonomy logic; consume imagery and vehicle state; command the aircraft through ArduPilot.

## Implementation constraints

- Keep edits to the nested `companion/comp2026` repository to the minimum required for new camera input, simulation timing, MAVLink wiring, and removal or disabling of incompatible LiDAR assumptions.
- Do not push changes from the nested repository.
- Keep container adapters and simulation-specific wrappers outside the nested repository whenever possible.

## Non-responsibilities

- Flight stabilization and actuator control
- Physics, sensors, ground truth, or direct Gazebo state changes
- Scenario effects and scoring

## Inputs

Simulation-timestamped camera frames, ROS 2 `/clock`, MAVLink telemetry, modes, and acknowledgements.

## Outputs

MAVLink flight and mission commands plus wall-clock performance diagnostics.

## Implementation stages

1. Receive camera frames and preserve `run_id`, frame identity, and capture time.
2. Run frame-triggered vision and mission decisions.
3. Issue MAVLink commands with causal frame metadata where supported.
4. Consume acknowledgements and telemetry.
5. Expose health and latency diagnostics without scheduling from wall time.

## Acceptance criteria

- Camera handling represents 20 frames per simulated second.
- Commands flow only through ArduPilot.
- Stale runs and duplicate frames are handled deterministically.
- Lost simulation time prevents new simulated decisions.

## Deferred decisions

- Mission and vision language
- MAVLink endpoint and causal-correlation encoding
