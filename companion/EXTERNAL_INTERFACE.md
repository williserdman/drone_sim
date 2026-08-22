# Companion External Interface

## ROS 2 inputs

- Gazebo camera frames through ROS 2 image transport at 20 frames per simulated second
- Authoritative `/clock` with `use_sim_time=true`

Each frame carries `run_id`, `frame_id`, and a simulation capture timestamp.

## MAVLink interface

- Output: flight and mission commands to ArduPilot SITL
- Input: vehicle telemetry, modes, and command acknowledgements

Commands derived from imagery retain `source_frame_id` where the adapter permits.

## Timing and ordering

Mission logic is frame- or event-triggered in simulation time. Duplicate frames are idempotently ignored, missing frames are diagnosed, stale `run_id` data is ignored, and loss of `/clock` prevents new simulated decisions. Wall time measures computation and infrastructure health only.

## Prohibited path

The companion must never directly command or mutate Gazebo.

## Deferred decisions

- Exact topic names, message schemas, and QoS values
- MAVLink ports, routing, and command-to-frame correlation encoding

