# First Gazebo Compose smoke

Date: 2026-08-24

Milestone: first Gazebo runtime smoke run through Docker Compose.

The `gazebo-runtime` service was built and launched with project
`vertical-gazebo-smoke`, `--pull never`, and the production Phase 3 image.
The bounded smoke exited zero after explicit Compose teardown.

## Runtime evidence

- Image: `drone-sim-gazebo-runtime:phase3`
- Image ID: `sha256:2bb824b8db75153e259ff5dd0ab971c2e6831be65123e14c0a1be364e4c02c9d`
- Preserved evidence root: `/tmp/drone-sim-compose-gazebo.eVBZ4A`
- Onboard frames: 4
- Observer frames: 4
- Ground-truth samples: 4
- Simulation timestamps for all three streams: 50, 100, 150, and 200 ms
- Gazebo server exit: graceful, return code 0
- `gazebo/server.log`: 1,208 bytes
- `gazebo/state/state.tlog`: 81,920 bytes
- Remaining Compose project containers: none
- Remaining Compose project networks: none

The first attempt exposed a real callback-ordering defect: no-contact ground
truth for timestamp N is closed by odometry at N+1, but the independent camera
subscription may deliver the N+1 pair first. The adapter now permits exactly
one bounded lookahead pair. The same Compose smoke then passed under the
existing host load without changing simulation timestamps or cadence.

This milestone proves the passive Gazebo/ROS runtime and native evidence path.
It does not claim an ArduPilot-controlled flight; that is the next milestone.
