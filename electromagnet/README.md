# Electromagnet module

[Project README](../README.md) · [Architecture](../docs/architecture.md) ·
[Runbook](../docs/runbook.md)

This module owns payload-request policy and the handoff to Gazebo for physical
attach/detach operations. In `competition_v1` it validates a request against
current physical facts, sends one coordinator command, waits for confirmation,
and publishes a truthful payload event. In `descent_v1` it only publishes the
deterministic inactive scenario event.

It does **not** fly the aircraft, mutate a pose or joint itself, calculate score,
or decide whether a whole run succeeded. Mission intent belongs to the companion,
physical mutation and recurrent truth to Gazebo, scoring to the scorekeeper, and
terminal status to orchestration and the validated manifest.

## Code map

- [runtime_node.py](src/drone_sim_electromagnet/runtime_node.py) is the process
  entry point and selects the resolved `descent_v1` or `competition_v1` path.
- [payload.py](src/drone_sim_electromagnet/payload.py) is the ROS-free policy:
  request validation, capacity rules, pickup eligibility, and command-ID ledger.
- [controller.py](src/drone_sim_electromagnet/controller.py) joins physical facts,
  serializes operations, waits for coordinator results, caches responses, and
  publishes confirmed events.
- [scenario.py](src/drone_sim_electromagnet/scenario.py) contains the preserved
  inactive descent policy.
- The installed command is defined in [pyproject.toml](pyproject.toml); Compose
  starts it in the [`electromagnet-runtime` service](../compose.yaml).
- Runtime readiness, failure, and quiescence use the shared
  [typed status contract](../artifacts/src/artifacts/runtime_status.py) and
  [container protocol adapter](../artifacts/src/artifacts/runtime_protocol.py).

## Interfaces and configuration

The competition path consumes [GroundTruth](../ros_ws/src/simulation_interfaces/msg/GroundTruth.msg),
[PayloadState](../ros_ws/src/simulation_interfaces/msg/PayloadState.msg), and
[RunState](../ros_ws/src/simulation_interfaces/msg/RunState.msg). It provides the
[PayloadCommand service](../ros_ws/src/simulation_interfaces/srv/PayloadCommand.srv)
and publishes [PayloadEvent](../ros_ws/src/simulation_interfaces/msg/PayloadEvent.msg).
The descent path publishes [ScenarioEvent](../ros_ws/src/simulation_interfaces/msg/ScenarioEvent.msg).
Topic names and QoS are authoritative in
[runtime_node.py](src/drone_sim_electromagnet/runtime_node.py).

Private coordinator wires are implemented at both ends in
[controller.py](src/drone_sim_electromagnet/controller.py) and Gazebo's
[PayloadCommandCoordinator.cc](../gazebo/plugin/PayloadCommandCoordinator.cc).
Competition inventory, pickup zones, capacity, and tolerances come from the
resolved run copies of [course.yaml](../config/course.yaml) and
[scenario.yaml](../config/scenario.yaml); loading and validation live in
[RuntimeConfig](src/drone_sim_electromagnet/runtime_node.py).

## Constraints worth preserving

- `accepted=true` means Gazebo returned the matching physical confirmation; a
  requested or published coordinator command is not success. Timeout, mismatch,
  stale/inconsistent facts, and coordinator errors fail closed without an event.
- A completed duplicate command ID replays its original response and sequence
  without another wire or event. Reusing the ID for different content conflicts.
  Operations are serialized through validation and confirmation.
- The gateway authorizes from the newest recent timestamp shared by vehicle and
  all three payload streams. It rejects regressing, stale, or multiply-attached
  truth rather than inventing a coherent state.
- Confirmation updates local state only for the immediate response. Subsequent
  monotonic `PayloadState.attached` samples remain the attachment authority.
- On finalization, the last structured event precedes the module's quiescence
  marker, after which the module must remain silent.
- Runtime failures use the shared first-wins policy. This module does not replace
  an earlier durable failure from another producer.

## Focused checks

Run from the repository root; these do not launch a mission:

```bash
uv run pytest electromagnet/tests -v
uv run pytest tests/contracts -v
```

When changing the Gazebo wire boundary too, also run the focused coordinator and
adapter tests under `gazebo/tests` described in the [runbook](../docs/runbook.md).
