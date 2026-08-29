# Electromagnet External Interface

The runtime selects one of two paths from the current resolved `run.json`.
`descent_v1` preserves the inactive scenario behavior. `competition_v1` is the
sole request-policy gateway to Gazebo's physical detachable joints.

## `competition_v1` ROS 2 inputs

| Name | Type | QoS |
| --- | --- | --- |
| `/simulation/ground_truth` | `simulation_interfaces/msg/GroundTruth` | reliable, volatile, depth 10 |
| `/simulation/payload_state` | `simulation_interfaces/msg/PayloadState` | reliable, volatile, depth 100 |
| `/simulation/run_state` | `simulation_interfaces/msg/RunState` | reliable, transient-local, depth 1 |
| `/gazebo/private/payload_2/result` | `std_msgs/msg/String` | reliable, volatile, depth 10 |
| `/gazebo/private/payload_3/result` | `std_msgs/msg/String` | reliable, volatile, depth 10 |
| `/gazebo/private/payload_4/result` | `std_msgs/msg/String` | reliable, volatile, depth 10 |

The runtime also reads the immutable current-run
`configuration/{run.json,course.yaml,scenario.yaml}` copies. It ignores state
facts whose `run_id` is not the configured run.

## `competition_v1` ROS 2 outputs

| Name | Type | QoS |
| --- | --- | --- |
| `/simulation/payload_command` | `simulation_interfaces/srv/PayloadCommand` | service |
| `/simulation/payload_events` | `simulation_interfaces/msg/PayloadEvent` | reliable, transient-local, depth 100 |
| `/gazebo/private/payload_2/command` | `std_msgs/msg/String` | reliable, volatile, depth 10 |
| `/gazebo/private/payload_3/command` | `std_msgs/msg/String` | reliable, volatile, depth 10 |
| `/gazebo/private/payload_4/command` | `std_msgs/msg/String` | reliable, volatile, depth 10 |

Attach requests require the current run and known marker, the vehicle and
intended payload in that marker's configured pickup zone, grounded vehicle and
payload truth, free capacity, and horizontal hardpoint-center error at or below
0.075 m. Release requests require that exact marker to be physically attached.
Rejections have a structured code and publish no coordinator command.

A new accepted request publishes exactly one
`payload-command-v1|<command_id>|attach|detach` wire to its marker-specific
coordinator. The service returns `accepted=true` only after the matching
`payload-result-v1` physical confirmation. It then publishes one payload event
whose `state` is `attached` or `detached`. Five wall seconds without a matching
result returns `PHYSICAL_CONFIRMATION_TIMEOUT`; the runtime does not infer a
state or publish a physical event. An identical completed request returns its
original response and sequence without republishing, including when a duplicate
arrives while the original is pending. Reusing a command ID for a different
request returns `COMMAND_ID_CONFLICT`. Physical operations are serialized, so a
second command is validated only after the first reaches a terminal result.

Vehicle and payload samples never regress their per-source timestamps. The
gateway keeps only the latest 0.5 simulated seconds per source and selects the
latest exact timestamp common to current vehicle truth and all three payload
facts. A request fails closed with `STALE_PHYSICAL_STATE` when no such recent
tick exists or when newer grounded or attachment truth conflicts with it. More
than one physically attached payload returns `INVALID_PHYSICAL_STATE`; it is
never treated as free capacity. Recurrent `PayloadState.attached` samples remain
the attachment authority after coordinator confirmations.

Competition readiness is emitted only after current-run vehicle truth, all
three payload states, all three result publishers, and the service exist.

## Preserved `descent_v1` behavior

The runtime consumes authoritative `/clock` with best-effort QoS depth 1 and
publishes exactly one reliable transient-local
`/simulation/scenario_events` inactive event on the first clock sample. The
event has ID 0, `magnet_id=descent-v1-magnet`, and `state=INACTIVE`; it applies
no physical force.

## Ownership and finalization

The module never commands the aircraft, writes a model pose, teleports a
payload, or awards score. Physical mutation belongs to Gazebo. On finalization,
its final structured event precedes
`.status/quiescence/electromagnet.json`; no module output follows that marker.

There is no retry service. A rejected, mismatched, or timed-out request returns
one fail-closed response to the current mission; retry policy remains outside
this module and is deferred for the MVP.
