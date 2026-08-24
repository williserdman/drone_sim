# Gazebo Internal Interface

All Phase 3 Python code lives under one `drone_sim_gazebo` import root. Its
immediate sibling packages have these exclusive boundaries:

| Package | Owned API boundary |
| --- | --- |
| `worlds` | Resolve and validate immutable local world resources and checksums. It performs no server control. |
| `models` | Resolve and validate immutable model resources, identities, provenance, and checksums. It performs no physics or ROS publication. |
| `server` | Construct and supervise the paused `gz sim` subprocess, private Gazebo Transport discovery/control, native recording, and raw server log. |
| `ros_adapter` | Validate private bridged native samples and publish the exact public ROS 2 camera, metadata, clock, and ground-truth contracts. |
| `runtime` | Own run lifecycle, readiness, completion, finalization, failure facts, and supervision of the other runtime units. |

These APIs are sibling seams, not permission to import another package's
private implementation files. Gazebo Transport names remain private to
`server` and `ros_adapter`; no other repository module discovers or consumes
them. Only `runtime` writes Gazebo lifecycle/quiescence facts, and neither it
nor its siblings writes the aggregate runtime freeze or terminal manifest.

No Phase 3 internal API exposes actuator exchange, ArduPilot lockstep,
electromagnet force mutation, or an in-process world reset.

`runtime.runtime_node` is the sole live composition root. It applies every
side effect returned by `RuntimeModel`, uses `RuntimeProtocol` for existing
durable lifecycle facts, and uses the Gazebo-owned writer only for
`gazebo-ready`. `runtime.children` supervises two new process groups: a
one-way parameter bridge for clock, odometry, and contact, plus one
`ros_gz_image` bridge for the fixed camera pair. Both quiesce before the
server.

`ros_adapter.aggregation` retains at most one odometry, one contact, and one
completed truth value. `ros_adapter.live` joins that truth to the current
camera pair before the public ROS node publishes it. Private sensor QoS may be
best effort; public QoS remains exactly the external table.
