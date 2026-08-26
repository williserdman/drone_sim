# Gazebo Internal Interface

All Phase 3 Python code lives under one `drone_sim_gazebo` import root. Its
immediate sibling packages have these exclusive boundaries:

| Package | Owned API boundary |
| --- | --- |
| `worlds` | Resolve and validate immutable local world resources and checksums. It performs no server control. |
| `models` | Resolve and validate immutable model resources, identities, provenance, and checksums. It performs no physics or ROS publication. |
| `server` | Construct and supervise the paused `gz sim` subprocess, native recording, and raw server log. |
| `ros_adapter` | Validate private bridged native samples and publish the exact public ROS 2 camera, metadata, clock, and ground-truth contracts. |
| `runtime` | Own run lifecycle, private Gazebo Transport discovery/control, flight exchange observation, completion, finalization, failure facts, and supervision of the other runtime units. |

These APIs are sibling seams, not permission to import another package's
private implementation files. Gazebo Transport names remain private to this
package: `runtime` owns world control and the flight-only ArduPilot status
service, while `ros_adapter` owns bridged native samples. No other repository
module discovers or consumes them. Only `runtime` writes Gazebo lifecycle/quiescence facts, and neither it
nor its siblings writes the aggregate runtime freeze or terminal manifest.

For `vertical_descent/iris_flight`, the downstream-patched plugin owns the
private UDP sensor/actuator lockstep seam and
`/model/iris/ardupilot/status`. Runtime readiness requires servo, motor-update,
and JSON-send counters to prove one bounded paused round trip with no frame gaps
or send errors before it freezes the stable observed counts into
`gazebo-ready`.
The downstream plugin patch removes upstream receive-queue draining: one
`ReceiveServoPacket` call owns one datagram, while the unchanged `PreUpdate`
loop handles duplicates and terminates on the first sequential frame. A
contiguous duplicate burst sends at most one recovery JSON; an empty receive or
accepted sequential frame re-arms recovery. This is the one-for-one boundary
between motor updates and controlled physics steps without duplicate feedback.
Orchestration owns aggregate peer readiness. No internal API exposes
electromagnet force mutation or an in-process world reset.

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
best effort; public QoS remains exactly the external table. The pure
`OutputEpochGate` receives the exact configured native target (Phase 3 default
`90.0` seconds) and arms at `RUNNING` before it. Late activation and a reliable
clock that skips the target fault closed. Native samples at or before the target
are dropped; a bounded pre-zero queue holds at most two public epochs (six
validated frame/truth outputs), faults on overflow, and drains only after
public `/clock=0` is published. The first target+50 ms camera/truth sample
maps to public 50 ms. The target is the single mapping origin for clock,
cameras, odometry, and contact, and the configured frame count caps public
clock output at the final frame timestamp.
`runtime.rendezvous.PublicEpochRendezvous` sequences public activation, a
confirmed paused-world observation, absolute run-to-target control, and the
durable initial-command delivery fact. Only after the current-run fact matches
`SET_GUIDED` at public zero does it release the world for the public epoch.
