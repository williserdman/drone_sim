# Electromagnet Internal Interface

`payload.py` is the ROS-free policy seam. `PayloadAuthority.decide()` consumes
an immutable `PayloadWorld` and `PayloadRequest`, and returns an immutable
`PayloadDecision`. It owns only request validation and the command-ID ledger.
`complete()` binds a physically confirmed or explicitly failed terminal result;
completed identical requests replay with no wire, while conflicting ID reuse
cannot reach Gazebo.

`PayloadGateway` in `controller.py` joins the latest current-run vehicle and
three payload facts, publishes a marker-specific coordinator command, and waits
on a per-command wall-time event. Result callbacks populate those events from a
different executor thread. Only an exact marker, command ID, status, and desired
physical state can produce an accepted response. The gateway updates its
confirmed one-slot attachment state and publishes one immutable
`PayloadEventRecord` only after that confirmation. Validation rejection,
coordinator error, mismatch, and timeout never change attachment state.

`runtime_node.py` owns ROS translation and resolved-config loading. The
competition boundary uses a `MultiThreadedExecutor` and one
`ReentrantCallbackGroup`, allowing result subscriptions to run while a service
callback waits. The exact private topic aliases are `payload_2`, `payload_3`,
and `payload_4`; the public ArUco IDs remain integers 2, 3, and 4.

The preserved `ScenarioPolicy`/`ScenarioController.observe_clock()` seam remains
the deterministic inactive `descent_v1` implementation. `ScenarioController`
also owns structured readiness/failure/finalization evidence and the shared
quiescence boundary for both runtime paths.
