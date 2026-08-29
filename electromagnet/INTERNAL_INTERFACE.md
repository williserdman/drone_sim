# Electromagnet Internal Interface

`payload.py` is the ROS-free policy seam. `PayloadAuthority.decide()` consumes
an immutable `PayloadWorld` and `PayloadRequest`, and returns an immutable
`PayloadDecision`. It owns only request validation and the command-ID ledger.
`complete()` binds a physically confirmed or explicitly failed terminal result;
completed identical requests replay with no wire, while conflicting ID reuse
cannot reach Gazebo.

`PayloadGateway` in `controller.py` retains at most the latest 0.5 simulated
seconds of monotonic current-run vehicle and three-payload facts. It joins the
latest exact common timestamp, publishes a marker-specific coordinator command,
and waits on a per-command wall-time event. A missing or expired common tick and
any newer grounded or attachment-state transition fail closed. Result callbacks
populate those events from a different executor thread. A dedicated operation
mutex spans validation through response caching; it is independent of the
fact/result mutex. Concurrent exact duplicates therefore wait for and replay
the original response, conflicts cannot replace that response, and distinct
physical operations revalidate in order without blocking result callbacks.

Only an exact marker, command ID, status, and desired physical state can produce
an accepted response. A confirmation updates the requested payload's local fact
for the immediate response, then later monotonic recurrent `PayloadState` facts
remain authoritative. Authorization uses the latest recent exact common
timestamp across the vehicle and all payloads and rejects multiple attached
facts explicitly. The gateway publishes one immutable `PayloadEventRecord` only
after confirmation.
Validation rejection, coordinator error, mismatch, and timeout publish no
physical event.

`runtime_node.py` owns ROS translation and resolved-config loading. The
competition boundary uses a `MultiThreadedExecutor` and one
`ReentrantCallbackGroup`, allowing result subscriptions to run while a service
callback waits. The exact private topic aliases are `payload_2`, `payload_3`,
and `payload_4`; the public ArUco IDs remain integers 2, 3, and 4.

The preserved `ScenarioPolicy`/`ScenarioController.observe_clock()` seam remains
the deterministic inactive `descent_v1` implementation. `ScenarioController`
also owns structured readiness/failure/finalization evidence and the shared
quiescence boundary for both runtime paths.

All physical freshness comparisons use accepted simulation timestamps already
present on vehicle and payload facts. The gateway adds no epoch, timestamp
mapping, activation barrier, or cross-service synchronization queue.
