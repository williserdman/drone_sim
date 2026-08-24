# Gazebo Runtime Internal Interface

`RuntimeModel.accept(event) -> tuple[RuntimeAction, ...]` is the sole
transition seam. Events and actions are frozen dataclasses with canonical UUID,
bool-versus-int, finite deadline, bounded text/path, exact terminal-state, and
typed-summary validation. Returned collections are immutable tuples. The
model stores only readiness bits, lifecycle/finalization latches, the one
accepted completion summary, and the one native artifact summary; it has no
callback or sample queue.

Action order carries part of the interface. Readiness publication precedes the
controlled step; unpause follows that step and `RUNNING`; pause precedes
source-finished and normal finalization; a first failure is written before
best-effort pause and `BeginFinalization`; and `WriteQuiescence` follows a
completed `StopServer` observation. A bare `FINALIZING` state pauses and closes
the normal path immediately, while the typed durable finalization request
supplies terminal intent, reason, and the absolute stop deadline.

A `COMPLETED` request is itself invalid until `_source_summary` contains the
exact accepted `AdapterSummary`; the failure write and failed begin action are
emitted before the one stop action. `FAILED` and `ABORTED` requests do not
depend on source completion. Child-exit and server-stop-failure events are
first-class typed facts, so Task 6 does not bypass the first-failure latch when
a server, bridge, image bridge, adapter, or native validation fails.

Every current-run `ServerStopped` summary is reconstructed through the native
summary validator before acceptance. Malformed, internally inconsistent,
wrong-run, or premature native evidence irreversibly latches a distinct native
stop failure and forbids later quiescence, even when another runtime failure was
already first. That native latch does not erase or replace the first diagnostic.
Conversely, an unrelated runtime failure followed by the first valid native
summary may still produce quiescence for its failed finalization.

The first invalid processing fact closes normal progress before any accepted
state can be repaired. Stale canonical run IDs are ignored as required by the
repository lifecycle contract. Once quiescence is returned, only identical
server-stop/finalization observations and terminal lifecycle repeats are
idempotent; all other current-run use raises `RuntimeModelError`.

The model validates that completion timestamps describe the fixed native
cadence but never samples time or invents an output timestamp. Infrastructure
timeout is an explicit caller fact. The process boundary alone consumes the
deadline through an injected monotonic clock. The current durable finalization
protocol does not store that deadline; Task 6/7 must hand one non-restarting
absolute deadline from durable intent to `StopServer` rather than create a new
budget in either layer.
