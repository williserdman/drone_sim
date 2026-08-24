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

The first invalid processing fact closes normal progress before any accepted
state can be repaired. Stale canonical run IDs are ignored as required by the
repository lifecycle contract. Once quiescence is returned, only identical
server-stop/finalization observations and terminal lifecycle repeats are
idempotent; all other current-run use raises `RuntimeModelError`.

The model validates that completion timestamps describe the fixed native
cadence but never samples time or invents an output timestamp. Infrastructure
timeout is an explicit caller fact. The process boundary alone consumes the
deadline through an injected monotonic clock.
