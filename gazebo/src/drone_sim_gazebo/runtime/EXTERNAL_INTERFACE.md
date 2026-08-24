# Gazebo Runtime External Interface

The `runtime` package exports a pure `RuntimeModel` and frozen typed facts and
actions. It has no ROS, Gazebo binding, subprocess, durable-file, or wall-clock
dependency. The later runtime entry point maps live observations into these
facts and applies the returned actions; it may not bypass their order.

Current-run `ArtifactsReady` and private `GazeboReady` are both required before
one `PublishGazeboReady`. One current-run `READY` then returns exactly one
`RequestSteps(1)`, and the following current-run `RUNNING` returns one
`SetPaused(False)`. Duplicates and canonical stale-run facts return no actions.

`AdapterCompleted` carries the frozen Task 4 `AdapterSummary`. Success requires
exactly the configured, aligned onboard, observer, paired, and ground-truth
counts and the fixed native 50 ms cadence. The model pauses before returning
`WriteSourceFinished` with the summary's unchanged last native timestamp.

`ChildExited` carries a bounded canonical identity for server, bridge, image
bridge, or adapter children, and its exact identity is retained in the first
failure reason. `ServerStopFailed` is the typed native-stop/validation failure
fact and carries named diagnostic paths. These facts, endpoint timeout,
malformed progress, premature state, or unsupported current-run input latch
one `WriteRuntimeFailure`; normal progress can never repair it.

`FINALIZING` preempts progress and pauses immediately.
`FinalizationRequested` carries the single positive finite absolute monotonic
deadline and returns the remaining ordered `BeginFinalization` and
`StopServer` actions. `COMPLETED` is accepted only after the exact frozen
adapter summary has produced `WriteSourceFinished`; otherwise it fails and can
emit only `BeginFinalization("FAILED", ...)`. `FAILED` and `ABORTED` remain
explicit preemptions. A valid current-run `NativeArtifactSummary` after a
successful stop yields one `WriteQuiescence` and freezes the model. A typed
stop failure, or any malformed, inconsistent, wrong-run, or premature stopped
summary, forbids later quiescence or repair while preserving the first runtime
diagnostic. An unrelated earlier runtime failure alone does not prevent a first
valid stopped summary from recording failed-run quiescence. Later
output-producing inputs are rejected; duplicate terminal observations are
silent.
