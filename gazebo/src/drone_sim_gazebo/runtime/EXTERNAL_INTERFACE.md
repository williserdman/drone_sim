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

Unexpected child exit, endpoint timeout, malformed progress, premature state,
or unsupported current-run input latches one `WriteRuntimeFailure`. Normal
progress can never repair that first failure. `FINALIZING` preempts progress
and pauses immediately; `FinalizationRequested` carries the single positive
finite absolute monotonic deadline and returns the remaining ordered
`BeginFinalization` and `StopServer` actions. A valid current-run
`NativeArtifactSummary` after stop yields one `WriteQuiescence` and freezes the
model. Later output-producing inputs are rejected; duplicate terminal
observations are silent.
