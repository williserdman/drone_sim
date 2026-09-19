# Artifacts

[Project README](../README.md) · [Architecture](../docs/architecture.md) ·
[Runbook](../docs/runbook.md)

Artifacts owns the evidence bundle for each run: ROS 2 recording, onboard and
observer video encoding, structured and Docker logs, Gazebo native evidence,
score outputs, validation, checksums, and atomic manifest publication. Its runtime
also reports aggregate recorder readiness and recorder-local finalization facts.
The shared durable protocol accepts `mission-execution-ready` with exact fields
`run_id`, `ready: true`, and a nonnegative integer `sim_timestamp_ns`.

It does **not** produce camera images or physical truth, fly the mission, calculate
the competition score, or choose a mission outcome. Orchestration requests a
terminal outcome; artifact validation may reject a requested completion when
required evidence is missing or invalid.

## Entry points and code map

- [`runtime_node.py`](src/artifacts/runtime_node.py) is the container entry point
  selected by [`compose.yaml`](../compose.yaml). It starts recorders, publishes
  readiness, watches health, and closes and validates recorder outputs.
- [`RosbagRecorder`](src/artifacts/_adapters/rosbag.py) defines the exact bag topic
  inventories and controls rosbag2. [`VideoStreamRecorder`](src/artifacts/_adapters/video.py)
  owns each FFmpeg process and validates finalized H.264 output.
- [`ArtifactSession`](src/artifacts/session.py) performs host-side bundle validation
  and returns the committed terminal result. [`manifest.py`](src/artifacts/manifest.py)
  owns the required inventory, manifest model, validation, and atomic publication.
- [`runtime_configuration.py`](src/artifacts/runtime_configuration.py) derives the
  recorder contract from the resolved run configuration.
- [`runtime_status.py`](src/artifacts/runtime_status.py) owns the typed runtime
  status schema, canonical JSON conversion, and write policy.
  [`RuntimeProtocol`](src/artifacts/runtime_protocol.py) is the container-side
  adapter over the strict, descriptor-safe persistence mechanics in
  [`protocol_files.py`](src/artifacts/protocol_files.py).
- [`acceptance.py`](src/artifacts/acceptance.py),
  [`competition_score_validation.py`](src/artifacts/competition_score_validation.py),
  and [`search_delivery_score_validation.py`](src/artifacts/search_delivery_score_validation.py)
  implement independent semantic acceptance for completed physical runs.

## Consumer and producer seams

The runtime consumes lifecycle state, camera pixels and frame metadata, public
simulation truth and events, module stdout, Gazebo state/logs, and scorekeeper
files. Wire shapes live in the actual
[`simulation_interfaces` definitions](../ros_ws/src/simulation_interfaces/msg),
while exact bag topics and types live in
[`_adapters/rosbag.py`](src/artifacts/_adapters/rosbag.py). Do not copy either
inventory into prose.

The bag is deliberately metadata-only for cameras: it stores frame IDs and
timestamps, not raw image pixels. Pixel recordings are `video/onboard.mp4` and
`video/observer.mp4`; losing an MP4 cannot be repaired from the bag. Competition
runs extend the base bag with their physical/scoring evidence as selected by
[`RecordingRuntimeConfig.topics`](src/artifacts/runtime_configuration.py).
`search_delivery_v1` uses that topic inventory with exactly one payload-state
sample for ID 3 on every physical tick and no descent scenario event. Artifact
replay independently checks its seven mission events and two payload events.
Replay allows the first physical sample within one 50 ms interval after mission
start, matching the public grid's first sample at 50 ms when SEARCH starts at
zero. Every later sample must remain exactly 50 ms apart; a missing first tick
or an interior gap still fails validation.

The search-delivery onboard stream is 640x480 and its observer stream is
1280x960. Resolved recording configuration carries the observer dimensions
separately; older scenarios default them to their onboard dimensions.

The deployed private rosbag QoS overrides are
[`recording-qos.yaml`](recording-qos.yaml), copied into the runtime image by the
[`Dockerfile`](Dockerfile). Treat that file and the publisher/subscriber code as
authority rather than maintaining a duplicate QoS table here.

Outputs live under `runs/<run_id>/`: two MP4s, `rosbag/`, configuration, Gazebo
evidence, per-module logs, scoring files, diagnostics, and `manifest.json`. The
required inventory is defined by
[`REQUIRED_ARTIFACT_PATHS`](src/artifacts/manifest.py), and recorder-local semantic
records are assembled in [`runtime_node.py`](src/artifacts/runtime_node.py).
Manifest paths use portable POSIX-relative syntax. The executable contract is
[`is_manifest_relative_path`](src/artifacts/manifest.py), with its JSON form in
the [`manifest.json` schema](schemas/manifest.schema.json).

## Constraints worth preserving

- Recorder readiness precedes public simulation output. A readiness message is
  not a completeness claim.
- Finalization begins only after orchestration's aggregate `runtime-frozen` fact.
  Videos and rosbag must stop, drain, validate, and become stable before
  `artifacts-final` is written and host-side bundle validation starts.
- An unconfirmed live rosbag fails closed: artifacts must not claim completeness
  or hash a mutable named bag. Partial and diagnostic outputs are preserved.
- The bag ends with `FINALIZING`. After host-side validation atomically publishes
  `manifest.json`, its terminal status and reason are authoritative; do not depend
  on a later live ROS notification or append terminal evidence to the frozen bag.
- Completed runs require every required artifact; failed and aborted runs retain
  explicit missing/invalid records. Finalization is no-clobber and scoped to the
  current run directory.
- The shared protocol helper operates within the cooperative trust boundary
  defined in the [architecture guide](../docs/architecture.md#shared-communication-guarantees).
  Its advisory lock is not a security boundary against a hostile process with
  the same filesystem permissions.
- Runtime components publish typed values from the shared
  [`runtime_status.py`](src/artifacts/runtime_status.py) contract. Do not add a
  module-local JSON writer or duplicate its field validation here.
- CPU and optional GPU encoding must satisfy the same video contract. Deployment
  and GPU setup belong in the [runbook](../docs/runbook.md#optional-nvidia-path).

## Focused tests

Run from the project root:

```bash
uv run pytest artifacts/tests/test_protocol_files.py \
  artifacts/tests/test_runtime_protocol.py artifacts/tests/test_runtime_configuration.py -q
uv run pytest artifacts/tests/test_runtime_node.py artifacts/tests/test_rosbag_adapter.py -q
uv run pytest artifacts/tests/test_manifest.py artifacts/tests/test_session.py \
  artifacts/tests/test_acceptance.py -q
uv run pytest artifacts/tests/test_search_delivery_score_validation.py -q
```
