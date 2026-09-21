# Operator and developer runbook

[Start here](../README.md) · [Architecture](architecture.md) · [Current status](handoff.md)

Run commands from the repository root. This guide separates cheap checks from
image builds and real flights; do not launch a mission just to check installation.

## Prerequisites

- Python 3.12+ and `uv` for the host CLI and tests.
- Docker Engine and the Compose plugin, with permission to use the daemon.
- A Linux/container environment able to run the pinned ROS 2 Jazzy, Gazebo
  Harmonic, and ArduPilot images. Image builds need network access and can be slow.
- Free disk space for images and `runs/` evidence. Check `df -h .` and
  `docker system df`; this guide does not delete old evidence or images.

ROS/Gazebo do not need to be installed on the host to operate the containers.
ROS-specific tests and the independent acceptance inspector have additional
dependencies described below. GPU acceleration is optional.

```bash
uv sync --locked
uv run --locked drone-sim --help
docker version
docker compose version
make test-unit
```

### Comp2026 mission source

`companion/comp2026` is tracked in this monorepo. A normal clone contains the
mission source required by the Phase 3 build. Its history before the monorepo
import remains available through Git.

Confirm the source and repository state before building:

```bash
test -f companion/comp2026/README.md
git rev-parse HEAD
git status --short
```

Commit or explicitly account for source changes before a reproducible build.
The Docker label records monorepo HEAD, not a hash of uncommitted source files.
The [Docker context allowlist](../.dockerignore) includes only the mission's
runtime import closure; adding a new mission module may require updating it.

## Build runtime images

The build **changes local image tags**, not source files. Avoid rebuilding tags
while another run is collecting provenance. Preserve old digests if you intend
to inspect an old run later.

```bash
SIM_COMP2026_REVISION=$(git rev-parse HEAD) \
  docker compose --profile phase3 build
```

`docker compose --profile phase3 build` requires this explicit monorepo HEAD build
argument. Compose leaves it empty when omitted so inactive profiles and
noncompanion configuration still resolve, but the companion build then fails
before package installation or source copies. At launch the CLI checks the
companion image's `org.opencontainers.image.comp2026.revision` label against
monorepo HEAD and fails closed on a mismatch. Check it without launching:

```bash
docker image inspect drone-sim-companion-runtime:phase3 \
  --format '{{ index .Config.Labels "org.opencontainers.image.comp2026.revision" }}'
docker compose --profile phase3 config --services
```

Expect seven services. `start` uses `--no-build`; rebuild affected images after
source/parameter changes. In particular, the SITL parameter overlay is copied
into the ArduPilot image. Editing it on the host does not change an existing image.

## Local developer workflow

After editing `companion/comp2026` or simulator code, run from the repository root:

1. Prepare the host environment and rebuild images with the current source:

   ```bash
   uv sync --locked
   SIM_COMP2026_REVISION=$(git rev-parse HEAD) docker compose --profile phase3 build
   ```

   `start` never builds images. For later edits confined to companion/Comp2026,
   append `companion-runtime` to that build command. Keep source and image tags
   unchanged through launch and acceptance; see [build provenance](#build-runtime-images).

2. Run the configured takeoff/hold/land example:

   ```bash
   uv run --locked drone-sim start --config config/configured-descent-run.json
   ```

   Use `config/default-run.json` for the existing Comp2026 competition path.
   Configured diagnostics exercise the new operation runner, not Comp2026's
   mission classes. To add a fixed sequence, copy the configured template and
   edit `mission_plan.steps` using the [tool contract](../companion/README.md#configured-diagnostic-missions).

3. Copy the UUID from the `run_starting` JSON line and follow [monitoring](#run-and-monitor).
   The final `run_result` reports the terminal state. Results and recordings live
   under `runs/RUN_ID/`; see [evidence paths](#find-evidence-and-debug-a-run).
   Configured operations record their arguments and outcomes in `logs/companion.jsonl`.

The automatic example targets 90 simulated seconds of warmup plus 30 recorded
seconds at one-tenth real time, about 20 wall minutes before startup overhead.
The operator example waits for external arming and GUIDED selection through an
existing connection; it does not provision one. Its 60-second public window must
cover the wait and flight, and targets 25 wall minutes including warmup.

### Run all automatic templates

There is no `run-all` command. This Bash sequence attempts all six automatic
templates, stops on the first failure or abort, and retains each normal run bundle:

```bash
(
  set -euo pipefail
  for mission_config in \
    config/configured-descent-run.json \
    config/vertical-descent-run.json \
    config/hover-roll-run.json \
    config/autotune-roll-run.json \
    config/default-run.json \
    config/realtime-run.json
  do
    uv run --locked drone-sim start --config "$mission_config"
  done
)
```

Run `configured-operator-run.json` separately with an operator present. The batch
includes the existing AutoTune experiment and both competition timing variants;
allow several hours. It checks process exit codes, not independent physical
acceptance. Inspect each bundle and use [competition acceptance](#independent-competition-acceptance)
for competition runs. AutoTune records candidates without promoting parameters.

## Run and monitor

```bash
uv run --locked drone-sim start --config config/default-run.json
```

Keep this foreground process alive. It owns startup, finalization, and teardown.
Use another terminal for the commands below, replacing `RUN_ID` with the UUID in
the start output / generated run directory:

```bash
uv run --locked drone-sim status RUN_ID
uv run --locked drone-sim collect-results RUN_ID
```

`collect-results` is for a terminal run: it validates and reports the manifest
location; it does not copy or download the bundle. With a custom output root,
pass the same **absolute** `--output-root /absolute/path` to status, abort, and
collect-results. The normal root is `runs/` relative to the invoking directory.

To request a safe stop:

```bash
uv run --locked drone-sim abort RUN_ID
```

An abort request is not proof of completed shutdown. Keep `start` alive and poll
status until terminal. Avoid killing containers or deleting the run directory;
recorders need finalization to publish their files. `start` exits 0 for
`COMPLETED`, 1 for `FAILED`, 130 for `ABORTED`; CLI/config errors use exit 2.

### Why it can take so long

The default has a 90-second **native simulation** warmup followed by a
600-second **public simulation** window. Its target real-time factor is 0.25:
even if achieved, these total about 46 minutes of wall time, plus startup and
finalization. Host contention can make it slower. The flight may land home well
before the 600-second recording/scoring window ends. Do not interpret quiet
mission logs alone as a stopped process.

| Template | Purpose | Public duration / warmup / target RTF |
| --- | --- | --- |
| [configured-descent-run.json](../config/configured-descent-run.json) | Fixed automatic takeoff/hold/land | 30 s / 90 s / 0.1 |
| [configured-operator-run.json](../config/configured-operator-run.json) | Operator arms/selects GUIDED, then takeoff/hold/land | 60 s / 90 s / 0.1 |
| [default-run.json](../config/default-run.json) | Full three-payload competition | 600 s / 90 s / 0.25 |
| [vertical-descent-run.json](../config/vertical-descent-run.json) | Controlled descent, not the payload mission | 60 s / 90 s / 0.1 |
| [hover-roll-run.json](../config/hover-roll-run.json) | Short roll/hover diagnostic | 45 s / 15 s / 0.1 |
| [autotune-roll-run.json](../config/autotune-roll-run.json) | Roll AutoTune experiment | 120 s / 15 s / 0.1 |
| [realtime-run.json](../config/realtime-run.json) | Competition with a higher speed target, not a speed guarantee | 600 s / 90 s / 1.0 |

Short diagnostic missions do not prove competition success. AutoTune promotion
is a separate, explicit source change using
[promote_roll_autotune.py](../scripts/promote_roll_autotune.py); it is not part of
normal launch. Inspect the saved gains and resulting parameter diff before reuse.

### Optional NVIDIA path

After provisioning and checking the host NVIDIA container runtime, EGL, NVENC,
and the device path required by [compose.gpu.yaml](../compose.gpu.yaml):

```bash
SIM_COMPOSE_OVERLAY=gpu uv run --locked drone-sim start --config config/default-run.json
```

The overlay requests a GPU for rendering and `h264_nvenc` encoding. It currently
hardcodes `/dev/nvidia-caps/nvidia-cap2`; do not assume that path exists on a new
host. The default path uses CPU video encoding. No host provisioning is performed
by the CLI.

The GPU overlay also joins the other Phase 3 services to Gazebo's network and
IPC namespaces for DDS shared-memory communication. Inspect the resolved overlay,
not just base Compose, when diagnosing discovery or container networking:

```bash
docker compose -f compose.yaml -f compose.gpu.yaml --profile phase3 config --quiet
```

## Find evidence and debug a run

Under `runs/<run_id>/`:

| Path | Use |
| --- | --- |
| `manifest.json` | Terminal status, failure reason, score summary, source/image provenance, and artifact validity |
| `configuration/run.json` | Resolved configuration for this run, not today's defaults |
| `video/onboard.mp4`, `video/observer.mp4` | Normal finalized recordings |
| `rosbag/` | Public state/events and camera metadata, not image pixels |
| `scoring/result.json`, `scoring/events.jsonl` | Awarded points and their evidence references |
| `logs/<module>.jsonl`, `logs/docker/` | Structured events and captured child-process output |
| `.status/`, `.control/` | Live lifecycle diagnostics; read them, do not edit them |
| `gazebo/` | Native state and server diagnostics |

Read the manifest reason first, then the failing module's log. While running,
there may not be a manifest yet; use status and available logs. Normal videos
may not have final names until recorders close. The `review_video/` files linked
from the payload issue note were an independent recording for that particular
run, **not** an artifact guaranteed by ordinary launches.

Three distinct questions must be answered: did the vehicle physically perform
the mission, what points were awarded, and did the full evidence bundle validate?
Use [the latest documented failure](payload-timestamp-fix.md) as an example of
150 points coexisting with a failed run.

Precision-landing recovery records one compact JSON object per rejected
observation and state transition. Inspect it without changing the bundle:

```bash
RUN_ID=replace-with-run-uuid
rg -n 'PRECISION_LANDING ' "runs/$RUN_ID/logs"
```

The records distinguish tracking, GUIDED hold, reacquisition, the single search
hover retry, touchdown, and deadline failure. During a GUIDED interval there
must be no accepted landing-target output.

Inspect the image-baked values recorded by the flight controller rather than
assuming the host parameter file reached the container:

```bash
BIN="runs/$RUN_ID/ardupilot_sitl/logs/00000001.BIN"
uv run --locked mavlogdump.py --types PARM --format csv "$BIN" \
  | rg 'LAND_SPD_MS|PLND_|ATC_ANG_RLL_P|ATC_RAT_RLL_|ATC_RAT_PIT_|ATC_ACC_R_MAX'
uv run --locked mavlogdump.py --types ATT,PL --format csv "$BIN" \
  > "/tmp/$RUN_ID-att-pl.csv"
```

Compare the parameter rows with
[descent.parm](../ardupilot_sitl/params/descent.parm). Pair each `ATT` sample to
its nearest `PL` sample and select `TAcq=1`; the guarded-run acceptance limits
are 5 degrees for desired roll/pitch magnitude and 8 degrees for actual
roll/pitch magnitude. Keep the CSV as analysis scratch, not as a replacement
for the original DataFlash log.

Validate the two finalized videos and bag independently when diagnosing an
otherwise passing score:

```bash
ffprobe -v error -show_streams -of json "runs/$RUN_ID/video/onboard.mp4"
ffprobe -v error -show_streams -of json "runs/$RUN_ID/video/observer.mp4"
ros2 bag info "runs/$RUN_ID/rosbag"
```

### Independent competition acceptance

```bash
make inspect-competition RUN_DIRECTORY=/absolute/path/to/runs/RUN_ID
```

This is read-only and requires the Python environment to provide ROS Jazzy
`rosbag2_py`, plus FFmpeg/FFprobe and access to the seven Docker image tags.
`uv sync` alone does not install those system dependencies. See
[acceptance implementation](../artifacts/src/artifacts/acceptance.py) and
[inspector](../scripts/inspect_competition_run.py).

It requires 150/150, valid evidence, and matching **current** parent/nested
revisions, dirty flags, and image digests. Running it after changing source or
retagging images can reject an old bundle on provenance alone. Preserve its
original checkout/images when establishing a baseline; never edit historical
evidence to match today's checkout. `collect-results` is not this full semantic
competition acceptance check.

## Test without a full flight

```bash
# Existing quick subset: orchestration, artifacts, and ROS schema contracts.
make test-unit

# Broader host suite across all seven modules; ROS-only cases may skip.
uv run --locked pytest orchestration/tests artifacts/tests companion/tests \
  ardupilot_sitl/tests gazebo/tests electromagnet/tests scorekeeper/tests tests/contracts -q
```

Neither command establishes real flight behavior. For infrastructure integration
(these commands build/run Docker containers and are slower):

```bash
docker compose --profile foundation build foundation
make test-foundation
make test-phase2
```

`make test` combines the host subset, foundation tests, and Phase 2 tests; it
does not first build the foundation image. There is no `make test-phase3`
production-flight target. A real competition check is the build → start →
physical evidence review → independent inspection workflow above.

To run the focused payload queue regression with ROS available in the existing
Gazebo image, exercising current source mounted read-only:

```bash
docker run --rm -v "$PWD:/workspace:ro" -w /workspace \
  drone-sim-gazebo-runtime:phase3 bash -lc \
  'source /opt/ros/jazzy/setup.bash && PYTHONPATH=/workspace/gazebo/src:$PYTHONPATH python3 -m pytest -p no:cacheprovider -o addopts= gazebo/tests/test_adapter_node.py gazebo/tests/test_payload_adapter.py -q'
```

This is a ROS adapter test, not a claim that the image contains all latest host
changes. Consult [current status](handoff.md) before launching a long rerun.

## Audit or refresh imported Gazebo assets

The [asset manifest](../gazebo/provenance/ardupilot_gazebo-assets.json) records
the immutable upstream revision, source/import paths, SHA-256 hashes, and license
for each imported copy. Preserve the [license](../gazebo/provenance/LICENSE.ardupilot_gazebo.md)
and [upstream snapshots](../gazebo/provenance/upstream). Compare each manifest
record's imported bytes with its digest; copied meshes have separate records.
Derived model SDF/config files are not represented as original upstream bytes.

To refresh, review a new immutable upstream commit, fetch and verify its files
outside this repository, then update every affected copy and manifest entry
together. Never import from a dirty/generated or neighboring worktree. This
manifest covers selected Iris model assets, not compiled plugins, worlds,
payloads, gimbals, or the camera pipeline. Keep licensing/provenance intact and
run the asset tests listed in the [Gazebo guide](../gazebo/README.md).
