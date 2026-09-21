# Operator and developer runbook

[Start here](../README.md) · [Architecture](architecture.md) · [Current status](handoff.md)

Run commands from the repository root. This guide separates cheap checks from
image builds and real flights; do not launch a mission just to check installation.

## Guarded `comp2026_auto` status

The source now has a guarded parent QGC host for FM1 and FM2. It validates the
complete immutable QGC input bundle and external attempt state before opening
ROS or DroneKit, passes sealed artifact bytes to the nested listener, and sends
no automatic command. QGC admission remains closed until the matching run is
`RUNNING`, public simulation time is accepted, ROS telemetry production is
alive, and the actual connected vehicle has a fresh heartbeat and literal
armable state. FM3 is disabled.

Repository default configurations do not provide the required QGC input bundle,
so there is no copy-paste `comp2026_auto` launch command in this runbook. Do not
invent site, calibration, RC, session, action, policy, or ledger values. Parent
build source and provenance select official ArduCopter 4.5.7 commit
`2a3dc4b7bf2507120f7378a7b2fde73185e0c325`. Matching Phase 3 images were rebuilt
for the configured diagnostic on 2026-09-19; QGC integration remains unverified.
See [handoff](handoff.md#2026-09-19-configured-mission-runner) for that separate
diagnostic's evidence. Check image provenance before trusting mutable local tags.
`FS_THR_ENABLE=0` and `FLTMODE_CH=0` remain legacy diagnostic settings, not a
QGC/aircraft failsafe or takeover profile.
Preserve previous run artifacts as historical evidence, not proof that current
source is runnable or flight-ready. The controlled-descent, AutoTune, and hover
operator workflows below are unchanged.
The configured competition plan below is a separate mission runner and does not
change these `comp2026_auto` admission requirements.

## Prerequisites

- Python 3.12+ and `uv` for the host CLI and tests.
- Docker Engine and the Compose plugin, with permission to use the daemon.
- A Linux/container environment able to run the pinned ROS 2 Jazzy, Gazebo
  Harmonic, and ArduPilot images. Image builds need network access and can be slow.
- Comp2026 source at `companion/comp2026` for the Phase 3 build/run path.
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

### Locate the Comp2026 source

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
It excludes user-generated action files, waypoint stores, sessions, ledgers,
deployment profiles, backups, hardware experiments, and secrets. Source
admission and parent composition do not authorize aircraft operation.

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

Before a companion build, check that Docker can assemble every explicitly
admitted source used by repository Dockerfiles:

```bash
docker build --file tests/docker-context/Dockerfile .
```

An import-only check can confirm that the QGC listener package is present without
opening MAVLink or constructing GPIO, I2C, camera, or payload devices:

```bash
PYTHONPATH=companion/comp2026/src uv run --locked python -c \
  'import drone.control.listener; print(drone.control.listener.start_repl.__name__)'
```

These checks prove packaging and inert import behavior only. They do not replace
a uniquely tagged matching image build, a QGC-to-SITL test, a scored simulation,
or aircraft acceptance.

## Local developer workflow

Run these steps from the repository root after changing code. First check the
[coverage map](../companion/README.md#which-code-does-a-local-mission-exercise):
the configured competition and search missions reuse Comp2026 camera/range
code, but their flight operations and mission sequence are implemented in the
parent package. A successful configured flight does not validate edits to
Comp2026's separate mission/controller implementation.

1. Prepare the Python environment and inspect source changes:

   ```bash
   uv sync --locked
   git status --short
   ```

   For a shared regression baseline, commit the relevant edits in their owning
   repository before building. Uncommitted allowlisted edits are copied into
   the image too, but a revision plus a dirty flag does not identify their exact
   contents. Keep the source unchanged through launch and acceptance.

2. Build the [seven runtime images](#build-runtime-images) on a fresh checkout
   or after changes spanning modules. If only Comp2026 or the parent companion
   code changed and the other images already match the checkout, rebuild just
   the companion:

   ```bash
   SIM_COMP2026_REVISION=$(git rev-parse HEAD) \
     docker compose --profile phase3 build companion-runtime
   ```

   `start` never rebuilds images. Docker copies only the runtime files admitted
   by [`.dockerignore`](../.dockerignore); a newly imported module must be
   included there. Changes to dependencies also need the corresponding
   [Dockerfile](../companion/Dockerfile) update. Even a commit-only change can
   require a companion rebuild when its recorded source revision changes.

3. Run one mission in the foreground:

   ```bash
   uv run --locked drone-sim start --config config/configured-search-delivery-run.json
   ```

   Substitute `config/configured-competition-run.json` for the three-payload
   competition or `config/configured-descent-run.json` for the takeoff/hold/land
   smoke mission. The [batch recipe below](#run-all-automatic-mission-templates)
   runs every automatic template on this branch.

4. Copy the UUID from the `run_starting` JSON line. In another terminal:

   ```bash
   uv run --locked drone-sim status RUN_ID
   ```

   To stop, use `uv run --locked drone-sim abort RUN_ID` and leave the original
   process alive until it finishes teardown. `start` emits multiple JSON lines;
   its final `run_result` line reports the terminal state. Exit codes are 0 for
   COMPLETED, 1 for FAILED, 130 for ABORTED, and 2 for CLI/configuration errors.

5. After completion, collect the result and open the recordings:

   ```bash
   uv run --locked drone-sim collect-results RUN_ID
   ```

   Read `runs/RUN_ID/manifest.json` and `scoring/result.json`; open
   `video/observer.mp4` and `video/onboard.mp4` under that directory. For configured
   missions, `logs/companion.jsonl` contains each operation's arguments and
   outcome. `collect-results` checks the stored manifest; independent physical
   replay is a separate [acceptance step](#independent-payload-mission-acceptance).

### Run all automatic mission templates

There is currently no `drone-sim run-all` command or Phase 3 suite target. After
building the images once, paste this Bash block from the repository root. It
runs the three automatic configured missions and the three existing diagnostics
sequentially, preserving console output in a unique directory. It stops on the
first failure or abort, including failures before a run directory is created.
Normal per-run evidence remains under `runs/RUN_ID/`.

```bash
(
  set -euo pipefail
  mkdir -p runs
  suite_logs=$(mktemp -d "$PWD/runs/local-suite.XXXXXX")
  printf 'Suite console logs: %s\n' "$suite_logs"
  mission_templates=(
    config/configured-descent-run.json
    config/configured-search-delivery-run.json
    config/configured-competition-run.json
    config/vertical-descent-run.json
    config/hover-roll-run.json
    config/autotune-roll-run.json
  )
  for mission_config in "${mission_templates[@]}"; do
    mission_name=${mission_config##*/}
    printf 'Running %s\n' "$mission_config"
    uv run --locked drone-sim start --config "$mission_config" 2>&1 \
      | tee "$suite_logs/${mission_name%.json}.log"
  done
)
```

This checks mission process exit codes; it does not automatically perform
independent semantic acceptance. Apply the acceptance commands below to each
payload mission's run ID before claiming a regression pass. Keep source and
image tags unchanged throughout the batch and inspection. The AutoTune
diagnostic records candidate gains; it does not promote them into source.

The remaining checked-in templates require separate handling:

| Template | Why it is excluded from the unattended batch |
| --- | --- |
| [configured-operator-run.json](../config/configured-operator-run.json) | Requires an operator to arm and select GUIDED through an existing verified connection; see [operator arming](#operator-arming). |
| [default-run.json](../config/default-run.json) | Quarantined `comp2026_auto` input; lacks the required QGC input bundle and attempt-state setup. |
| [realtime-run.json](../config/realtime-run.json) | The same guarded QGC requirements apply; it is not an automatic alternative to the configured competition. |

Allow several hours for the six-template batch. The most recent search flight
took 47 wall minutes and the configured competition took 63; those are dated
measurements, not deadlines or guarantees. See [handoff](handoff.md) and
[recording windows](#recording-windows-and-historical-competition-timing).

## Run and monitor a current diagnostic

### Configured mission runner

The configured runner accepts automatic and operator-wait plans. See
[handoff](handoff.md#2026-09-19-configured-mission-runner) for dated verification.
Build matching runtime images before using either template, with the monorepo
revision argument described in [Build runtime images](#build-runtime-images):

```bash
SIM_COMP2026_REVISION=$(git rev-parse HEAD) \
  docker compose --profile phase3 build
uv run --locked drone-sim start --config config/configured-descent-run.json
```

The automatic example selects GUIDED, arms, takes off to 1.5 m, holds for two
simulation seconds, and lands. Edit its `mission_plan.steps` to add a regression
case; tool arguments and outcome checks are in the
[companion guide](../companion/README.md#configured-diagnostic-missions).
`world` selects the Gazebo environment: this example uses
[`vertical_descent`](../gazebo/resources/worlds/vertical_descent.sdf), a flat
ground plane with a yellow landing circle, the Iris drone, and an observer camera.
The automatic template targets one-tenth real time: 90 simulated seconds of
private warmup plus 30 recorded seconds take 20 wall minutes at that rate,
excluding startup. The operator template records 60 seconds, taking 25 minutes
at the same target rate before additional startup overhead.
`timeout_sim_s` bounds each step, default 60. The simulation recording duration
must accommodate the whole sequence, including any operator wait and landing.

#### Configured competition plan

Launch the fixed three-payload sequence with matching rebuilt Phase 3 images:

```bash
uv run --locked drone-sim start --config config/configured-competition-run.json
```

This template selects `mission: configured` and `scenario: competition_v1`. Its
42 explicit steps run FM1, FM2, both FM3 payload cycles, and HOME. The configured
runner exposes `precision_land`, `attach_payload`, `release_payload`, and
`mission_event` for this sequence; use the
[mission tool contract](../companion/src/drone_sim_companion/mission_plan.py) for
arguments and the [run-template schema](../config/run-template.schema.json) for
the configuration envelope. The plan uses the frozen repository course and
scenario inputs and does not supply QGC inputs.

The public recording window is 420 simulated seconds after a 90-second native
warmup, with target real-time factor 1.0. Recording and scoring continue through
the full 420-second window even if the mission reaches HOME earlier. The
2026-09-19 run completed the mission at public time 250.15 seconds and passed
independent 150/150 acceptance with valid artifacts. The complete run took
3,762.64 wall seconds on this machine; target factor 1.0 does not guarantee real
time performance. See [dated evidence](handoff.md#2026-09-19-configured-competition-conversion).

#### Search-and-deliver mission

Build matching Phase 3 images, then launch:

```bash
uv run --locked drone-sim start --config config/configured-search-delivery-run.json
```

The 24-step plan flies a compact route with turns and altitude changes, stops
2 m west of ArUco 3, searches and lands, picks up, delivers from 10 m, and lands
home. The drone starts empty. Its separate
[course](../config/course-search-delivery.yaml),
[scenario](../config/scenario-search-delivery.yaml), and generated
[world](../gazebo/resources/worlds/search_delivery.sdf) keep the competition
baseline intact. The legacy `competition` configuration field binds and freezes
these physical inputs for either payload scenario.

The observer records at 1280 × 960; onboard vision remains at its calibrated
640 × 480. Both streams record at 20 FPS. The public window is 240 simulated
seconds after 90 seconds of private warmup. Target RTF is 1.0; CPU rendering may
run substantially slower. Actual timing and verified flight status belong in
[handoff](handoff.md). Search, pickup, delivery, and home each
carry 25 points; mission completion and valid recordings remain separate checks.

With the ROS/FFmpeg inspection dependencies available, require full independent
acceptance using the existing inspector's ruleset selector:

```bash
uv run --locked python scripts/inspect_competition_run.py runs/RUN_ID --ruleset search_delivery_v1
```

#### Operator arming

To wait for external arming and GUIDED selection, use:

```bash
uv run --locked drone-sim start --config config/configured-operator-run.json
```

That plan sends neither ARM nor GUIDED. It starts TAKEOFF only after observing
both states. It does not set up a new operator/QGC connection; use an existing
verified connection to this simulator. The present Compose stack does not expose
an additional operator control port through this change.

The companion log records operation IDs, arguments, simulation timestamps, and
terminal results. The exact normalized plan is in the run's
`configuration/run.json`. Failure stops later steps; a local LAND recovery does
not make that regression pass. Use the status/abort/collect commands below and
check physical outcome, score, and artifact validity separately.

### Existing descent diagnostic

This example uses the repository-default controlled-descent route. The guarded
`default-run.json` and `realtime-run.json` templates still omit the required QGC
bundle and attempt-state binding.

```bash
uv run --locked drone-sim start --config config/vertical-descent-run.json
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

### Recording windows and historical competition timing

The historical competition default specified a 90-second **native simulation**
warmup followed by a 600-second **public simulation** window. Its target
real-time factor is 0.25:
even if achieved, these total about 46 minutes of wall time, plus startup and
finalization. Host contention can make it slower. The flight may land home well
before the 600-second recording/scoring window ends. Do not interpret quiet
mission logs alone as a stopped process.

| Template | Purpose | Public duration / warmup / target RTF |
| --- | --- | --- |
| [configured-competition-run.json](../config/configured-competition-run.json) | Configured three-payload plan; 150/150 accepted on 2026-09-19 | 420 s / 90 s / 1.0 |
| [default-run.json](../config/default-run.json) | Quarantined competition default, historical timing only | 600 s / 90 s / 0.25 |
| [vertical-descent-run.json](../config/vertical-descent-run.json) | Controlled descent, not the payload mission | 60 s / 90 s / 0.1 |
| [hover-roll-run.json](../config/hover-roll-run.json) | Short roll/hover diagnostic | 45 s / 15 s / 0.1 |
| [autotune-roll-run.json](../config/autotune-roll-run.json) | Roll AutoTune experiment | 120 s / 15 s / 0.1 |
| [realtime-run.json](../config/realtime-run.json) | Quarantined competition variant, historical timing only | 600 s / 90 s / 1.0 |

Short diagnostic missions do not prove competition success. AutoTune promotion
is a separate, explicit source change using
[promote_roll_autotune.py](../scripts/promote_roll_autotune.py); it is not part of
normal launch. Inspect the saved gains and resulting parameter diff before reuse.

### Optional NVIDIA path

After provisioning and checking the host NVIDIA container runtime, EGL, NVENC,
and the device path required by [compose.gpu.yaml](../compose.gpu.yaml), run the
available controlled-descent diagnostic:

```bash
SIM_COMPOSE_OVERLAY=gpu uv run --locked drone-sim start --config config/vertical-descent-run.json
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

Historical Comp2026 precision-landing runs record one compact JSON object per
rejected observation and state transition. Inspect it without changing the bundle:

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
  | rg 'LAND_SPD_MS|LAND_SPEED|PLND_|ATC_ANG_RLL_P|ATC_RAT_RLL_|ATC_RAT_PIT_|ATC_ACC_R_MAX|ATC_ACCEL_R_MAX'
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

### Independent payload mission acceptance

```bash
# Search-and-deliver: requires 100/100.
uv run --locked python scripts/inspect_competition_run.py \
  runs/SEARCH_RUN_ID --ruleset search_delivery_v1

# Configured competition: requires 150/150.
uv run --locked python scripts/inspect_competition_run.py \
  runs/COMPETITION_RUN_ID --ruleset competition_v1
```

This is read-only and requires the Python environment to provide ROS Jazzy
`rosbag2_py`, plus FFmpeg/FFprobe and access to the seven Docker image tags.
`uv sync` alone does not install those system dependencies. If they are missing,
the mission launch/recording workflow still works through Docker, but this host
inspection command is unavailable. A portable local/CI acceptance wrapper remains
future work. Do not report independent acceptance from `collect-results` alone.
`make inspect-competition RUN_DIRECTORY=/absolute/path/to/runs/RUN_ID` remains
the shorthand for the competition ruleset only. See
[acceptance implementation](../artifacts/src/artifacts/acceptance.py) and
[inspector](../scripts/inspect_competition_run.py).

Both commands require their maximum score, valid evidence, and matching **current** source
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
