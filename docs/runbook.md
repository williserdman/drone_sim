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

Check the deployed competition import closure before a flight:

```bash
docker run --rm drone-sim-companion-runtime:phase3 \
  python3 -m drone_sim_companion.comp2026_smoke
```

This imports the original FM1/FM2/FM3 functions without opening devices. A
nonzero exit identifies a packaging failure; success is not flight evidence.

The manual operator template waits for external GUIDED/arming. The suite supplies
that operator through the private `operator-wait` Compose profile, retaining the
original mission plan. Ordinary `start` keeps manual operator behavior.

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
The operator example waits for external arming and GUIDED selection. Native
SERIAL1 exposes private TCP 5762 for that operator; the suite supplies the
external actor while direct `start` retains manual operation. Its 60-second
public window must cover the wait and flight.

### Calibrate and validate saved gains

This two-run workflow runs the calibration gates individually. The
[full suite](#run-the-full-mission-suite) applies these dependencies automatically. Use a clean checkout and build all seven images
with the [build command](#build-runtime-images); capture expected provenance using
the [acceptance block](#moving-pad-landing) before either run. Keep that checkout
and those image tags fixed through both flights and acceptance.

1. Run roll, pitch and yaw AutoTune on the unloaded competition airframe:

   ```bash
   uv run --locked drone-sim start --config config/autotune-run.json
   ```

   The mission settles in LOITER, tunes all axes, reactivates the tuned gains,
   settles again and uses native LAND. The public/warmup windows total 990
   simulated seconds at target RTF 0.25, about 66 wall minutes plus startup.
   The final 60 public seconds remain reserved for LAND or failure recovery;
   calibration stops tuning at public 840 seconds. The wall limit remains
   7,200 seconds.
   A failed tune or landing cannot release accepted parameters.

2. Save its UUID and prepare a temporary validation template:

   ```bash
   calibration_run=REPLACE_WITH_RUN_UUID
   validation_config=$(mktemp /tmp/drone-sim-validation.XXXXXX.json)
   python3 - "$calibration_run" "$validation_config" <<'PY'
   import json
   from pathlib import Path
   import sys
   template = json.loads(Path("config/calibration-validation-run.json").read_text())
   template["calibration"]["source_run_directory"] = str((Path("runs") / sys.argv[1]).resolve())
   Path(sys.argv[2]).write_text(json.dumps(template, indent=2) + "\n")
   PY
   uv run --locked drone-sim start --config "$validation_config"
   ```

   Before launching, the importer independently accepts the source at 100/100
   and checks aircraft, base parameters, firmware and image compatibility. It
   copies the exact gain file and source manifest into the new run. Fresh SITL
   loads the gains last, and companion verifies readback before arming. This
   flight takes off to 5 m, holds 10 seconds and lands. Its 210 simulated seconds
   including warmup target about 14 wall minutes plus startup.

3. Independently inspect both bundles with the captured provenance. Use
   `scorekeeper/rules/calibration_v1.json` for AutoTune and
   `scorekeeper/rules/descent_v1.json` for validation, retaining
   `--require-maximum-score`. Report physical outcome, score and artifact
   acceptance separately. The validation inspector checks recorded hover
   stability as well as landing and loaded-gain evidence.

   Current descent rules use settling-policy version 2. Inspect older descent
   bundles with `scorekeeper/rules/descent_v1_legacy.json` when their scoring
   checksum matches that file. A replay under revised rules is a separate
   diagnostic; it does not change historical acceptance. Accepted calibration
   gains may be reused for validation when the importer confirms physical-profile
   compatibility, without rerunning AutoTune.

The source artifact is `runs/RUN_ID/ardupilot_sitl/autotune.parm`. The validation
copy is `runs/RUN_ID/configuration/calibration.parm`; its source identity is frozen
in `configuration/run.json`. This workflow does not edit tracked defaults.
Other scenarios need fresh flights with the shared aircraft before claiming no
regressions. The old `autotune-roll-run.json` remains a historical diagnostic.

### Run the full mission suite

Commit runtime edits, then run from this checkout:

```bash
uv sync --locked
uv run --locked drone-sim suite --config config/ci-suite.json
```

The [catalog](../config/ci-suite.json) covers all 11 checked-in flight templates,
including the stationary fixture, operator wait and both competition timings.
The command builds seven Phase 3 images once, checks the original competition
imports, calibrates all axes, then validates saved gains in fresh SITL. These
first two cases gate the other nine. Every consumer loads the same accepted
artifact after its base/scenario overlays and verifies live parameters before
flight. Each case has fresh SITL storage, Compose project and normal recordings.

Individual mission failures do not skip later independent cases. Failed gates,
changed source/images or unconfirmed teardown block remaining cases. Keep source
and image tags unchanged until the command returns. One account-wide workstation
lock prevents concurrent suites across checkouts and output roots. Ctrl-C
requests the active run's existing bounded abort/finalization and retains a
partial report; keep the process alive until teardown returns.

The final `suite_result` JSON points to `runs/suites/SUITE_ID/report.json` and
adjacent `report.md`. Reports separate lifecycle, physical outcome, raw score,
independent acceptance and teardown, linking to `runs/RUN_ID` bundles. Recordings
remain in each bundle's `video/onboard.mp4` and `video/observer.mp4`. Diagnostic
safe landing can pass at 60/100; other cases require their maximum score. Exit
codes are 0 passed, 1 failed/blocked/unrun, 2 setup failure, 130 interrupted.

Use an absolute persistent destination when needed:

```bash
uv run --locked drone-sim suite --config config/ci-suite.json \
  --output-root /home/willis/projects/drone_sim/runs/local-ci
```

The existing seeds, plans, timing and recording settings remain unchanged. The
catalog's target timing totals about 4 h 40 min before build, startup and
finalization; use the report's timestamps for actual duration. Configuration
coverage fails if a new top-level flight template is absent from the catalog.

### Manual workstation CI

The [Mission suite workflow](../.github/workflows/mission-suite.yml) uses the same
catalog and CLI, with no scheduled, push or PR trigger. It selects this workstation
by `self-hosted`, `linux`, `x64`, `drone-sim` labels and queues dispatches without
cancelling an active suite. Its persistent outputs are
`/home/willis/projects/drone_sim/runs/ci/GITHUB_RUN_ID-GITHUB_RUN_ATTEMPT`.
Full MP4s and bags remain local; Actions publishes the report and compact logs,
configuration, manifests, score results and parameters.

The official runner is registered at `/home/willis/actions-runner-drone-sim`.
Service installation needs interactive workstation sudo:

```bash
cd /home/willis/actions-runner-drone-sim
sudo ./svc.sh install willis
sudo ./svc.sh start
```

Confirm the service with `sudo ./svc.sh status` and the repository runner page.
Registration alone does not establish that the service is online. Machine setup
and verification are recorded in `/home/willis/SETUP_REPLICATION.md`.

After the workflow definition is present on `main`, dispatch through Actions →
Mission suite → Run workflow, or from this repository:

```bash
gh workflow run mission-suite.yml --ref main
gh run list --workflow mission-suite.yml --limit 1
```

Watch that run's job summary for the report. The self-hosted job allows 24 hours;
the local command still enforces its own build, run, inspection and finalization
bounds. Existing [handoff](handoff.md) evidence distinguishes local sweeps from
actual Actions dispatches; neither registration nor workflow syntax proves a flight.

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
| [configured-moving-pad-run.json](../config/configured-moving-pad-run.json) | Takeoff/transit/camera-guided moving-deck landing | 90 s / 90 s / 0.1 |
| [configured-operator-run.json](../config/configured-operator-run.json) | Operator arms/selects GUIDED, then takeoff/hold/land | 60 s / 90 s / 0.1 |
| [default-run.json](../config/default-run.json) | Full three-payload competition | 600 s / 90 s / 0.25 |
| [vertical-descent-run.json](../config/vertical-descent-run.json) | Controlled descent, not the payload mission | 60 s / 90 s / 0.1 |
| [hover-roll-run.json](../config/hover-roll-run.json) | Short roll/hover diagnostic | 45 s / 90 s / 0.1 |
| [autotune-roll-run.json](../config/autotune-roll-run.json) | Roll AutoTune experiment | 120 s / 90 s / 0.1 |
| [realtime-run.json](../config/realtime-run.json) | Competition with a higher speed target, not a speed guarantee | 600 s / 90 s / 1.0 |

Short diagnostic missions do not prove competition success. AutoTune promotion
is a separate, explicit source change using
[promote_roll_autotune.py](../scripts/promote_roll_autotune.py); it is not part of
normal launch. Inspect the saved gains and resulting parameter diff before reuse.
The calibrated roll diagnostics use the same 90-second empirical private warmup
as the other calibrated consumers so their complete parameter readback can finish
before the fixed public epoch. This is a runtime allowance, not a MAVLink timing
guarantee; the calibration gate still requires every configured parameter.

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

### Moving-pad landing

Read the [current flight evidence](handoff.md#moving-pad-verification) before a demo.
The EKF3 moving run passed independent acceptance with 100/100 and full recordings.
A separate stationary flight landed but its recording failed on a missing pad
contact sample. That intermittent evidence-stream failure remains unresolved.
The current moving-only experiment selects stock `AHRS_EKF_TYPE=3`, native
precision `PLND_EST_TYPE=1`, and the original `PSC_NE_POS_P=1`. It retains
option 5 and 40 ms lag with exposure-stamped target messages. EKF3 avoids the
diagnosed SIM attitude delta-velocity frame error. Native LAND controls the
accepted moving descent; the camera and pad speed were unchanged.

Build from a committed checkout using [the image-build command](#build-runtime-images).
The configured mission takes off to 5 m, flies 35 m east, then watches marker 7
and precision-lands. The 3 m deck moves east at 0.5 m/s from public time zero,
including after disarm. Camera observation continues during transit. The mission
must settle by 45 s and acquire two seconds of fresh observations by 60 s;
failure ends the attempt. The [architecture contract](architecture.md#moving-pad-landing)
defines tracking loss and final touchdown behavior.

Use the stationary fixture for controller isolation and the moving configuration
for the actual mission. These commands launch different courses:

```bash
uv run --locked drone-sim start --config tests/fixtures/configured-stationary-pad-run.json
uv run --locked drone-sim start --config config/configured-moving-pad-run.json
```

If public time reaches 90 s without mission completion, the current runtime can
wait for its wall deadline: the precision-operation deadline exceeds the capped
public clock. Use `uv run --locked drone-sim abort RUN_ID` to finalize a stalled
experiment and preserve evidence. That result is `ABORTED`, never an accepted
mission. A deadline/recording-boundary fix remains separate work.

Run each command separately and inspect its result before continuing. Each
includes 90 s of warmup and 90 s of public simulation at target RTF 0.1:
30 wall minutes at that rate, plus startup/finalization. On the validation host,
90 s of warmup took about 18 wall minutes; allow about 36 minutes for the full run.
The stationary fixture
uses the same deck, camera, landing controller, and scoring, with its pad held at
the approach point. It is a control experiment, not part of the automatic batch.

For independent acceptance, capture expected provenance **before launching**,
in the same Bash session used to inspect the result:

```bash
mission_revision=$(git rev-parse HEAD)
mission_acceptance_args=(
  --rules-path "$PWD/scorekeeper/rules/moving_pad_v1.json"
  --require-maximum-score
  --expected-source "drone_sim=$mission_revision"
  --expected-source-dirty-entry drone_sim=false
  --expected-source "comp2026=$mission_revision"
  --expected-source-dirty-entry comp2026=false
)
while IFS= read -r mission_image; do
  mission_digest=$(docker image inspect --format '{{.Id}}' "$mission_image")
  mission_acceptance_args+=(--expected-image-digest "$mission_image=${mission_digest#sha256:}")
done < <(docker compose --profile phase3 config --images | sort -u)

# After the run finishes, replace RUN_ID with its UUID:
uv run --locked python -m artifacts.acceptance "$PWD/runs/RUN_ID" \
  "${mission_acceptance_args[@]}"
```

Keep the checkout clean and image tags unchanged between capture, flight, and
acceptance. The inspector runs ROS-dependent checks inside the artifacts image
and verifies Compose teardown on the host. A physical pass requires deck contact,
observed disarm, and two continuous simulated seconds aboard. Acceptance
independently recomputes the 100-point result from recorded vehicle/pad truth
and arm transitions, then validates both recordings and provenance. Inspect
`video/onboard.mp4`, `video/observer.mp4`, and `scoring/result.json` under the run
directory. Report physical landing, score, and artifact validity separately.

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
