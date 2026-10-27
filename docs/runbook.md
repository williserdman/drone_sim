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
uv run --locked pytest orchestration/tests/test_cli.py -q
docker version
docker compose version
```

These commands provide a quick cross-platform CLI and installation check. The
complete supported source gate runs in the Linux harness described in
[Test without a full flight](#test-without-a-full-flight). It supplies Linux
`/proc`, anonymous-file, and FFmpeg descriptor behavior, so macOS developers do
not need to interpret those expected host limitations as product failures.

### Comp2026 mission source

`companion/comp2026` is tracked in this monorepo. A normal clone contains the
mission source required by the Phase 3 build. Its history before the monorepo
import remains available through Git.
The active FM3 remains
[`src/drone/mock_mission.py`](../companion/comp2026/src/drone/mock_mission.py).
Simulation adapters remain under
[`companion/src/drone_sim_companion`](../companion/src/drone_sim_companion/).
Use the [flight-code map](../companion/README.md#where-to-change-comp-2026-code)
for ownership and entry points instead of copying mission logic into this guide.

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

### Native local image build

The local builder supports native `linux/arm64` and `linux/amd64` engines. It
derives the target from the detected host and accepts Docker's ARM64 aliases
`arm64` and `aarch64`, and AMD64 aliases `amd64` and `x86_64`. Use a local
engine from the configured provider. Remote `DOCKER_HOST` or context endpoints,
TLS endpoint overrides, emulation, and a conflicting `DOCKER_DEFAULT_PLATFORM`
are rejected before Compose build or simulation preparation.

Run the same commands on each native workstation:

```bash
./scripts/local-build.sh --build
./scripts/local-build.sh --check
```

The build holds the shared workstation lease, builds the seven Phase 3 runtime
services and then the local Gazebo viewer, verifies all eight actual image
architectures and identities, and publishes one schema-1 record. The read-only
check never rewrites an old record. Keep at least 4 CPUs and 8 GiB memory, with
6 CPUs and 12 GiB recommended, plus the existing 8 GiB free-disk minimum.

If an exact Ubuntu package version has disappeared from the package indexes,
resolve replacement locks explicitly:

```bash
python3 scripts/resolve-amd64-locks.py --architecture arm64 --refresh-unavailable-ubuntu
python3 scripts/resolve-amd64-locks.py --architecture amd64 --refresh-unavailable-ubuntu
```

Despite its historical filename, the resolver supports both architectures. It
uses the native ROS image as an apt utility and the selected architecture's
pinned base package database. It verifies signed Ubuntu and ROS indexes,
downloads every selected archive, and checks its SHA-256. Available exact pins
and ROS versions stay fixed. Only unavailable Ubuntu pins are refreshed, with
any required dependency changes included in each complete layer delta.

The resolver writes candidates and evidence under
`.local-runtime/apt-lock-resolution/<architecture>/runs/`. It does not edit the
repository. Review the successful run's `resolution-summary.json`, compare
its candidate locks with their repository counterparts, and apply the reviewed
changes. The artifacts direct-package candidate lists pins from
`artifacts/Dockerfile`; update those explicitly if they changed, along with
matching common pins in the other runtime Dockerfiles. Keep the lock
contract tests and recorded checksums consistent, then build and check images
with the commands above. Ordinary builds never refresh dependencies themselves.

Metadata resolution does not execute foreign package scripts or prove native
compilation and runtime behavior. AMD64 acceptance still requires an AMD64
workstation. The [current lock audit](package-lock-resolution-summary.json)
records both architectures' signed indexes, replacements, and archive-manifest
checksums. The earlier [AMD64 resolution report](amd64-lock-resolution.md)
and [signed-index inventory](amd64-lock-resolution-summary.json) remain dated
historical evidence.

The unattended/cloud workflow remains a seven-image build. It does not build or
depend on the local QGC/Gazebo viewer image.

### Local Mac console

The same console supports macOS supported by the installed Docker Desktop
release, Ubuntu 24.04 desktop, and Ubuntu
24.04 under WSL2 on Windows 11 x86-64. Every target needs Python 3.12, locked
`uv` dependencies, Docker Compose, 8 GiB free for run evidence, at least 4 CPUs
and 8 GiB RAM, and preferably 6 CPUs and 12 GiB. Install QGroundControl 5.0.8
using the [QGC installation guide](https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/getting_started/download_and_install.html).

On macOS, install QGC at `/Applications/QGroundControl.app` and install
[Docker Desktop for your Mac's architecture](https://docs.docker.com/desktop/setup/install/mac-install/).
Open Docker Desktop and complete its first-run setup. In **Settings > Resources**,
allocate at least 4 CPUs and 8 GiB RAM, preferably 6 CPUs and 12 GiB RAM, and
allow at least 80 GiB of virtual disk capacity for images, caches, and recordings.
These are capacity settings; the host also needs sufficient free disk space.
Docker Desktop manages its own Linux VM, so no Colima or Lima installation is
needed. Choose native ARM64 images on Apple Silicon and native AMD64 on Intel.

```bash
open -a Docker
docker context use desktop-linux
docker info
uv sync --python 3.12 --locked
uv run --locked drone-sim console
```

Wait for Docker Desktop to report that its engine is running before the Docker
commands. The normal context is `desktop-linux`; the builder also accepts a
default or renamed local context when its effective engine is Docker Desktop.
Remove stale `DOCKER_HOST` or `DOCKER_CONTEXT` shell overrides if they select the
old engine. Remote endpoints, TLS overrides, and emulated image architectures
remain unsupported. The console reports a mismatch instead of switching engines.

Keep the checkout and run directories in a directory shared with Docker Desktop.
`/Users` is shared by default on macOS. If using another location, add it under
**Settings > Resources > File sharing**. The existing loopback port mappings work
through Docker Desktop; host networking is not required.

When migrating an existing installation, finish the old run with **Stop
simulation** while its original engine is available. Confirm cleanup is complete
before stopping Colima or selecting Docker Desktop. An empty new engine does not
prove cleanup of the old engine. Preserve the old VM until any needed evidence
has been published. Docker Desktop has a separate image and volume store, so run
`./scripts/local-build.sh --build` and `--check` there before preparing a new run.
Do not copy or edit the old build record to make it match the new engine.

On Ubuntu 24.04 desktop, install Docker Engine and the Compose plugin from the
[official Ubuntu instructions](https://docs.docker.com/engine/install/ubuntu/).
Use the official QGC AppImage on x86-64. On ARM64, select an architecture-matched
5.0.8 installation with its AppStream metadata in `.local-runtime/host.json`.
Then run:

```bash
uv sync --python 3.12 --locked
./scripts/local-build.sh --build
./scripts/local-build.sh --check
uv run --locked drone-sim console
```

On Windows, install `Ubuntu-24.04` and verify WSL version 2 with the
[Microsoft WSL commands](https://learn.microsoft.com/en-us/windows/wsl/basic-commands).
Enable Docker Desktop's WSL2 engine and **Resources > WSL Integration** for that
distribution, following the [Docker WSL guide](https://docs.docker.com/desktop/features/wsl/).
Do not install a second Docker daemon inside the distribution. Clone the repo
under the Linux home directory, for example `~/projects/drone_sim`; `/mnt/c`,
`drvfs`, `9p`, and symlinks into them are rejected for checkout and state paths.
Install `uv` inside Ubuntu using the
[official installation guide](https://docs.astral.sh/uv/getting-started/installation/),
then run the same sync, build, check, and console commands there.

Install native Windows QGC 5.0.8. Its defaults are
`%ProgramFiles%\QGroundControl\bin\QGroundControl.exe` and
`%APPDATA%\QGroundControl\QGroundControl.ini`. Windows PowerShell 5.1 must be
allowed to execute the repository helper. Inspect policy with
`Get-ExecutionPolicy -List`; do not use an execution-policy bypass, elevation,
or a service. See [Microsoft's execution-policy reference](https://learn.microsoft.com/en-us/powershell/module/microsoft.powershell.core/about/about_execution_policies?view=powershell-5.1).
If native paths need explicit selection, create the schema-1 host file shown in
the [orchestration reference](../orchestration/README.md#host-configuration) and
launch `uv run --locked drone-sim console --host-config .local-runtime/host.json`.

The console uses port 8765. The local overlay publishes Gazebo desktop on 6080,
onboard camera on 8081, and the dedicated SITL GCS channel on TCP 5762, all on
loopback. A host bridge connects that channel to native QGC's UDP 14550 discovery
listener, independently of the companion's connection. Keep QGC's normal UDP
auto-connect enabled. Opening an already-running QGC focuses it; cleanup does
not close the application.

1. Click **Build matching images** if the source/image check requires it. This
   is explicit and uses Docker's build cache. Build output appears in the log
   panel and `.local-runtime/build.log`. The equivalent terminal commands are:

   ```bash
   ./scripts/local-build.sh --build
   ./scripts/local-build.sh --check
   ```

   The build record binds the current revision, runtime input content, native
   platform, Docker context, and all eight image IDs. Source or image changes
   require a matching build before another preparation. The terminal wrapper,
   console builder, direct runs, and suites use the same account-wide
   workstation lease. Preserve image tags during a run and finalization.

2. Choose the world and a compatible preset. These are complete tuples from
   `config/ci-suite.json`; the UI does not synthesize arbitrary combinations.
   Reload validation requires the absolute directory of an independently
   accepted calibration run. The console still validates that run's calibration
   identity against the current firmware, airframe, and images.

3. Click **Prepare simulation**. While it owns the workstation lease, the console
   allocates the run and its action token, writes immutable configuration, and
   publishes the exact action bytes under QGC's configured
   `savePath/MavlinkActions` directory. It displays the full path, then opens or
   focuses QGC and starts Compose. It does not change QGC preferences or the
   currently selected action file. Publication failure retains the run identity
   for diagnosis and starts no simulator.

4. In QGC, open **Application Settings > Fly View > MAVLink Actions > Fly View
   Actions**. Reopen that settings page if its file list is stale. Select the
   exact `drone-sim-RUN_ID.json` filename shown by the console. This selection is
   a native QGC step; the console does not claim it succeeded. Native physics
   and camera frames advance while the vehicle stays disarmed. The separate
   preflight limit defaults to 30 wall minutes. Warmup and rendering may take
   substantial wall time at a preset's configured pace.

5. Use **Start selected mission** in QGC after the console reports fresh vehicle,
   native clock, calibration, and producer readiness. Browser focus, polling,
   Gazebo desktop connection, and camera-view freshness do not authorize the
   command. One immutable request admits one mission epoch. Automatic presets
   retain onboard command ownership. For **Operator Wait** only, use Start first,
   then select GUIDED and arm in QGC so its companion sequence can continue.

6. Use **Abort and recover** in QGC to invoke guarded onboard recovery. The
   console reports admission, mission, and recovery observations alongside
   viewer and camera freshness. A rejected command, stale telemetry, timeout,
   unconfirmed landing, or manual takeover remains visible. **Stop simulation**
   requests infrastructure finalization and teardown; it does not request a
   landing and may interrupt an airborne vehicle. Wait for cleanup before **Run
   again**, which allocates a new run identity, token, and action file.

The streamed Gazebo GUI supports navigation, selection, and read-only inspection.
World mutation controls are excluded. QGC retains its native operator controls;
manual commands are recorded in `logs/docker/qgc-interventions.jsonl` and flight
supervisors observe changes in authority. A stale camera image is visibly marked
and remains separate from the private camera-producer readiness used by QGC
admission. Closing a browser tab leaves the session owner running; reload the
console to reconnect. Interrupting the console process asks the active controller
to finalize before exit.
Desktop status uses the actual browser RFB connection. A disconnected desktop is
reported as stale but cannot block an otherwise eligible QGC action. Losing the
viewer during flight leaves healthy flight services running. Reopen the console
to reconnect and inspect them.

QGC transport status is separate from SITL telemetry and the streamed desktop.
If the native datagram transport fails, the console reports QGC disconnected
while healthy simulation services continue. Stop Simulation still uses the
normal bounded finalization path; it does not create a landing or recovery
command. At finalization, the console closes the run transport and its host log
writer after runtime and media closure, before the manifest inventory begins.

The local artifacts service uses a run-scoped **Docker named volume** for its
recording spool. Earlier Colima checks found that the macOS shared directory and
the container writable layer rejected `O_TMPFILE`; Docker Desktop retains the
native Linux volume contract. Closed media is published to the host evidence
directory before recorder finalization is reported. Do not replace the spool
with a macOS bind mount or remove it while recording/finalizing.
Cleanup removes the volume only after verified publication; if publication fails,
the retained volume appears in the remaining resources and blocks another run.

CLI actions use the same persistent console, running in another terminal:

```bash
uv run --locked drone-sim prepare --preset configured-descent
uv run --locked drone-sim stop RUN_ID
uv run --locked drone-sim status RUN_ID
uv run --locked drone-sim collect-results RUN_ID
```

Descriptor-backed runs reject the historical `start-mission` and `abort-flight`
HTTP and CLI actions before writing a request file. Descriptor-absent operator
runs retain those commands for compatibility. `stop` without a run ID cancels
the current preparation as well. Add `--port` to console actions when using a
nondefault console port. The original `start` and `suite` commands remain
unattended workflows; original `abort` means simulator finalization.

Evidence remains under `runs/RUN_ID/`. The console displays physical completion,
score, artifact validity, and cleanup separately. `configuration/run.json` stays
frozen; `configuration/start-admission.json` records the actual native/public
clock mapping. Read the committed manifest as terminal authority. If the owner
crashes, the next console identifies the orphaned session and blocks another
preparation until cleanup. Its interrupted evidence does not acquire a fabricated
landing or artifact-validity claim. Use **Stop simulation** on that orphan to
enter exact-run cleanup. A run that failed before Compose startup can clear its
retained owner after the console verifies that no run resources exist. If
Compose started without verified media publication, the retained media volume
and native owner continue to block builds and preparations.

### Windows QGroundControl prerequisites for WSL2

The production console selects the packaged helper on WSL2. Native acceptance
is still pending. Before that run, use Windows 11 x86-64 with Ubuntu 24.04 in
WSL2. Keep the checkout, run evidence, session receipts, and token ledger on the
distribution's Linux ext4 filesystem. Do not use `/mnt/c`, another `drvfs` or
`9p` mount, or a symlink into one of those mounts for simulator state.

Install the official QGroundControl 5.0.8 Windows build. The native defaults are:

```text
%ProgramFiles%\QGroundControl\bin\QGroundControl.exe
%APPDATA%\QGroundControl\QGroundControl.ini
```

The settings file must contain an absolute `[General] savePath`. The helper
publishes only to its `MavlinkActions` child on local NTFS. It reads the actual
settings and effective 255/190 MAVLink identity, validates the executable's
5.0.8 version resources, and checks any running QGroundControl process against
that executable before focus. It does not write preferences, select an action,
terminate QGroundControl, or invoke an undocumented version command.

The logged-in Windows user's `powershell.exe` must be able to run the
repository-owned helper script through WSL interoperability. Review policy with
`Get-ExecutionPolicy -List`. Do not add `-ExecutionPolicy Bypass`, change policy,
request elevation, or install a service. A repository on the WSL Linux
filesystem is normally exposed to Windows through a UNC path; Windows policy
can impose additional UNC restrictions. If policy blocks the script, ask the
workstation administrator to permit the repository script and record that
prerequisite. Do not copy simulator state to a Windows mount as a workaround.

The native acceptance command requires policy already permitting the script:

```powershell
powershell.exe -NoLogo -NoProfile -NonInteractive -File tests/windows/test_host_helper.ps1
```

Run it from the Windows view of the repository. The test uses a temporary local
NTFS directory, child Windows PowerShell processes, an isolated HKCU test key,
the actual helper pipes, and Windows loopback UDP. It sends arbitrary binary
datagrams in both directions and verifies that only `127.0.0.1:14550` is
accepted. Retain its raw output with Windows, Windows PowerShell, filesystem,
WSL, and network-mode versions. Run the WSL integration test once under default
NAT and once under mirrored networking. A non-Windows parse, macOS PowerShell
run, fixture process, or mocked Win32 call is not a passing native result.

```bash
uv run --locked pytest tests/integration/test_wsl_host.py -q
```

### Inspect QGC control evidence

After finalization, inspect one bounded QGC claim with the read-only evidence
reader:

```bash
uv run --locked python scripts/inspect_qgc_control_run.py runs/RUN_ID \
  --expect routing
```

Choose `routing`, `start`, `abort`, `ordinary-land`, or `precision-loss`. The
JSON result reports `command`, `physical`, `profile`, and `cleanup` separately.
A dimension that the selected claim does not need is `not_required`; it is not
reported as passed. The process exits zero only when every dimension required
for that claim passes.

The reader accepts evidence selected by the committed manifest. It checks the
immutable run, action, request, and admission bindings before interpreting the
native MAVLink log, DataFlash log, companion events, landing-profile events, or
precision-loss stimulus. Missing, malformed, stale, ambiguous, unsupported, or
checksum-mismatched evidence fails the affected finding. An ACK never proves
airborne flight, touchdown, disarm, profile restoration, or target loss by
itself.

Every selected `.status/qgc-command-NNNN.json` must appear in the final manifest,
with no extra status file outside that inventory. Landing-profile inspection
accepts Task 5's one shared generation boundary for the batched full-profile
request; each parameter verification must be newer than that boundary. Later
set requests may capture the current generation, and their matching verification
must be newer. Ordinary LAND also requires both restoration events after the
independent touchdown observation and an eventual independent disarm. Abort
status wall times must not regress, and adjacent native times must not regress
when both records provide one.

Native command inspection uses `logs/docker/native-qgc.tlog`. The acceptance
workflow must copy a labeled snapshot of the genuine QGC telemetry log to that
path while the run still owns its `.control` lock and before artifact inventory
and finalization. Preserve the original QGC log and record the selected source
and capture boundary outside the bundle. Do not copy into or edit a finalized
bundle. The retained packet stream establishes wire identity, command fields,
heartbeat, ACK, progress, and status text. Native file selection and button
clicks remain a separate operator observation.

This command does not replace full artifact acceptance. Video, public rosbag,
score, source and image provenance, and live Compose cleanup still use the
existing acceptance workflow.

Focused host checks, including the new typed operator boundary:

```bash
uv run --locked mypy
uv run --locked pytest orchestration/tests/test_operator_controller.py \
  orchestration/tests/test_operator_session.py orchestration/tests/test_console_http.py -q
```

Some existing fault-injection and recorder tests require Linux `/proc` and
`O_TMPFILE`; run those on Linux storage rather than treating macOS collection
compatibility as a recorder verification.

### Existing unattended image build

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

### Develop Comp 2026 with QGC

Use the [flight-code map](../companion/README.md#where-to-change-comp-2026-code)
to choose the mission, shared controller, or simulation adapter to edit. These
files are tracked in `drone_sim`; commit them on the same feature branch.
The local console source supports the matrix documented above. Native Intel Mac,
Linux desktop, and Windows/WSL acceptance remain pending and must not be inferred
from focused software tests.

1. Edit the source and run tests that cover the changed behavior. For example,
   these tests cover automatic sequencing and mission phase outcomes:

   ```bash
   PYTHONPATH=companion/comp2026/src:companion/comp2026/tests \
     uv run --locked pytest companion/comp2026/tests/test_auto_attempt.py \
       companion/comp2026/tests/test_mission_phase_results.py -q
   ```

   For shared controller or simulator adapter changes, also run the affected
   tests under `companion/comp2026/tests` or `companion/tests`.

2. After the previous simulation has stopped and cleanup is complete, build
   matching images. From the repository root:

   ```bash
   ./scripts/local-build.sh --build
   ./scripts/local-build.sh --check
   uv run --locked drone-sim console
   ```

   If the console is already running, use its **Build matching images** button.
   Source is copied into the companion image; editing a file does not update a
   running simulation. Committing after a build changes the required revision,
   so build again before the next run. If you add an imported mission module,
   check the [Docker context allowlist](../.dockerignore).

3. Select a competition preset that runs `comp2026_auto`, then click
   **Prepare simulation**. Select its new action file in QGC and use
   **Start selected mission** there, following the
   [console workflow](#local-mac-console). Configured diagnostic presets invoke
   a separate operation runner and do not exercise the Comp2026 mission phases.

4. Inspect the run's flight outcome, score, and recorded evidence separately.
   Use **Abort and recover** in QGC when flight recovery is needed. Use
   **Stop simulation** for teardown, then wait for cleanup before rebuilding
   and preparing another run. Each preparation creates a new run and action file.

### Run an unattended development check

For the existing unattended CLI, run from the repository root after editing
`companion/comp2026` or simulator code:

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

   After a healthy heartbeat, the mission applies run-local roll P/I seeds of
   `0.0675` and `AUTOTUNE_AXES=7`, then requires a fresh, complete,
   float32-aware parameter readback before arming. Missing or mismatched seed
   values fail within 10 simulated seconds; tracked base parameters remain
   unchanged. It takes off in GUIDED, settles in LOITER for two seconds, then
   commands and observes ALT_HOLD. It tunes all axes, reactivates the tuned
   gains, and settles again.
   Neutral overrides remain active through ALT_HOLD entry and tuning. It then
   returns in GUIDED to the launch zone at 5 m before native LAND. The return
   must reach within 0.5 m and remain steady for two seconds;
   `autotune_return_target` and `autotune_return_arrived` identify it in the logs.
   The public/warmup windows total 990
   simulated seconds at target RTF 0.25, about 66 wall minutes plus startup.
   The final 60 public seconds remain reserved for LAND or failure recovery;
   calibration stops tuning at public 840 seconds. The overall host limit is
   10,800 seconds (three hours), allowing for the measured workstation slowdown.
   This allowance leaves the simulated windows and complete-recording requirement
   unchanged; finalization still has its separate 300-second budget.
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

Individual mission failures, including template preparation errors, do not skip
later independent cases. A preparation failure has no allocated run or recording;
its cause appears in that case's report row. Failed gates,
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

For Comp2026 ordinary landing, also inspect the durable parameter transaction:

```bash
rg -n 'landing_profile_transition' "runs/$RUN_ID"
```

The expected order is a requested and verified fresh read for every approved
profile parameter, requested and verified `PLND_ENABLED=0`, LAND/touchdown
evidence, then requested and verified `PLND_ENABLED=1`. A failed event must have
`detail`. Its generation may be absent only when the request or write never
reached the guarded enqueue boundary. Event timestamps are public
mission-relative nanoseconds; add the admitted native epoch before correlating
them with DataFlash. Keep the live PLND switching proof pending until the Task 7
native QGC flight captures both lifecycle events and genuine FC evidence.

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

### Acceptance-only precision-target-loss stimulus

Use this helper only for the Task 7 negative acceptance run prepared with the
`stationary-pad` preset. Start that run from its generated action in native QGC.
Wait until QGC shows the mission's own transition to LAND, then run:

```bash
RUN_ID=replace-with-run-uuid
uv run --locked python scripts/induce_precision_target_loss.py "$PWD/runs/$RUN_ID"
```

The helper fails closed unless the immutable configuration and native-QGC Start
admission match, the host lifecycle is still `RUNNING`, and the owned
`gazebo-runtime` and `companion-runtime` containers are bound to the run. It
reads the current configured `precision_land` operation from a bounded tail of
the live companion container log because `logs/companion.jsonl` is published
only during finalization. The active operation must emit a
`precision_mode_requested` LAND event after its command enqueue succeeds. This
event attributes the request to that operation but does not claim an FC ACK.
The latest live DataFlash mode must then show LAND strictly after the mission
event and no more than 5 native seconds before the current odometry. The latest
DataFlash `PL.TAcq=1` row and `PL.LastMeasMS` must place acquisition within 0.5
native seconds of current odometry. The helper also requires downward clearance
greater than 0.75 m. An active BIN file that has not flushed those rows within
the bounds causes a refusal; do not infer readiness from an older row.

The helper binds the accepted Start to its native QGC intervention row and
rejects any later QGC takeover evidence before or during the stimulus. Do not
click native QGC LAND, GUIDED, Pause, Abort, or another action while the helper
runs. Invalid, finalizing, or finalized bundles are rejected without creating
or changing any file.

One invocation reserves
`logs/docker/precision-loss-stimulus.jsonl` without overwrite while holding the
same `.control` directory lock used to request finalization. The lock covers the
final eligibility and freshness checks, physical command, bounded native
observations, and fsynced evidence appends. It discovers
`/world/moving_pad_stationary/set_pose` on the derived run partition and asks
Gazebo's `gz.msgs.Pose` to move the physical `moving_pad` entity from x=35 m to
x=55 m with y=z=0 and identity orientation. Before dispatch it fsyncs an
`attempted` JSONL row with the exact request. It then appends a returned response
or an explicit `unknown` outcome for a timeout or interruption, followed by the
terminal result when known. Success requires a true `gz.msgs.Boolean` reply and
strictly increasing native odometry showing the approximately 20 m displacement
for more than 0.5 simulated seconds. A failure or unknown outcome remains in the
log and blocks another invocation for that run; prepare a fresh run rather than
hiding or retrying a physical action.

This record establishes the bounded physical stimulus and its immediate native
observations. The later `--expect precision-loss` inspection must still prove
acquisition then loss in DataFlash, the policy-owned GUIDED request and exact
failure reason, unchanged `PLND_ENABLED=1` and `PLND_STRICT=2`, the rendered
marker's disappearance on video, matching public pad motion in the rosbag,
native-state corroboration, manifest hashing, and cleanup.

## Test without a full flight

```bash
# Quick cross-platform CLI sanity.
uv run --locked pytest orchestration/tests/test_cli.py -q

# Complete supported source suite, including Linux filesystem behavior.
scripts/test-linux.sh orchestration/tests artifacts/tests companion/tests \
  electromagnet/tests gazebo/tests ardupilot_sitl/tests scorekeeper/tests tests -q

# Configured typing and browser behavior.
uv run --locked mypy
uv run --with playwright python tests/browser/check_console_views.py
```

Run imported Comp2026 tests in a separate process so their module namespace does
not collide with the top-level companion tests. The exact current command and
result are retained in the
[portable-console verification record](verification/2026-10-10-portable-local-console.md#supported-software-gates).

These commands do not establish real flight behavior. For infrastructure
integration (these commands build/run Docker containers and are slower):

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
