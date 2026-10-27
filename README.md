# Drone Simulation

Dockerized drone competition simulation built with ROS 2, Gazebo, ArduPilot SITL, and a companion mission controller.

The mission delivers three payloads, then returns home. Gazebo
provides physical truth; a separate scorer evaluates it; recordings and a
validated run bundle establish what actually happened.

## Start here

1. [Architecture and code map](docs/architecture.md): what each component owns,
   how one mission flows through the system, and where to make a change.
2. [Operator and developer runbook](docs/runbook.md): prerequisites, builds,
   running, finding videos, stopping safely, and testing.
3. [Human handoff / current status](docs/handoff.md): known failures, evidence,
   unfinished work, and the next useful tasks.

**Verification scope:** the current QGC control implementation has automated
test and matching image-build evidence. Native QGC command routing and flight
acceptance remain pending. Earlier configured-descent, calibration, and reload
results apply to their recorded revisions. See [current status](docs/handoff.md)
for exact evidence and remaining checks. The dated
[portable-console verification record](docs/verification/2026-10-10-portable-local-console.md)
separates software, native workstation, and pending platform results.

## Platform support

The console resolves macOS, Ubuntu desktop, or Ubuntu 24.04 under WSL2. Use Docker
Desktop on macOS and Windows, or Docker Engine on native Ubuntu. It uses
native QGroundControl 5.0.8 and native `linux/arm64` or `linux/amd64` container
images. Windows runs the Python controller inside WSL; it does not run a native
Windows controller.

| Target | Source-level status | Native acceptance |
| --- | --- | --- |
| macOS supported by Docker Desktop, Apple Silicon | Implemented; native ARM64 images | Previous Colima checks are historical; see the handoff for Docker Desktop verification |
| macOS supported by Docker Desktop, Intel | Implemented | Pending on Intel hardware |
| Ubuntu 24.04 desktop, ARM64 or x86-64 | Implemented | Desktop QGC and runtime acceptance pending |
| Windows 11 x86-64, Ubuntu 24.04 in WSL2 | Implemented, including the bounded Windows helper | Windows PowerShell 5.1, NTFS, UDP, NAT, and mirrored-network acceptance pending |

Windows ARM64 and emulated or remote container runtimes are outside the initial
matrix. The [runbook](docs/runbook.md#local-mac-console) gives exact setup and
recovery steps. The [handoff](docs/handoff.md) keeps native results separate
from software tests and historical flights.

Docker Desktop manages the Linux VM on macOS; Colima and Lima are no longer
prerequisites. QGC, the browser, and the Python console still run on the host.

The Desktop host migration is complete. Ubuntu package locks have been refreshed
for fresh machines. Check the
[current build status](docs/handoff.md#docker-desktop-migration--2026-10-26)
before you prepare a run.

## Local flight console

From this checkout, after completing the platform setup in the runbook:

```bash
uv sync --python 3.12 --locked
./scripts/local-build.sh --build
./scripts/local-build.sh --check
uv run --locked drone-sim console
```

The console opens at `http://127.0.0.1:8765`. Build matching images when prompted,
choose a world and mission preset, then click **Prepare simulation**. Inspect
native QGC, the interactive Gazebo desktop, and the onboard camera. Prepare
publishes a run-named QGC action file and shows its exact path. In QGC, select
that file under **Application Settings > Fly View > MAVLink Actions > Fly View
Actions**, then use **Start selected mission** or **Abort and recover**.
Preparation keeps the aircraft disarmed and does not consume the mission budget.

**Stop simulation** finalizes and stops the runtime; it does not request a
landing. **Run again** waits for cleanup and prepares a fresh run, token, and
action file. See the [console workflow](docs/runbook.md#local-mac-console) for
macOS, Ubuntu, and WSL2 prerequisites, Operator Wait guidance, CLI controls,
recovery, and evidence paths.

## First local check

From the repository root, with Python 3.12+ and `uv` available:

```bash
uv sync --locked
uv run --locked drone-sim --help
uv run --locked pytest orchestration/tests/test_cli.py -q
```

This is a quick cross-platform CLI check. It does **not** launch a flight or
prove ROS/Gazebo integration. Run the complete supported software suite through
the Linux harness in the [runbook](docs/runbook.md#test-without-a-full-flight),
which supplies the Linux descriptor and FFmpeg behavior that macOS lacks.

Once the runtime images are ready, the normal flight command is:

```bash
uv run --locked drone-sim start --config config/default-run.json
```

It runs in the foreground and can take tens of minutes or longer. It does not
build images automatically. Do not start with a bare `docker compose up`.

## How to read this repository

- `config/` selects a run; `compose.yaml` connects its seven runtime services.
- Each of the seven modules has a concise README linked from the
  [module map](docs/architecture.md#where-to-read-or-change-code), covering its
  responsibilities, interfaces, entry points, tests, and constraints.
- `runs/<run_id>/` contains local generated evidence and is ignored by Git.
- Superseded plans and interface documents live in Git history, not the active
  documentation path. Dated verification notes remain evidence, not current specs.

## Contributing or adding a mission

Develop Comp 2026 flight behavior in [`companion/comp2026`](companion/comp2026).
It is tracked source in this repository, with its earlier Git history preserved.
Commit those edits on your `drone_sim` feature branch. See the
[flight-code map](companion/README.md#where-to-change-comp-2026-code) for the
active FM1, FM2, and FM3 entry points, shared controls, and simulation adapters.
Use the [QGC development loop](docs/runbook.md#develop-comp-2026-with-qgc) to test
and rebuild before trying the change in a new run.

For a fixed sequence, copy [configured-descent-run.json](config/configured-descent-run.json)
and edit its `mission_plan.steps`. The [companion guide](companion/README.md#configured-diagnostic-missions)
defines the tools; the [local workflow](docs/runbook.md#local-developer-workflow)
covers rebuilding changed code, running one template, and running the automatic set.

[configured-moving-pad-run.json](config/configured-moving-pad-run.json) adds a
camera-guided landing on a deck traveling at 0.5 m/s. The stock EKF3 profile
completed two independently accepted moving landings with 100/100 and full
90-second recordings. The latest stationary control also passed. An earlier
intermittent contact-stream fault did not recur, but remains unresolved. Use the
[moving-pad workflow](docs/runbook.md#moving-pad-landing) for the stationary
control, moving flight, and independent acceptance. Current verification is
recorded in [handoff](docs/handoff.md).

[autotune-run.json](config/autotune-run.json) tunes roll, pitch and yaw on the
shared unloaded aircraft. The [calibration workflow](docs/runbook.md#calibrate-and-validate-saved-gains)
then loads accepted gains into fresh SITL for a hover/landing validation.
Run every setup with `uv run --locked drone-sim suite --config config/ci-suite.json`.
The [local suite workflow](docs/runbook.md#run-the-full-mission-suite) covers
calibration gates, reports and recordings. Its current end-to-end verification
and remaining provider setup are recorded in [handoff](docs/handoff.md).

Read [AGENTS.md](AGENTS.md) for contribution rules, then the affected module
README. For a mission, start with [companion](companion/README.md): define its
behavior and success/failure conditions, implement and register it through the
existing selector, add a run template and focused tests, then follow the
[runbook](docs/runbook.md) to build and verify. Competition mission edits may
belong under `companion/comp2026`; read its local documentation before editing.

Update every affected module README and shared guide **in the same change**.
Report documentation impact (or why none is needed), tests, and any unverified
runtime behavior when handing off.
