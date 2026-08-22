# Phase 1 Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and verify the shared ROS 2 contracts, immutable run configuration, lifecycle state machine, structured logging, manifest builder, and a synthetic Dockerized foundation run.

**Architecture:** ROS message definitions live in one build-time package under `ros_ws/src/simulation_interfaces`. Pure Python domain logic stays inside the owning `orchestration` and `artifacts` modules, with ROS and Docker treated as adapters. A synthetic ROS container proves discovery, `/clock`, run state, and structured logging before Gazebo is introduced.

**Tech Stack:** Ubuntu 24.04 LTS, ROS 2 Jazzy, Python 3.12, pytest, JSON Schema Draft 2020-12, Docker Compose v2, Fast DDS

**Spec:** `docs/superpowers/specs/2026-08-22-runnable-simulation-design.md`

## Global Constraints

- Gazebo remains authoritative for physical truth and future `/clock`; the Phase 1 synthetic clock is test-only.
- Simulation-relevant data carries `run_id` and a simulation timestamp.
- Wall time is restricted to infrastructure deadlines, diagnostics, and artifact finalization.
- Every process emits structured JSON Lines with `run_id`, `module`, `severity`, `event`, `sim_timestamp`, and `wall_timestamp`.
- Lifecycle states are `CREATED`, `STARTING`, `READY`, `RUNNING`, `FINALIZING`, `COMPLETED`, `FAILED`, and `ABORTED`.
- Completed, failed, and aborted runs all preserve a manifest; only a validated full artifact set may be completed in later phases.
- Use ROS 2 Jazzy defaults with Fast DDS.
- Do not copy generated legacy build, log, or recording output.
- Do not modify or push `companion/comp2026` in this phase.
- Preserve the user's existing uncommitted changes in `companion/PLAN.md` and `ardupilot_sitl/PLAN.md`.

## File Structure

```text
config/
├── default-run.json
└── run.schema.json
ros_ws/src/simulation_interfaces/
├── CMakeLists.txt
├── package.xml
└── msg/
    ├── ArtifactStatus.msg
    ├── FrameMetadata.msg
    ├── GroundTruth.msg
    ├── RunState.msg
    ├── ScenarioEvent.msg
    └── ScoreEvent.msg
orchestration/
├── pyproject.toml
├── src/orchestration/{__init__,config,lifecycle}.py
└── tests/{test_config,test_lifecycle}.py
artifacts/
├── pyproject.toml
├── schemas/manifest.schema.json
├── src/artifacts/{__init__,manifest,structured_log}.py
└── tests/{test_manifest,test_structured_log}.py
tests/
├── contracts/test_ros_interfaces.py
├── foundation/Dockerfile
├── foundation/foundation_node.py
└── integration/test_foundation_compose.py
compose.yaml
Makefile
.gitignore
pyproject.toml
uv.lock
```

---

### Task 1: Shared ROS 2 Interface Package

**Files:**
- Create: `ros_ws/src/simulation_interfaces/CMakeLists.txt`
- Create: `ros_ws/src/simulation_interfaces/package.xml`
- Create: `ros_ws/src/simulation_interfaces/msg/RunState.msg`
- Create: `ros_ws/src/simulation_interfaces/msg/FrameMetadata.msg`
- Create: `ros_ws/src/simulation_interfaces/msg/GroundTruth.msg`
- Create: `ros_ws/src/simulation_interfaces/msg/ScenarioEvent.msg`
- Create: `ros_ws/src/simulation_interfaces/msg/ScoreEvent.msg`
- Create: `ros_ws/src/simulation_interfaces/msg/ArtifactStatus.msg`
- Create: `tests/contracts/test_ros_interfaces.py`
- Create: `pyproject.toml`
- Create: `uv.lock`

**Interfaces:**
- Consumes: ROS 2 `builtin_interfaces`, `geometry_msgs`, and the approved topic/QoS contract
- Produces: package `simulation_interfaces` and the six message types imported by later modules

- [ ] **Step 1: Create the development test environment and contract test**

Create the root `pyproject.toml` with project name `drone-sim-dev`, Python requirement `>=3.12`, runtime dependency `jsonschema>=4.23,<5`, development dependency `pytest>=8.3,<9`, and pytest `pythonpath` entries `orchestration/src` and `artifacts/src`. Run `uv lock` and commit the generated lockfile.

Create a parameterized pytest that parses each `.msg` file as text and asserts exact required declarations:

```python
from pathlib import Path

ROOT = Path(__file__).parents[2]
MSG = ROOT / "ros_ws/src/simulation_interfaces/msg"

EXPECTED = {
    "RunState.msg": ["string run_id", "builtin_interfaces/Time sim_timestamp", "uint8 state", "string reason", "string config_sha256"],
    "FrameMetadata.msg": ["string run_id", "builtin_interfaces/Time sim_timestamp", "uint64 frame_id", "string stream"],
    "GroundTruth.msg": ["string run_id", "builtin_interfaces/Time sim_timestamp", "string vehicle_id", "geometry_msgs/Pose pose", "geometry_msgs/Twist twist", "bool in_contact"],
    "ScenarioEvent.msg": ["string run_id", "builtin_interfaces/Time sim_timestamp", "uint64 event_id", "string magnet_id", "string state"],
    "ScoreEvent.msg": ["string run_id", "builtin_interfaces/Time sim_timestamp", "uint64 event_id", "string event_type", "float64 value", "string evidence_ref"],
    "ArtifactStatus.msg": ["string run_id", "builtin_interfaces/Time sim_timestamp", "bool ready", "bool complete", "string[] missing", "string manifest_path"],
}

def test_message_contracts_are_exact():
    for filename, declarations in EXPECTED.items():
        text = (MSG / filename).read_text()
        for declaration in declarations:
            assert declaration in text, f"{filename} missing {declaration}"
```

Also assert that `RunState.msg` declares numeric constants `CREATED=0` through `ABORTED=7` in lifecycle order.

- [ ] **Step 2: Run the test and observe the missing package failure**

Run: `uv run pytest tests/contracts/test_ros_interfaces.py -v`

Expected: failure because `ros_ws/src/simulation_interfaces/msg` does not exist.

- [ ] **Step 3: Implement the message and build metadata files**

Use the exact declarations from Step 1. Add the eight `uint8` constants to `RunState.msg`. Configure `rosidl_generate_interfaces` for all six messages with dependencies `builtin_interfaces` and `geometry_msgs`; export `rosidl_default_runtime` from `package.xml`.

- [ ] **Step 4: Verify the source contract and ROS build**

Run:

```bash
uv run pytest tests/contracts/test_ros_interfaces.py -v
docker run --rm -v "$PWD/ros_ws:/workspace/ros_ws" -w /workspace/ros_ws ros:jazzy-ros-base \
  bash -lc 'apt-get update >/dev/null && apt-get install -y python3-colcon-common-extensions >/dev/null && source /opt/ros/jazzy/setup.bash && colcon build --packages-select simulation_interfaces'
```

Expected: pytest passes and `colcon build` exits zero.

- [ ] **Step 5: Commit the interface package**

```bash
git add pyproject.toml uv.lock ros_ws/src/simulation_interfaces tests/contracts/test_ros_interfaces.py
git commit -m "feat: define shared simulation ROS interfaces"
```

### Task 2: Immutable Run Configuration and Lifecycle

**Files:**
- Create: `config/run.schema.json`
- Create: `config/default-run.json`
- Create: `orchestration/pyproject.toml`
- Create: `orchestration/src/orchestration/__init__.py`
- Create: `orchestration/src/orchestration/config.py`
- Create: `orchestration/src/orchestration/lifecycle.py`
- Create: `orchestration/tests/test_config.py`
- Create: `orchestration/tests/test_lifecycle.py`

**Interfaces:**
- Consumes: lifecycle names and run-bundle requirements from the design spec
- Produces: `RunConfig`, `load_run_config(path)`, `RunLifecycle`, `LifecycleState`, `LifecycleEvent`, and `InvalidTransition`

- [ ] **Step 1: Write configuration tests**

Test with `jsonschema.Draft202012Validator.check_schema` that `config/run.schema.json` is a valid schema, validate `default-run.json`, and test that `load_run_config` returns a frozen `RunConfig` with fields:

```python
run_id: str
world: str
vehicle: str
mission: str
scenario: str
output_root: pathlib.Path
max_wall_seconds: int
config_sha256: str
```

Use a valid UUID, require `max_wall_seconds >= 1`, reject unknown keys, and verify `config_sha256` is the SHA-256 of canonical JSON encoded with `sort_keys=True` and separators `(',', ':')`.

- [ ] **Step 2: Write lifecycle tests**

Test this exact success path:

```python
lifecycle = RunLifecycle.created(run_id="00000000-0000-4000-8000-000000000001")
lifecycle = lifecycle.apply(LifecycleEvent.START)
lifecycle = lifecycle.apply(LifecycleEvent.MODULES_READY)
lifecycle = lifecycle.apply(LifecycleEvent.CLOCK_STARTED)
lifecycle = lifecycle.apply(LifecycleEvent.COMPLETE, reason="mission_complete")
assert lifecycle.state is LifecycleState.FINALIZING
assert lifecycle.pending_terminal is LifecycleState.COMPLETED
lifecycle = lifecycle.apply(LifecycleEvent.ARTIFACTS_FINALIZED)
assert lifecycle.state is LifecycleState.COMPLETED
```

Add cases for failure and abort from `STARTING`, `READY`, and `RUNNING`; finalization failure yielding `FAILED`; and invalid transition errors containing current state and event.

- [ ] **Step 3: Run both test files and observe import failures**

Run: `uv run pytest orchestration/tests -v`

Expected: collection fails because package `orchestration` is absent.

- [ ] **Step 4: Implement the minimal configuration module**

Use frozen dataclasses, `uuid.UUID`, `json`, and `hashlib` from the standard library. Reject non-object JSON, missing/extra keys, invalid UUIDs, empty string fields, non-integer timeouts, and timeouts below one. Store a resolved `Path` for `output_root` without creating it.

Create `config/run.schema.json` using JSON Schema Draft 2020-12 with `additionalProperties: false` and the same constraints. Create `config/default-run.json` with UUID `00000000-0000-4000-8000-000000000001`, world `competition`, vehicle `iris`, mission `descent`, scenario `maximum_score`, output root `runs`, and wall timeout `3600`.

- [ ] **Step 5: Implement the immutable lifecycle state machine**

Use `Enum` values with the exact uppercase state/event names. `RunLifecycle` is frozen and returns a new value from `apply`. Supported events are `START`, `MODULES_READY`, `CLOCK_STARTED`, `COMPLETE`, `FAIL`, `ABORT`, `ARTIFACTS_FINALIZED`, and `FINALIZATION_FAILED`. Terminal states reject every event.

- [ ] **Step 6: Verify configuration and lifecycle**

Run: `uv run pytest orchestration/tests -v`

Expected: all tests pass.

- [ ] **Step 7: Commit orchestration domain logic**

```bash
git add config orchestration/pyproject.toml orchestration/src orchestration/tests
git commit -m "feat: add run configuration and lifecycle"
```

### Task 3: Structured Logging and Manifest Domain

**Files:**
- Create: `artifacts/pyproject.toml`
- Create: `artifacts/schemas/manifest.schema.json`
- Create: `artifacts/src/artifacts/__init__.py`
- Create: `artifacts/src/artifacts/structured_log.py`
- Create: `artifacts/src/artifacts/manifest.py`
- Create: `artifacts/tests/test_structured_log.py`
- Create: `artifacts/tests/test_manifest.py`

**Interfaces:**
- Consumes: `run_id`, lifecycle terminal states, and required run-bundle paths
- Produces: `StructuredEvent`, `write_event(stream, event)`, `ArtifactRecord`, `RunManifest`, `build_manifest`, and `write_manifest_atomic`

- [ ] **Step 1: Write structured-log tests**

Construct `StructuredEvent` with a timezone-aware wall timestamp and optional decimal simulation seconds. Assert `to_json_line()` contains exactly one compact JSON object followed by `\n`, includes the six common fields, merges event-specific fields under `fields`, rejects naive wall timestamps, and rejects collisions with common field names.

- [ ] **Step 2: Write manifest tests**

Use a temporary run directory with representative files. Assert:

- SHA-256, relative path, byte size, and validation status are recorded;
- required paths are classified as present, missing, or invalid;
- a completed manifest is rejected when any required artifact is absent;
- failed and aborted manifests retain missing-artifact records;
- `write_manifest_atomic` leaves valid JSON at `manifest.json` and no temporary sibling;
- achieved score, maximum available score, scoring checksum, and evidence paths round-trip.

- [ ] **Step 3: Run tests and observe import failures**

Run: `uv run pytest artifacts/tests -v`

Expected: collection fails because package `artifacts` is absent.

- [ ] **Step 4: Implement structured logging**

Use frozen dataclasses and standard-library JSON. Serialize simulation seconds as a JSON number or `null`, wall timestamps as UTC ISO 8601 with `Z`, stable key order, compact separators, and UTF-8. `write_event` writes and flushes one complete line.

- [ ] **Step 5: Implement manifest domain and schema**

Required artifact categories are configuration, Gazebo server log, Gazebo state, onboard MP4, observer MP4, ROS bag, all seven module logs, score events, and score result. Use terminal status strings `COMPLETED`, `FAILED`, and `ABORTED`. Compute checksums by streaming 1 MiB chunks. Write to `manifest.json.tmp`, flush and `fsync`, then replace `manifest.json` atomically.

- [ ] **Step 6: Verify artifacts domain**

Run: `uv run pytest artifacts/tests -v`

Expected: all tests pass.

- [ ] **Step 7: Commit artifact domain logic**

```bash
git add artifacts/pyproject.toml artifacts/schemas artifacts/src artifacts/tests
git commit -m "feat: add structured logs and run manifests"
```

### Task 4: Synthetic ROS Foundation Container

**Files:**
- Create: `tests/foundation/Dockerfile`
- Create: `tests/foundation/foundation_node.py`
- Create: `tests/foundation/entrypoint.sh`
- Create: `compose.yaml`
- Create: `.gitignore`
- Create: `Makefile`
- Create: `tests/integration/test_foundation_compose.py`

**Interfaces:**
- Consumes: built `simulation_interfaces`, `/simulation/run_state`, standard `/clock`, and structured JSON Lines
- Produces: Compose service `foundation`, health file `/run/foundation/ready`, synthetic clock messages, run-state messages, and captured JSONL

- [ ] **Step 1: Write the integration test**

The test creates a temporary output directory and uses run ID `00000000-0000-4000-8000-000000000099`, then invokes this Python subprocess argument list with the temporary directory converted to an absolute string:

```python
[
    "docker", "compose", "run", "--rm",
    "-e", "SIM_RUN_ID=00000000-0000-4000-8000-000000000099",
    "-e", "SIM_OUTPUT_ROOT=/output",
    "-v", f"{output_dir.resolve()}:/output",
    "foundation",
]
```

Assert exit zero, `logs/foundation.jsonl` exists, every line parses as JSON with the common fields, the events appear in order `starting`, `ready`, `clock_started`, `finalizing`, and the last event has a non-null simulation timestamp.

- [ ] **Step 2: Run the integration test and observe the missing Compose service failure**

Run: `uv run pytest tests/integration/test_foundation_compose.py -v`

Expected: failure because `compose.yaml` or service `foundation` is absent.

- [ ] **Step 3: Implement the synthetic foundation node**

Use `rclpy` to publish `/clock` with best-effort depth 1 and `/simulation/run_state` with reliable, transient-local depth 1. Publish deterministic simulated times `0.0`, `0.05`, and `0.10`; never sleep to determine those values. Emit the five JSONL events to stdout and `/output/logs/foundation.jsonl`. Exit after flushing the final event.

- [ ] **Step 4: Implement the container and Compose service**

Build from `ros:jazzy-ros-base`, install `python3-colcon-common-extensions`, copy and build `simulation_interfaces`, copy `artifacts/src` plus the node, and use an entrypoint that sources ROS, the built workspace, and adds the artifacts source directory to `PYTHONPATH`. Configure Compose with `init: true`, the run/output environment variables, and no host networking.

- [ ] **Step 5: Add developer commands and ignore rules**

`Makefile` targets:

```make
test-unit:
	uv run pytest orchestration/tests artifacts/tests tests/contracts -v

test-foundation:
	uv run pytest tests/integration/test_foundation_compose.py -v

test: test-unit test-foundation
```

Ignore Python caches, `.pytest_cache`, ROS `build/install/log`, `runs/`, and `.superpowers/` while retaining all source and schemas.

- [ ] **Step 6: Verify the Dockerized foundation**

Run:

```bash
docker compose build foundation
uv run pytest tests/integration/test_foundation_compose.py -v
docker compose config --quiet
```

Expected: image builds, integration test passes, and Compose config validates.

- [ ] **Step 7: Commit the foundation container**

```bash
git add tests/foundation tests/integration/test_foundation_compose.py compose.yaml Makefile .gitignore
git commit -m "test: add synthetic ROS foundation stack"
```

### Task 5: Phase 1 Integration Gate and Contract Alignment

**Files:**
- Modify if behavior differs: root and affected module `PLAN.md`, `INTERNAL_INTERFACE.md`, and `EXTERNAL_INTERFACE.md`
- Create: `docs/verification/phase-1-foundation.md`

**Interfaces:**
- Consumes: all Phase 1 packages, tests, Compose service, and approved documentation contracts
- Produces: evidence that Phase 2 can rely on fixed lifecycle, schema, logging, and ROS interfaces

- [ ] **Step 1: Run the complete Phase 1 test surface**

Run:

```bash
make test
docker compose config --quiet
git diff --check
```

Expected: every command exits zero.

- [ ] **Step 2: Verify exact interface alignment**

Compare generated ROS interface names, lifecycle states, common log fields, required artifact categories, topic names, and QoS against the runnable-simulation design and every affected interface document. Correct documentation only when implementation matches the approved spec; correct implementation when it diverges.

- [ ] **Step 3: Record evidence**

Create `docs/verification/phase-1-foundation.md` containing the tested commit, UTC verification time, exact commands, exit codes, test counts, built image digest, lifecycle transition coverage, and the statement that Gazebo, ArduPilot, companion, electromagnet, and scoring behavior are not yet claimed by this gate.

- [ ] **Step 4: Verify repository scope**

Run:

```bash
git status --short
git -C companion/comp2026 status --short
```

Expected: only intentional Phase 1 changes appear in the outer repository, and no Phase 1 task changed the nested companion repository.

- [ ] **Step 5: Commit the Phase 1 gate**

```bash
git add docs/verification/phase-1-foundation.md PLAN.md INTERNAL_INTERFACE.md EXTERNAL_INTERFACE.md orchestration artifacts
git diff --cached --quiet || git commit -m "docs: record phase 1 foundation gate"
```
