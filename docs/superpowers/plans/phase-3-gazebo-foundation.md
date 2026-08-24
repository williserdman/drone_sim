# Phase 3 Gazebo Physical Foundation Implementation Plan

> **Execution pivot (2026-08-24):** Tasks 1–5 are complete through `416a180`. Tasks 6–8 are no longer executed as a separate passive-physics phase; their launch-critical work is incorporated into `docs/superpowers/plans/runnable-vertical-descent.md`. Historical constraints and task detail below remain design input.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the Phase 2 synthetic physical source with a deterministic, paused-first Gazebo Harmonic runtime that produces authoritative simulation time, two 20-sim-Hz camera streams, ground truth, native state, and server logs in the existing validated run bundle.

**Architecture:** One `gazebo-runtime` Compose service supervises a headless Gazebo Sim 8 server, private unidirectional Gazebo-to-ROS bridges, and a thin public ROS adapter. Gazebo Transport stays private; the existing ROS 2 and durable-file lifecycle contracts remain the module boundary. Phase 3 retains controlled test doubles for companion, ArduPilot, electromagnet, and scorekeeper and makes no flight or scoring claim.

**Tech Stack:** Docker Compose, ROS 2 Jazzy, Gazebo Harmonic / Gazebo Sim 8, `ros_gz` 1.0.22, Python 3.12, `rclpy`, SDF, pytest, rosbag2 MCAP, FFmpeg

**Spec:** `docs/superpowers/specs/2026-08-24-phase-3-gazebo-foundation-design.md`

## Global Constraints

- Gazebo is the only producer of physical truth and simulation time.
- The production Gazebo runtime never consumes `/simulation/camera_pair_ack`.
- Public camera topics remain `320x240`, `rgb8`, reliable, volatile, depth 5, at exactly 20 frames per simulated second.
- `/clock` is unidirectional Gazebo-to-ROS with best-effort, volatile, depth 1 QoS.
- Ground truth is world-frame ENU and uses best-effort, volatile, depth 10 QoS.
- The server starts paused and no simulation sample is released before artifacts and Gazebo readiness.
- Run reset means a fresh container, server, Compose project, and `GZ_PARTITION`; never reuse an in-process world across run IDs.
- Preserve the fixed ten-topic bag, two MP4s, seven structured module logs, manifest authorities, deadlines, and completed/failed/aborted bundle behavior.
- Phase 3 must not add ArduPilot, MAVLink, actuation, mission, electromagnet physics, competition course, or real scoring behavior.
- Build from `ros:jazzy-ros-base@sha256:2589a8fba5257307857890173c069852c2abf913a0be7970f172478baecb09e4` and verify a committed apt package delta.
- Import only the reviewed Iris files from `ArduPilot/ardupilot_gazebo` commit `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`, with exact license and provenance.
- Do not modify or push `companion/comp2026` in this phase.
- Keep the existing untracked nested `companion/comp2026/` repository out of every commit.

---

### Task 1: Phase 3 configuration, topology, and interface contracts

**Files:**
- Modify: `config/default-run.json`
- Modify: `config/run.schema.json`
- Modify: `config/run-template.schema.json`
- Modify: `orchestration/src/orchestration/config.py`
- Modify: `orchestration/src/orchestration/_adapters/compose.py`
- Modify: `orchestration/src/orchestration/controller.py`
- Modify: `orchestration/tests/test_config.py`
- Modify: `orchestration/tests/test_controller.py`
- Modify: `EXTERNAL_INTERFACE.md`
- Modify: `INTERNAL_INTERFACE.md`
- Modify: `gazebo/EXTERNAL_INTERFACE.md`
- Modify: `gazebo/INTERNAL_INTERFACE.md`
- Modify: `gazebo/PLAN.md`
- Modify: `orchestration/EXTERNAL_INTERFACE.md`
- Modify: `orchestration/INTERNAL_INTERFACE.md`

**Interfaces:**
- Consumes: existing `RunConfig`, `ComposeRuntime`, seven-owner service checks, Phase 2 default profile behavior.
- Produces: `SimulationConfig(seed: int, duration_ns: int, target_real_time_factor: float)`, `RunConfig.runtime_profile`, `RunConfig.expected_camera_frames`, and an immutable profile-to-service ownership map used by every Compose operation.

- [ ] **Step 1: Write failing schema and parser tests**

Add tests proving that omission resolves to the Phase 2 regression profile, the repository default resolves to Phase 3, and invalid seed, duration, target RTF, or non-integral frame counts fail before Compose construction:

```python
def test_phase3_simulation_config_derives_exact_frame_count(tmp_path):
    value = json.loads(DEFAULT_TEMPLATE.read_text(encoding="utf-8"))
    value["runtime_profile"] = "phase3"
    value["simulation"] = {
        "seed": 7,
        "duration_sim_seconds": 2.0,
        "target_real_time_factor": 0.1,
    }
    path = tmp_path / "phase3.json"
    path.write_text(json.dumps(value), encoding="utf-8")
    config = resolve_run_config(path, run_id_factory=lambda: FIXED_RUN_ID)
    assert config.runtime_profile == "phase3"
    assert config.simulation.seed == 7
    assert config.simulation.duration_ns == 2_000_000_000
    assert config.simulation.target_real_time_factor == 0.1
    assert config.expected_camera_frames == 40


@pytest.mark.parametrize(
    "simulation",
    [
        {"seed": -1, "duration_sim_seconds": 2.0, "target_real_time_factor": 0.1},
        {"seed": 2**32, "duration_sim_seconds": 2.0, "target_real_time_factor": 0.1},
        {"seed": 1, "duration_sim_seconds": 0, "target_real_time_factor": 0.1},
        {"seed": 1, "duration_sim_seconds": 0.075, "target_real_time_factor": 0.1},
        {"seed": 1, "duration_sim_seconds": 2.0, "target_real_time_factor": 0},
        {"seed": 1, "duration_sim_seconds": 2.0, "target_real_time_factor": 1.0},
    ],
)
def test_invalid_phase3_timing_is_rejected_before_compose(tmp_path, simulation):
    value = json.loads(DEFAULT_TEMPLATE.read_text(encoding="utf-8"))
    value.update(runtime_profile="phase3", simulation=simulation)
    path = tmp_path / "invalid-phase3.json"
    path.write_text(json.dumps(value), encoding="utf-8")
    with pytest.raises(ValueError):
        resolve_run_config(path, run_id_factory=lambda: FIXED_RUN_ID)
```

- [ ] **Step 2: Run the focused tests and verify red**

Run: `uv run pytest orchestration/tests/test_config.py orchestration/tests/test_controller.py -q`

Expected: failures show that `runtime_profile`, `simulation`, and profile-specific service ownership do not exist.

- [ ] **Step 3: Implement exact configuration types and validation**

Add immutable values in `config.py` and derive nanoseconds through `Decimal(str(value))`, never binary-float multiplication:

```python
CAMERA_INTERVAL_NS = 50_000_000


@dataclass(frozen=True)
class SimulationConfig:
    seed: int
    duration_ns: int
    target_real_time_factor: float

    @property
    def expected_camera_frames(self) -> int:
        return self.duration_ns // CAMERA_INTERVAL_NS


@dataclass(frozen=True)
class RuntimeTopology:
    profile: str
    ownership: tuple[tuple[str, str], ...]
```

The exact ownership maps are:

```python
PHASE2_OWNERSHIP = (
    ("orchestration-runtime", "orchestration"),
    ("artifacts-runtime", "artifacts"),
    ("synthetic-companion", "companion"),
    ("synthetic-ardupilot-sitl", "ardupilot_sitl"),
    ("synthetic-gazebo", "gazebo"),
    ("synthetic-electromagnet", "electromagnet"),
    ("synthetic-scorekeeper", "scorekeeper"),
)
PHASE3_OWNERSHIP = tuple(
    ("gazebo-runtime", module) if module == "gazebo" else (service, module)
    for service, module in PHASE2_OWNERSHIP
)
```

`ComposeRuntime` accepts `topology: RuntimeTopology`; it sets
`COMPOSE_PROFILES=topology.profile`, and controller health/log/image checks use
`topology.ownership`. No operation reads an ambient topology selector.

- [ ] **Step 4: Update schemas and the default configuration**

Add optional `runtime_profile` with enum `phase2|phase3` and conditional schema
rules: `phase3` requires `simulation`, while an omitted profile resolves to
`phase2` and rejects `simulation`. Require `target_real_time_factor` to equal
`0.1`, add the exact remaining `simulation` bounds, and retain
`additionalProperties: false`. Set `config/default-run.json` to:

```json
"world": "phase3_foundation",
"vehicle": "iris",
"mission": "physical_foundation",
"scenario": "passive_descent",
"runtime_profile": "phase3",
"simulation": {
  "seed": 1,
  "duration_sim_seconds": 2.0,
  "target_real_time_factor": 0.1
}
```

- [ ] **Step 5: Replace deferred interface decisions with the approved contract**

Document the public profile selector, simulation fields, fresh-server reset,
paused-first readiness, no production camera acknowledgement, exact public QoS,
private Gazebo Transport, Phase 3 exclusions, and directory API boundaries.
Remove the now-resolved Gazebo distribution/world/reset deferrals. Do not claim
ArduPilot lockstep is implemented.

- [ ] **Step 6: Run all affected unit tests**

Run: `uv run pytest orchestration/tests tests/contracts -q`

Expected: all tests pass; the Phase 2 profile still produces the original exact
seven-service map, and the Phase 3 profile differs only at the Gazebo service.

- [ ] **Step 7: Commit**

```bash
git add config orchestration EXTERNAL_INTERFACE.md INTERNAL_INTERFACE.md gazebo/EXTERNAL_INTERFACE.md gazebo/INTERNAL_INTERFACE.md gazebo/PLAN.md tests/contracts
git commit -m "feat: define phase 3 runtime contracts"
```

---

### Task 2: Pinned Gazebo image and reviewed asset provenance

**Files:**
- Create: `gazebo/Dockerfile`
- Create: `gazebo/harmonic-packages.lock`
- Create: `gazebo/pyproject.toml`
- Create: `gazebo/provenance/EXTERNAL_INTERFACE.md`
- Create: `gazebo/provenance/INTERNAL_INTERFACE.md`
- Create: `gazebo/provenance/LICENSE.ardupilot_gazebo.md`
- Create: `gazebo/provenance/ardupilot_gazebo-assets.json`
- Create: `gazebo/provenance/upstream/iris_with_standoffs/model.config`
- Create: `gazebo/provenance/upstream/iris_with_standoffs/model.sdf`
- Create: `gazebo/provenance/upstream/iris_with_standoffs/meshes/iris.dae`
- Create: `gazebo/provenance/upstream/iris_with_standoffs/meshes/iris_collision.stl`
- Create: `gazebo/provenance/upstream/iris_with_standoffs/meshes/iris_prop_ccw.dae`
- Create: `gazebo/provenance/upstream/iris_with_standoffs/meshes/iris_prop_cw.dae`
- Create: `gazebo/resources/models/iris_phase3/model.config`
- Create: `gazebo/resources/models/iris_phase3/model.sdf`
- Create: `gazebo/resources/models/iris_phase3/meshes/iris.dae`
- Create: `gazebo/resources/models/iris_phase3/meshes/iris_collision.stl`
- Create: `gazebo/resources/models/iris_phase3/meshes/iris_prop_ccw.dae`
- Create: `gazebo/resources/models/iris_phase3/meshes/iris_prop_cw.dae`
- Create: `gazebo/tests/test_provenance.py`
- Create: `gazebo/tests/test_image_contract.py`
- Modify: `pyproject.toml`
- Modify: `uv.lock`

**Interfaces:**
- Consumes: immutable ROS base digest and the approved upstream commit.
- Produces: build target `drone-sim-gazebo-runtime:phase3`, a verified apt delta, local-only resources, and a provenance manifest whose entries use `source_path`, `imported_path`, `sha256`, and `license`.

- [ ] **Step 1: Write failing provenance tests**

```python
EXPECTED = {
    "LICENSE.md": "1a45b1d0a8603dfe2cfc644f9dab970b1762f92babe2aac6eb2f5d4572c4a680",
    "models/iris_with_standoffs/model.config": "e419cc7681f730edf14398baa577a4f05cd34e2cebaa764c508119c4465b34b7",
    "models/iris_with_standoffs/model.sdf": "2c4e8ccf4385f329af905cce1cbceff96656da4ace9353c0ec6ca09df3c442d0",
    "models/iris_with_standoffs/meshes/iris.dae": "697956cfc0fe608afc52a0f688de50354b89e5201056451cf6bb767de8f46323",
    "models/iris_with_standoffs/meshes/iris_collision.stl": "66b85caa021497ea73dbaed08c72da381a6157116244ea3860a1ba445f50102c",
    "models/iris_with_standoffs/meshes/iris_prop_ccw.dae": "5f8b01668ee24a5b663ca8c4dd56e294fd6480db7eec1e5a7c11e45ba9004e99",
    "models/iris_with_standoffs/meshes/iris_prop_cw.dae": "6cbc686772dccd46253fb65ece000e7ffa6a74b9c097bda349884bd1e78cd879",
}


def test_imported_assets_match_pinned_upstream_bytes():
    manifest = json.loads(MANIFEST.read_text())
    assert manifest["origin"] == "https://github.com/ArduPilot/ardupilot_gazebo"
    assert manifest["revision"] == "082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5"
    assert {item["source_path"]: item["sha256"] for item in manifest["files"]} == EXPECTED
    for item in manifest["files"]:
        assert sha256(ROOT / item["imported_path"]) == item["sha256"]
        assert item["license"] == "LGPL-3.0-only"
```

The model SDF test must also reject `ArduPilotPlugin`, `.so`, `.dylib`, remote
HTTP resources, gimbal, LiDAR, payload, and Gst camera references.

- [ ] **Step 2: Run tests and verify red**

Run: `uv run pytest gazebo/tests/test_provenance.py gazebo/tests/test_image_contract.py -q`

Expected: missing image, lock, resources, and provenance files.

- [ ] **Step 3: Import only pinned upstream bytes**

Fetch each file from
`https://raw.githubusercontent.com/ArduPilot/ardupilot_gazebo/082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5/<source_path>`, verify the exact hashes
above before copying, and never copy from the dirty transfer worktree. Preserve
the upstream `model.config`, `model.sdf`, and mesh bytes under
`gazebo/provenance/upstream/iris_with_standoffs/`. Copy the four verified mesh
bytes unchanged into the Phase 3 resource model and record both imported paths
for each copied mesh. Create the Phase 3 `model.config` and `model.sdf` as new
local files; do not represent their adapted bytes as upstream originals.

- [ ] **Step 4: Build and freeze the exact package delta**

The Dockerfile installs these directly used package versions:

```text
ros-jazzy-ros-gz=1.0.22-1noble.20260616.074726
ros-jazzy-ros-gz-bridge=1.0.22-1noble.20260615.142443
ros-jazzy-ros-gz-image=1.0.22-1noble.20260615.145009
ros-jazzy-ros-gz-interfaces=1.0.22-1noble.20260615.112415
ros-jazzy-ros-gz-sim=1.0.22-1noble.20260615.173223
ros-jazzy-gz-sim-vendor=0.0.10-1noble.20260604.111001
ros-jazzy-sdformat-vendor=0.0.11-1noble.20260604.104102
```

Use the existing FFmpeg lock pattern: capture sorted `dpkg-query` output before
and after installation, commit every added package and exact version in
`harmonic-packages.lock`, and make the image build compare the actual delta to
the lock. The test stage asserts `gz sim --versions` reports Sim major 8,
`ros2 pkg prefix ros_gz_bridge` succeeds, and no ArduPilot plugin library exists.

- [ ] **Step 5: Add the installable Python project and test target**

Create `drone-sim-gazebo` with Python `>=3.12`, the workspace dependency
`drone-sim-artifacts`, no third-party PyPI runtime dependencies, and an entry
point reserved for Task 5. The Dockerfile copies and installs both local
projects:

```toml
[project.scripts]
drone-sim-gazebo-runtime = "drone_sim_gazebo.runtime.runtime_node:main"
```

Add it to the uv workspace and lock it without upgrading unrelated packages:

Run: `uv lock`

- [ ] **Step 6: Verify source and image contracts**

Run:

```bash
uv run pytest gazebo/tests/test_provenance.py gazebo/tests/test_image_contract.py -q
docker build --target test -f gazebo/Dockerfile -t drone-sim-gazebo-test:phase3 .
docker run --rm --network none drone-sim-gazebo-test:phase3
```

Expected: exact provenance and apt-delta tests pass without network access at
runtime.

- [ ] **Step 7: Commit**

```bash
git add gazebo pyproject.toml uv.lock
git commit -m "build: pin gazebo harmonic runtime and assets"
```

---

### Task 3: Deterministic local world and resource resolver

**Files:**
- Create: `gazebo/resources/worlds/phase3_foundation.sdf`
- Create: `gazebo/src/drone_sim_gazebo/worlds/__init__.py`
- Create: `gazebo/src/drone_sim_gazebo/worlds/api.py`
- Create: `gazebo/src/drone_sim_gazebo/worlds/EXTERNAL_INTERFACE.md`
- Create: `gazebo/src/drone_sim_gazebo/worlds/INTERNAL_INTERFACE.md`
- Create: `gazebo/src/drone_sim_gazebo/models/__init__.py`
- Create: `gazebo/src/drone_sim_gazebo/models/api.py`
- Create: `gazebo/src/drone_sim_gazebo/models/EXTERNAL_INTERFACE.md`
- Create: `gazebo/src/drone_sim_gazebo/models/INTERNAL_INTERFACE.md`
- Create: `gazebo/tests/test_world_resources.py`

**Interfaces:**
- Consumes: `SimulationConfig`, image-owned resource root, and provenance manifest.
- Produces: `WorldConfig` and `ResolvedWorld(path, world_name, vehicle_id, resource_path, world_sha256, resource_sha256s)`; no ROS or subprocess behavior.

- [ ] **Step 1: Write failing resolver and SDF tests**

```python
def test_resolve_phase3_world_is_local_and_stable():
    resolved = resolve_world(WorldConfig(world="phase3_foundation", vehicle="iris"))
    assert resolved.world_name == "phase3_foundation"
    assert resolved.vehicle_id == "iris"
    assert resolved.path.is_file()
    assert resolved.resource_path.is_dir()
    assert len(resolved.world_sha256) == 64
    assert tuple(sorted(resolved.resource_sha256s)) == resolved.resource_sha256s


def test_sdf_has_exact_physics_and_camera_contract():
    root = ET.parse(WORLD).getroot()
    cameras = root.findall(".//sensor[@type='camera']")
    assert {item.attrib["name"] for item in cameras} == {"onboard_camera", "observer_camera"}
    for camera in cameras:
        assert camera.findtext("update_rate") == "20"
        assert camera.findtext("camera/image/width") == "320"
        assert camera.findtext("camera/image/height") == "240"
        assert camera.findtext("camera/image/format") == "R8G8B8"
```

Also assert one dynamic `iris` model, one static landing marker, one ground
plane, local `model://iris_phase3` resolution, fixed poses, explicit
`max_step_size`, real-time update rate derived for target RTF 0.1, contact
sensor coverage, pose publication, and absence of remote URIs and ArduPilot
plugins.

- [ ] **Step 2: Run focused tests and verify red**

Run: `uv run pytest gazebo/tests/test_world_resources.py -q`

Expected: world and resolver are missing.

- [ ] **Step 3: Implement immutable resource resolution**

```python
@dataclass(frozen=True)
class WorldConfig:
    world: str
    vehicle: str


@dataclass(frozen=True)
class ResolvedWorld:
    path: Path
    world_name: str
    vehicle_id: str
    resource_path: Path
    world_sha256: str
    resource_sha256s: tuple[tuple[str, str], ...]


def resolve_world(config: WorldConfig, *, package_root: Path | None = None) -> ResolvedWorld:
    if config != WorldConfig("phase3_foundation", "iris"):
        raise ValueError("Phase 3 supports only phase3_foundation/iris")
    root = (
        package_root
        if package_root is not None
        else Path(__file__).resolve().parents[3] / "resources"
    ).resolve(strict=True)
    path = (root / "worlds/phase3_foundation.sdf").resolve(strict=True)
    if not path.is_relative_to(root):
        raise ValueError("world path escapes the Gazebo resource root")
    resource_hashes = tuple(
        (str(item.relative_to(root)), _sha256_regular(item, root=root))
        for item in sorted(root.rglob("*"))
        if item.is_file()
    )
    return ResolvedWorld(
        path=path,
        world_name="phase3_foundation",
        vehicle_id="iris",
        resource_path=root,
        world_sha256=_sha256_regular(path, root=root),
        resource_sha256s=resource_hashes,
    )
```

The implementation resolves symlinks, requires regular single-link files under
the package resource root, rejects escaping paths, hashes by descriptor-safe
reads through `_sha256_regular(path: Path, *, root: Path) -> str`, and returns
only frozen values.

- [ ] **Step 4: Create the minimal SDF**

Use Gazebo Harmonic SDF with Physics, Sensors, UserCommands, SceneBroadcaster,
PosePublisher, and Contact systems. Make `iris` passive and dynamic at a fixed
height, attach the onboard camera downward, and place the observer camera in a
static world model. Give native topics run-independent private names; the
adapter supplies run identity.

- [ ] **Step 5: Validate SDF inside the pinned image**

Run:

```bash
uv run pytest gazebo/tests/test_world_resources.py -q
docker run --rm --network none drone-sim-gazebo-test:phase3 bash -lc 'gz sdf -k /opt/drone_sim/gazebo/resources/worlds/phase3_foundation.sdf'
```

Expected: Python contract tests pass and `gz sdf -k` exits 0.

- [ ] **Step 6: Commit**

```bash
git add gazebo/resources gazebo/src/drone_sim_gazebo/worlds gazebo/src/drone_sim_gazebo/models gazebo/tests/test_world_resources.py
git commit -m "feat: add deterministic phase 3 world"
```

---

### Task 4: Pure camera and ground-truth adapter model

**Files:**
- Create: `gazebo/src/drone_sim_gazebo/ros_adapter/__init__.py`
- Create: `gazebo/src/drone_sim_gazebo/ros_adapter/model.py`
- Create: `gazebo/src/drone_sim_gazebo/ros_adapter/EXTERNAL_INTERFACE.md`
- Create: `gazebo/src/drone_sim_gazebo/ros_adapter/INTERNAL_INTERFACE.md`
- Create: `gazebo/tests/test_adapter_model.py`

**Interfaces:**
- Consumes: native image samples, pose/twist/contact samples, canonical run ID, expected frame count.
- Produces: validated `PublicFrame`, `PublicGroundTruth`, `AdapterSummary`, and `AdapterFault`; contains no ROS imports so sequencing is exhaustively unit-testable.

- [ ] **Step 1: Write failing frame-sequence tests**

```python
def test_camera_sequence_assigns_contiguous_ids_and_preserves_native_stamp():
    adapter = CameraSequence(run_id=RUN_ID, stream="onboard", expected_frames=2)
    first = adapter.accept(native_image(stamp_ns=50_000_000))
    second = adapter.accept(native_image(stamp_ns=100_000_000))
    assert (first.frame_id, second.frame_id) == (0, 1)
    assert first.sim_timestamp_ns == first.header_timestamp_ns == 50_000_000
    assert second.sim_timestamp_ns == 100_000_000
    assert adapter.complete


@pytest.mark.parametrize(
    "samples",
    [
        [50_000_000, 50_000_000],
        [100_000_000, 50_000_000],
        [50_000_000, 100_000_001],
    ],
)
def test_camera_sequence_fails_closed_on_duplicate_regression_or_off_grid(samples):
    adapter = CameraSequence(run_id=RUN_ID, stream="onboard", expected_frames=2)
    adapter.accept(native_image(stamp_ns=samples[0]))
    with pytest.raises(AdapterFault):
        adapter.accept(native_image(stamp_ns=samples[1]))
```

Test wrong dimensions, encoding, stride, payload size, overrun, wrong stream,
pair timestamp mismatch, ground-truth timestamp regression, non-finite pose or
twist, contact epoch changes, and frozen adapter rejection.

- [ ] **Step 2: Run focused tests and verify red**

Run: `uv run pytest gazebo/tests/test_adapter_model.py -q`

Expected: adapter model does not exist.

- [ ] **Step 3: Implement frozen input/output values**

```python
@dataclass(frozen=True)
class NativeImage:
    sim_timestamp_ns: int
    width: int
    height: int
    encoding: str
    step: int
    data: bytes


@dataclass(frozen=True)
class PublicFrame:
    run_id: str
    stream: str
    frame_id: int
    sim_timestamp_ns: int
    header_timestamp_ns: int
    width: int
    height: int
    encoding: str
    step: int
    data: bytes


@dataclass(frozen=True)
class NativeGroundTruth:
    sim_timestamp_ns: int
    position_xyz: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    linear_velocity_xyz: tuple[float, float, float]
    angular_velocity_xyz: tuple[float, float, float]
    in_contact: bool


@dataclass(frozen=True)
class PublicGroundTruth:
    run_id: str
    vehicle_id: str
    sim_timestamp_ns: int
    position_xyz: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    linear_velocity_xyz: tuple[float, float, float]
    angular_velocity_xyz: tuple[float, float, float]
    in_contact: bool


class AdapterFault(RuntimeError):
    pass
```

`CameraSequence.accept()` validates exact geometry, lets the first positive
native timestamp establish the capture epoch, then requires every delta to be
exactly 50,000,000 ns. It never sleeps, invents, retimes, or drops a sample.

- [ ] **Step 4: Implement paired completion and ground truth**

`AdapterModel.accept_frame(stream, sample)` returns one `PublicFrame` and only
reports `camera_pair_complete(frame_id, stamp)` when both streams have that
same ID and native timestamp. `accept_ground_truth()` maps ENU values unchanged
and requires nondecreasing native time. A pair can complete publicly only when
one pose/twist/contact sample exists at the identical timestamp; completion
returns that one aligned `PublicGroundTruth`, giving exactly one ground-truth
sample per pair. `freeze()` prevents further output and returns:

```python
@dataclass(frozen=True)
class AdapterSummary:
    onboard_frames: int
    observer_frames: int
    paired_frames: int
    ground_truth_samples: int
    first_sim_timestamp_ns: int | None
    last_sim_timestamp_ns: int | None
```

- [ ] **Step 5: Run tests**

Run: `uv run pytest gazebo/tests/test_adapter_model.py -q`

Expected: all pure sequencing, geometry, pairing, contact, completion, and
freeze cases pass without ROS or Gazebo installed on the host.

- [ ] **Step 6: Commit**

```bash
git add gazebo/src/drone_sim_gazebo/ros_adapter gazebo/tests/test_adapter_model.py
git commit -m "feat: validate gazebo public sample sequences"
```

---

### Task 5: Gazebo server process boundary and runtime state machine

**Files:**
- Create: `gazebo/src/drone_sim_gazebo/server/__init__.py`
- Create: `gazebo/src/drone_sim_gazebo/server/process.py`
- Create: `gazebo/src/drone_sim_gazebo/server/EXTERNAL_INTERFACE.md`
- Create: `gazebo/src/drone_sim_gazebo/server/INTERNAL_INTERFACE.md`
- Create: `gazebo/src/drone_sim_gazebo/runtime/__init__.py`
- Create: `gazebo/src/drone_sim_gazebo/runtime/model.py`
- Create: `gazebo/src/drone_sim_gazebo/runtime/EXTERNAL_INTERFACE.md`
- Create: `gazebo/src/drone_sim_gazebo/runtime/INTERNAL_INTERFACE.md`
- Create: `gazebo/tests/test_server_process.py`
- Create: `gazebo/tests/test_runtime_model.py`

**Interfaces:**
- Consumes: `ResolvedWorld`, `SimulationConfig`, canonical run paths, monotonic infrastructure deadline, current-run lifecycle inputs.
- Produces: exact shell-free server argv/environment, bounded process control, `RuntimeAction` values, and `NativeArtifactSummary`; it does not import `rclpy`.

- [ ] **Step 1: Write failing command and state-machine tests**

```python
def test_server_spec_is_paused_local_partitioned_and_records_native_state(tmp_path):
    spec = server_spec(run_context(tmp_path), resolved_world(), simulation(seed=9))
    assert spec.argv == (
        "gz", "sim", "-s", "--headless-rendering", "--seed", "9",
        "--record-path", str(tmp_path / "gazebo/state"),
        str(resolved_world().path),
    )
    assert "-r" not in spec.argv
    assert spec.environment["GZ_PARTITION"] == f"drone_sim_{RUN_ID.replace('-', '_')}"
    assert spec.environment["GZ_SIM_RESOURCE_PATH"] == str(resolved_world().resource_path)


def test_runtime_releases_one_step_only_after_both_readiness_facts():
    model = RuntimeModel(run_id=RUN_ID, expected_frames=40)
    assert model.accept(ArtifactsReady(RUN_ID)) == ()
    assert model.accept(GazeboReady(RUN_ID)) == (PublishGazeboReady(),)
    assert model.accept(RunStateEvent(RUN_ID, "READY")) == (RequestSteps(1),)
    assert model.accept(RunStateEvent(RUN_ID, "READY")) == ()
    assert model.accept(RunStateEvent(RUN_ID, "RUNNING")) == (SetPaused(False),)
```

Also test stale IDs, duplicate readiness, child exit, endpoint timeout, final
pair completion, `FINALIZING` preemption, pause-before-finish, graceful-stop
timeout escalation, and output rejection after freeze.

- [ ] **Step 2: Run tests and verify red**

Run: `uv run pytest gazebo/tests/test_server_process.py gazebo/tests/test_runtime_model.py -q`

Expected: server and runtime units do not exist.

- [ ] **Step 3: Implement a shell-free subprocess boundary**

```python
@dataclass(frozen=True)
class ServerSpec:
    argv: tuple[str, ...]
    environment: Mapping[str, str]
    partial_log_path: Path
    final_log_path: Path
    native_state_path: Path


@dataclass(frozen=True)
class NativeArtifactSummary:
    server_log_path: Path
    state_log_path: Path
    server_returncode: int
    graceful: bool
```

`GazeboServer.start()` uses `shell=False`, one already-open append-only partial
log descriptor for stdout/stderr, a minimal validated environment, and a new
process group. Before spawning, it writes one escaped diagnostic preamble with
the exact argv, run ID, partition, world checksum, and seed to that raw log;
the preamble is evidence, never shell input. `stop(deadline)` requests graceful termination, waits only for
the remaining deadline, escalates once, fsyncs files/directories, verifies a
nonempty regular `state.tlog`, and publishes `server.log` with an atomic
same-directory link/rename protocol consistent with artifact security rules.

- [ ] **Step 4: Implement the pure lifecycle transition model**

Represent every side effect as a frozen action:

```python
@dataclass(frozen=True)
class PublishGazeboReady:
    pass


@dataclass(frozen=True)
class RequestSteps:
    count: int


@dataclass(frozen=True)
class SetPaused:
    paused: bool


@dataclass(frozen=True)
class WriteSourceFinished:
    sim_timestamp_ns: int


@dataclass(frozen=True)
class WriteRuntimeFailure:
    reason: str
    diagnostic_paths: tuple[str, ...]


@dataclass(frozen=True)
class BeginFinalization:
    requested_terminal: str
    reason: str


@dataclass(frozen=True)
class StopServer:
    deadline_monotonic: float


@dataclass(frozen=True)
class WriteQuiescence:
    native_artifacts: NativeArtifactSummary


RuntimeAction = (
    PublishGazeboReady
    | RequestSteps
    | SetPaused
    | WriteSourceFinished
    | WriteRuntimeFailure
    | BeginFinalization
    | StopServer
    | WriteQuiescence
)
```

The model never calls wall time. Its caller supplies current-run events and
uses wall time only to bound infrastructure actions. Completion occurs after
the configured number of exact camera pairs and matching ground-truth progress.

- [ ] **Step 5: Run focused tests**

Run: `uv run pytest gazebo/tests/test_server_process.py gazebo/tests/test_runtime_model.py -q`

Expected: exact argv/environment, idempotence, stale-run rejection, one-step
release, completion, failure, and bounded finalization tests pass.

- [ ] **Step 6: Commit**

```bash
git add gazebo/src/drone_sim_gazebo/server gazebo/src/drone_sim_gazebo/runtime gazebo/tests/test_server_process.py gazebo/tests/test_runtime_model.py
git commit -m "feat: supervise paused gazebo lifecycle"
```

---

### Task 6: ROS adapter node, private bridges, and production runtime entry point

**Files:**
- Create: `gazebo/config/bridge.yaml`
- Create: `gazebo/src/drone_sim_gazebo/ros_adapter/node.py`
- Create: `gazebo/src/drone_sim_gazebo/runtime/runtime_node.py`
- Create: `gazebo/src/drone_sim_gazebo/runtime/children.py`
- Create: `gazebo/tests/test_adapter_node.py`
- Create: `gazebo/tests/test_runtime_node.py`
- Modify: `gazebo/Dockerfile`

**Interfaces:**
- Consumes: pure adapter/runtime/server APIs, private bridged image/pose/contact topics, `/simulation/run_state`, artifacts durable readiness, finalize request.
- Produces: the six public Gazebo ROS topics, `.status/gazebo-ready.json`, `source-finished`, `runtime-failure`, structured Gazebo JSONL, native artifacts, and Gazebo quiescence.

- [ ] **Step 1: Write failing ROS contract tests**

Use the same Jazzy container-test pattern as `artifacts/tests/test_artifact_status_ros.py` and assert exact endpoint types and QoS:

```python
assert_endpoint(node, "/clock", Clock, reliability="BEST_EFFORT", depth=1)
for stream in ("onboard", "observer"):
    assert_endpoint(node, f"/camera/{stream}/image_raw", Image, reliability="RELIABLE", depth=5)
    assert_endpoint(node, f"/camera/{stream}/frame_metadata", FrameMetadata, reliability="RELIABLE", depth=5)
assert_endpoint(node, "/simulation/ground_truth", GroundTruth, reliability="BEST_EFFORT", depth=10)
```

Publish private synthetic native messages and assert exact public image bytes,
matching metadata, run ID, frame ID, timestamps, ENU ground truth, and durable
failure on an invalid sample. Assert there is no subscription to
`/simulation/camera_pair_ack` and no second ROS `/clock` publisher.

- [ ] **Step 2: Run ROS tests and verify red**

Run: `docker build --target test -f gazebo/Dockerfile -t drone-sim-gazebo-test:phase3 . && docker run --rm --network none drone-sim-gazebo-test:phase3`

Expected: node and bridge configuration are missing.

- [ ] **Step 3: Add unidirectional private bridges**

`bridge.yaml` contains only `GZ_TO_ROS` entries. Bridge native clock to a
private ROS clock topic, two images to private image topics, and pose/contact
data to private topics. Use explicit type names, queues, and private names.
Never bridge ROS `/clock` back to Gazebo. The runtime supervises bridge and
image-bridge child processes and treats unexpected exit as `runtime-failure`.

- [ ] **Step 4: Implement the public ROS adapter node**

The node converts ROS messages into pure `NativeImage`/ground-truth values,
calls `AdapterModel`, and constructs public messages. Use explicit QoS factory
functions, bounded callback queues, and a freeze flag checked before every
publish. `FrameMetadata.sim_timestamp` and `Image.header.stamp` come from the
same validated integer nanoseconds.

- [ ] **Step 5: Implement the production runtime entry point**

`runtime_node.main()` validates env/config, resolves the world, creates the
server spec, starts the server paused, discovers required private endpoints,
starts bridge/adapter children, waits for artifacts readiness, writes Gazebo
readiness, applies pure `RuntimeAction` values, and finalizes through
`RuntimeProtocol`. It emits structured events through `artifacts.StructuredEvent`.

Child-process polling and durable-file fallback must continue while ROS is
idle; use bounded `rclpy.spin_once` calls, not simulation-relevant sleeps.

- [ ] **Step 6: Verify container unit and ROS tests**

Run:

```bash
docker build --target test -f gazebo/Dockerfile -t drone-sim-gazebo-test:phase3 .
docker run --rm --network none drone-sim-gazebo-test:phase3
```

Expected: pure tests and live Jazzy QoS/message tests pass, and the image test
asserts one public `/clock` publisher and zero production camera-ack consumers.

- [ ] **Step 7: Commit**

```bash
git add gazebo/config gazebo/src/drone_sim_gazebo gazebo/tests gazebo/Dockerfile
git commit -m "feat: publish gazebo simulation contracts"
```

---

### Task 7: Compose, readiness, and native-artifact integration

**Files:**
- Modify: `compose.yaml`
- Modify: `orchestration/src/orchestration/controller.py`
- Modify: `orchestration/src/orchestration/runtime_node.py`
- Modify: `orchestration/tests/test_controller.py`
- Modify: `orchestration/tests/test_runtime_node.py`
- Modify: `artifacts/src/artifacts/validation.py`
- Modify: `artifacts/src/artifacts/session.py`
- Modify: `artifacts/tests/test_validation.py`
- Modify: `artifacts/tests/test_session.py`
- Modify: `artifacts/EXTERNAL_INTERFACE.md`
- Modify: `artifacts/INTERNAL_INTERFACE.md`

**Interfaces:**
- Consumes: `PHASE3_OWNERSHIP`, `gazebo-ready`, production image target, existing artifacts readiness/finalization protocol.
- Produces: exact seven-service `phase3` topology, a startup barrier requiring artifacts plus Gazebo readiness, and semantic native-state validation.

- [ ] **Step 1: Write failing controller and validator tests**

```python
def test_phase3_compose_has_exact_service_ownership():
    assert configured_services("phase3") == {
        "orchestration-runtime", "artifacts-runtime", "synthetic-companion",
        "synthetic-ardupilot-sitl", "gazebo-runtime",
        "synthetic-electromagnet", "synthetic-scorekeeper",
    }


def test_phase3_ready_requires_artifacts_and_gazebo(tmp_path):
    readiness = RuntimeReadiness(run_id=RUN_ID, runtime_profile="phase3")
    assert readiness.accept_artifacts(run_id=RUN_ID, ready=True) == ()
    assert readiness.accept_gazebo(run_id=RUN_ID, ready=True) == (PublishReady(),)
    assert readiness.accept_gazebo(run_id=RUN_ID, ready=True) == ()


def test_native_state_requires_one_nonempty_regular_state_tlog(tmp_path):
    state = tmp_path / "gazebo/state"
    state.mkdir(parents=True)
    (state / "state.tlog").write_bytes(b"native-state")
    assert validate_gazebo_state(state).valid
```

Add symlink, hard-link, empty log, escaping directory, wrong filename, stale-run
readiness, duplicate service, ambient profile, and failed/aborted partial-state
cases.

- [ ] **Step 2: Run affected tests and verify red**

Run: `uv run pytest orchestration/tests artifacts/tests -q`

Expected: Phase 3 readiness/topology and native semantic validation are absent.

- [ ] **Step 3: Add the exact Phase 3 Compose profile**

Factor shared run mounts/environment into a profile-neutral anchor. Keep Phase
2 names and behavior unchanged. Add `gazebo-runtime` using
`drone-sim-gazebo-runtime:phase3`, the Gazebo Dockerfile runtime target, one
read-write run bind, one read-only resolved config bind, `SIM_MODULE=gazebo`,
and no host network, display, privileged mode, device mount, or published port.

Give the five retained test doubles `profiles: [phase2, phase3]` without
changing their module logs or lifecycle behavior.

- [ ] **Step 4: Extend readiness without moving authority**

For `phase3`, orchestration waits for both current-run `artifacts-ready` and
`gazebo-ready` before publishing `READY`. Phase 2 continues to require only
artifacts readiness. The first clock remains the trigger for `RUNNING`.
Current wall deadlines and durable-file fallbacks remain unchanged.

Implement `RuntimeReadiness` as a pure idempotent current-run gate in
`orchestration/runtime_node.py`. The ROS runtime feeds it live `ArtifactStatus`
and durable `gazebo-ready`; the host controller independently waits for both
durable records before considering startup complete. `PublishReady` is emitted
exactly once and only the existing `OrchestrationRuntime.start()` method maps
that action to the public `RunState.READY` sample.

Define the gate output explicitly:

```python
@dataclass(frozen=True)
class PublishReady:
    pass
```

- [ ] **Step 5: Add descriptor-safe native state validation**

`validate_gazebo_state()` opens the directory without following symlinks,
requires a nonempty regular single-link `state.tlog`, rejects special files and
unexpected nested links, and verifies identity before/after hashing. Completed
runs require valid native state and final server log. Failed/aborted runs record
missing or partial evidence without preventing manifest publication.

- [ ] **Step 6: Run unit and Compose configuration tests**

Run:

```bash
uv run pytest orchestration/tests artifacts/tests -q
docker compose --profile phase2 config --services
docker compose --profile phase3 config --services
```

Expected: both profiles contain exactly seven services; their sets differ only
by synthetic versus production Gazebo; all unit tests pass.

- [ ] **Step 7: Commit**

```bash
git add compose.yaml orchestration artifacts
git commit -m "feat: integrate phase 3 gazebo lifecycle"
```

---

### Task 8: Slow-real-time Gazebo acceptance gate and evidence

**Files:**
- Create: `tests/integration/test_phase3_gazebo.py`
- Create: `tests/phase3/inspect_gazebo_bundle.py`
- Modify: `Makefile`
- Modify: `docs/IMPLEMENTATION_ROADMAP.md`
- Create: `docs/verification/phase-3-gazebo-foundation.md`
- Modify: `PLAN.md`
- Modify: `gazebo/PLAN.md`

**Interfaces:**
- Consumes: complete Phase 3 image/profile/runtime and existing offline bundle inspector patterns.
- Produces: `make test-phase3`, fresh acceptance bundles, and recorded evidence for normal, slow, paused, deterministic, faulted, and isolated runs.

- [ ] **Step 1: Write the failing end-to-end acceptance module**

Create session-scoped image build and module-scoped output fixtures patterned on
`test_phase2_compose.py`, but inspect real Gazebo evidence. The normal-run test
must assert:

```python
assert result["terminal_status"] == "COMPLETED"
assert bag["counts"]["/camera/onboard/image_raw"] == 40
assert bag["counts"]["/camera/observer/image_raw"] == 40
assert bag["counts"]["/camera/onboard/frame_metadata"] == 40
assert bag["counts"]["/camera/observer/frame_metadata"] == 40
assert bag["counts"]["/simulation/ground_truth"] == 40
assert exact_deltas(bag, "/camera/onboard/frame_metadata") == {50_000_000}
assert exact_deltas(bag, "/camera/observer/frame_metadata") == {50_000_000}
assert bundle["videos"]["onboard"] == {"width": 320, "height": 240, "fps": 20.0}
assert bundle["videos"]["observer"] == {"width": 320, "height": 240, "fps": 20.0}
assert bundle["gazebo"]["state_tlog_bytes"] > 0
assert bundle["gazebo"]["server_log_bytes"] > 0
assert set(bundle["structured_logs"]) == set(MODULES)
```

Add tests for target RTF 0.1, an injected pre-release wall pause, equal-seed
normalized determinism, different-seed provenance, clock/camera/child-exit
faults, finalization while paused, completed/failed/aborted preservation, two
fresh partitions, exact service/image maps, and teardown with no project
containers or networks left behind.

- [ ] **Step 2: Run the gate and verify red**

Run: `uv run pytest tests/integration/test_phase3_gazebo.py -x -v`

Expected: the not-yet-wired acceptance path fails with specific missing runtime
behavior or evidence.

- [ ] **Step 3: Add only acceptance-facing fault controls**

If required by the tests, add validated Phase 3-only fault values:

```text
hold_before_first_step
clock_stall_after_5
camera_discontinuity_after_5
gazebo_child_exit_after_5
```

Fault controls may use wall time only to hold or terminate infrastructure; they
must never assign simulation timestamps. They are rejected outside the test
image/profile and recorded in structured diagnostics.

- [ ] **Step 4: Make the full Phase 3 gate pass**

Run:

```bash
docker compose --profile phase3 build
DRONE_SIM_PHASE3_IMAGES_BUILT=1 uv run pytest tests/integration/test_phase3_gazebo.py -v
```

Expected: every acceptance case passes. Capture actual image digests, Gazebo
and ros_gz versions, representative run IDs, simulation/wall timings, frame
counts, state/log sizes, manifest checksums, terminal distributions, and cleanup
evidence in `docs/verification/phase-3-gazebo-foundation.md`.

- [ ] **Step 5: Add the Make target and run regressions**

Add:

```make
test-phase3:
	docker compose --profile phase3 build
	DRONE_SIM_PHASE3_IMAGES_BUILT=1 uv run pytest tests/integration/test_phase3_gazebo.py -v
```

Run:

```bash
make test-unit
make test-foundation
make test-phase2
make test-phase3
```

Expected: all prior gates and the new physical Gazebo gate pass freshly.

- [ ] **Step 6: Close only Phase 3 documentation claims**

Mark the Phase 3 roadmap and Gazebo physical-foundation stages complete with
links to verification evidence. Leave ArduPilot lockstep, companion, scenario,
real scoring, complete descent, and maximum-score run explicitly pending.

- [ ] **Step 7: Commit**

```bash
git add tests/integration/test_phase3_gazebo.py tests/phase3 Makefile docs/IMPLEMENTATION_ROADMAP.md docs/verification/phase-3-gazebo-foundation.md PLAN.md gazebo/PLAN.md
git commit -m "test: prove phase 3 gazebo foundation"
```

---

## Execution ordering and review gates

1. Task 1 is the shared contract gate and runs first.
2. Tasks 2 and 4 are independent after Task 1 and may run concurrently in
   separate file ownership zones.
3. Task 3 follows Task 2 because it consumes the pinned resources.
4. Task 5 follows Tasks 3 and 4.
5. Task 6 follows Task 5.
6. Task 7 follows Tasks 1 and 6.
7. Task 8 follows every implementation task.

Every task receives a fresh implementer, then a spec-compliance review and a
code-quality review before the next dependent task begins. Fixes return to the
original implementer. Reviewers must compare against both this plan and the
Phase 3 design, and must reject any apparent shortcut that restores a synthetic
clock, makes physics wait on recorder acknowledgement, weakens artifact
validation, adds unreviewed legacy code, or claims Phase 4/5/6 completion.
