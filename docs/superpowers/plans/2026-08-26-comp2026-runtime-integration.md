# Comp2026 Runtime Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make one normal `drone-sim start` invocation run the original nested Comp2026 mission through FM1, FM2, both physical FM3 payload cycles, and the final Home landing, producing a preserved and independently verified `150/150` bundle.

**Architecture:** Keep the existing seven-service runtime. A thin host in the current companion service runs a narrow automatic entry point in the original nested repository; Gazebo owns course geometry and payload physics, electromagnet validates physical attach/release requests, and scorekeeper independently evaluates ordered physical evidence. The implementation ports only the prior simulator's course/asset generator inputs and detachable-joint coordinator, not its companion rewrite or broad infrastructure.

**Tech Stack:** Python 3.12, ROS 2 Jazzy, Gazebo Harmonic, ArduPilot SITL, DroneKit 2.9.2, PyMAVLink 2.4.49, OpenCV ArUco, NumPy, Docker Compose, pytest, C++17 Gazebo plugins, JSON/YAML configuration, rosbag2 MCAP, FFmpeg.

**Spec:** `docs/superpowers/specs/2026-08-26-comp2026-runtime-integration-design.md`

## Global Constraints

- Use the original nested repository at `companion/comp2026`, based on `b903edd`; do not import the later mission-runtime rewrite.
- Nested-repository changes are limited to an automatic entry point and necessary dependency, clock, perception, stability, attachment, and disarm seams.
- Commit nested changes only to a local `integration/drone-sim-mvp` branch and do not push.
- Retain exactly seven services: orchestration, artifacts, companion, ArduPilot SITL, Gazebo, electromagnet, and scorekeeper.
- Gazebo is the sole authority for pose, contact, joints, and payload motion; no payload teleport or pose-setting path is permitted.
- The companion requests payload actions only through electromagnet; scorekeeper remains read-only.
- The automatic sequence is FM1, FM2, FM3 marker 3, FM3 marker 4, and Home; there is one full attempt and no QGC fallback.
- Course targets are H `(0, 0)`, L `(-91.44, 0)`, F2 `(-152.40, 0)`, WA `(-45.72, -9.144)`, and WM `(-45.72, 9.144)` in H-relative Gazebo ENU meters.
- Camera output is `640x480` RGB at exactly 20 frames per simulated second.
- Marker acquisition occurs at 4.572 m AGL and requires five distinct fresh detections with lateral error at most 0.50 m.
- Pickup requires a grounded configured payload, grounded aircraft in the correct pickup zone, free capacity, and horizontal center error at most 0.075 m.
- Release eligibility requires 10 m AGL, position error at most 0.15 m, horizontal speed at most 0.10 m/s, and a continuous two-simulated-second hold.
- The score is physical and ordered: FM2 cumulative `80`, marker 3 cumulative `145`, marker 4 cumulative `150`, followed by a vertical Home landing and disarm before 600 simulated seconds.
- Preserve the existing lifecycle, fixed public epoch, lockstep, complete artifact finalization, seven logs, both videos, Gazebo state, and rosbag evidence.
- QGC handling, full-attempt retries, dashboards, generalized payload infrastructure, multiple vehicles, and unrelated hardening remain deferred.

## File and responsibility map

### Configuration and interfaces

- `config/default-run.json`: default competition attempt.
- `config/vertical-descent-run.json`: preserved prior baseline profile.
- `config/course.yaml`: course geometry and attempt limits.
- `config/scenario.yaml`: sensors, payload geometry, payload layout, and mission cycles.
- `config/run.schema.json`, `config/run-template.schema.json`: resolved/template validation.
- `orchestration/src/orchestration/config.py`: immutable resolution, source-config hashing, and snapshot copying.
- `ros_ws/src/simulation_interfaces/msg/{PayloadState,PayloadEvent,MissionEvent}.msg`: explicit physical and mission evidence.
- `ros_ws/src/simulation_interfaces/srv/PayloadCommand.srv`: companion-to-electromagnet request/response.

### Gazebo

- `gazebo/src/drone_sim_gazebo/competition_config.py`: strict course/scenario loader used by asset generation.
- `gazebo/scripts/prepare_competition_assets.py`: deterministic source-to-SDF/model generator.
- `gazebo/resources/worlds/competition_mission.sdf`: generated course world.
- `gazebo/resources/models/{iris_competition,payload_2,payload_3,payload_4}/`: generated vehicle and payload resources.
- `gazebo/plugin/PayloadCommandCoordinator.cc`: idempotent bridge to Gazebo's detachable-joint system.
- `gazebo/plugin/test/PayloadCommandCoordinatorIntegrationTest.cc`: real plugin confirmation/idempotency test.
- `gazebo/src/drone_sim_gazebo/ros_adapter/payload.py`: joins pose/contact/joint state into public payload samples.
- `gazebo/src/drone_sim_gazebo/ros_adapter/{model,node,topics}.py`: config-sized images and public vehicle/payload outputs.
- `gazebo/config/bridge-competition.yaml`: competition private bridge topics.

### Electromagnet and scoring

- `electromagnet/src/drone_sim_electromagnet/payload.py`: pure physical policy and command state.
- `electromagnet/src/drone_sim_electromagnet/runtime_node.py`: ROS service, state subscriptions, Gazebo command/result bridge, and payload events.
- `scorekeeper/rules/competition_v1.json`: versioned 150-point rules.
- `scorekeeper/src/drone_sim_scorekeeper/models.py`: scorer-neutral immutable result/event types.
- `scorekeeper/src/drone_sim_scorekeeper/competition.py`: physical sequence, settling, containment, and points.
- `scorekeeper/src/drone_sim_scorekeeper/competition_runtime.py`: lifecycle wrapper for competition inputs.
- `scorekeeper/src/drone_sim_scorekeeper/runtime_node.py`: scenario-selected ROS boundary.

### Original mission and host

- `companion/comp2026/src/drone/timebase.py`: wall-time default with injectable simulation provider.
- `companion/comp2026/src/drone/auto_attempt.py`: original FM function sequence.
- `companion/comp2026/src/drone/{mock_mission.py,control/drone_control.py,control/mission_info.py}`: narrow clock, acquisition, stability, attach, and disarm fixes.
- `companion/comp2026/src/drone/sensors/camera/{camera.py,_camera_manager.py}`: injected frame source and calibration path.
- `companion/src/drone_sim_companion/comp2026_host.py`: ROS sensor/payload adapters, coordinate conversion, worker, and mission events.
- `companion/src/drone_sim_companion/runtime_node.py`: select the original automatic host for `comp2026_auto`.
- `companion/Dockerfile`, `companion/pyproject.toml`: original source and pinned runtime dependencies.

### Artifacts and handoff

- `artifacts/src/artifacts/_adapters/rosbag.py`: profile-aware topic/image/payload evidence validation.
- `artifacts/src/artifacts/competition_score_validation.py`: independent 150-point recomputation.
- `artifacts/src/artifacts/acceptance.py`: ruleset dispatch and two-repository provenance validation.
- `artifacts/recording-qos.yaml`, `config/recording-qos.yaml`: new reliable evidence topics.
- `orchestration/src/orchestration/controller.py`: parent plus nested source revisions.
- `compose.yaml`: competition build revision and unchanged seven-service topology.
- `scripts/inspect_competition_run.py`: one-command independent bundle inspection.
- `scripts/write_competition_verification.py`: generate the final handoff evidence note from the accepted manifest.

---

### Task 1: Resolve the competition configuration and make it the operator default

**Files:**
- Create: `config/course.yaml`
- Create: `config/scenario.yaml`
- Create: `config/vertical-descent-run.json`
- Modify: `config/default-run.json`
- Modify: `config/run.schema.json`
- Modify: `config/run-template.schema.json`
- Modify: `orchestration/src/orchestration/config.py`
- Modify: `orchestration/src/orchestration/cli.py`
- Modify: `orchestration/pyproject.toml`
- Modify: `orchestration/tests/test_config.py`
- Modify: `orchestration/tests/test_cli.py`
- Modify: `uv.lock`

**Interfaces:**
- Consumes: an operator template whose optional `competition` object names `course.yaml` and `scenario.yaml` relative to the template.
- Produces: `RunConfig.competition: CompetitionSources | None`, copied immutable `configuration/course.yaml` and `configuration/scenario.yaml`, their SHA-256 values in resolved `run.json`, and default `drone-sim start` config selection.

- [ ] **Step 1: Write failing configuration and CLI tests**

Add tests that assert the new default, exact course values, copied config snapshots, and optional CLI argument:

```python
def test_default_template_resolves_complete_competition_attempt(tmp_path):
    resolved = resolve_run_config(DEFAULT_TEMPLATE, run_id_factory=lambda: FIXED_RUN_ID)
    assert (resolved.world, resolved.vehicle, resolved.mission, resolved.scenario) == (
        "competition_mission",
        "iris_competition",
        "comp2026_auto",
        "competition_v1",
    )
    assert resolved.recording == RecordingConfig(640, 480, 20, "rgb8")
    assert resolved.simulation.duration_ns == 600_000_000_000
    assert resolved.competition is not None
    destination = tmp_path / "configuration" / "run.json"
    write_resolved_config(resolved, destination)
    assert (destination.parent / "course.yaml").read_bytes() == (CONFIG / "course.yaml").read_bytes()
    assert (destination.parent / "scenario.yaml").read_bytes() == (CONFIG / "scenario.yaml").read_bytes()


def test_start_defaults_to_repository_competition_config(monkeypatch, tmp_path):
    monkeypatch.chdir(ROOT.parent)
    controller = FakeController()
    assert main(["start"], controller_factory=lambda **kwargs: controller) == 0
    assert controller.started == ROOT.parent / "config/default-run.json"
```

- [ ] **Step 2: Run the focused tests and confirm the old contract fails**

Run: `uv run pytest orchestration/tests/test_config.py orchestration/tests/test_cli.py -q`

Expected: FAIL because recording is fixed to `320x240`, competition sources do not exist, duration is 60 seconds, and `--config` is required.

- [ ] **Step 3: Add the authoritative course, scenario, and preserved baseline files**

Use the approved physical values. `course.yaml` must normalize to these H-relative meters:

```yaml
schema_version: 1
units: meters
origin: H
waypoints:
  H: {x: 0.0, y: 0.0, width: 4.572, height: 4.572, role: home}
  L: {x: -91.44, y: 0.0, width: 4.572, height: 4.572, role: landing}
  F2: {x: -152.40, y: 0.0, width: 0.9144, height: 0.9144, role: fire}
  WA: {x: -45.72, y: -9.144, width: 6.096, height: 6.096, role: autonomous_pickup}
  WM: {x: -45.72, y: 9.144, width: 6.096, height: 6.096, role: manual_pickup}
attempt:
  duration_seconds: 600
  acquisition_agl_m: 4.572
  transit_agl_m: 10.0
  release_agl_m: 10.0
```

`scenario.yaml` must include payload capacity one, `640x480`/20 Hz/0.60 rad camera geometry, a 20 Hz range sensor, 0.075 m pickup tolerance, one-second settling, 6x6x2 inch 2.5 lb payloads, 100 mm markers, and IDs 2/3/4 in the approved initial locations.

Copy the current `default-run.json` byte-for-byte to `vertical-descent-run.json`, then change `default-run.json` to competition, 600 simulated seconds, `640x480`, seed 2026, and target real-time factor `0.25`. Set `max_wall_seconds` to 5400 so a slower-than-target lockstep attempt can still finalize.

- [ ] **Step 4: Implement immutable source resolution and config copying**

Add the exact value type and serialize only safe relative artifact names plus hashes:

```python
@dataclass(frozen=True)
class CompetitionSources:
    course_source: Path
    scenario_source: Path
    course_sha256: str
    scenario_sha256: str


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
```

Resolve both paths against the template directory, require regular non-symlink files, parse them with `yaml.safe_load`, validate the exact schema/waypoints/payload IDs, and copy with exclusive creation inside `write_resolved_config`. Allow `320x240` for the preserved baseline and phase 2; require `640x480` for `comp2026_auto`. Allow exact target factors `0.1` and `0.25` and keep the 50 ms public grid requirement.

Make `start --config` optional with `Path.cwd() / "config/default-run.json"` as its repository-workspace default.

- [ ] **Step 5: Run config, schema, and CLI tests**

Run: `uv run pytest orchestration/tests/test_config.py orchestration/tests/test_cli.py tests/contracts -q`

Expected: PASS, including the preserved explicit vertical-descent profile.

- [ ] **Step 6: Commit the configuration boundary**

```bash
git add config orchestration/pyproject.toml orchestration/src/orchestration/config.py orchestration/src/orchestration/cli.py orchestration/tests/test_config.py orchestration/tests/test_cli.py uv.lock
git commit -m "Add competition runtime configuration"
```

### Task 2: Add explicit payload and mission ROS contracts

**Files:**
- Create: `ros_ws/src/simulation_interfaces/msg/PayloadState.msg`
- Create: `ros_ws/src/simulation_interfaces/msg/PayloadEvent.msg`
- Create: `ros_ws/src/simulation_interfaces/msg/MissionEvent.msg`
- Create: `ros_ws/src/simulation_interfaces/srv/PayloadCommand.srv`
- Modify: `ros_ws/src/simulation_interfaces/CMakeLists.txt`
- Modify: `tests/contracts/test_ros_interfaces.py`

**Interfaces:**
- Consumes: ROS standard `builtin_interfaces/Time`, `geometry_msgs/Pose`, and `geometry_msgs/Twist`.
- Produces: `/simulation/payload_state`, `/simulation/payload_events`, `/simulation/mission_events`, and `/simulation/payload_command` typed contracts used by Tasks 4-9.

- [ ] **Step 1: Extend the contract test with exact declarations**

Add service parsing and these expected declarations:

```python
EXPECTED["PayloadState.msg"] = [
    "string run_id",
    "builtin_interfaces/Time sim_timestamp",
    "uint16 aruco_id",
    "geometry_msgs/Pose pose",
    "geometry_msgs/Twist twist",
    "bool grounded",
    "bool attached",
]
EXPECTED["PayloadEvent.msg"] = [
    "string run_id",
    "builtin_interfaces/Time sim_timestamp",
    "uint64 event_id",
    "uint16 aruco_id",
    "string command_id",
    "string action",
    "string state",
    "string code",
]
EXPECTED["MissionEvent.msg"] = [
    "string run_id",
    "builtin_interfaces/Time sim_timestamp",
    "uint64 event_id",
    "string phase",
    "string state",
    "string detail",
]
```

Require `PayloadCommand.srv` request fields `run_id`, `aruco_id`, action constants `ATTACH=1` and `RELEASE=2`, `action`, and `command_id`; require response fields `accepted`, `code`, `detail`, `command_id`, and `response_sequence`.

- [ ] **Step 2: Run the contract test and confirm missing files fail**

Run: `uv run pytest tests/contracts/test_ros_interfaces.py -q`

Expected: FAIL with missing payload and mission interface files.

- [ ] **Step 3: Add the message and service files and register them**

Create the files exactly as asserted. Update `rosidl_generate_interfaces`:

```cmake
  "msg/PayloadState.msg"
  "msg/PayloadEvent.msg"
  "msg/MissionEvent.msg"
  "srv/PayloadCommand.srv"
```

Keep existing declarations unchanged so phase-2 and descent consumers remain compatible.

- [ ] **Step 4: Verify source contracts and a real ROS interface build**

Run: `uv run pytest tests/contracts/test_ros_interfaces.py -q`

Run: `docker build --target runtime -f electromagnet/Dockerfile -t drone-sim-electromagnet-contract .`

Expected: both commands PASS and rosidl generates the new service and messages.

- [ ] **Step 5: Commit the shared contracts**

```bash
git add ros_ws/src/simulation_interfaces tests/contracts/test_ros_interfaces.py
git commit -m "Add payload mission ROS contracts"
```

### Task 3: Generate the competition scene, physical payloads, and detachable-joint coordinator

**Files:**
- Create: `gazebo/src/drone_sim_gazebo/competition_config.py`
- Create: `gazebo/scripts/prepare_competition_assets.py`
- Create: `gazebo/resources/worlds/competition_mission.sdf`
- Create: `gazebo/resources/models/iris_competition/`
- Create: `gazebo/resources/models/payload_2/`
- Create: `gazebo/resources/models/payload_3/`
- Create: `gazebo/resources/models/payload_4/`
- Create: `gazebo/plugin/PayloadCommandCoordinator.cc`
- Create: `gazebo/plugin/test/PayloadCommandCoordinatorIntegrationTest.cc`
- Modify: `gazebo/plugin/CMakeLists.txt`
- Modify: `gazebo/Dockerfile`
- Create: `gazebo/tests/test_competition_assets.py`
- Modify: `gazebo/tests/test_flight_resources.py`
- Modify: `pyproject.toml`
- Modify: `uv.lock`

**Interfaces:**
- Consumes: `config/course.yaml`, `config/scenario.yaml`, and the current `iris_flight` source model.
- Produces: byte-reproducible committed competition resources; Gazebo string command/result topics using `payload-command-v1|<id>|attach|detach` semantics; stock detachable-joint triggers.

- [ ] **Step 1: Write failing strict config and generated-asset tests**

Test H-relative positions, current source geometry rather than stale transfer output, sensor placement, marker texture identity, and the absence of pose-setting plugins:

```python
def test_generated_competition_world_has_exact_course_and_payload_layout(tmp_path):
    output = prepare_assets(ROOT / "gazebo/resources", tmp_path, COURSE, SCENARIO)
    world = ET.parse(output).getroot().find("world")
    assert world.find("model[@name='pad_h']/pose").text.split()[:2] == ["0", "0"]
    assert world.find("model[@name='pad_l']/pose").text.split()[:2] == ["-91.44", "0"]
    assert world.find("model[@name='pad_f2']/pose").text.split()[:2] == ["-152.4", "0"]
    poses = {node.findtext("name"): node.findtext("pose") for node in world.findall("include")}
    assert poses["payload_3"].split()[:2] == ["-45.72", "-9.144"]
    assert poses["payload_4"].split()[:2] == ["-45.72", "9.144"]
    assert "set_pose" not in output.read_text(encoding="utf-8")


def test_generated_payload_uses_current_scenario_geometry(tmp_path):
    prepare_assets(ROOT / "gazebo/resources", tmp_path, COURSE, SCENARIO)
    payload = ET.parse(tmp_path / "models/payload_3/model.sdf")
    assert payload.findtext(".//inertial/mass") == "1.133981"
    assert payload.findtext(".//collision/geometry/box/size") == "0.1524 0.1524 0.0508"
    assert (tmp_path / "models/payload_3/materials/textures/marker_3.png").stat().st_size > 1000
```

- [ ] **Step 2: Run the tests and confirm no competition generator exists**

Run: `uv run pytest gazebo/tests/test_competition_assets.py gazebo/tests/test_flight_resources.py -q`

Expected: FAIL on missing generator and assets.

- [ ] **Step 3: Implement the strict loader and trimmed deterministic generator**

Use immutable dataclasses and make H the only world origin:

```python
@dataclass(frozen=True)
class Waypoint:
    name: str
    x_m: float
    y_m: float
    width_m: float
    height_m: float
    role: str


def world_xy(course: CourseConfig, name: str) -> tuple[float, float]:
    point = course.waypoints[name]
    home = course.waypoints["H"]
    return point.x_m - home.x_m, point.y_m - home.y_m
```

Port only the prior generator's useful SDF construction ideas. Generate the five pads, observer camera, centered downward camera, offset single-beam range sensor, centered hardpoint link, payload pose publishers, contacts, inertial values, colored bodies, and ArUco 4x4-250 textures. Configure payload 2's detachable joint as initially attached; configure 3 and 4 detached. Raise on any unexpected source model motor layout.

Write generated output deterministically: fixed float formatting, sorted payload IDs, no timestamps, and stable XML indentation. Run the generator into `gazebo/resources` and commit its outputs.

Add `numpy==2.1.3` and `opencv-contrib-python-headless==4.10.0.84` to the root development dependency group so marker generation is available to developers but not installed in the Gazebo runtime image. Lock both versions.

- [ ] **Step 4: Add the narrow idempotent Gazebo coordinator**

Port the reviewed `PayloadCommandCoordinator` only. Keep the wire grammar:

```text
payload-command-v1|<command_id>|attach
payload-command-v1|<command_id>|detach
payload-result-v1|<command_id>|confirmed|attached|OK
payload-result-v1|<command_id>|confirmed|detached|OK
```

Reject malformed IDs and conflicting action reuse; replay a completed identical command without retriggering physics. The coordinator may publish only the stock attach/detach trigger and result topics.

Add the C++17 integration test that loads the plugin into a minimal server, sends duplicate attach/detach commands, observes one physical transition per action, and verifies `COMMAND_ID_ACTION_CONFLICT` for conflicting reuse.

- [ ] **Step 5: Build both Gazebo plugins and execute the coordinator integration test in the image**

Update CMake to build `ArduPilotPlugin` and `cwru_payload_command_coordinator`; configure with `BUILD_TESTING=ON`, run `ctest --output-on-failure`, and copy both shared libraries into `/opt/drone_sim/gazebo/plugins`.

Run: `docker build --target runtime -f gazebo/Dockerfile -t drone-sim-gazebo-assets .`

Expected: image build PASS, including the C++ coordinator integration test.

- [ ] **Step 6: Regenerate and verify committed resources are byte-identical**

Run: `uv run python gazebo/scripts/prepare_competition_assets.py --course config/course.yaml --scenario config/scenario.yaml --source-root gazebo/resources --output-root /tmp/drone-sim-generated-assets`

Run: `diff -ru gazebo/resources/worlds/competition_mission.sdf /tmp/drone-sim-generated-assets/worlds/competition_mission.sdf`

Run: `uv run pytest gazebo/tests/test_competition_assets.py gazebo/tests/test_flight_resources.py -q`

Expected: no diff and all tests PASS.

- [ ] **Step 7: Commit the scene and physical plugin**

```bash
git add gazebo config/course.yaml config/scenario.yaml pyproject.toml uv.lock
git commit -m "Add physical competition scene"
```

### Task 4: Publish configuration-sized images, range, and physical payload state from Gazebo

**Files:**
- Create: `gazebo/src/drone_sim_gazebo/ros_adapter/payload.py`
- Modify: `gazebo/src/drone_sim_gazebo/ros_adapter/model.py`
- Modify: `gazebo/src/drone_sim_gazebo/ros_adapter/node.py`
- Modify: `gazebo/src/drone_sim_gazebo/ros_adapter/topics.py`
- Modify: `gazebo/src/drone_sim_gazebo/runtime/paths.py`
- Modify: `gazebo/src/drone_sim_gazebo/runtime/runtime_node.py`
- Create: `gazebo/config/bridge-competition.yaml`
- Create: `gazebo/tests/test_payload_adapter.py`
- Modify: `gazebo/tests/test_adapter_model.py`
- Modify: `gazebo/tests/test_adapter_node.py`
- Modify: `gazebo/tests/test_image_contract.py`
- Modify: `gazebo/tests/test_runtime_node.py`
- Modify: `gazebo/Dockerfile`

**Interfaces:**
- Consumes: private Gazebo camera, odometry, payload pose/contact/joint-state, and downward range topics.
- Produces: current public camera topics at configured dimensions, unchanged `/simulation/ground_truth`, `/simulation/payload_state` at 20 Hz for IDs 2/3/4, and `/competition/range/downward`.

- [ ] **Step 1: Write failing model and payload-join tests**

Cover both preserved `320x240` and competition `640x480`, finite-difference velocity, contact freshness, and attachment state:

```python
def test_payload_tracker_emits_one_complete_sample_per_pose_tick():
    tracker = PayloadTracker(run_id=RUN_ID, aruco_id=3, interval_ns=50_000_000)
    tracker.accept_contact(50_000_000, True)
    tracker.accept_attachment(50_000_000, False)
    first = tracker.accept_pose(50_000_000, (1.0, 2.0, 0.0254), UNIT_QUATERNION)
    second = tracker.accept_pose(100_000_000, (1.005, 2.0, 0.0254), UNIT_QUATERNION)
    assert first.grounded is True and first.attached is False
    assert second.linear_velocity_xyz == pytest.approx((0.1, 0.0, 0.0))


@pytest.mark.parametrize("width,height", [(320, 240), (640, 480)])
def test_adapter_uses_resolved_image_geometry(width, height):
    model = AdapterModel(run_id=RUN_ID, expected_frames=1, width_px=width, height_px=height)
    frame = model.accept_frame("onboard", native_image(width, height))
    assert (frame.width, frame.height, frame.step) == (width, height, width * 3)
```

- [ ] **Step 2: Run the focused tests and confirm fixed-size/missing-payload failures**

Run: `uv run pytest gazebo/tests/test_payload_adapter.py gazebo/tests/test_adapter_model.py gazebo/tests/test_adapter_node.py gazebo/tests/test_image_contract.py -q`

Expected: FAIL because images are frozen at `320x240` and no payload tracker or publisher exists.

- [ ] **Step 3: Parameterize the image adapter from resolved recording config**

Replace fixed width/height/step/payload constants with constructor fields validated as either exact approved profile. Propagate `recording.width_px` and `height_px` through runtime construction. Do not rescale frames; reject native frames that disagree with the resolved config.

- [ ] **Step 4: Implement payload pose/contact/joint aggregation**

Use this public immutable output:

```python
@dataclass(frozen=True)
class PublicPayloadState:
    run_id: str
    sim_timestamp_ns: int
    aruco_id: int
    position_xyz: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    linear_velocity_xyz: tuple[float, float, float]
    grounded: bool
    attached: bool
```

Require exact 50 ms pose cadence. Derive velocity only from consecutive positions. Latch failure on regression, gaps, nonfinite poses, unknown physical-state strings, or contact data older than one sample. Publish one `PayloadState` per payload pose tick. Never infer attachment from proximity.

- [ ] **Step 5: Add the competition bridge and runtime topic selection**

Bridge private clock, Iris odometry/contact, downward `LaserScan`, three payload pose vectors, three payload contact topics, and three command/state/result topic sets. Use `ros_gz_image` for onboard and observer images. Add `competition_mission` to approved world/topic/path selectors and require all competition publishers before `gazebo-ready`.

- [ ] **Step 6: Run Gazebo adapter and resource tests**

Run: `uv run pytest gazebo/tests -q`

Run: `docker build --target runtime -f gazebo/Dockerfile -t drone-sim-gazebo-payload-state .`

Expected: all tests and image build PASS.

- [ ] **Step 7: Commit public physical-state publication**

```bash
git add gazebo
git commit -m "Publish competition payload state"
```

### Task 5: Replace the inert electromagnet with physical payload authority

**Files:**
- Create: `electromagnet/src/drone_sim_electromagnet/payload.py`
- Modify: `electromagnet/src/drone_sim_electromagnet/runtime_node.py`
- Modify: `electromagnet/src/drone_sim_electromagnet/controller.py`
- Modify: `electromagnet/tests/test_runtime_node.py`
- Create: `electromagnet/tests/test_payload.py`
- Modify: `electromagnet/Dockerfile`
- Modify: `electromagnet/EXTERNAL_INTERFACE.md`
- Modify: `electromagnet/INTERNAL_INTERFACE.md`

**Interfaces:**
- Consumes: current-run `GroundTruth`, `PayloadState`, resolved course/scenario, ROS `PayloadCommand`, and Gazebo coordinator result strings.
- Produces: one physically confirmed service response, `/simulation/payload_events`, and one coordinator command; no pose writes.

- [ ] **Step 1: Write failing pure policy tests for every approved decision**

Use immutable snapshots and exact structured rejection codes:

```python
@pytest.mark.parametrize(
    "mutation,code",
    [
        (lambda state: replace(state, vehicle_grounded=False), "NOT_LANDED"),
        (lambda state: replace(state, vehicle_xy=(-40.0, -9.144)), "OUTSIDE_PICKUP_ZONE"),
        (lambda state: replace(state, payload_grounded=False), "PAYLOAD_NOT_GROUNDED"),
        (lambda state: replace(state, attached_id=2), "CAPACITY_OCCUPIED"),
        (lambda state: replace(state, vehicle_xy=(-45.60, -9.144)), "CENTER_ERROR"),
    ],
)
def test_attach_rejections_have_no_command(mutation, code):
    decision = authority().decide(mutation(centered_state()), attach_request(3))
    assert decision == PayloadDecision(False, code, None)


def test_centered_grounded_attach_emits_one_wire_command():
    decision = authority().decide(centered_state(), attach_request(3))
    assert decision.accepted is True
    assert decision.wire_command == "payload-command-v1|run:3:attach:1|attach"
```

Also test run mismatch, unknown marker, wrong zone, duplicate identical request replay, conflicting command ID reuse, releasing the wrong marker, and successful release of the attached marker.

- [ ] **Step 2: Run tests and confirm the inert implementation fails**

Run: `uv run pytest electromagnet/tests/test_payload.py electromagnet/tests/test_runtime_node.py -q`

Expected: FAIL because only `descent_v1` inactive-event policy exists.

- [ ] **Step 3: Implement the pure authority**

Define these stable interfaces:

```python
@dataclass(frozen=True)
class PayloadRequest:
    run_id: str
    aruco_id: int
    action: str
    command_id: str


@dataclass(frozen=True)
class PayloadDecision:
    accepted: bool
    code: str
    wire_command: str | None


@dataclass(frozen=True)
class PayloadWorld:
    vehicle_xy: tuple[float, float]
    vehicle_grounded: bool
    payload_xy: tuple[float, float]
    payload_grounded: bool
    attached_id: int | None


class PayloadAuthority:
    def decide(self, world: PayloadWorld, request: PayloadRequest) -> PayloadDecision:
        if request.run_id != self.run_id:
            return PayloadDecision(False, "STALE_RUN", None)
        return self._decide_current_run(world, request)
```

Validate horizontal center distance with `math.hypot`, zone containment from resolved config, capacity from confirmed attached state, and grounded facts from Gazebo. Store completed commands by ID and replay only exact duplicates.

- [ ] **Step 4: Implement the ROS service and confirmed-result bridge**

Use a `MultiThreadedExecutor` and reentrant callback group so a service callback can wait while result subscriptions run. The callback sequence is: validate, publish one coordinator string, wait at most five wall seconds for matching plugin confirmation, update confirmed attachment state, publish one `PayloadEvent`, and return. A timeout returns `PHYSICAL_CONFIRMATION_TIMEOUT` with `accepted=false`; it never guesses physical state.

Retain the old inactive `descent_v1` path when the resolved scenario is the preserved baseline. Competition readiness requires all three payload states, vehicle ground truth, all three coordinator result publishers, and the service.

- [ ] **Step 5: Run electromagnet tests and build its real image**

Run: `uv run pytest electromagnet/tests -q`

Run: `docker build --target runtime -f electromagnet/Dockerfile -t drone-sim-electromagnet-payload .`

Expected: tests and ROS interface image build PASS.

- [ ] **Step 6: Commit physical payload authority**

```bash
git add electromagnet
git commit -m "Add physical payload authority"
```

### Task 6: Add independent `competition_v1` physical scoring

**Files:**
- Create: `scorekeeper/rules/competition_v1.json`
- Create: `scorekeeper/src/drone_sim_scorekeeper/models.py`
- Create: `scorekeeper/src/drone_sim_scorekeeper/competition.py`
- Create: `scorekeeper/src/drone_sim_scorekeeper/competition_runtime.py`
- Modify: `scorekeeper/src/drone_sim_scorekeeper/descent.py`
- Modify: `scorekeeper/src/drone_sim_scorekeeper/output.py`
- Modify: `scorekeeper/src/drone_sim_scorekeeper/runtime.py`
- Modify: `scorekeeper/src/drone_sim_scorekeeper/runtime_node.py`
- Modify: `scorekeeper/Dockerfile`
- Create: `scorekeeper/tests/test_competition_score.py`
- Create: `scorekeeper/tests/test_competition_runtime.py`
- Modify: `scorekeeper/tests/test_descent_score.py`
- Modify: `scorekeeper/EXTERNAL_INTERFACE.md`
- Modify: `scorekeeper/INTERNAL_INTERFACE.md`

**Interfaces:**
- Consumes: 20 Hz `GroundTruth`, three 20 Hz `PayloadState` streams, ordered `PayloadEvent` and `MissionEvent`, and `competition_v1.json`.
- Produces: eight ordered score events, `scoring/result.json`, cumulative checkpoints 80/145/150, and `score-finished` only after the valid Home completion.

- [ ] **Step 1: Write failing perfect-attempt and fail-closed scorer tests**

Build a compact exact-grid trace and assert the official allocation:

```python
def test_perfect_physical_attempt_scores_150_at_required_checkpoints():
    scorer = CompetitionScorer(RUN_ID, load_competition_rules(RULES))
    feed_fm1_landing(scorer, at_xy=(-91.44, 0.0))
    feed_scored_drop(scorer, marker=2, source="H", release_xy=(-152.40, 0.0))
    feed_scored_pickup_drop(scorer, marker=3, source="WA")
    feed_scored_pickup_drop(scorer, marker=4, source="WM")
    feed_home_landing(scorer, elapsed_ns=420_000_000_000)
    result = scorer.finalize()
    assert result.complete is True
    assert result.achieved_score == result.maximum_available_score == 150.0
    assert cumulative_values(result.events[:-1]) == (20.0, 50.0, 60.0, 80.0, 95.0, 145.0, 150.0)
```

Add independent failures for missing FM1-complete mission evidence, release before a two-second stability window, speed `0.100001`, position `0.150001`, release below 10 m tolerance, payload bounds outside F2, missing attach before FM3 release, payload pose jump at pickup, capacity greater than one, Home after 600 seconds, and out-of-order marker 4.

- [ ] **Step 2: Run scorekeeper tests and confirm no competition scorer exists**

Run: `uv run pytest scorekeeper/tests/test_competition_score.py scorekeeper/tests/test_competition_runtime.py -q`

Expected: FAIL on missing competition types and rules.

- [ ] **Step 3: Extract scorer-neutral result models without changing descent behavior**

Move `RuleResult`, `ScoreEvent`, and `ScoreResult` unchanged into `models.py`; update imports in `descent.py`, `output.py`, and `runtime.py`. Keep serialization byte-compatible.

Run: `uv run pytest scorekeeper/tests/test_descent_score.py scorekeeper/tests/test_output.py scorekeeper/tests/test_runtime.py -q`

Expected: PASS with the existing `100/100` descent behavior unchanged.

- [ ] **Step 4: Implement the versioned rules and competition scorer**

The rules file must allocate exactly:

```json
{
  "schema_version": 1,
  "ruleset_id": "competition_v1",
  "sample_interval_ns": 50000000,
  "pickup_center_tolerance_m": 0.075,
  "release_position_tolerance_m": 0.15,
  "release_horizontal_speed_mps": 0.10,
  "release_stability_ns": 2000000000,
  "payload_settle_ns": 1000000000,
  "release_agl_m": 10.0,
  "deadline_ns": 600000000000,
  "points": {
    "fm1_landing": 20.0,
    "fm1_autonomy": 30.0,
    "payload_2": 10.0,
    "fm2_autonomy": 20.0,
    "payload_3": 15.0,
    "fm3_autonomy": 50.0,
    "payload_4": 5.0
  }
}
```

Emit rule events in the listed order and `score.finalized` as event 7. Compute full payload XY bounds from configured 0.1524 m dimensions and final yaw; require full containment in the 0.9144 m F2 rectangle. A release event itself awards nothing. Require continuous grounded/low-speed payload samples for one simulated second before a physical payload rule passes.

- [ ] **Step 5: Add the competition lifecycle and ROS boundary**

Create `CompetitionScorekeeperRuntime` with `accept_ground_truth`, `accept_payload_state`, `accept_payload_event`, `accept_mission_event`, `accept_source_finished`, and `begin_finalization`. Select it only when resolved `scenario == "competition_v1"`; retain the descent runtime otherwise. Subscribe with reliable volatile QoS for 20 Hz state and reliable transient-local QoS for mission/payload events.

- [ ] **Step 6: Run all scorekeeper tests and build the image**

Run: `uv run pytest scorekeeper/tests -q`

Run: `docker build --target runtime -f scorekeeper/Dockerfile -t drone-sim-scorekeeper-competition .`

Expected: all tests and image build PASS.

- [ ] **Step 7: Commit competition scoring**

```bash
git add scorekeeper
git commit -m "Add physical competition scoring"
```

### Task 7: Add only the necessary simulation seams to the original nested Comp2026 repository

**Files:**
- Create: `companion/comp2026/src/drone/timebase.py`
- Create: `companion/comp2026/src/drone/auto_attempt.py`
- Modify: `companion/comp2026/src/drone/control/drone_control.py`
- Modify: `companion/comp2026/src/drone/control/mission_info.py`
- Modify: `companion/comp2026/src/drone/missions/fm2.py`
- Modify: `companion/comp2026/src/drone/mock_mission.py`
- Modify: `companion/comp2026/src/drone/sensors/camera/camera.py`
- Modify: `companion/comp2026/src/drone/sensors/camera/_camera_manager.py`
- Create: `companion/comp2026/tests/test_timebase.py`
- Create: `companion/comp2026/tests/test_auto_attempt.py`
- Create: `companion/comp2026/tests/test_simulation_mission_seams.py`

**Interfaces:**
- Consumes: duck-typed controller, camera, lidar, per-marker payload action objects, phase-event callback, and an injected `Clock.now()/sleep()` provider.
- Produces: `run_auto_attempt(...)` that invokes original FM1/FM2/active FM3 functions in the required order and returns only after Home land/disarm.

- [ ] **Step 1: Create the local nested integration branch**

Run: `git -C companion/comp2026 switch -c integration/drone-sim-mvp`

Expected: local branch created from `b903edd`; no remote operation occurs.

- [ ] **Step 2: Write dependency-free sequencing and timebase tests**

Use injected fake mission functions so the test does not need hardware modules:

```python
def test_auto_attempt_invokes_original_phases_and_home_in_order():
    calls = []
    run_auto_attempt(
        tracker=FakeTracker(calls),
        controller=FakeController(calls),
        camera=object(),
        lidar=object(),
        payloads={2: FakePayload(2, calls), 3: FakePayload(3, calls), 4: FakePayload(4, calls)},
        waypoints=fake_waypoints(),
        emit=lambda phase, state: calls.append((phase, state)),
        mission_functions=FakeMissionFunctions(calls),
    )
    assert phase_names(calls) == ["FM1", "FM2", "FM3_3", "FM3_4", "HOME"]
    assert calls[-2:] == [("land", "H"), ("disarm", "H")]


def test_configured_timebase_sleeps_on_injected_clock():
    fake = FakeClock(now_value=12.0)
    with configured(fake):
        assert timebase.time() == 12.0
        timebase.sleep(2.5)
    assert fake.sleeps == [2.5]
```

- [ ] **Step 3: Run tests and confirm the entry point/timebase are absent**

Run: `PYTHONPATH=companion/comp2026/src uv run pytest companion/comp2026/tests/test_timebase.py companion/comp2026/tests/test_auto_attempt.py -q`

Expected: FAIL on missing `drone.timebase` and `drone.auto_attempt`.

- [ ] **Step 4: Add a wall-default injectable timebase and automatic entry point**

Expose this minimal clock seam:

```python
class Clock(Protocol):
    def now(self) -> float: ...
    def sleep(self, seconds: float) -> None: ...


@contextmanager
def configured(clock: Clock):
    global _active
    previous, _active = _active, clock
    try:
        yield
    finally:
        _active = previous
```

Use the standard library as the default. Import it under the existing `time` name in `drone_control.py`, `mission_info.py`, and `mock_mission.py`, preserving call sites while making mission waits simulation-driven. Add postponed annotations and move the hardware-only `Lidar` and `Dropper` imports in `fm2.py` and `mock_mission.py` behind `TYPE_CHECKING` so importing the automatic path never imports GPIO or I2C packages.

Implement `run_auto_attempt` to call original `missions.fm1.fm1`, `missions.fm2.fm2`, and active `mock_mission.fm3` for singleton ID sets `{3}` and `{4}`. Pass payload adapter 2 to FM2 and adapters 3/4 to their corresponding FM3 invocation. Emit `STARTED` and `COMPLETE` around each phase; after marker 4, command H at 10 m, call `simple_land`, call `disarm`, and emit Home complete.

- [ ] **Step 5: Make the active original behavior satisfy the approved physical gates**

Apply only these focused edits:

- set `TARGET_HOVER_HEIGHT = 4.572`;
- after correcting over a marker, require five fresh `vec_to_marker_3d` results whose horizontal error is at most 0.50 m before LAND;
- after precision touchdown, call `controller.disarm()` and `dropper.attach(target_id)` before takeoff;
- make `hold_waypoint_until_stable` use horizontal speed `hypot(vehicle.velocity[0], vehicle.velocity[1]) <= 0.10`, distance `<= 0.15`, and two seconds on the injected clock;
- implement `DroneControl.disarm()` by setting `vehicle.armed = False` and waiting up to 15 simulated seconds for confirmation;
- preserve 10 m AGL for transit/release and return failure when the stability gate times out;
- make `fm2` and `fm3` refuse to call `drop()` when the gate returns false.

Update the test fakes to prove exactly five distinct camera timestamps are consumed, four equal timestamps cannot acquire, speed `0.100001` resets the stability window, rejected attachment prevents takeoff, and disarm waits for the DroneKit state change.

- [ ] **Step 6: Inject frames without opening camera index zero**

Change `CameraManager(frame_source=None, calibration_path=None)` so its default remains hardware-compatible, while an injected source supplies BGR arrays. Resolve default calibration with `Path(__file__).with_name("calibration.json")`. Change `Camera(marker_size_mm, manager=None)` to accept the injected manager and retain all original ArUco pose calculations.

- [ ] **Step 7: Run nested seam tests and syntax validation**

Run: `PYTHONPATH=companion/comp2026/src uv run --with dronekit==2.9.2 --with numpy==2.1.3 --with opencv-contrib-python-headless==4.10.0.84 pytest companion/comp2026/tests -q`

Run: `python3 -m compileall -q companion/comp2026/src/drone`

Expected: all nested tests and compilation PASS without hardware access.

- [ ] **Step 8: Commit only in the nested repository and do not push**

```bash
git -C companion/comp2026 add src/drone tests
git -C companion/comp2026 commit -m "Add simulation adapter seams"
git -C companion/comp2026 status --short --branch
```

Expected: the local integration branch contains one focused commit; any pre-existing unrelated untracked nested files remain uncommitted; no push command is run.

### Task 8: Host the original mission in the current companion service

**Files:**
- Create: `companion/src/drone_sim_companion/comp2026_host.py`
- Modify: `companion/src/drone_sim_companion/runtime_node.py`
- Modify: `companion/pyproject.toml`
- Modify: `companion/Dockerfile`
- Create: `companion/tests/test_comp2026_host.py`
- Modify: `companion/tests/test_runtime_node.py`
- Modify: `companion/EXTERNAL_INTERFACE.md`
- Modify: `companion/INTERNAL_INTERFACE.md`
- Modify: `uv.lock`

**Interfaces:**
- Consumes: `/clock`, onboard image plus metadata, downward `LaserScan`, `PayloadCommand`, DroneKit TCP endpoint, and resolved competition config.
- Produces: a worker-running original attempt, `/simulation/mission_events`, existing lifecycle statuses, and payload actions that block until confirmed.

- [ ] **Step 1: Write failing adapter and coordinate tests**

Cover exact ENU-to-WGS84 conversion, simulation waits, fresh frame delivery, stale range rejection, command identity, and phase publication:

```python
def test_world_xy_to_gps_maps_x_east_and_y_north():
    home = GPSCoord(41.501900, -81.675000, 0.0)
    target = world_xy_to_gps(home, x_m=-152.40, y_m=0.0)
    assert target.lat == pytest.approx(home.lat)
    assert target.long < home.long
    assert horizontal_distance_m(home, target) == pytest.approx(152.40, abs=0.02)


def test_frame_source_never_delivers_one_frame_twice():
    source = RosFrameSource(width_px=640, height_px=480)
    source.accept(metadata(1, 50_000_000), rgb_image(50_000_000))
    assert source.capture_frame().shape == (480, 640, 3)
    with pytest.raises(StaleSensorError):
        source.capture_frame(deadline_sim_ns=100_000_000)


def test_payload_dropper_waits_for_matching_confirmation():
    client = FakePayloadClient()
    dropper = PayloadDropper(RUN_ID, 3, client, FakeClock())
    client.complete(PayloadResponse(True, "OK", "", "run:3:release:1", 1))
    dropper.drop()
    assert client.requests[0].aruco_id == 3
    assert client.requests[0].action == client.requests[0].RELEASE
```

- [ ] **Step 2: Run tests and confirm the host is absent**

Run: `uv run pytest companion/tests/test_comp2026_host.py companion/tests/test_runtime_node.py -q`

Expected: FAIL on missing host and competition runtime branch.

- [ ] **Step 3: Implement the adapter host**

Define these focused classes in `comp2026_host.py`:

```python
class SimulationClock:
    def accept(self, timestamp_ns: int) -> None: ...
    def now(self) -> float: ...
    def sleep(self, seconds: float) -> None: ...
    def stop(self, reason: str) -> None: ...


class RosFrameSource:
    def accept_image(self, message: object) -> None: ...
    def accept_metadata(self, message: object) -> None: ...
    def capture_frame(self, quality: int = 4, deadline_sim_ns: int | None = None): ...


class RosLidar:
    def accept(self, message: object, sim_timestamp_ns: int) -> None: ...
    def get_distance(self) -> float: ...


class PayloadDropper:
    def attach(self, aruco_id: int) -> None: ...
    def drop(self, delay_hold: float = 0.0) -> None: ...
```

Implement `world_xy_to_gps` with radius 6,378,137 m, `north=y`, and `east=x`. Convert ROS RGB bytes to BGR NumPy arrays without `cv_bridge`. Pair images with current-run metadata by timestamp/frame ID and block the mission worker until a strictly newer pair is available. Treat range older than 0.5 simulated seconds as stale.

- [ ] **Step 4: Add pinned original mission dependencies and image contents**

Add `dronekit==2.9.2`, `numpy==2.1.3`, and `opencv-contrib-python-headless==4.10.0.84` to the companion package and lock. Copy `companion/comp2026/src` into `/opt/drone_sim/comp2026/src`; add it to `PYTHONPATH`. Do not copy nested `.git`, recordings, or unrelated docs.

Add only the Python 3.12 compatibility aliases required before DroneKit import: `collections.MutableMapping = collections.abc.MutableMapping` and `inspect.getargspec = inspect.getfullargspec` when absent.

- [ ] **Step 5: Select the host in the existing runtime without adding a service**

Read resolved `mission`. For `controlled_descent`, retain the current PyMAVLink path. For `comp2026_auto`, create one ROS node with subscriptions/service client, connect original `DroneControl` to `tcp:ardupilot-sitl:5760`, wait for heartbeat/armable/current frame/current range/payload service, write `mission-ready`, and start `run_auto_attempt` in a worker.

Publish ordered `MissionEvent` rows from the nested emit callback. On success, require Home complete and write the existing `mission-finished`; on exception, log the phase, request best-effort RTL/land/disarm, and write runtime failure. Keep spinning callbacks until orchestration finalization, then stop the simulation clock, join the worker, write quiescence, and close DroneKit.

- [ ] **Step 6: Run companion tests and build the production image**

Run: `uv run pytest companion/tests -q`

Run: `docker build --target runtime -f companion/Dockerfile -t drone-sim-companion-comp2026 .`

Run: `docker run --rm --entrypoint python3 drone-sim-companion-comp2026 -c "import cv2, dronekit; from drone.auto_attempt import run_auto_attempt; print(cv2.__version__)"`

Expected: tests, image build, DroneKit import, OpenCV ArUco import, and nested entry-point import PASS.

- [ ] **Step 7: Commit the parent companion host locally**

```bash
git add companion/Dockerfile companion/pyproject.toml companion/src companion/tests companion/EXTERNAL_INTERFACE.md companion/INTERNAL_INTERFACE.md uv.lock
git commit -m "Host original comp2026 mission"
```

### Task 9: Wire competition lifecycle, provenance, recordings, and independent acceptance

**Files:**
- Modify: `compose.yaml`
- Modify: `orchestration/src/orchestration/controller.py`
- Modify: `orchestration/src/orchestration/_adapters/compose.py`
- Modify: `orchestration/tests/test_controller.py`
- Modify: `orchestration/tests/test_cli.py`
- Modify: `artifacts/src/artifacts/_adapters/rosbag.py`
- Create: `artifacts/src/artifacts/competition_score_validation.py`
- Modify: `artifacts/src/artifacts/score_validation.py`
- Modify: `artifacts/src/artifacts/acceptance.py`
- Modify: `artifacts/src/artifacts/runtime_configuration.py`
- Modify: `artifacts/recording-qos.yaml`
- Modify: `config/recording-qos.yaml`
- Modify: `artifacts/tests/test_rosbag_adapter.py`
- Create: `artifacts/tests/test_competition_score_validation.py`
- Modify: `artifacts/tests/test_acceptance.py`
- Create: `scripts/inspect_competition_run.py`
- Create: `scripts/write_competition_verification.py`
- Modify: `Makefile`
- Modify: `EXTERNAL_INTERFACE.md`
- Modify: `INTERNAL_INTERFACE.md`
- Modify: module interface documents affected by new topics and scenario selection

**Interfaces:**
- Consumes: unchanged seven-image Compose topology, parent and nested Git worktrees, competition messages, rules, and complete bundle.
- Produces: manifest source revisions for both repositories, profile-aware bag validation, independent physical `150/150` acceptance, and generated handoff evidence.

- [ ] **Step 1: Write failing provenance, topic, and acceptance tests**

Require two source revisions and competition-specific bag evidence:

```python
def test_source_revisions_include_parent_and_nested_repositories(controller):
    assert controller._source_revisions(15.0) == (
        SourceRevision("drone_sim", "a" * 40, False),
        SourceRevision("comp2026", "b" * 40, True),
    )


def test_competition_bag_contract_adds_physical_and_mission_topics():
    assert COMPETITION_TOPICS == BASE_TOPICS + (
        "/simulation/payload_state",
        "/simulation/payload_events",
        "/simulation/mission_events",
    )


def test_acceptance_requires_exact_competition_score_and_nested_revision(run_dir):
    report = inspect_phase3_bundle(
        run_dir,
        rules_path=COMPETITION_RULES,
        expected_source_revisions={"drone_sim": "a" * 40, "comp2026": "b" * 40},
        expected_source_dirty={"drone_sim": False, "comp2026": True},
        expected_image_digests=EXPECTED_IMAGE_DIGESTS,
        require_maximum_score=True,
    )
    assert report.achieved_score == report.maximum_available_score == 150.0
```

- [ ] **Step 2: Run focused tests and confirm descent-only assumptions fail**

Run: `uv run pytest orchestration/tests/test_controller.py artifacts/tests/test_rosbag_adapter.py artifacts/tests/test_competition_score_validation.py artifacts/tests/test_acceptance.py -q`

Expected: FAIL because provenance contains one repository and artifact semantics are hard-coded to descent.

- [ ] **Step 3: Record parent and nested source revisions and bind the companion image label**

Generalize `_source_revisions` to run `rev-parse HEAD` and `status --porcelain --untracked-files=normal` against the parent and `companion/comp2026`, using one shared deadline. Return names `drone_sim` then `comp2026` in fixed order.

Pass `SIM_COMP2026_REVISION` into the Compose build args and set `org.opencontainers.image.comp2026.revision` on the companion image. Assert the label equals the nested source revision before launch. Do not add an eighth service or a remote Git operation.

- [ ] **Step 4: Make rosbag recording profile-aware**

Keep `BASE_TOPICS` unchanged for phase 2 and descent. Add the three competition topics with exact message types and QoS. Validate configured image width, height, step, and payload bytes rather than fixed `320x240`. For competition require exactly three payload states per 50 ms grid tick, one for each ID, contiguous event IDs, and timestamp-monotonic mission/payload events. Preserve lifecycle and camera/ground-truth alignment checks.

- [ ] **Step 5: Independently recompute competition scoring from the bag**

Implement a separate validator that does not trust scorekeeper result booleans. Decode vehicle, payload, mission, and payload-event evidence; re-evaluate FM1 landing, ordered physical attach/release, two-second release gates, one-second settling, rotated payload bounds, Home deadline, and the seven point components. Require persisted events 0-7 and result JSON to match the independent result exactly. Dispatch by `ruleset_id`; retain the existing descent validator unchanged.

- [ ] **Step 6: Add operator inspection and verification-note scripts**

`scripts/inspect_competition_run.py RUN_DIRECTORY` must query current parent/nested revisions and dirty flags, inspect the seven local image digests, call `inspect_phase3_bundle` with `competition_v1.json`, require maximum score, and print one canonical JSON report.

`scripts/write_competition_verification.py RUN_DIRECTORY OUTPUT` must read only an already accepted bundle and write Markdown containing the run ID, both source revisions, terminal state, elapsed simulated time, score, rules checksum, manifest path, onboard/observer video paths, rosbag path, and the three cumulative checkpoints. Refuse a non-`COMPLETED` or non-`150/150` manifest.

- [ ] **Step 7: Update Makefile and interface documentation**

Add:

```make
inspect-competition:
	test -n "$(RUN_DIRECTORY)"
	uv run python scripts/inspect_competition_run.py "$(RUN_DIRECTORY)"
```

Document exact topic ownership, service behavior, nested local-commit handoff limitation, default command, failure behavior, and deferred QGC/retry work. Do not add a review/hardening milestone.

- [ ] **Step 8: Run all non-live tests and build all seven images**

Run: `uv run pytest orchestration/tests artifacts/tests companion/tests electromagnet/tests gazebo/tests scorekeeper/tests tests/contracts -q`

Run: `docker compose --profile phase3 build`

Expected: all tests PASS and all seven production images build.

- [ ] **Step 9: Commit lifecycle, evidence, and handoff tooling**

```bash
git add compose.yaml orchestration artifacts config/recording-qos.yaml scripts Makefile EXTERNAL_INTERFACE.md INTERNAL_INTERFACE.md companion/EXTERNAL_INTERFACE.md companion/INTERNAL_INTERFACE.md electromagnet/EXTERNAL_INTERFACE.md electromagnet/INTERNAL_INTERFACE.md gazebo/EXTERNAL_INTERFACE.md gazebo/INTERNAL_INTERFACE.md scorekeeper/EXTERNAL_INTERFACE.md scorekeeper/INTERNAL_INTERFACE.md
git commit -m "Wire competition runtime evidence"
```

### Task 10: Produce and preserve the real `150/150` MVP run

**Files:**
- Create after acceptance: `docs/verification/comp2026-mvp.md`
- Modify only when a failing live fact requires it: the owning files and focused tests from Tasks 3-9

**Interfaces:**
- Consumes: the committed default configuration, seven built images, local nested integration commit, and at least 35 GiB free in the run filesystem.
- Produces: one accepted run directory, both visually inspected recordings, a generated verification note, and final local commits with no remote push.

- [ ] **Step 1: Preflight source state, disk, and stale services without deleting data**

Run: `git status --short --branch`

Run: `git -C companion/comp2026 status --short --branch`

Run: `df -BG runs .`

Run: `docker compose --profile phase3 down --remove-orphans`

Expected: both exact revisions are readable, no integration code is unintentionally dirty, at least 35 GiB is available, and no old service remains. If capacity is below 35 GiB, stop before launching and use the user's separately authorized recording-cleanup decision against explicit old run directories.

- [ ] **Step 2: Run the focused full local verification before the expensive attempt**

Run: `uv run pytest orchestration/tests artifacts/tests companion/tests electromagnet/tests gazebo/tests scorekeeper/tests tests/contracts -q`

Run: `docker compose --profile phase3 build`

Expected: all tests and all image builds PASS.

- [ ] **Step 3: Start one complete automatic attempt through the operator command**

Run:

```bash
uv run drone-sim start | tee /tmp/comp2026-start.jsonl
COMP_RUN_DIRECTORY="$(uv run python -c 'import json, pathlib; rows=[line for line in pathlib.Path("/tmp/comp2026-start.jsonl").read_text().splitlines() if line.startswith("{")]; result=json.loads(rows[-1]); print(pathlib.Path("runs") / result["run_id"])')"
test -d "$COMP_RUN_DIRECTORY"
```

Expected: the command blocks through the run lifecycle and returns exit code 0 with canonical JSON naming the new run ID and `COMPLETED` terminal state. It must not require QGC input.

- [ ] **Step 4: Inspect the bundle independently**

Set the run path from the command result, then run:

```bash
COMP_RUN_DIRECTORY="$(uv run python -c 'import json, pathlib; rows=[line for line in pathlib.Path("/tmp/comp2026-start.jsonl").read_text().splitlines() if line.startswith("{")]; result=json.loads(rows[-1]); print(pathlib.Path("runs") / result["run_id"])')"
make inspect-competition RUN_DIRECTORY="$COMP_RUN_DIRECTORY"
```

Expected: PASS with independently recomputed `150/150`, FM2 checkpoint 80, marker-3 checkpoint 145, marker-4 checkpoint 150, Home landed/disarmed before 600 simulated seconds, two source revisions, seven image digests, and complete required artifacts.

- [ ] **Step 5: Inspect both camera recordings and payload continuity**

Run:

```bash
COMP_RUN_DIRECTORY="$(uv run python -c 'import json, pathlib; rows=[line for line in pathlib.Path("/tmp/comp2026-start.jsonl").read_text().splitlines() if line.startswith("{")]; result=json.loads(rows[-1]); print(pathlib.Path("runs") / result["run_id"])')"
ffprobe -v error -show_entries stream=width,height,avg_frame_rate,nb_frames -of json "$COMP_RUN_DIRECTORY/video/onboard.mp4"
ffprobe -v error -show_entries stream=width,height,avg_frame_rate,nb_frames -of json "$COMP_RUN_DIRECTORY/video/observer.mp4"
ffmpeg -y -i "$COMP_RUN_DIRECTORY/video/observer.mp4" -vf "fps=1/60,scale=960:-1,tile=5x2" -frames:v 1 /tmp/comp2026-observer-contact-sheet.png
ffmpeg -y -i "$COMP_RUN_DIRECTORY/video/onboard.mp4" -vf "fps=1/60,scale=960:-1,tile=5x2" -frames:v 1 /tmp/comp2026-onboard-contact-sheet.png
```

Open both contact sheets and verify visible H takeoff, L landing, all three physical falls at F2, centered payload pickups without snapping, and final Home landing. Confirm each video is `640x480`, 20 fps, and has the configured frame count.

- [ ] **Step 6: Resolve only evidence-backed live blockers with a red test first**

If Steps 3-5 fail, identify the first failing phase from `logs/companion.jsonl`, `logs/gazebo.jsonl`, payload events, score events, ground truth, and both videos. Add one focused regression test in the owning module reproducing that exact fact, confirm it fails, apply the smallest patch, rerun that module's tests, rebuild only affected images, and start a fresh full attempt. Do not weaken physical thresholds, fabricate events, bypass ArUco frames, teleport payloads, or accept a partial score.

- [ ] **Step 7: Generate the handoff verification note from accepted evidence**

Run:

```bash
COMP_RUN_DIRECTORY="$(uv run python -c 'import json, pathlib; rows=[line for line in pathlib.Path("/tmp/comp2026-start.jsonl").read_text().splitlines() if line.startswith("{")]; result=json.loads(rows[-1]); print(pathlib.Path("runs") / result["run_id"])')"
uv run python scripts/write_competition_verification.py "$COMP_RUN_DIRECTORY" docs/verification/comp2026-mvp.md
```

Read the generated note and compare its revisions, score, timing, and paths against `manifest.json`.

- [ ] **Step 8: Commit the accepted evidence note and any focused blocker fixes**

```bash
git add docs/verification/comp2026-mvp.md
git commit -m "Record verified comp2026 MVP run"
```

If live fixes were needed, each owning red-test/fix pair must already have its own local commit before this evidence commit. Do not commit run recordings and do not push either repository.

- [ ] **Step 9: Perform the completion audit**

Run: `git log --oneline --decorate -12`

Run: `git -C companion/comp2026 log --oneline --decorate -5`

Run: `git status --short --branch`

Run: `git -C companion/comp2026 status --short --branch`

Run:

```bash
COMP_RUN_DIRECTORY="$(uv run python -c 'import json, pathlib; rows=[line for line in pathlib.Path("/tmp/comp2026-start.jsonl").read_text().splitlines() if line.startswith("{")]; result=json.loads(rows[-1]); print(pathlib.Path("runs") / result["run_id"])')"
make inspect-competition RUN_DIRECTORY="$COMP_RUN_DIRECTORY"
```

Compare the independent report item-by-item with every acceptance requirement in the design spec. Completion is proven only when the report passes, both videos were inspected, the verification note names the same bundle and revisions, no required implementation change is uncommitted, and no remote push occurred.
