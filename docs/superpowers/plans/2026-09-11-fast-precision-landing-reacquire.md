# Fast Precision-Landing Reacquisition Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Preserve a 0.50 m/s precision descent while making every payload landing reject unhealthy camera data, hold and reacquire safely, retry acquisition once, and produce a fresh valid 150/150 run artifact.

**Architecture:** The camera returns each marker vector and its source timestamp atomically. The mission layer owns a small precision-landing state machine and fixed earth-frame target anchor; it filters observations before they reach ArduPilot, moves to GUIDED for a fixed-position hold after 0.50 simulated seconds without healthy data, and either resumes LAND after five healthy frames or returns once to the 4.572 m acquisition hover. The controller provides only the shallow MAVLink and runtime-parameter operations needed by that policy.

**Tech Stack:** Python 3.11, pytest, DroneKit/pymavlink, ROS 2 camera bridge, ArduPilot Copter 4.7 SITL, Gazebo, Docker Compose, MCAP/rosbag2, DataFlash logs

**Spec:** `docs/superpowers/specs/2026-09-11-fast-precision-landing-reacquire-design.md`

**Completed:** 2026-09-11. Fresh run
`b3dfad75-4630-4233-84e3-836943459903` passed canonical semantic acceptance at
150/150 with all payloads delivered and Home complete. The run also exposed and
verified a final-descent refinement: below `PLND_ALT_MIN=0.75`, normal marker
loss no longer interrupts LAND before touchdown.

## Global Constraints

- Keep `LAND_SPD_MS=0.50` and `PLND_OPTIONS=4`; do not slow the final descent to mask bad observations.
- Keep the promoted AutoTune roll gains and verified pitch gains unchanged.
- The exact profile is `PLND_ENABLED=1`, `PLND_TYPE=1`, `PLND_EST_TYPE=0`, `PLND_LAG=0.08`, `PLND_XY_DIST_MAX=0.50`, `PLND_STRICT=2`, `PLND_RET_MAX=1`, `PLND_TIMEOUT=0.50`, `PLND_ALT_MIN=0.75`, `PLND_ALT_MAX=8.0`, `PLND_OPTIONS=4`, and `LAND_SPD_MS=0.50`.
- Integer parameters must match exactly; fractional parameters use `math.isclose(..., rel_tol=0.0, abs_tol=1e-3)`.
- Use simulated time for all freshness windows, holds, retries, and the existing 60-second landing deadline.
- Accept only a strictly newer, non-future camera timestamp no older than 0.25 simulated seconds; finite body-FRD coordinates with positive down; a LiDAR sample no older than 0.50 simulated seconds; at most 0.20 m earth-frame anchor drift; and at most 0.50 m horizontal marker error during reacquisition.
- Initial acquisition requires five distinct healthy centered observations whose projected earth target locations fit within 0.20 m; use their coordinate-wise median as the immutable anchor.
- After 0.50 simulated seconds without an acceptable observation, confirm GUIDED and hold current latitude, longitude, and relative altitude by reissuing an integer waypoint every 0.20 simulated seconds. Send no `LANDING_TARGET` while GUIDED.
- Resume LAND after five consecutive acceptable hold observations. If none arrive within 5.0 simulated seconds, return to 4.572 m AGL and run the existing acquisition once more. Fail closed if that second landing attempt fails.
- Never disarm or attach without confirmed touchdown. Do not change the scorekeeper or public ROS schemas.
- Emit compact sorted JSON diagnostics prefixed by `PRECISION_LANDING ` for every state transition and rejected observation.
- Parent and nested repositories are committed separately. Parent runtime image provenance must identify the exact nested `companion/comp2026` commit.
- Completion requires a fresh `COMPLETED` artifact scoring exactly 150/150, physical attach/lift/release/delivery evidence for payloads 2, 3, and 4, home touchdown, valid onboard and observer videos, a valid bag, the exact runtime profile and promoted gains in DataFlash, and acquired-target attitude limits of desired roll/pitch at most 5 degrees and actual roll/pitch at most 8 degrees.

---

### Task 1: Pin the fast ArduPilot precision-landing profile

**Files:**
- Modify: `ardupilot_sitl/tests/test_config.py`
- Modify: `ardupilot_sitl/params/descent.parm`

**Interfaces:**
- Consumes: `_descent_parameters() -> dict[str, str]`
- Produces: the exact runtime parameter profile listed in Global Constraints for image builds and companion preflight validation

- [ ] **Step 1: Replace the slow-speed assertion and add one exact profile test**

```python
def test_descent_parameters_use_fast_guarded_precision_landing_profile() -> None:
    parameters = _descent_parameters()

    assert {name: parameters[name] for name in (
        "PLND_ENABLED", "PLND_TYPE", "PLND_EST_TYPE", "PLND_STRICT",
        "PLND_RET_MAX", "PLND_OPTIONS",
    )} == {
        "PLND_ENABLED": "1", "PLND_TYPE": "1", "PLND_EST_TYPE": "0",
        "PLND_STRICT": "2", "PLND_RET_MAX": "1", "PLND_OPTIONS": "4",
    }
    assert {name: float(parameters[name]) for name in (
        "LAND_SPD_MS", "PLND_LAG", "PLND_XY_DIST_MAX", "PLND_TIMEOUT",
        "PLND_ALT_MIN", "PLND_ALT_MAX",
    )} == pytest.approx({
        "LAND_SPD_MS": 0.50, "PLND_LAG": 0.08, "PLND_XY_DIST_MAX": 0.50,
        "PLND_TIMEOUT": 0.50, "PLND_ALT_MIN": 0.75, "PLND_ALT_MAX": 8.0,
    })
```

- [ ] **Step 2: Run the focused test and verify it fails on `LAND_SPD_MS=0.10` or a missing key**

Run: `uv run --locked pytest ardupilot_sitl/tests/test_config.py::test_descent_parameters_use_fast_guarded_precision_landing_profile -q`

Expected: FAIL because the checked-in parameter overlay does not yet contain the approved profile.

- [ ] **Step 3: Update the parameter overlay with the exact approved values**

```text
LAND_SPD_MS 0.50
PLND_ENABLED 1
PLND_TYPE 1
PLND_EST_TYPE 0
PLND_LAG 0.08
PLND_XY_DIST_MAX 0.50
PLND_STRICT 2
PLND_RET_MAX 1
PLND_TIMEOUT 0.50
PLND_ALT_MIN 0.75
PLND_ALT_MAX 8.0
PLND_OPTIONS 4
```

Keep all `ATC_*` values byte-for-byte unchanged and update the adjacent comments to explain that companion-side gating makes the fast final descent safe.

- [ ] **Step 4: Run the complete parent ArduPilot configuration tests**

Run: `uv run --locked pytest ardupilot_sitl/tests/test_config.py -q`

Expected: PASS.

- [ ] **Step 5: Commit the parent change**

```bash
git add ardupilot_sitl/tests/test_config.py ardupilot_sitl/params/descent.parm
git commit -m "fix: restore guarded fast precision landing profile"
```

### Task 2: Return marker data and frame time atomically

**Files:**
- Modify: `companion/comp2026/src/drone/sensors/camera/camera.py`
- Modify: `companion/comp2026/src/drone/sensors/camera/_camera_manager.py`
- Modify: `companion/comp2026/tests/test_camera_body_mapping.py`
- Modify: `companion/comp2026/tests/test_simulation_mission_seams.py`

**Interfaces:**
- Consumes: frame sources exposing `capture_frame(quality: int = 4, deadline_sim_ns: int | None = None)` and `last_frame_timestamp: int | None`
- Produces: immutable `MarkerObservation(vector: RelPosComplete | None, frame_timestamp_ns: int)` and `Camera.observe_marker_3d(id: int, lidar_alt: float | None = None, quality: int = 4, deadline_sim_ns: int | None = None) -> MarkerObservation | None`
- Preserves: `Camera.vec_to_marker_3d(...) -> RelPosComplete | None`

- [ ] **Step 1: Add tests for atomic timestamps, marker misses, and bounded no-frame behavior**

```python
def test_marker_observation_keeps_vector_and_source_timestamp_together():
    camera = Camera(100, manager=FixedMarkerPose())
    observation = camera.observe_marker_3d(3)
    assert observation is not None
    assert observation.frame_timestamp_ns == 170_000_000_000
    assert observation.vector == RelPosComplete(0.3, 0.2, 4.1)

def test_marker_observation_returns_timestamp_even_when_target_is_absent():
    observation = Camera(100, manager=NoMarkerFrame()).observe_marker_3d(3)
    assert observation is not None
    assert observation.vector is None
    assert observation.frame_timestamp_ns == NoMarkerFrame.last_frame_timestamp

def test_marker_observation_propagates_capture_deadline():
    manager = DeadlineRecordingMarkerPose()
    Camera(100, manager=manager).observe_marker_3d(3, deadline_sim_ns=125)
    assert manager.deadlines == [125]
```

Update camera test doubles to accept the optional deadline argument.

- [ ] **Step 2: Run the focused tests and verify the new API is absent**

Run: `PYTHONPATH=companion/comp2026/src uv run --locked pytest companion/comp2026/tests/test_camera_body_mapping.py -q`

Expected: FAIL with `AttributeError: 'Camera' object has no attribute 'observe_marker_3d'`.

- [ ] **Step 3: Add the immutable observation and a deadline-aware capture seam**

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class MarkerObservation:
    vector: RelPosComplete | None
    frame_timestamp_ns: int

def observe_marker_3d(self, id, lidar_alt=None, quality=4, deadline_sim_ns=None):
    frame = self.cm.capture_frame(quality=quality, deadline_sim_ns=deadline_sim_ns)
    timestamp = self.cm.last_frame_timestamp
    if timestamp is None:
        return None
    self._buffer_frame(frame)
    vector = self._vector_from_frame_3d(frame, id, lidar_alt)
    return MarkerObservation(vector=vector, frame_timestamp_ns=int(timestamp))
```

Extract the existing mapping into `_vector_from_frame_3d`. Implement `vec_to_marker_3d` as a compatibility wrapper over `observe_marker_3d`. In `CameraManager.capture_frame`, accept `deadline_sim_ns`; pass it to the injected frame source, while the physical webcam path ignores it.

- [ ] **Step 4: Run all nested camera and host frame-source tests**

Run: `PYTHONPATH=companion/comp2026/src uv run --locked pytest companion/comp2026/tests/test_camera_body_mapping.py companion/comp2026/tests/test_simulation_mission_seams.py companion/tests/test_comp2026_host.py -q`

Expected: PASS.

- [ ] **Step 5: Commit the nested change**

```bash
git -C companion/comp2026 add src/drone/sensors/camera/camera.py src/drone/sensors/camera/_camera_manager.py tests/test_camera_body_mapping.py tests/test_simulation_mission_seams.py
git -C companion/comp2026 commit -m "feat: expose atomic marker observations"
```

### Task 3: Add exact parameter validation and nonblocking hold commands

**Files:**
- Modify: `companion/comp2026/src/drone/control/drone_control.py`
- Modify: `companion/comp2026/tests/test_drone_control_connection.py`
- Modify: `companion/comp2026/tests/test_simulation_mission_seams.py`

**Interfaces:**
- Consumes: `DroneControl.vehicle.parameters`, `GPSCoord(lat, long, alt)`
- Produces: `DroneControl.send_guided_waypoint(coord: GPSCoord) -> int` and `DroneControl.require_precision_landing_profile() -> bool`

- [ ] **Step 1: Add focused controller tests**

```python
def test_send_guided_waypoint_preserves_integer_lat_lon():
    controller = controller_without_connect(StableVehicle(FakeClock()))
    assert controller.send_guided_waypoint(GPSCoord(41.12345678, -81.87654321, 4.572)) == 0
    args = controller.vehicle._master.mav.items[-1]
    assert args[11:14] == (411234568, -818765432, 4.572)

def test_precision_landing_profile_accepts_exact_runtime_values():
    controller = controller_without_connect(ParameterVehicle(valid_profile()))
    assert controller.require_precision_landing_profile() is True

@pytest.mark.parametrize("name,value", [("PLND_OPTIONS", 0), ("LAND_SPD_MS", 0.10)])
def test_precision_landing_profile_rejects_mismatch(name, value):
    parameters = valid_profile()
    parameters[name] = value
    controller = controller_without_connect(ParameterVehicle(parameters))
    assert controller.require_precision_landing_profile() is False
```

- [ ] **Step 2: Run the focused tests and verify both public methods are absent**

Run: `PYTHONPATH=companion/comp2026/src uv run --locked pytest companion/comp2026/tests/test_drone_control_connection.py companion/comp2026/tests/test_simulation_mission_seams.py -q`

Expected: FAIL with missing `send_guided_waypoint` and `require_precision_landing_profile` attributes.

- [ ] **Step 3: Implement the two shallow controller operations**

```python
PRECISION_LANDING_PARAMETERS = {
    "LAND_SPD_MS": 0.50, "PLND_ENABLED": 1, "PLND_TYPE": 1,
    "PLND_EST_TYPE": 0, "PLND_LAG": 0.08, "PLND_XY_DIST_MAX": 0.50,
    "PLND_STRICT": 2, "PLND_RET_MAX": 1, "PLND_TIMEOUT": 0.50,
    "PLND_ALT_MIN": 0.75, "PLND_ALT_MAX": 8.0, "PLND_OPTIONS": 4,
}

def send_guided_waypoint(self, coord: GPSCoord) -> int:
    _send_guided_waypoint(self.vehicle, coord.lat, coord.long, coord.alt)
    return 0

def require_precision_landing_profile(self) -> bool:
    for name, expected in PRECISION_LANDING_PARAMETERS.items():
        actual = self.vehicle.parameters.get(name)
        if actual is None:
            return False
        if isinstance(expected, int):
            matches = int(actual) == expected and float(actual).is_integer()
        else:
            matches = math.isclose(float(actual), expected, rel_tol=0.0, abs_tol=1e-3)
        if not matches:
            return False
    return True
```

Log each missing or mismatched name with expected and observed values; do not write parameters at runtime.

- [ ] **Step 4: Run the complete nested controller tests**

Run: `PYTHONPATH=companion/comp2026/src uv run --locked pytest companion/comp2026/tests/test_drone_control_connection.py companion/comp2026/tests/test_simulation_mission_seams.py -q`

Expected: PASS.

- [ ] **Step 5: Commit the nested change**

```bash
git -C companion/comp2026 add src/drone/control/drone_control.py tests/test_drone_control_connection.py tests/test_simulation_mission_seams.py
git -C companion/comp2026 commit -m "feat: guard precision landing controller inputs"
```

### Task 4: Implement observation filtering and hold/reacquire landing

**Files:**
- Modify: `companion/comp2026/src/drone/mock_mission.py`
- Modify: `companion/comp2026/tests/test_simulation_mission_seams.py`

**Interfaces:**
- Consumes: `MarkerObservation`, `DroneControl.send_guided_waypoint`, mode methods, current GPS/attitude, LiDAR range, immutable anchor `GPSCoord`, and a caller-supplied absolute landing deadline
- Produces: `aruco_land_precision(controller, camera, lidar, target_id, anchor, deadline) -> Literal["touchdown", "retry", "failed"]`

- [ ] **Step 1: Add deterministic state-machine fixtures and rejection tests**

Create a scripted camera whose observations include vector and nanosecond timestamp, a range fixture with sample timestamps, and a controller that records requested/observed modes, landing targets, and earth-fixed hold waypoints. Add tests proving:

```python
def test_landing_sends_only_new_fresh_finite_anchor_consistent_observations(): ...
def test_single_bad_frame_does_not_leave_land_mode(): ...
def test_half_second_without_healthy_target_enters_guided_hold(): ...
def test_guided_hold_sends_no_landing_targets_and_reissues_fixed_waypoint(): ...
def test_five_consecutive_healthy_hold_frames_resume_land_once(): ...
def test_bad_hold_frame_resets_reacquisition_count(): ...
def test_five_second_hold_requests_one_search_hover_retry(): ...
def test_landing_deadline_failure_never_disarms_or_attaches(): ...
def test_precision_landing_diagnostics_are_sorted_compact_json(capsys): ...
```

Each fixture uses `FakeClock`, so a 0.50-second health timeout, 0.20-second waypoint cadence, 5.0-second hold timeout, and 60-second shared deadline are asserted without wall-clock sleeps.

- [ ] **Step 2: Run the new tests and verify the legacy loop fails the recovery contract**

Run: `PYTHONPATH=companion/comp2026/src uv run --locked pytest companion/comp2026/tests/test_simulation_mission_seams.py -k 'landing and (fresh or hold or reacqui or diagnostic or deadline)' -q`

Expected: FAIL because the legacy landing forwards all detected vectors and never transitions through GUIDED hold.

- [ ] **Step 3: Implement explicit state and acceptance helpers**

```python
class LandingResult(Enum):
    TOUCHDOWN = "touchdown"
    RETRY = "retry"
    FAILED = "failed"

def _precision_log(**fields):
    print("PRECISION_LANDING " + json.dumps(fields, sort_keys=True, separators=(",", ":")))

def _observation_reason(observation, *, now_ns, last_timestamp_ns, lidar_age_s,
                        horizontal_error_m, anchor_drift_m):
    if observation is None: return "no_frame"
    if observation.frame_timestamp_ns <= last_timestamp_ns: return "not_new"
    if observation.frame_timestamp_ns > now_ns: return "future_frame"
    if (now_ns - observation.frame_timestamp_ns) / 1e9 > 0.25: return "stale_frame"
    if observation.vector is None: return "target_absent"
    if not all(math.isfinite(v) for v in (observation.vector.x, observation.vector.y, observation.vector.z)): return "non_finite"
    if observation.vector.z <= 0: return "non_positive_down"
    if lidar_age_s > 0.50: return "stale_lidar"
    if anchor_drift_m > 0.20: return "anchor_drift"
    if horizontal_error_m > 0.50: return "horizontal_error"
    return None
```

Use a bounded `observe_marker_3d(..., deadline_sim_ns=...)` call each cycle. Keep the latest valid LiDAR value and timestamp. In TRACKING, send only accepted observations. Once `now - last_healthy >= 0.50`, confirm GUIDED, snapshot current GPS including relative altitude, then reissue that exact coordinate every 0.20 seconds. In HOLD_REACQUIRE, send no landing targets; count consecutive accepted observations, reset on rejection, and enter LAND exactly once at five. Return `RETRY` at 5.0 seconds and `FAILED` at the shared deadline or any failed mode confirmation.

- [ ] **Step 4: Run the state-machine suite and all nested tests**

Run: `PYTHONPATH=companion/comp2026/src uv run --locked pytest companion/comp2026/tests -q`

Expected: PASS.

- [ ] **Step 5: Commit the nested change**

```bash
git -C companion/comp2026 add src/drone/mock_mission.py tests/test_simulation_mission_seams.py
git -C companion/comp2026 commit -m "feat: hold and reacquire unhealthy precision landings"
```

### Task 5: Anchor acquisition and permit exactly one landing retry

**Files:**
- Modify: `companion/comp2026/src/drone/mock_mission.py`
- Modify: `companion/comp2026/tests/test_simulation_mission_seams.py`
- Modify: `companion/comp2026/tests/test_auto_attempt.py`

**Interfaces:**
- Consumes: filtered atomic observations, `_marker_offset_ne`, `LandingResult`, exact runtime profile validation
- Produces: `pickup_sequence(...) -> bool` with five-sample median anchoring and no more than two landing attempts inside one 60-second budget

- [ ] **Step 1: Add acquisition and retry integration tests**

```python
def test_pickup_requires_runtime_precision_landing_profile_before_land(): ...
def test_acquisition_anchor_is_coordinate_median_of_five_projected_targets(): ...
def test_acquisition_rejects_five_points_with_more_than_point_two_metre_spread(): ...
def test_pickup_returns_to_4572m_and_reacquires_after_first_hold_timeout(): ...
def test_pickup_allows_only_one_reacquisition_retry(): ...
def test_failed_second_landing_never_disarms_or_attaches(): ...
def test_touchdown_after_retry_disarms_then_attaches_exact_target(): ...
```

Assert five distinct timestamps, the exact `4.572` m retry altitude, the immutable median anchor passed to both landing attempts, one call to `require_precision_landing_profile`, and event ordering `touchdown -> disarm -> attach`.

- [ ] **Step 2: Run the focused tests and verify failure under the one-shot legacy pickup**

Run: `PYTHONPATH=companion/comp2026/src uv run --locked pytest companion/comp2026/tests/test_simulation_mission_seams.py companion/comp2026/tests/test_auto_attempt.py -k 'acquisition or retry or profile or touchdown' -q`

Expected: FAIL because acquisition has no median anchor and pickup has no bounded second attempt.

- [ ] **Step 3: Extract acquisition and wire the bounded retry loop**

```python
def _acquire_target_anchor(controller, camera, lidar, target_id, deadline):
    # Existing grid search and centering behavior, but collect exactly five
    # accepted projected target coordinates with distinct source timestamps.
    # Reject a set whose max pairwise spread exceeds 0.20 m.
    # Return GPSCoord(median latitude, median longitude, 0.0), or None.

def pickup_sequence(...):
    deadline = time.time() + 60.0
    if not controller.require_precision_landing_profile():
        return False
    for attempt in range(2):
        anchor = _acquire_target_anchor(..., deadline)
        if anchor is None:
            return False
        result = aruco_land_precision(..., anchor, deadline)
        if result is LandingResult.TOUCHDOWN:
            if controller.disarm() != 0:
                return False
            return dropper.attach(target_id) is True
        if result is not LandingResult.RETRY or attempt == 1:
            return False
        if controller.set_guided_mode() != 0:
            return False
        retry_hover = controller.get_current_gps()
        retry_hover.alt = TARGET_HOVER_HEIGHT
        if controller.goto_waypoint(retry_hover, position_tol=0.15) != 0:
            return False
    return False
```

Do not reset `deadline` between acquisition, hold, or retry. Preserve the existing public `pickup_sequence -> bool` interface used by `fm3` and `auto_attempt`.

- [ ] **Step 4: Run every nested companion test**

Run: `PYTHONPATH=companion/comp2026/src uv run --locked pytest companion/comp2026/tests -q`

Expected: PASS.

- [ ] **Step 5: Commit the nested integration**

```bash
git -C companion/comp2026 add src/drone/mock_mission.py tests/test_simulation_mission_seams.py tests/test_auto_attempt.py
git -C companion/comp2026 commit -m "fix: retry one unhealthy payload approach"
```

### Task 6: Update operator and module documentation

**Files:**
- Modify: `companion/README.md`
- Modify: `ardupilot_sitl/README.md`
- Modify: `docs/architecture.md`
- Modify: `docs/runbook.md`
- Modify: `docs/handoff.md`

**Interfaces:**
- Consumes: implemented behavior and exact executable locations from Tasks 1-5
- Produces: one accurate human path from `README.md` to module responsibilities, runtime checks, recovery diagnostics, build provenance, and verification limits

- [ ] **Step 1: Update affected documentation without duplicating the parameter table**

Document in `companion/README.md` that `mock_mission.py` owns acquisition, filtering, hold/reacquire, one retry, and fail-closed attachment; link exact fields to `ardupilot_sitl/params/descent.parm`. Document in `ardupilot_sitl/README.md` that the overlay owns the fast guarded profile and that source changes require an image rebuild. Update `docs/architecture.md` with the atomic camera observation and ownership boundary. Add runbook commands for `PRECISION_LANDING` log inspection, runtime parameter extraction, bag/video validation, and ATT/PL angle checks. Update handoff with implementation status but explicitly mark the 150/150 run unverified until Task 8.

- [ ] **Step 2: Verify documentation links and command names**

Run: `uv run --locked pytest companion/tests ardupilot_sitl/tests -q`

Run: `rg -n 'PRECISION_LANDING|descent.parm|150/150|4\.572' companion/README.md ardupilot_sitl/README.md docs/architecture.md docs/runbook.md docs/handoff.md`

Expected: tests PASS; each behavior appears in its owning guide, while the exact parameter list appears only in the executable `.parm` file and its config test.

- [ ] **Step 3: Commit parent documentation and record the nested revision**

```bash
git add companion/README.md ardupilot_sitl/README.md docs/architecture.md docs/runbook.md docs/handoff.md
git commit -m "docs: explain precision landing recovery"
git status --short --branch
git -C companion/comp2026 status --short --branch
```

Expected: both repositories clean except the parent listing the preserved independent nested repository as untracked.

### Task 7: Verify code and build revision-matched runtime images

**Files:**
- Verify: all files changed in Tasks 1-6
- Generated outside Git: Docker images and build metadata

**Interfaces:**
- Consumes: clean parent and nested commits
- Produces: runtime images labeled with the exact nested commit and a tested source tree ready for an end-to-end run

- [ ] **Step 1: Run focused and complete automated checks**

Run: `uv run --locked pytest ardupilot_sitl/tests companion/tests -q`

Run: `PYTHONPATH=companion/comp2026/src uv run --locked pytest companion/comp2026/tests -q`

Expected: all tests PASS; no skips or failures relevant to the changed behavior.

- [ ] **Step 2: Check formatting and repository state**

Run: `git diff --check HEAD~4..HEAD`

Run: `git -C companion/comp2026 diff --check HEAD~3..HEAD`

Run: `git status --short --branch && git -C companion/comp2026 status --short --branch`

Expected: no whitespace errors and no uncommitted tracked changes.

- [ ] **Step 3: Build with explicit nested-revision provenance**

```bash
SIM_COMP2026_REVISION=$(git -C companion/comp2026 rev-parse HEAD) docker compose --profile phase3 build
```

Expected: build succeeds. The companion image label and subsequent run manifest identify the exact value printed by `git -C companion/comp2026 rev-parse HEAD`.

- [ ] **Step 4: Re-run the focused tests after the build**

Run: `uv run --locked pytest ardupilot_sitl/tests companion/tests -q && PYTHONPATH=companion/comp2026/src uv run --locked pytest companion/comp2026/tests -q`

Expected: PASS.

### Task 8: Produce and audit the fresh 150/150 artifact

**Files:**
- Generated: `runs/<new-run-id>/manifest.json`
- Generated: `runs/<new-run-id>/score.json`
- Generated: `runs/<new-run-id>/logs/**`
- Generated: `runs/<new-run-id>/recordings/**`
- Generated: `runs/<new-run-id>/bags/**`
- Modify after verification: `docs/handoff.md`

**Interfaces:**
- Consumes: Task 7 images and `config/default-run.json`
- Produces: the completion artifact required by the goal plus dated handoff evidence

- [ ] **Step 1: Start one fresh full simulation run**

Run: `uv run --locked drone-sim start --config config/default-run.json`

Expected: command prints a new run UUID and remains active until the orchestrator reaches a terminal state. Monitor the same process; do not start a replacement merely because the target real-time factor makes the run take roughly 70-80 wall-clock minutes.

- [ ] **Step 2: Check artifact integrity and score**

Run the runbook's artifact validation command against `runs/<new-run-id>`, then inspect `manifest.json` and `score.json`.

Expected: manifest terminal state `COMPLETED`, exact score `150/150`, bag opens, and both observer and onboard videos are present, non-empty, and decodable.

- [ ] **Step 3: Prove each physical payload lifecycle and home touchdown**

Search authoritative scorekeeper/electromagnet/runtime events for payload IDs 2, 3, and 4.

Expected: each ID has attachment, lift/carry, release, and delivery evidence in order; the vehicle subsequently lands at home. Report these physical facts separately from the numerical score.

- [ ] **Step 4: Prove the flight-controller profile and attitude limits**

Use the runbook DataFlash extraction commands on the new `.BIN` file.

Expected: all precision-landing parameters equal the Global Constraints profile; the promoted roll and pitch gains remain loaded; for every ATT sample paired to the nearest PL sample with `TAcq=1`, desired roll and pitch magnitudes are at most 5 degrees and actual roll and pitch magnitudes are at most 8 degrees.

- [ ] **Step 5: Inspect recovery behavior and recordings**

Run: `rg -n '^PRECISION_LANDING ' runs/<new-run-id>/logs`

Expected: transitions and rejected observations are parseable compact JSON, every hold/retry remains inside the shared mission window, and no landing target is sent during GUIDED. Visually inspect both videos for continuity, approach stability, all three payload interactions, and home landing.

- [ ] **Step 6: Update the dated handoff with exact evidence and commit**

Add the run UUID, parent commit, nested commit, image provenance, terminal state, score, physical outcome, artifact validation, runtime parameters, attitude maxima, and any recovery transitions to `docs/handoff.md`. Do not describe historical evidence as this run's result.

```bash
git add docs/handoff.md
git commit -m "docs: record verified 150 point precision landing run"
```

- [ ] **Step 7: Run final repository and evidence checks**

Run: `git diff --check HEAD~1..HEAD && git status --short --branch`

Run: `git -C companion/comp2026 status --short --branch`

Expected: no tracked changes remain; the independent nested repository remains preserved; the linked run directory contains the validated completion bundle.
