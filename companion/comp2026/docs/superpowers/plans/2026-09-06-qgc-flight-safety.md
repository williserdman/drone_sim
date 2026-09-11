# QGC flight safety implementation plan

> For execution after this planning task: use `superpowers:subagent-driven-development` or `superpowers:executing-plans` task by task, subject to the user's instructions. Use Sol for work involving flight behavior or interfaces. Do not add review cycles unless requested. Checkboxes record implementation progress; none are completed by writing this plan.

**Goal:** Correct the reviewed flight and deployment defects in the QGC command path before operating the 30 lb aircraft.

**Architecture:** Keep synchronous FM1, FM2, and active FM3 functions. A small mission supervisor owns command admission, cancellation, and recovery; the controller owns MAVLink translation and verified flight operations. Sensor acquisition may have workers, but only one execution path may issue mission or recovery commands.

**Tech stack:** Python, DroneKit, pymavlink, OpenCV, GPIO Zero, LIDAR-Lite, QGroundControl custom actions, and ArduCopter 4.x. Resolve the exact installed firmware and hardware dependencies in Task 1 instead of assuming all 4.x releases have identical parameters or behavior.

**Spec:** The agreed design and acceptance contract in this document. This is a plan-only deliverable, not authorization to fly or a claim that fixes exist.

## Agreed design and acceptance contract

The user selected QGC commands as the real-aircraft entry point and confirmed an independent RC transmitter. The pilot selects LOITER or STABILIZE to take over. Recovery preference is return home at cruise altitude and land, with local landing as backup. A fault is never evidence that a pilot is controlling the aircraft.

Use the original launch point H throughout FM1, FM2, and FM3. Preserve it across landing and rearming at L and pickup sites. This carries forward the proposed original-home interpretation accepted in the discussion. Record and display that point before accepting FM1. Never substitute L, a pickup site, or a saved point from an earlier flight.

For recovery, cruise is a minimum transit height relative to the original launch altitude datum. Retain a higher current altitude until reaching home, subject to the approved operating envelope. The existing listener uses 10.0 metres; retain that as the simulation starting value, not an independently established safe clearance over terrain.

### Recovery and authority

| Observed situation | Required behavior |
| --- | --- |
| Mission exception, failed operation, or QGC abort; companion authority and navigation remain valid | Terminate mission progression and payload actions; confirm a vertical climb if below return height; transit to original H; LAND; confirm touchdown and disarm. |
| Home transit is unavailable, rejected, or makes insufficient progress while companion authority remains valid | Request local LAND and confirm its state. No repeated return/land oscillation. |
| Already in a confirmed landing descent | Preserve the landing descent; do not climb away merely to return home. Report recovery as landing locally if applicable. |
| Fresh evidence confirms already landed | Stop the mission. Do not rearm to perform recovery. Disarm only with valid touchdown evidence. |
| Confirmed pilot takeover | Cancel the entire attempt, including future phases, and issue no further companion flight or payload commands. Continue monitoring/reporting. |
| Explicit flight-controller failsafe | Cancel mission progression and preserve the flight controller's safety action. Do not restore GUIDED. |
| Missing/stale telemetry or unexplained mode change | Mark authority UNKNOWN, never PILOT. Stop normal mission outputs; use the independently tested flight-controller failure policy. Do not blindly change mode or claim recovery succeeded. |

The UNKNOWN case is not permission to abandon an uncontrolled aircraft. An independent flight-controller response to companion crash, mission-loop hang, and link loss is a flight prerequisite. A live DroneKit heartbeat can conceal a stuck mission thread; test those failures separately. A companion cannot guarantee a delivered LAND command over a failed link.

Takeover evidence should combine a fresh, healthy RC flight-mode-switch transition with the resulting accepted LOITER/STABILIZE mode. A switch already held in a manual slot before QGC selects GUIDED is not a new takeover. A mode alone does not identify its requester. Missing RC samples and simultaneous mode requests create ambiguity; prove RC precedence or an FC-side autonomy inhibit on the actual firmware before claiming that the companion can never override a pilot. Do not hide that limitation with a guessed mode-change reason.

Recovery is separate from the 600-second mission deadline. Expiry stops mission progression; it must not make every recovery sleep immediately raise the same expired deadline. Recovery operations have their own bounded waits and remain subject to pilot/failsafe cancellation.

An abort is latched against the mission, not against its recovery commands. Entering recovery changes the guard context: the original abort/deadline cannot cancel the requested return or LAND, but authority loss and recovery deadlines still can. Repeated abort requests do not restart recovery or clear the terminal mission state.

### Selected approach

Use a small synchronous supervisor plus cooperative cancellation checks at every wait and before each command/actuation. This fits the current mission code and keeps one command writer.

Two alternatives were considered. Return-value patches alone leave replay, ownership, and cancellation holes. A mission worker thread keeps dispatch responsive but cannot safely interrupt a blocked Python worker and introduces competing command writers. Neither is the selected MVP.

For a healthy companion, use GUIDED return to the pinned original H followed by LAND. Native RTL remains a candidate for the independent FC failure policy, but only after proving its home/rally destination, altitude reference, and landing configuration. Copter can reset home on rearm and RTL can select a rally point. Do not silently write home, rally, or RTL parameters during a mission. [ArduPilot RTL behavior](https://ardupilot.org/copter/docs/rtl-mode.html).

## Global constraints

- Scope includes the current dirty worktree at `54cdeffd36f6936036973e4a93bd38f78d62add2`. Preserve pre-existing modifications and the separate parent/nested Git repositories.
- No autonomous STABILIZE transition, force-arm bypass, airborne disarm, fabricated sensor value, fabricated attachment success, or automatic mission restart.
- Successful transport/ACK is not confirmed motion, touchdown, or payload attachment.
- Each phase requires a fresh explicit QGC request. FM1 success permits FM2; FM2 success permits FM3. Failure or takeover blocks later phases for that attempt.
- All commanded distances and altitudes use metres. Label original-home-relative altitude, FC-home-relative altitude, AMSL, and local AGL distinctly.
- Use focused regression tests for the observed defects. Preserve simulation contracts; change tests that encoded unsafe behavior only with an explicit replacement assertion.
- No aircraft connection, parameter change, physical actuation, or flight is performed as part of writing this plan.
- Execution should use small commits containing only its own changes. Do not stage this dirty worktree wholesale or add authorship attribution.
- If later execution changes this machine's configuration, packages, or services, update `/home/willis/SETUP_REPLICATION.md` and its verification date without secret values. Ordinary source/plan edits do not require machine setup changes.

## Code map and shared interfaces

Paths below are relative to `companion/comp2026` unless marked as parent paths. New paths are planned files, not existing implementations.

| File | Responsibility after the change |
| --- | --- |
| `src/drone/control/mission_supervisor.py`, new | Command definitions/envelopes, locked admission ledger, attempt/authority state, cancellation checks, and the single recovery coordinator. Pure imports, no hardware construction. |
| `src/drone/control/flight_state.py`, new | Immutable observations with source identity and monotonic receipt times; freshness and authority-evidence checks. No flight commands. |
| `src/drone/control/drone_control.py` | Decode observations, encode MAVLink, and implement bounded operations using fresh evidence. No mission-specific recovery. |
| `src/drone/control/listener.py` | Construct hardware, install supervisor callbacks, dispatch one phase, publish final status, and own shutdown. |
| `src/drone/common_types.py` | Existing coordinates plus the pinned mission-home datum and documented body FRD convention. |
| `src/drone/timebase.py` | Separate elapsed mission time from persisted epoch timestamps, retaining simulation injection. |
| `src/drone/control/mission_info.py` | Validated atomic waypoint persistence, actual age, and mission timing. |
| `src/drone/missions/fm1.py`, `fm2.py`, `src/drone/mock_mission.py` | Checked mission sequencing; active FM3 remains `mock_mission.py`. |
| `src/drone/sensors/` | Timestamped measurements, calibrated transforms, honest payload capability, and bounded sensor/recording cleanup. |
| `src/gc/custom_missions.json`, new `src/gc/prepare_attempt.py` | Canonical QGC actions and generation of an attempt-specific action file. No connection to an aircraft. |
| New `tests/test_flight_state.py`, `test_flight_safety.py`, `test_mission_supervisor.py`, `test_listener.py`, `test_lidar.py`, `test_payload_contract.py`, `test_safe_imports.py` | Focused offline regression coverage. Extend existing tests where their current fixtures already cover the operation. |
| `README.md`, `docs/SYSTEM_DIAGRAM.md` | Current setup, operating contract, recovery, and validation evidence. Replace obsolete statements in place. |

Shared definitions to introduce in their owning tasks:

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class MissionHome:
    lat: float
    lon: float
    amsl_m: float

@dataclass(frozen=True)
class CommandEnvelope:
    source_system: int
    source_component: int
    target_system: int
    target_component: int
    command: int
    attempt_id: int
    params: tuple[float, ...]
    received_at: float
```

`MissionHome` belongs in `common_types.py`; `CommandEnvelope` belongs in `mission_supervisor.py`. `FlightOperationError`, `MissionAbort`, and `AuthorityLost` are distinct exceptions defined in `mission_supervisor.py`, each derived directly from `RuntimeError`. They must retain a reason string. An authority error never falls through to ordinary companion recovery.

Keep supervisor imports independent of `DroneControl`; use type-only imports or a structural controller interface for recovery. The controller may import the shared exceptions without a circular runtime import.

The controller retains its public operation names and their existing success convention where applicable: `goto_waypoint`, `simple_land`, `disarm`, `set_guided_mode`, and `guide_move_relative_frame` return 0 only after their specified success condition; a flight failure raises `FlightOperationError`. Takeoff/climb wrappers raise on failure instead of returning normally. `fm1`, `fm2`, and `fm3` return `True` only on phase success; the QGC supervisor also treats any legacy non-True result as failure. Boolean stability predicates remain Boolean and their callers must check them.

`DroneControl.set_mission_home(home: MissionHome) -> None` fixes the datum for an attempt. Once configured, `get_current_gps().alt` and mission `GPSCoord.alt` use original-H-relative metres. Translation to FC takeoff altitude or MAVLink AMSL happens only in the controller. Update the simulation host at this interface rather than maintaining contradictory altitude meanings.

The supervisor exposes `admit(envelope: CommandEnvelope) -> bool`, `begin(command: int) -> None`, `finish(command: int, result: str) -> None`, `request_abort(reason: str) -> None`, and `check_permission() -> None`. `admit` returns True only for the one envelope to enqueue. Admission also records the ACK result for the transport adapter. Terminal result strings are `SUCCEEDED`, `FAILED`, and `ABORTED`; recovery outcome is recorded separately as `HOME_LANDED`, `LOCAL_LANDED`, `PILOT`, `FC_FAILSAFE`, or `UNCONFIRMED`.

## Execution sequence

Run Tasks 1-8 in order. Task 9 can proceed independently once shared frame/measurement contracts are fixed. Task 10 can proceed alongside sensor work. Task 11 integrates the finished paths. Passing an intermediate task is not permission to fly.

### Task 1: Establish the deployment facts and an import-safe test entry point

**Files:** Modify `src/drone/control/listener.py`, `README.md`, and `docs/SYSTEM_DIAGRAM.md`; create `tests/test_safe_imports.py`. Inspect parent `companion/README.md` and `docs/runbook.md` before changing deployment instructions.

**Interfaces:** `start_repl` remains the QGC entry point. Sensor/controller construction moves into a callable factory invoked only by the live entry point; importing the listener performs no hardware initialization.

- [ ] Add an isolated-process regression that imports `drone.control.listener` with GPIO/I2C modules unavailable. Expect the current eager hardware imports to fail; then move those imports into hardware construction and require the regression to pass.

```python
def test_listener_import_does_not_load_hardware_drivers():
    import subprocess
    import sys
    subprocess.run(
        [sys.executable, "-c",
         "import sys; import drone.control.listener; "
         "assert 'gpiozero' not in sys.modules; "
         "assert 'board' not in sys.modules"],
        check=True,
    )
```

- [ ] Record the exact firmware version/build, FC model, QGC version, MAVLink dialect/version, actual source/target IDs, RC receiver protocol, mode-switch channel/PWM bands, and mode mapping. Obtain a read-only parameter export with motors inhibited. Store evidence locations and checksums in the deployment notes; keep secrets out of documentation.
- [ ] Verify GCS, RC, battery, EKF, fence, and companion-loss behavior on that exact release. Cover a companion crash, a frozen mission loop with DroneKit still alive, a one-way link fault, and a pilot switch during a pending companion mode request. Identify which heartbeat/source is monitored. A QGC heartbeat alone is not a mission watchdog.
- [ ] Establish the independent FC action: original-H return and land where that is verified possible, otherwise local LAND as the user's backup. Preserve an already active FC safety action. If native configuration cannot cover a stuck companion or enforce RC precedence, record that as a closed flight gate and plan the smallest FC-side watchdog/inhibit needed; do not substitute a companion polling claim.
- [ ] Confirm an obstacle-cleared return corridor, altitude envelope, reserve policy, and local-landing operating area. The current code has no general obstacle-aware return planner. Approve bounds for this aircraft before flight rather than treating 10 m as universally safe.

**Done when:** Import tests pass and each hardware fact has an evidence source or a specific failed deployment gate. Software work can continue with injected observations while aircraft use remains disabled.

### Task 2: Separate elapsed clocks from persisted waypoint time

**Files:** Modify `src/drone/timebase.py`, `src/drone/control/mission_info.py`, `src/drone/control/drone_control.py`, `src/drone/mock_mission.py`, `src/drone/auto_attempt.py`; extend `tests/test_timebase.py`, `tests/test_mission_info.py`, and affected simulation tests.

**Interfaces:** `timebase.monotonic()` is the elapsed mission clock, defaulting to `time.monotonic()` and using the injected `Clock.now()` in simulation. Add `timebase.epoch()` for Unix timestamps. Keep `timebase.time()` as a documented compatibility alias for elapsed time while migrating all internal duration calculations to `monotonic()`.

- [ ] Add clock-jump and persistence regressions. This example must pass after the split without installing or setting a system clock:

```python
def test_epoch_jump_does_not_change_elapsed_time(monkeypatch):
    from drone import timebase
    monkeypatch.setattr(timebase._wall_time, "monotonic", lambda: 12.0)
    monkeypatch.setattr(timebase._wall_time, "time", lambda: 1_800_000_000.0)
    assert timebase.monotonic() == 12.0
    assert timebase.epoch() == 1_800_000_000.0
    monkeypatch.setattr(timebase._wall_time, "time", lambda: 1_700_000_000.0)
    assert timebase.monotonic() == 12.0
```

- [ ] Change `loaded_at` writes and age checks to `epoch()`. Keep mission start, auxiliary timers, stability, operation deadlines, and the automatic 600-second deadline on elapsed time. Handle auxiliary start time zero with `is not None`.
- [ ] Retain injected clock sleep/deadline behavior and nested context restoration. Audit every `time.time()` use by purpose; do not mechanically convert persisted dates into boot-relative values.
- [ ] Add a regression where the mission deadline expires, its context unwinds, and recovery can use a fresh bounded deadline. Preserve outer host cancellation and infrastructure timeouts.
- [ ] Run the clock, mission-info, auto-attempt, and simulation-seam tests; commit only the clock-related change and its documentation.

**Done when:** Wall-clock jumps cannot extend flight waits; persisted age remains meaningful across restarts; simulation timing tests still use simulation time.

### Task 3: Collect fresh flight observations and classify authority

**Files:** Create `src/drone/control/flight_state.py`, `tests/test_flight_state.py`; modify `src/drone/control/drone_control.py`. Add the exception definitions in `src/drone/control/mission_supervisor.py` without hardware imports.

**Interfaces:** Define immutable `Observation(value: object, received_at: float, sequence: int)` and `is_fresh(observation: Observation, now: float, max_age: float) -> bool`. A locked `FlightState` stores source-filtered heartbeat/mode, location, velocity, attitude, landed state, home, RC input, and available explicit failsafe observations. Snapshot reads return one coherent copy. Each measurement retains its own receipt time and sequence.

```python
def test_future_and_stale_observations_are_not_fresh():
    from drone.control.flight_state import Observation, is_fresh
    sample = Observation(value=10.0, received_at=5.0, sequence=1)
    assert is_fresh(sample, now=5.2, max_age=0.5)
    assert not is_fresh(sample, now=6.0, max_age=0.5)
    assert not is_fresh(sample, now=4.9, max_age=0.5)
```

- [ ] Write failing tests for wrong-source telemetry, stale fields under a fresh heartbeat, missing/NaN values, and mixed observation updates; implement source filtering and atomic snapshots.
- [ ] Maintain authority values `COMPANION`, `PILOT`, `FC_FAILSAFE`, and `UNKNOWN`. Track the baseline RC switch state and expected companion mode changes before sending them. Only fresh corroborated RC transitions identify pilot takeover. A same-slot retransmission, old RC value, failed GUIDED request, or missing heartbeat does not.
- [ ] Suspend new mission output as soon as an unexpected mode change is observed, without naming it pilot takeover until supported. Never reassert GUIDED while authority is unresolved. Receive-path callbacks update cancellation state but issue no navigation or payload commands.
- [ ] Make source identities, freshness bounds, and RC mapping validated profile inputs. Start offline tests with heartbeat age 1.5 s and position/velocity/attitude/range age 0.5 s; those are test settings, not measured aircraft limits. Request and verify the corresponding message rates during preflight on the supported firmware.
- [ ] Test takeover during ascent, return, landing, and the interval between a command check and transmission. Test masked/missing RC edges and explicitly mark unresolved cases UNKNOWN. Carry Task 1's FC-side precedence requirement into the acceptance result.

**Done when:** Missing data cannot become PILOT, stale telemetry cannot satisfy an operation, and neither PILOT nor FC_FAILSAFE nor UNKNOWN can trigger a companion mode reassertion.

### Task 4: Make flight primitives bounded and confirm their outcomes

**Files:** Modify `src/drone/control/drone_control.py`; create `tests/test_flight_safety.py`; extend `tests/test_drone_control_connection.py` and simulation-seam tests.

**Interfaces:** Apply the controller success/error contract above. Add `DroneControl.check_permission() -> None`, delegating to the installed supervisor guard. For standalone simulation use, install an explicit simulation guard; the real listener must never start with a no-op guard.

```python
def test_missing_altitude_cannot_confirm_arrival(monkeypatch):
    from types import SimpleNamespace
    from drone.control import drone_control
    ticks = iter([0.0, 0.1, 1.1])
    monkeypatch.setattr(drone_control.time, "monotonic", lambda: next(ticks))
    monkeypatch.setattr(drone_control.time, "sleep", lambda seconds: None)
    vehicle = SimpleNamespace(location=SimpleNamespace(
        global_relative_frame=SimpleNamespace(lat=41.0, lon=-81.0, alt=None)))
    assert drone_control.wait_pos(vehicle, 41.0, -81.0,
                                  alt_m=10.0, timeout=1.0) is False
```

- [ ] Add failing tests for denied GUIDED, denied arming, no ascent, missing arrival altitude, stale position, false low-altitude touchdown, denied LAND, and delayed disarm. Use injected time and fake MAVLink; no serial connection.
- [ ] Require valid preflight state, confirm GUIDED, then arm and take off. Remove STABILIZE and force-arm behavior. Reject a takeoff request when already airborne; use an explicit climb operation for airborne height changes. Preserve FC arming checks.
- [ ] Give mode, arm, ascent, navigation, landing, and disarm waits bounded intervals; check permission before transmission and during each wait. Initial offline bounds: 10 s mode, 15 s arm, 45 s ascent, 120 s navigation, 180 s landing, and 16 s disarm. Task 1's tested aircraft profile must justify operational bounds and reserve.
- [ ] Require relevant observations newer than the command before reporting motion complete. `wait_pos` must reject missing/nonfinite required altitude and handle missing coordinates without an unhandled arithmetic exception.
- [ ] Confirm LAND before reporting it selected. Confirm touchdown using fresh FC ON_GROUND after a landing transition, or fresh auto-disarm without contradictory airborne evidence. Range, elapsed time, low home-relative altitude, or a previous landing's cached state cannot independently establish touchdown. Explicit disarm must itself require current touchdown evidence.
- [ ] Check ACKs where the installed protocol provides them and confirm actual state regardless of ACK. Never add blind retransmission of a relative-offset command, since the second command could apply the offset again.
- [ ] Make unimplemented movement/arming APIs raise `NotImplementedError` instead of returning success. Fix the unused pitch-stability helper's timer reset while preserving its limited scope.

**Done when:** Every reported successful operation has current supporting evidence, and failures propagate without local mission-specific RTL/LAND handling.

### Task 5: Admit QGC commands once per attempt and fix their mapping

**Files:** Implement `src/drone/control/mission_supervisor.py`; modify `src/drone/control/drone_control.py`, `src/drone/control/listener.py`, `src/gc/custom_missions.json`; create `src/gc/prepare_attempt.py` and `tests/test_mission_supervisor.py`; extend `tests/test_listener.py` as introduced here.

**Interfaces:** Use `CommandEnvelope` and the supervisor methods above. `MissionSupervisor(attempt_id: int, *, admission_check: Callable[[CommandEnvelope], None])` owns the command ledger; its `admit` return is the sole permission to enqueue. The listener supplies a nonblocking admission validator that checks configured identity and current prerequisite snapshots and raises on rejection. Phase, token, authority, and ledger checks remain supervisor-owned. QGC transport decoding and ACK transmission stay in the MAVLink adapter. A permissive validator is allowed only in a focused test, never the real entry point.

```python
def test_duplicate_start_reserves_only_one_slot():
    from drone.control.mission_supervisor import CommandEnvelope, MissionSupervisor
    supervisor = MissionSupervisor(attempt_id=7, admission_check=lambda request: None)
    request = CommandEnvelope(200, 190, 1, 191, 31000, 7,
                              (7.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 1.0)
    assert supervisor.admit(request) is True
    assert supervisor.admit(request) is False
    supervisor.begin(31000)
    supervisor.finish(31000, "SUCCEEDED")
    assert supervisor.admit(request) is False
```

- [ ] Add concurrent admission tests and reproduce queued/running/terminal duplicates. Reserve under one lock before returning to the callback; never leave the reserve operation to the later listener dequeue.
- [ ] Validate configured source/target identity, command allowlist, finite parameters, attempt ID, expected phase, prerequisites, and current authority before admission. Mission starts use explicit targets; do not accept arbitrary broadcast starts. Queue the full envelope. Coordinate-recording actions may remain available before an attempt, but reject updates and clears during a flight attempt.
- [ ] Use `QUEUED -> RUNNING -> TERMINAL` per mission ID. A completed, failed, or aborted command never runs again within the attempt, including after disarm. Reject out-of-order or busy starts. Repeated admitted requests report their existing status rather than enqueueing. Use IN_PROGRESS for admitted missions, terminal ACKs for completion, and appropriate busy/denied/unsupported results. Verify exact retry behavior with the installed QGC version. [MAVLink command protocol](https://mavlink.io/en/services/command.html).
- [ ] Prevent cross-attempt replay with `param1=attempt_id` in generated mission actions. Generate a new monotonically increasing integer from 1 through 16,777,215 while preparing a new flight on the ground; reject zero, fractional values, and reused/consumed IDs. This range is exactly representable in MAVLink float parameters. Persist consumption before accepting FM1, and never resume a consumed attempt after process restart. Keep the same ID across planned L/pickup disarms. The ID is replay context, not authentication.
- [ ] Fail closed if the attempt ledger is missing unexpectedly, corrupt, unwritable, or exhausted; never reset its counter automatically. Make consumption durable before ACK/admission, and reject mismatched session/action IDs after interrupted preparation. The live listener fixes its session at startup and refuses session changes while armed or while an attempt is active.
- [ ] `prepare_attempt.py` takes a validated local deployment profile and emits a matching listener session file and QGC action JSON. Use an exclusive local preparation lock and atomic replacements. It does not connect to a vehicle. The operator loads the generated action file before the attempt; a stale file must be denied. Provide no in-flight reset command and no automatic reconnect/restart that re-enables a consumed attempt. QGC supports fixed parameters in custom actions. [QGC action format](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/custom_actions/custom_actions.html).
- [ ] Align clear-pickups to 31012 and clear-all to 31013. Retire the old listener-only 31014 by returning UNSUPPORTED; never reuse it for a flight action. Add proposed `ABORT_AND_RECOVER=31015` only after checking the deployed custom command registry for collisions. Reject startup on a collision. Remove the duplicate no-op action. Abort sets the shared flag immediately in the receive path, bypassing the mission queue; the single supervisor execution path performs recovery.
- [ ] Test wrong vehicle/source/token, duplicate after auto-disarm, listener restart with consumed token, stale QGC files, FM2 before FM1, clear-all retaining no navigation targets, and abort during a blocked mission wait. Test that a wrong file cannot silently fall back to tokenless acceptance.

**Done when:** One accepted mission action causes at most one phase execution; failed/aborted attempts cannot advance or restart; the actual QGC JSON and receiver agree.

### Task 6: Pin home and make one supervisor own recovery

**Files:** Modify `src/drone/common_types.py`, `src/drone/control/drone_control.py`, `src/drone/control/mission_supervisor.py`, `src/drone/control/listener.py`; extend `tests/test_flight_safety.py`, `tests/test_mission_supervisor.py`, `tests/test_listener.py`. Adapt parent `companion/src/drone_sim_companion/comp2026_host.py` and its focused tests if needed for the new datum/guard interfaces.

**Interfaces:** `MissionHome` and `set_mission_home` as defined above. Add the pure `return_altitude_amsl(home: MissionHome, cruise_m: float, current_amsl_m: float) -> float` to `mission_supervisor.py`, and `recover(controller, home: MissionHome, cruise_m: float) -> str` on the supervisor. The controller supplies confirmed operations, permission checks, and flight snapshots.

```python
def test_recovery_keeps_the_higher_transit_altitude():
    from drone.common_types import MissionHome
    from drone.control.mission_supervisor import return_altitude_amsl
    home = MissionHome(41.0, -81.0, 100.0)
    assert return_altitude_amsl(home, 10.0, 103.0) == 110.0
    assert return_altitude_amsl(home, 10.0, 125.0) == 125.0
```

- [ ] Capture a fresh finite original H and AMSL datum before first takeoff. Keep them immutable for the attempt, display them, and require the operating-area checks in Task 8. Restart while airborne never creates a new home or resumes a mission.
- [ ] Encode fixed-datum navigation in `MAV_FRAME_GLOBAL_INT` using AMSL, and evaluate arrival in the same frame. For takeoff, translate the required target into the FC's current home-relative altitude. Obtain the current FC datum after each rearm and verify it before taking off. Do not rely on DroneKit's implicit location conversion, change FC home, or mix FC-relative and original-H-relative values.
- [ ] Implement recovery stages `ASSESS -> CLIMB -> RETURN -> LAND -> CONFIRM`. Skip climb when already high enough. Use local LAND on return rejection, no progress, invalid home/navigation, or exhausted permitted return budget when authority and the command link still permit it. Preserve landing already underway. A rejected or unconfirmed LAND remains UNCONFIRMED, never success or an airborne disarm request.
- [ ] Check authority before every stage and before every transmitted command. PILOT/FC_FAILSAFE/UNKNOWN stop companion recovery; record the appropriate separate outcome and keep the mission terminal. Do not retry GUIDED to regain ownership. Use Task 1's independently verified FC response for absent/uncertain control.
- [ ] Keep failure and recovery results separate: landing successfully after a failed FM2 produces mission FAILED plus recovery HOME_LANDED or LOCAL_LANDED, never FM2 SUCCEEDED. A timeout must not advance to payload release, a later WM target, or a new mission.
- [ ] Test the guard-context change explicitly: a latched QGC abort permits recovery commands, a repeated abort does not restart recovery, and subsequent pilot takeover still stops every companion command. Do not globally clear the abort to make recovery work.
- [ ] Add fake-controller order assertions for climb-before-transit, no initial descent when above cruise, original H after multiple rearm/home changes, return failure to LAND, failure during LAND, landed abort without takeoff, and takeover during each recovery stage. Include a 600-second mission timeout followed by independently bounded recovery.

**Done when:** The user's recovery order is enforced by one owner, the original home and altitude datum survive rearming, and no recovery result is inferred from a command merely being sent.

### Task 7: Propagate all mission failures into that supervisor

**Files:** Modify `src/drone/missions/fm1.py`, `src/drone/missions/fm2.py`, `src/drone/mock_mission.py`, `src/drone/control/listener.py`, `src/drone/auto_attempt.py`; extend `tests/test_simulation_mission_seams.py`, `tests/test_auto_attempt.py`, and `tests/test_listener.py`.

**Interfaces:** Successful phases return True. Failures raise or return a value the supervisor rejects. Normal completion and recovery remain distinct. Both the QGC caller and simulation host consume the same phase outcome contract.

```python
def test_fm1_does_not_disarm_after_failed_landing():
    from unittest.mock import Mock
    import pytest
    from drone.common_types import GPSCoord
    from drone.missions.fm1 import fm1
    controller = Mock()
    controller.force_arm_takeoff.return_value = 0
    controller.goto_waypoint.return_value = 0
    controller.simple_land.return_value = -1
    with pytest.raises(RuntimeError):
        fm1(Mock(), controller, 10, GPSCoord(41.0, -81.0, 10))
    controller.disarm.assert_not_called()
```

- [ ] Check takeoff, navigation, landing, disarm, GUIDED, climb, acquisition correction, stability, and attachment outcomes at every active call site. Include initial FM3 grid and recenter moves whose return values are currently only printed.
- [ ] Remove FM3's local RTL and swallowed Ctrl-C handling. Route exceptions and explicit aborts into the supervisor; diagnostic recording errors must not replace the original failure. Configure the listener's SIGINT handling to request abort and let cooperative unwinding reach the one recovery owner, rather than abruptly exiting or repeatedly interrupting recovery.
- [ ] Start the mission timer only after valid FM1 admission. Advance each phase only on success. Keep FM2's intended successful airborne wait for the next QGC command under supervision and the attempt deadline. On deadline expiry between commands, invoke recovery; do not idle indefinitely waiting for FM3.
- [ ] Replace the 1,000-iteration precision-landing cutoff with the declared elapsed deadline. Stream targets at a bounded rate and check cancellation even when markers disappear. On lost landing-target/range evidence, use the tested local landing policy without declaring pickup success. Never substitute GPS-relative altitude for AGL or touchdown.
- [ ] Confirm the intended payload landing location/attachment before any subsequent takeoff. Preserve the autopilot's low-altitude landing behavior; do not start a return climb from an uncertain touchdown.
- [ ] Test FM1 goto failure prevents its normal LAND/disarm path; FM1 LAND failure prevents disarm; FM2 sensor/stability failure prevents release; FM3 attachment/nav failure prevents later WM calls; Ctrl-C does not resume the listener sequence; and simulation `auto_attempt` still terminates on failure.

**Done when:** Every failed or cancelled phase stops normal sequencing and reaches exactly one terminal result. No swallowed exception or unchecked result can trigger later flight or payload actions.

### Task 8: Validate waypoints, range, and payload capability before flight

**Files:** Modify `src/drone/control/mission_info.py`, `src/drone/control/listener.py`, `src/drone/sensors/lidar/lidar.py`, `src/drone/sensors/servo/servo.py`, `src/drone/missions/fm2.py`, `src/drone/mock_mission.py`; extend/create `tests/test_mission_info.py`, `tests/test_lidar.py`, `tests/test_payload_contract.py`.

**Interfaces:** Preserve `get_waypoint(name, max_age_seconds)` while making invalid records unavailable with a reported reason. Lidar returns a validated sample value whose producer stores value/time atomically. Add `Dropper.supports_attachment: bool` and `attach(target_id: int) -> bool`; unsupported hardware advertises False and raises `NotImplementedError` if called anyway. FM3 admission requires attachment capability before moving.

```python
def test_hardware_dropper_does_not_claim_attachment():
    import pytest
    from drone.sensors.servo.servo import Dropper
    dropper = object.__new__(Dropper)
    assert dropper.supports_attachment is False
    with pytest.raises(NotImplementedError):
        dropper.attach(3)
```

- [ ] Validate finite numeric latitude/longitude/altitude, latitude/longitude bounds, actual persisted epoch age, and the approved operating area. Reject malformed JSON/schema without replacing it with an empty success-shaped store. Use same-directory temporary writes and atomic replacement; a failed write preserves the last valid file. Report actual loaded_at, including TARGET, and reject unknown or implausibly future timestamps.
- [ ] Apply the existing four-hour age limit to flight admission, with a stricter approved flight-site check for H/L/targets. Snapshot and freeze validated mission coordinates for the attempt. Missing return home or required L/TARGET prevents admission. Keep intentional waypoint recording available during site setup under the safe operator procedure.
- [ ] Remove the fabricated initial 10 m LiDAR value. Require a first valid hardware sample within a startup bound; store value and monotonic timestamp under one lock. Validate raw centimetres against the sensor's documented envelope before applying the measured mounting offset. Reject negative/nonfinite corrected ranges; zero clearance can be a valid ground measurement but is never standalone touchdown proof.
- [ ] Reset the release-stability timer whenever any required position, velocity, attitude, range, or permission observation is missing, stale, invalid, or outside limits. Require distinct current observations throughout the hold. Convert body-beam range through the calibrated attitude/mounting model before applying the vertical clearance requirement; reject unsupported tilt/slope cases. Use the configured desired drop height consistently instead of a separate hard-coded 10 m gate.
- [ ] Initially reject real FM3 at admission because the current hardware has no attachment mechanism/confirmation interface. Then document and implement the actual pickup actuator or passive latch plus independent confirmation using the hardware chosen for this aircraft. Require confirmed attachment before rearming. A fake always-True implementation is not completion of this task.
- [ ] Move GPIO imports/construction behind the payload hardware factory or constructor so capability-contract tests can import `Dropper` without installed GPIO drivers or actuator initialization.
- [ ] Distinguish commanded release from observed release. Existing FM2 rules permit an unsuccessful drop, so an open-loop servo command may satisfy the command attempt, but its log must not claim physical release. Keep retained-load assumptions explicit for subsequent flight; gate any behavior that needs known attachment/release on actual evidence.
- [ ] Test dead-at-start and stale-during-flight LiDAR, wall-clock rollback, value/timestamp races, zero/negative/raw-invalid range, unsupported attachment rejection before takeoff, failed attachment after landing, and drop refusal after interrupted stability. Hardware FM3 remains disabled until its physical subtask passes.

**Done when:** Bad waypoints/sensors/capabilities cannot trigger flight; stale/invalid data cannot authorize release; real FM3 can be enabled only with demonstrated attachment confirmation.

### Task 9: Correct vision frames and bound camera data age

**Files:** Modify `src/drone/common_types.py`, `src/drone/control/drone_control.py`, `src/drone/sensors/camera/_camera_manager.py`, `src/drone/sensors/camera/camera.py`, `src/drone/sensors/camera/calibration.json`, and `src/drone/sensors/camera/mounting.json`; extend `tests/test_camera_body_mapping.py` and `tests/test_drone_control_connection.py`.

**Interfaces:** `RelPosComplete` is forward/right/down in metres. Preserve `Camera.last_frame_timestamp` for existing consumers and add explicit frame sequence/receipt metadata at the manager boundary. Simulation capture retains its injected timestamp; hardware metadata must describe its actual capture/receipt meaning.

```python
def test_body_offset_keeps_forward_axis():
    from unittest.mock import Mock
    from drone.common_types import RelPosComplete
    from drone.control.drone_control import DroneControl
    controller = object.__new__(DroneControl)
    controller.vehicle = Mock()
    controller.check_permission = Mock()
    controller.guide_move_relative_frame(RelPosComplete(1, 0, 0))
    packet = controller.vehicle.message_factory.set_position_target_local_ned_encode
    assert packet.call_args.args[5:8] == (1.0, 0.0, 0.0)
```

- [ ] Fix the horizontal body-offset swap/sign error and add camera-to-body-to-command tests for forward, right, and down, including yaw/tilt projection. Preserve the already verified legacy LANDING_TARGET angular convention; it is not the same encoding as body-offset XYZ.
- [ ] Require finite, orthonormal, right-handed mounting rotation and measured translation. Verify marker dimensions and physical signs. Retrieve calibrated image dimensions from the calibration source or recalibrate; do not infer them from the principal point. Request and verify the matching camera mode and transform intrinsics for supported resizing; reject unmodeled crop/aspect changes.
- [ ] Validate camera readiness before a vision-dependent phase. Use a single bounded latest-frame acquisition path so navigation waits do not leave stale queued images waiting to be treated as new. A sensor capture worker is allowed; it issues no flight commands. Consumers must time out if acquisition blocks.
- [ ] Do not treat `read()` completion time as proof of exposure age. Use backend timestamps where available and measure buffer latency on the selected camera/backend. Reject replayed/out-of-order samples and frames older than the approved acquisition limit. If exposure age cannot be bounded, keep precision pickup disabled on that backend.
- [ ] Require the five centered samples to be distinct and consecutive within the acquisition window; missing marker, stale sample, invalid AGL, or correction resets the count. Correlate attitude/position with image time within measured skew limits instead of projecting an old image through a later attitude.
- [ ] Save recordings without holding the acquisition buffer lock during encoding. Own and join recording workers within a bounded shutdown, surface write failures as diagnostics, and prevent concurrent writers overwriting the same path. Logging failures never trigger a second flight recovery.

**Done when:** Direction tests agree through the full path, malformed calibration/mounting is rejected, and precision landing consumes bounded-age observations rather than newly stamped old frames.

### Task 10: Repair deployment inputs and isolate development tools

**Files:** Modify `README.md`, `docs/SYSTEM_DIAGRAM.md`, `r.txt`, `src/drone/entry.py`, `src/drone/aruco_land_only.py`, `src/drone/waypoint_only.py`, `src/drone/control/sender.py`, `src/gc/main.py`, `src/drone/sitl/simulation.py`, `src/lidar_test.py`; create `requirements-hardware.in` and a platform-verified lock. Update parent `.dockerignore`, `companion/Dockerfile`, and `companion/README.md` for the actual import/config closure.

**Interfaces:** Importing any project module has no flight or actuator side effect. Experimental hardware entry points require an explicit invocation and remain excluded from the production QGC launch path.

- [ ] Extend import-isolation tests to every former module-level flight script. Move construction and execution under guarded entry points. Remove hard-coded automatic flight from accidental imports and quarantine the nonfunctional legacy `entry.py`/inactive `missions/fm3.py` route instead of making it a second production stack.
- [ ] Replace the servo-test utility's navigation message with a correctly scoped actuator test or disable it with a clear error until its actual receiver is defined. It must never send NAV_WAYPOINT to test a servo. Label the demo sender as synthetic and require an explicit test endpoint/identity. Fix the SITL import and make its success depend on observed outcomes; leave simulated safety-check relaxation confined to explicit SITL use.
- [ ] Correct the LiDAR diagnostic's metres label and exception handling. Close sensor resources in diagnostic exits. Do not change camera/system settings by importing backup utilities.
- [ ] Replace the invalid requirements instructions with parseable hardware dependency inputs including DroneKit, pymavlink, OpenCV/NumPy, GPIO Zero, the Adafruit driver/bus support, and DroneKit's compatibility dependency. Resolve and pin versions against the selected onboard OS/architecture. The parent Python 3.12 environment is a known offline-test baseline, not proof of onboard GPIO compatibility.
- [ ] Include the untracked mounting file and every new imported module/configuration in reproducible source/image inputs. Update the parent allowlist for `flight_state.py` and `mission_supervisor.py` if they enter the simulation import closure. Keep hardware-only modules out of the simulation image unless required. Test import/config loading inside the built image without launching a mission.
- [ ] Rewrite current documentation with the actual QGC entry point, datum/units, attempt-action-file preparation, RC takeover rules, recovery outcomes, and accurate disarm/landing behavior. Keep one current operating guide rather than copying old contradictory instructions into another file.

**Done when:** A clean target installation can import and configure the selected runtime; all developer imports are inert; simulator images include the code/config they claim; operator docs match the new behavior.

### Task 11: Verify the complete QGC path and record the flight gate

**Files:** Complete `tests/test_listener.py`, `tests/test_mission_supervisor.py`, and affected existing tests. Update verification notes in `README.md` and `docs/SYSTEM_DIAGRAM.md`; update parent guides only where the parent contract changes.

- [ ] Exercise the actual receiver callback and listener with fake transport/sensors. Assert sent command order and absence of forbidden later commands, not only exception messages. Cover FM1/FM2/FM3 success, every operation failure above, abort during all phases, stale telemetry, duplicate commands, rejected commands, grounded failure, confirmed takeover, explicit FC failsafe, UNKNOWN authority, and failed recovery.
- [ ] Run the nested suite with the available parent environment from this repository:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src \
  /home/willis/projects/drone_sim/.venv/bin/python -m pytest \
  tests -q -p no:cacheprovider -o addopts= --rootdir=.
```

- [ ] Run the affected parent host/camera/runtime tests from the parent root after adapting interfaces:

```bash
uv run --locked pytest companion/tests/test_comp2026_host.py \
  companion/tests/test_simulator_camera_calibration.py \
  companion/tests/test_runtime_node.py -q
```

- [ ] Build matching simulation images and run the actual QGC/listener path against SITL through an isolated test transport with injected camera/range/payload adapters. Do not treat `auto_attempt` alone as QGC coverage. Test two rearm cycles, changed FC home, aborted return, camera/range failure, process kill, mission-loop hang, one-way telemetry loss, and RC takeover at command boundaries. Preserve the exact source/config/image evidence.
- [ ] Perform propellers-removed bench checks with the actual FC, receiver, QGC, companion, and restrained payload mechanism. Demonstrate mode/command precedence, startup outputs, link/watchdog behavior, sensor timing, attachment feedback, and no rearming after abort. No deliberate airborne disarm or physical destructive fault injection.
- [ ] Flight authorization remains closed until all critical offline, SITL, and bench cases pass, the original-H return corridor and reserve are approved, the deployment profile matches the aircraft, and hardware FM3 attachment is demonstrated or FM3 is explicitly disabled. Then use staged supervised flight testing under the aircraft team's procedures, starting with the minimum flight/payload scope.
- [ ] Record outcomes separately: software checks, physical mission result, payload evidence, and simulation score/artifact validity where relevant. A recovered landing does not turn a failed mission into a pass. A prior 65-test pass is historical component evidence only.

**Done when:** The exact QGC path and aircraft configuration have explicit passing evidence for each flight gate, with every untested or failed case reported as such.

## Coverage of the review

| Review finding | Planned work |
| --- | --- |
| FM1/final-FM3 ignored navigation/landing outcomes and premature disarm | Tasks 4, 6, 7 |
| Relative-home/negative-range false touchdown | Tasks 3, 4, 7, 8 |
| Swallowed abort, continued WM attempts, pilot takeover | Tasks 1, 3, 5, 6, 7 |
| Duplicate/misaddressed QGC starts and premature ACKs | Task 5 |
| STABILIZE arming, unbounded waits, failed ascent treated as success | Task 4 |
| Missing hardware attachment interface | Tasks 8, 11 |
| Dead/stale LiDAR after takeoff and unhandled FM2 exception | Tasks 4, 7, 8 |
| Missing/stale arrival altitude and interrupted stability window | Tasks 3, 4, 8 |
| Old/malformed waypoints, wrong clear IDs, lost launch point | Tasks 5, 6, 8 |
| Wrong phase/timer prerequisites and conflicting landing limits | Tasks 2, 5, 7 |
| Adjustable-clock deadlines/freshness | Tasks 2, 3, 8 |
| Slant range mistaken for AGL | Tasks 8, 9 |
| Standalone import-triggered flight and rotated body commands | Tasks 9, 10 |
| Missing camera configuration/build inputs/dependencies | Tasks 9, 10 |
| Broken legacy tools and stale operating documentation | Task 10 |
| No complete QGC/hardware failure-path coverage | Tasks 1, 11 |

## Deferred work and limits

Keep richer mission UI, automatic resume/retry, in-flight waypoint editing, general obstacle/path planning, and broad refactoring outside this plan. Do not replace the controller stack or migrate off DroneKit as an incidental fix. Restrict operation to the verified flight area and tested aircraft profile.

The plan intentionally adds one operator preparation step: load the generated attempt-specific QGC actions before each new flight attempt. That avoids inventing a custom GCS extension while making stale commands distinguishable across attempts. Physical attachment, precise sensor geometry, and FC-side failure/RC precedence evidence are mandatory enablement work, not debts that can be waved through by passing mocks.

During this task only this plan is created. Production code, existing documentation, aircraft parameters, services, packages, and hardware remain unchanged.
