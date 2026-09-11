# comp2026 mission control

## Purpose

This repository contains the mission-control, QGroundControl command, recovery,
and sensor contracts for the comp2026 companion process.

Top-level code under `src/` includes:

- `drone/control/` for command admission, aircraft observations, guarded control,
  recovery, and waypoint persistence.
- `drone/missions/` and `drone/mock_mission.py` for FM1, FM2, and FM3 logic.
- `drone/sensors/` for camera, LiDAR, and payload interfaces.
- `gc/prepare_attempt.py` for offline preparation of an attempt-specific QGC
  action file and listener session.

## Aircraft deployment status

Aircraft flight is disabled. The authority, recovery, sensor, offline
preparation, and live runtime contracts do not form a deployment by themselves.
The parent `comp2026_auto` entry in
`../src/drone_sim_companion/runtime_node.py` now provides the guarded QGC host
for both the limited FM1/FM2 and validated full-phase simulator compositions.
The full composition supplies the nested camera from the parent's ROS RGB8
frame source and fixed simulator calibration/mounting. That host has source and
offline test evidence only. No matching current image or integrated QGC/SITL
result exists, and no verified aircraft configuration or supported aircraft
shell invocation is available in this checkout.

The settled programmatic entry point is
`start_repl(files, runtime_config, *, factories=None, diagnostics=None,
monitoring_stop=None) -> LiveRunResult`.
It requires fully constructed and validated configuration objects; this is an
API signature, not a copy-paste aircraft command. Simulator composition must
inject explicit validated `LiveComponentFactories`. Do not use legacy
`create_live_components()` or `run_auto_attempt` paths as substitutes. Import
checks are not permission to open a MAVLink endpoint or construct GPIO, I2C,
camera, or payload devices.

## Prepare one attempt offline

Run the preparation script from the repository root with `PYTHONPATH=src`.
Python's built-in `gc` module makes `python -m gc.prepare_attempt` unsuitable.

Initialize a new durable ledger once:

```bash
PYTHONPATH=src python src/gc/prepare_attempt.py \
  initialize-ledger /durable/operator/attempt-ledger.json
```

While the aircraft is confirmed on the ground, prepare a new session and action
file from a verified deployment profile:

```bash
PYTHONPATH=src python src/gc/prepare_attempt.py prepare \
  --profile /operator/deployment-profile.json \
  --ledger /durable/operator/attempt-ledger.json \
  --session /operator/current-attempt-session.json \
  --actions /operator/current-qgc-actions.json \
  --acknowledge-on-ground
```

Load the generated action file into the exact QGC installation named by the
profile. The generated actions bind the profile and its raw SHA-256, session
generation, action-file SHA-256, QGC source, companion target, wire protocol,
firmware, QGC release, and current unconsumed attempt token. The canonical
zero-token `src/gc/custom_missions.json` is not ready to fly.

Preparation advances the ledger. FM1 admission consumes the current token. A
consumed token, malformed ledger, failed attempt, takeover, process restart, or
recovery cannot resume an attempt. Prepare a new session and send a new explicit
request for each phase. Duplicate packets for the same accepted request report
the recorded queued, running, or terminal state; they do not consume another
token or rerun a phase.

### Command map

| ID | Prepared action | Admission behavior |
| ---: | --- | --- |
| 31000 | FM1 | First phase. Starts the single 600-second attempt deadline when execution begins. |
| 31001 | FM2 | New explicit request after successful FM1. |
| 31002 | FM3 | New explicit request after successful FM2, only when the selected runtime enables verified attachment and vision dependencies. |
| 31003 | Update WA | Coordinate mutation, allowed only before the attempt is active. |
| 31004-31009 | Update WM1-WM6 | Coordinate mutation, allowed only before the attempt is active. |
| 31010 | Update L | Coordinate mutation, allowed only before the attempt is active. |
| 31011 | Update TARGET | Coordinate mutation, allowed only before the attempt is active. |
| 31012 | Clear pickup waypoints | Clear `WA` and `WM1` through `WM6` before the attempt is active. |
| 31013 | Clear all waypoints | Clear the complete waypoint set before the attempt is active. |
| 31014 | Retired | Always unsupported. It is not generated. |
| 31015 | Abort and recover | Generated only after the deployment registry supplies verified collision-check evidence. It latches abort immediately and bypasses the ordinary queue. |

The listener accepts only source- and target-matched `COMMAND_LONG` packets with
seven finite parameters. Parameter 1 is the exact prepared token and parameters
2 through 7 are zero. `COMMAND_INT` is unsupported because its `x`, `y`, and `z`
fields have no defined meaning for these actions.

An admission ACK, `MAV_RESULT_IN_PROGRESS`, and a terminal ACK describe command
handling. They do not prove flight, touchdown, payload state, or score. Live
flight profiles require MAVLink 2 because `COMMAND_ACK` must match the pinned
flight-controller source and the unique companion target. MAVLink 1 cannot carry
that target binding and is rejected before live construction. Every GUIDED
`MISSION_ITEM_INT current=2` also requires a post-enqueue, source-, target-, and
mission-type-matched `MISSION_ACK`; state telemetry must still confirm motion.
`MISSION_ACK` carries no per-transaction nonce, so an arbitrarily delayed ACK can
be indistinguishable from a later `MISSION_ITEM_INT current=2` transaction. A
timed-out, rejected, or interrupted GUIDED mission-item transaction therefore
latches out every later `current=2` output. Recovery may select the separate,
freshly validated local LAND action, but it does not resend the ambiguous mission
item; accepted ACKs still require causal state or motion confirmation.
See the [MAVLink command protocol](https://mavlink.io/en/services/command.html),
[common message definitions](https://mavlink.io/en/messages/common), and
[QGC custom action format](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/custom_actions/custom_actions.html).

## Runtime configuration contract

The final factory must validate all files and configuration before opening a
transport or creating output devices. The current contract requires:

- Distinct fixed paths for the deployment profile, prepared session, generated
  actions, durable ledger, and waypoint store.
- Separate QGC source, companion target, and flight-controller identities, plus
  the observed wire protocol and exact firmware evidence.
- One shared monotonic clock for controller construction, observations, mission
  deadlines, recovery, range samples, and camera receipt time. Persisted waypoint
  timestamps use Unix epoch time.
- An approved operating-site check, original-H check, recovery corridor and
  local-LAND policy, telemetry cadence policy, and FC-home consistency limits.
- One measured `ClearanceCalibration` shared by precision and release consumers,
  explicit release-stability limits, one cruise height, and one desired drop
  height.
- Explicit sensor source identities, age, skew, transport-latency, continuity,
  hold, and timeout limits. Test fixture values are not deployment values.

| Configuration object | Required inputs |
| --- | --- |
| `ListenerStartupFiles` | Distinct profile, session, generated-action, and ledger paths |
| `ConnectionConfig` | Endpoint, companion source, flight-controller target, wire protocol, ready behavior, and heartbeat timeout |
| `OperatingSitePolicy` | Waypoint, original-H, and recovery checks plus an evidence reference |
| `TelemetryStartupPolicy` and `AutopilotVersionContract` | Bounded request/collection timing, distinct-sample cadence, exact packed version bytes, firmware label, and evidence reference |
| `HardwareComponentConfig` | Validated `RangeConfig` and `PayloadConfig` values for the default physical adapters |
| `InjectedComponentConfig` | Explicit backend label and evidence reference for caller-supplied adapters; it carries no GPIO or raw I2C limits |
| `ClearanceCalibration` and `ReleaseStabilityConfig` | One measured projection geometry and explicit motion, attitude, position, clearance, skew, gap, hold, timeout, polling, and reissue limits |
| `VisionConfig` and `PrecisionMissionPolicy` | FM3-only verified calibration/mounting paths, source/receipt timing, shared clock, and bounded acquisition controls |
| `RuntimeConfiguration` | Fixed waypoint path, enabled phases, common cruise/drop heights, FC-home tolerances, attempt deadline, idle poll, and cleanup bound |

`LiveComponentFactories.telemetry_startup_mode` defaults to `complete`. That
mode preserves the full configure-and-collect proof before QGC listener
installation for physical hardware and ordinary injected tests. Only the
explicit injected `drone-sim-ros-confirmed-v1` backend may select
`staged_simulation`, for either the exact FM1/FM2 phase set or the exact full
phase set. It proves request ACKs and firmware metadata while Gazebo is paused,
then keeps the source-filtered collector active. The admitted FM1's first
guarded GUIDED delivery releases Gazebo; ARM and TAKEOFF remain denied until
complete requested telemetry arrives after that gate on strictly advancing
shared simulation time. A full-phase staged runtime then performs its bounded
camera preparation; it never waits for a frame during listener installation.
Cleanup closes the collector and startup request capability even if QGC sends
no command.

The stable composition interfaces are:

```python
construct_after_full_validation(
    files,
    runtime_config,
    *,
    component_factory,
)

build_live_listener(
    artifacts,
    config,
    *,
    factories=None,
    diagnostics=None,
    startup_admission_check=None,
) -> LiveListenerRuntime

start_repl(
    files,
    runtime_config,
    *,
    factories=None,
    diagnostics=None,
    monitoring_stop=None,
    startup_admission_check=None,
    on_listener_ready=None,
    phase_observer=None,
) -> LiveRunResult
```

`start_repl` owns the synchronous run and calls `LiveListenerRuntime.close()` in
its `finally` path. A supplied synchronous, nonblocking
`startup_admission_check` runs before every existing admission check and must
return `None`; it can keep host admission closed without replacing telemetry,
firmware, authority, site, RC, or phase validation. `on_listener_ready` runs
once after the listener is fully built and installed, before `run()`, and must
also return `None`. Its failure skips `run()`, attempts to close the runtime,
and propagates the original error. If cleanup also fails, `start_repl` chains
that cleanup error as the original readiness failure's cause rather than
replacing it. A host that needs durable readiness ordering must supply
both its initially closed admission barrier and its ready callback. Hookless
standalone callers retain their existing behavior. These interfaces do not
supply deployment values.

`phase_observer(phase, state)` is also optional and must return normally. The
execution owner calls it only for FM1 and FM2. STARTED follows `begin()` and the
deadline check immediately before the handler. COMPLETE follows a finalized
SUCCEEDED result; for a final two-phase FM2 it also follows confirmed
`HOME_LANDED` recovery. Failed, aborted, rejected, replayed, and waypoint
commands emit no COMPLETE event. Recovery emits no HOME or FM3 event. An
observer exception preserves terminal ACK and queue cleanup, runs at most one
recovery, and propagates as an infrastructure failure. If COMPLETE publication
fails, the ACK and supervisor result still report the already finalized physical
phase result; the parent host separately fails its lifecycle because public
evidence publication was not confirmed.

Live cleanup treats telemetry, LiDAR, camera, payload, vehicle-worker, attempt
descriptor, and diagnostic failures as independent facts. A `BaseException`
from one stage cannot skip later safe stages or replace the original run or
observer error. Authority loss stops the payload cleanup operation that
encountered it; cleanup does not retry flight or actuator output.

`RuntimeConfiguration.components` selects exactly one component binding. Calling
`build_live_listener` without explicit factories requires
`HardwareComponentConfig` and selects the fixed `physical-hardware-v1` backend.
Passing `LiveComponentFactories` requires `InjectedComponentConfig`, and their
validated backend labels must match exactly before any controller or device
factory runs. The backend label and evidence reference identify the caller's
binding. They do not prove aircraft capability, authority, firmware, sensor
validity, or flight approval.

`RuntimeConfiguration.enabled_phases` must contain FM1 and FM2, with FM3 added
only when its extra dependencies pass. The physical `Dropper` in this repository
has `supports_attachment = False`, so hardware FM3 stays disabled. Missing
disabled-FM3 vision prerequisites do not block FM1, FM2, or an independently
authorized local `LAND`. The parent simulator payload gateway is a different
adapter contract; a physical profile must not invent GPIO pins, camera facts, or
sensor measurements from simulator configuration.

The mission supervisor accepts only the ordered FM1/FM2 prefix or the full
FM1/FM2/FM3 sequence. In a two-phase runtime, FM2 is final. After FM2's payload
work returns literal `True`, the command owner runs the normal original-H
recovery path. FM2 succeeds only for `HOME_LANDED`; every other recovery outcome
leaves the mission non-successful without starting recovery a second time. In a
three-phase runtime, successful FM2 remains airborne and waits for FM3.

`flight_profile.py` contains source-filtered heartbeat mode/failsafe decoders and
an independent `SYS_STATUS` RC-health decoder. The profile must bind their exact
message identities, mode/status mapping, freshness, evidence hash, and supported
decoder reference. The live `AutopilotVersionContract` accepts only the exact
stable `ArduCopter 4.5.7` label, packed version `0x040507FF`, configured exact
eight-byte `flight_custom_version`, and an explicit evidence reference. This
narrow software compatibility rule does not identify the installed aircraft or
establish compatibility with another 4.x release. The rule is tied to the
[Copter 4.5.7 version declaration](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/ArduCopter/version.h)
and MAVLink's [firmware version and release-type fields](https://mavlink.io/en/messages/standard.html#FIRMWARE_VERSION_TYPE).

## Clock and deadline contract

Mission durations, stability windows, operation timeouts, and the 600-second
attempt deadline use `drone.timebase.monotonic()`. A simulation host must enter
the caller-owned `drone.timebase.configured(clock)` scope before runtime
construction and retain it through callbacks, mission work, recovery, and
`close()`. Without that scope, the runtime uses the host monotonic clock. The
factory validates clock-dependent configuration but does not enter or own the
scope. A deadline applies only to the execution context that entered it, so a
blocked mission does not rewrite callback timestamps in other threads. Nested
deadlines keep the earlier outer limit.

Waypoint `loaded_at` persistence and age checks use `drone.timebase.epoch()`,
which is Unix time and never simulation time. `drone.timebase.time()` remains an
elapsed-time compatibility alias for callers that have not migrated.

## Coordinates, observations, and authority

`MissionHome` is the original launch datum H. The controller pins it once from
fresh, source-filtered, disarmed, exact `ON_GROUND`, no-failsafe observations
after the operating-site check. It cannot be replaced on rearm, restart, or
recovery.

Mission-flight `GPSCoord.alt` values and `get_current_gps()` use metres relative
to original H. Global waypoint outputs convert a mission-flight value to AMSL
with `H.amsl_m + coord.alt`. Only takeoff uses the current flight-controller
home: after arming, the controller requests and confirms a fresh causal
`HOME_POSITION`, checks it against the approved tolerances, and translates the
original-H target into the FC-home-relative takeoff parameter. Rearming may
change FC home without changing H.

QGC waypoint setup is different. It captures MAVLink
[`GLOBAL_POSITION_INT.relative_alt`](https://mavlink.io/en/messages/common#GLOBAL_POSITION_INT)
before original H is pinned and stores that number in `GPSCoord.alt`. The stored
altitude is historical metadata relative to the FC home that existed at capture
time. It is not original-H-relative, AMSL, terrain height, clearance, or a
mission flight target. Horizontal operating-site checks may validate the stored
latitude and longitude, but must not use the setup altitude as original-H or
AMSL evidence.

Every active phase replaces stored setup altitude before horizontal transit.
FM1 and FM2 use their configured original-H-relative cruise height. FM3 creates
fresh pickup and delivery coordinates from stored latitude and longitude plus
the validated precision-policy cruise height. FM3 also passes the fresh delivery
coordinate into its calibrated clearance correction. It does not mutate caller
coordinates, rewrite the waypoint store, or infer a datum conversion before H
exists.

LiDAR clearance is a calibrated projection of a corrected beam range and a
time-aligned attitude sample onto an explicitly approved locally horizontal
plane. It is not GPS-relative altitude. Camera marker `z` is body-FRD visual
geometry and is not vertical AGL. Precision and release consumers require
distinct fresh samples, bounded source age, receipt latency and cross-stream
skew, plus continuous stability evidence before payload output.

FM3 builds an earth-fixed target anchor from five fresh centered camera
observations. During LAND it stops forwarding rejected target measurements. If
the target stays unhealthy for the configured target-loss timeout (0.50
simulated seconds in the simulator full policy), FM3 confirms GUIDED and
reissues one fixed hold waypoint every 0.20 simulated seconds. The configured
reacquisition count (five in that policy) of consecutive healthy observations
resumes LAND. A 5.0-second hold timeout permits one return to the 4.572 m
acquisition hover before pickup fails. At or below the configured
0.75 m precision-landing floor, FM3 keeps LAND active without requiring another
camera observation. Before the first LAND command it also checks the connected
flight controller against `PRECISION_LANDING_PARAMETERS` in
`drone/control/drone_control.py`.

For enabled FM3 in ordinary complete-startup mode, live construction obtains one
bounded real camera observation before installing QGC callbacks and leaves the
latest-frame producer running. Staged simulation instead constructs the camera
without consuming a frame, then prepares it after the first guarded GUIDED
delivery has opened public simulation progress. FM3 admission always inspects
current readiness; it does not wait for a frame on the receive callback.

Authority starts as `UNKNOWN`. The companion can acquire authority once per
attempt from the fresh ground baseline. Every outbound flight or payload boundary
rechecks current permission.

MAVLink output uses the production writer transaction and requires proof that it
reached DroneKit's real queue boundary. A successful evidence-only release
handover and each GPIO call use separate transactions and do not fabricate a
MAVLink enqueue receipt. Every servo release output revalidates the current
range and stability boundary against the consumed confirmation. If a later
channel fails that boundary, no further channel releases and the partial
failure propagates. Return-to-mid neutralization remains an authority-guarded
continuation output. After a refused or failed release, each neutral position is
attempted once while authority remains current; an authority-loss denial stops
later attempts. These attempts prove neither a physical neutral position nor a
physical payload release.

| Observation | Result |
| --- | --- |
| Fresh healthy RC switch edge plus accepted `LOITER` or `STABILIZE` | `PILOT`; stop all companion flight and payload output. |
| Explicit flight-controller failsafe evidence | `FC_FAILSAFE`; preserve the independent FC response. |
| Missing, stale, unhealthy, unexplained, or contradictory authority evidence | `UNKNOWN`; never infer `PILOT`, and require the independent FC response. |
| Expected companion mode accepted with current permission | Remain `COMPANION`; an expected-mode token records intent but grants no authority. |

Authority loss is one-way for the attempt. Landing, disarming, later telemetry,
or expected-mode registration cannot reacquire it.

## Failure and recovery

A phase failure or abort remains a failed mission even if recovery lands the
aircraft. The 600-second mission deadline is separate from the approved recovery
budget. Recovery runs after the mission deadline scope unwinds, using the same
clock; a clock failure records `UNCONFIRMED` and propagates rather than switching
to wall time.

The one recovery owner preserves an existing LAND or confirmed ground state. If
companion authority and fresh evidence remain, it checks the approved envelope,
climbs only as needed, and returns to original H at
`max(H AMSL + cruise height, current AMSL)`, then commands and confirms `LAND`.
One independently approved local-LAND fallback is allowed if return fails. It
never rearms during recovery. Outcomes are `HOME_LANDED`, `LOCAL_LANDED`,
`PILOT`, `FC_FAILSAFE`, or `UNCONFIRMED` and remain separate from the mission
result.

## Deployment evidence gates

No deployment evidence files or checksums were supplied. Each row therefore has
evidence location `not supplied` and checksum `not available`.

### Deployment facts gate: closed

| Required fact | Evidence required to open the gate | Current evidence location / SHA-256 |
| --- | --- | --- |
| Flight-controller firmware version and exact build | Read-only version/build output from the installed controller | `not supplied` / `not available` |
| Flight-controller model and hardware revision | Controller identity output and a photographed or inventoried hardware label | `not supplied` / `not available` |
| QGroundControl version and build | QGC About/version export from the operator station used for flight | `not supplied` / `not available` |
| MAVLink dialect and protocol version | QGC/link configuration plus a packet capture or message inspection from the exact release | `not supplied` / `not available` |
| Actual MAVLink source and target system/component IDs | Read-only routed-link capture that identifies QGC, flight controller, and companion endpoints | `not supplied` / `not available` |
| RC receiver protocol | Receiver and flight-controller configuration export for the installed hardware | `not supplied` / `not available` |
| Mode-switch channel, PWM bands, and mode mapping | Read-only RC calibration/mode parameter export plus an inhibited bench observation of every switch position | `not supplied` / `not available` |
| Full flight-controller parameter set | Read-only export taken with motors inhibited, with the export path and SHA-256 recorded here | `not supplied` / `not available` |

Source constants do not prove installed routing, firmware, hardware, or safe
altitudes. The only known aircraft firmware label is `4.x.x`, which is not an
exact release. The Copter 4.5.7 decoder reference does not prove all 4.x releases
or the installed aircraft.

### Independent safety gate: closed

Record raw logs, parameter exports, test date, reviewer, evidence path, and
SHA-256 for each result.

| Behavior to verify | Passing evidence required; otherwise flight remains disabled |
| --- | --- |
| GCS loss | Flight-controller action, trigger/timeout, monitored heartbeat source, and observed mode/action |
| RC loss and pilot takeover | Receiver failsafe plus proof that the pilot switch wins during a pending companion mode request |
| Battery failsafe | Thresholds, ordering, and observed independent flight-controller action |
| EKF failure | Trigger condition and observed independent flight-controller action |
| Fence breach | Enabled fence geometry/action and observed enforcement |
| Companion process crash | Observed flight-controller response without companion assistance |
| Frozen mission loop while DroneKit remains connected | Independent watchdog detection and response; companion polling does not count |
| One-way telemetry/command link fault | Results for each failed direction, including which heartbeat/source the flight controller monitors |

### Operating-area gate: closed

No obstacle-cleared return corridor, altitude envelope, energy reserve policy,
or local-landing area has been supplied. Approval requires a dated site survey,
measured obstacles, applicable rules, approved altitude limits, a reserve policy
tied to battery evidence, and a marked local-landing area. The controller has no
general obstacle-aware return planner. A source constant is not a safety case.

The runtime's frozen result separates mission, recovery, monitoring exit, and
cleanup. After `PILOT` or `UNCONFIRMED` recovery it remains command-silent while
observing. Only new trusted fresh exact `ON_GROUND` plus disarmed observations
after the terminal boundary establish natural exit. A terminal SIGINT or the
optional `threading.Event`/`stop_monitoring()` seam records an explicit
unconfirmed stop; an active-phase SIGINT still requests abort and recovery.
Timeout, `UNKNOWN`, or `PILOT` alone never establishes safe handoff.

### Integration and verification gate: closed

Reviewed contract tests are software evidence only. The historical
`final-fix-head` nested suite passed 928 tests in 14.08 seconds with no
exclusions. The reviewed `residual-corrections-head` snapshot passed 935 tests
in 13.75 seconds in the independent review run and 14.19 seconds in its
implementation run. The accepted `residual-fix1-head` software checkpoint
passed 939 tests in 15.51 seconds in its implementation run and 14.92 seconds
in the canonical verification run. Its scoped review accepted
R1-R5 with no new breakage. The historical precision and complete-path subset
passed 75 tests in 6.56 seconds. Its active-FM3 cases vary
captured pickup and delivery altitudes across zero, negative, and large values,
check that caller coordinates remain unchanged, and require both emitted transit
waypoints to use the policy cruise altitude. The public live-factory trace uses
captured altitudes of -6 m and 4000 m, then checks that both FM3 transit commands
encode 260 m AMSL from original H at 250 m and cruise height at 10 m.

The typed physical/injected component binding changes source after that accepted
939-test checkpoint. The canonical local suite for this newer source passed 947
tests in 19.95 seconds. That result is software evidence only. Scoped review,
parent composition, and matching source/image evidence were still pending at
that checkpoint. The guarded parent composition now exists in source and has
offline test coverage, but it has no matching current image or integrated
QGC/SITL evidence.

The Task 5c runtime, bounded Task 11 trace, FM3 altitude,
operating-documentation, and whole-change reviews are historical completed
reviews for `final-fix-head`. The scoped `residual-fix1-head` review is also
accepted as software evidence only. Parent range, camera, guarded payload,
packaging, and early legacy-entry quarantine changes are accepted. The full
parent companion suite passed 165 tests in 2.96 seconds. The obsolete
`comp2026_auto` entry failed before external construction and could not reach
the legacy flight route at that historical checkpoint; the guarded parent QGC
host was still absent then. The current guarded parent host supersedes that
quarantine in source, but none of these software results is evidence of actual
QGC application routing, aircraft AMSL behavior, sensor integration, or
physical payload state.

Before this work resumed, the Docker daemon was unavailable, no SITL binary was
present, and the parent runtime and allowlist were outside the task's writable
scope. Those are historical constraints, not the current build status. The
resumed parent work built the source and corrected companion image. Inert
inspection of image
`sha256:907d481abadfb4a4f54c7a5ea4597ab9ae58a4b24a8c39a0efab2e550812c9fe`
matched all 44 expected source/configuration hashes, found no waypoint store,
and confirmed early quarantine under an inert, network-disabled smoke test.

The isolated non-flight r2 lane ran native ArduCopter 4.5.7 over its private
MAVLink network. It observed packed version `0x040507FF` and custom-version bytes
hex `3261336463346237`, then failed telemetry validation with `requested
telemetry cadence was not achieved`. The retained result does not identify a
particular failed stream, so none is named here. The handler and owner each
processed zero commands, and no test, mission, flight, or payload command ran.
The run removed both owned containers and its private network, retained its
logs, and left unrelated services unchanged. This is an honest failed
non-flight check, not QGroundControl application, guarded parent-host, mission,
or flight evidence.

The aircraft OS/architecture lock remains unresolved, and
`requirements-hardware.in` remains parseable input rather than a verified
aircraft lock. Keep flight authorization closed until the exact aircraft build,
Ubuntu 22.04 target, architecture, and platform lock are verified; a matching
image and integrated QGC/SITL flight/fault matrix pass; and the
propellers-removed bench, hardware, and staged supervised flight checks pass.
Hardware FM3 remains disabled. Report
software status, physical mission result, payload evidence, simulation score,
and artifact validity separately. A bounded shutdown may leave native cleanup
unconfirmed; do not report guaranteed resource release without matching
worker/device evidence. The runtime does not call raw servo `close()` as if it
were passive. If the physical payload adapter lacks a verified
`cleanup_passive()` contract, the runtime suppresses payload cleanup, sets
`payload_closed` to false, and records incomplete cleanup. A supported passive
hook that times out, raises, or returns anything other than literal `True` also
reports incomplete cleanup. This conservative software contract does not
establish a physical cleanup method.
