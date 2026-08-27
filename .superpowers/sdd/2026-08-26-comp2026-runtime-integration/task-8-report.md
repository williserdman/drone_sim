# Task 8 Report: Host the Original Mission in the Companion Service

## Status

Implemented and verified the thin parent adapter from parent base
`cae07bd8794bb042b4ea7adfad42e5e9884e11c9`. The current companion service now
selects the original Comp2026 automatic attempt for resolved
`mission=comp2026_auto`; the nested checkout remains the phase and flight-decision
authority at exact local commit `2241d00444db6414a5a1c646c8971c84943849a8`.

No remote operation or push was performed. The nested checkout was neither
edited nor committed.

## Files

Created:

- `companion/src/drone_sim_companion/comp2026_host.py`
- `companion/src/sitecustomize.py`
- `companion/tests/test_comp2026_host.py`
- this report

Modified:

- `.dockerignore`
- `companion/Dockerfile`
- `companion/pyproject.toml`
- `companion/src/drone_sim_companion/runtime_node.py`
- `companion/tests/test_runtime_node.py`
- `companion/EXTERNAL_INTERFACE.md`
- `companion/INTERNAL_INTERFACE.md`
- `uv.lock`

The pre-existing untracked parent `SYSTEM_DIAGRAM.md` and nested `docs/` were
preserved and were not staged.

## Implemented behavior

- Resolved `controlled_descent` retains its existing PyMAVLink runtime body.
  Resolved `comp2026_auto` creates one ROS node and responsive multithreaded
  executor, then runs the exact nested `run_auto_attempt` in one blocking worker.
- The host consumes the existing current-run `/clock`, onboard image plus
  `FrameMetadata`, downward range, payload service, run-state, and DroneKit TCP
  interfaces. It publishes only the typed mission events and established
  lifecycle/status outputs.
- ENU conversion uses radius 6,378,137 m with x=east and y=north around the one
  resolved DroneKit Home/course source. All required H/L/F2/WA/WM points are
  passed to the original attempt at its required 10 m transit altitude.
- `SimulationClock` exposes the existing accepted monotonic simulation timestamp.
  Each sleep saves its own start and waits for
  `current_timestamp - saved_start >= duration`; stop wakes the waiter. No epoch,
  rebase, activation barrier, pre-zero buffer, lockstep queue, or cross-service
  synchronization was added.
- `RosFrameSource` filters current-run onboard metadata, joins an image only at
  the exact same genuine timestamp, validates the resolved 640x480 `rgb8`
  contract, converts RGB bytes to BGR NumPy without `cv_bridge`, and blocks for a
  strictly newer pair. It never fabricates or reuses `last_timestamp_ns`, and its
  bounded four-timestamp join window is not a recording queue.
- `RosLidar` retains one timestamped finite, in-range beam. `get_distance()` fails
  closed with no current data, a future sample, or age greater than exactly
  500,000,000 elapsed simulation nanoseconds.
- `PayloadDropper` maps nested attach/release calls to blocking typed service
  calls. Per-action command IDs are
  `run:<aruco>:<attach|release>:<sequence>`. Return requires the identical ID, a
  positive response sequence, and literal `accepted is True`; attach returns
  literal `True`, while every missing/rejected/mismatched response raises.
- The phase emitter preserves nested callback order with contiguous IDs from zero
  and genuine current `/clock` timestamps. The nested authority still observes
  DroneKit `armed is False` before `HOME/DISARMED`, then advances 50 ms before
  `HOME/COMPLETE`; the host requires Home complete before `mission-finished`.
- On exception, the host records the active phase, writes the current-run runtime
  failure once, and requests RTL, LAND, and DISARM best-effort. ROS callbacks stay
  responsive through orchestration finalization; waits then stop, the worker
  joins, quiescence is written, and DroneKit closes.

## Authorized lifecycle correction

The implementation follows the parent ruling that durable `mission-ready` for
this branch describes the initialized companion/ROS/DroneKit/blocked-worker seam
and therefore may precede sensor samples that begin only after `RUNNING`.
`Comp2026StartGate` independently prevents the worker from entering the nested
attempt until it has observed current-run `RUNNING`, clock, exact frame pair,
range, live payload service, heartbeat, and `is_armable is True` facts. Thus no
`FM1/STARTED` event or original arming path can precede the live-start gate.

The existing public-zero lifecycle rendezvous remains unchanged in shape: the
host sends GUIDED and records the established `mission-command-delivered` status
at simulation timestamp zero. This transport/lifecycle adaptation does not arm
the vehicle or enter the nested phase machine.

Focused coverage proves `mission_ready is True` immediately after process
readiness while `mission_start_ready is False`; the latter remains false after
all samples except armability and becomes true only once armability is actually
observed.

## RED evidence

The required initial focused command was run before the host existed:

```text
$ uv run pytest companion/tests/test_comp2026_host.py companion/tests/test_runtime_node.py -q
ERROR collecting companion/tests/test_comp2026_host.py
ModuleNotFoundError: No module named 'drone_sim_companion.comp2026_host'
1 error in 3.30s
```

After the host/runtime slice, a separate production-import regression was added
before the startup compatibility hook. It reproduced the Python 3.12 failure:

```text
AttributeError: module 'collections' has no attribute 'MutableMapping'
1 failed in 1.26s
```

The implementation then added only the two required startup aliases and kept
`future` as an explicit production dependency. Intermediate GREEN observations
were 9 host tests passed, then 31 combined host/runtime tests passed, followed by
the raw DroneKit import test passing.

## Fresh GREEN evidence

```text
$ uv run pytest companion/tests/test_comp2026_host.py companion/tests/test_runtime_node.py -q
32 passed in 0.95s

$ uv run pytest companion/tests -q
64 passed in 1.14s

$ uv run python -m compileall -q \
    companion/src/drone_sim_companion companion/src/sitecustomize.py
# exit 0

$ git diff --check
# exit 0
```

The timing test starts a one-second sleep at absolute simulation time 10 seconds,
proves 10.999999999 seconds does not release it, and proves exactly 11 seconds
does. Frame tests prove same-timestamp pairing, RGB-to-BGR output, genuine
50,000,000 ns identity, no duplicate delivery, and blocking until the exact
100,000,000 ns image/metadata pair exists. Range passes at the inclusive
500,000,000 ns age boundary and fails at 500,000,001 ns. Payload tests prove the
worker remains blocked before confirmation, then returns only for the exact
`run:3:release:1` response and rejects mismatched correlation.

## Production image and context evidence

The exact requested build passed from the final source and narrowed root Docker
context:

```text
$ docker build --target runtime -f companion/Dockerfile \
    -t drone-sim-companion-comp2026 .
Successfully built efbd4b7bdc3f
Successfully tagged drone-sim-companion-comp2026:latest
```

Image identity:

```text
sha256:efbd4b7bdc3f50a39720df4d1807f678131556bf39b3983cc74225591c30f173
```

Fresh in-image imports reported:

```text
4.10.0 True dronekit drone.auto_attempt drone_sim_companion.runtime_node
future=1.0.0 dronekit=2.9.2 numpy=2.1.3 opencv=4.10.0.84
pymavlink=2.4.49 PyYAML=6.0.3
aliases True True
```

The root `.dockerignore` now allows the required
`companion/comp2026/src` input while excluding the nested `.git`, docs, tests,
README, recordings/caches, backup/ground-control/SITL tools, standalone entry
scripts, scratch mission data, and hardware-only lidar/servo adapters. Image
inspection found `/opt/drone_sim/comp2026` contains only `src` and its narrowed
automatic-mission dependency closure; all named history/non-runtime paths were
absent.

## Repository identities and statuses

Before the parent commit, the protected identities were rechecked as:

```text
parent HEAD: cae07bd8794bb042b4ea7adfad42e5e9884e11c9
nested HEAD: 2241d00444db6414a5a1c646c8971c84943849a8

parent protected untracked paths:
?? SYSTEM_DIAGRAM.md
?? companion/comp2026/

nested status:
## integration/drone-sim-mvp
?? docs/
```

The parent implementation commit is the commit containing this report. Its hash
and the final post-commit statuses are recorded in the task handoff.

## Self-review

- Confirmed the parent calls the exact nested `drone.auto_attempt.run_auto_attempt`
  and does not duplicate its phase order, mission retry, flight logic, landing,
  or payload decisions.
- Confirmed the selected source does not import ground truth, command Gazebo,
  add a service, use QGC, or introduce an outer mission state machine.
- Confirmed one executor remains independent of the blocking mission worker and
  services asynchronous payload responses while that worker waits.
- Confirmed callbacks discard pre-RUNNING samples and stale-run metadata; the
  worker gate cannot start from durable process readiness alone.
- Confirmed clock, frame, range, phase, and command identities are always taken
  from existing run-scoped inputs rather than synthesized.
- Confirmed controlled-descent runtime logic remains in its original function
  body and is selected unchanged for its resolved mission value.
- Confirmed the staged scope contains only the files listed above and neither
  protected untracked artifact.

## Concerns and deferred work

No Task 8 implementation blocker remains. A live seven-service preserved
competition attempt was not required by the brief and was not fabricated; that
end-to-end run remains the next integration check. Docker also reports that its
legacy builder is deprecated, but the exact requested build and all image smoke
checks completed successfully.

## Fix Round 1

### Status and scope

Resolved all four Important review findings from parent fix base
`a89f8bd0bc6a220aa0c4e90541358f933a2d606e`. Modified only:

- `.dockerignore`
- `companion/src/drone_sim_companion/comp2026_host.py`
- `companion/src/drone_sim_companion/runtime_node.py`
- `companion/tests/test_comp2026_host.py`
- `companion/tests/test_runtime_node.py`
- `companion/EXTERNAL_INTERFACE.md`
- `companion/INTERNAL_INTERFACE.md`
- this report

The nested checkout remained exact at
`2241d00444db6414a5a1c646c8971c84943849a8` with only its preserved untracked
`docs/`. Parent `SYSTEM_DIAGRAM.md` and the nested checkout remained untracked
and unstaged in the parent.

### RED evidence

Each behavior was pinned before its production correction:

1. The five readiness-invalidation regressions initially failed collection
   because `refresh_comp2026_start_gate` did not exist. The previous gate could
   only OR/latch facts and had no API capable of replacing the live snapshot.
2. The fatal-input regression initially failed collection because
   `AttemptFailureCoordinator` did not exist. A callback error therefore had no
   shared cancellation, success guard, or single recovery claim.
3. The teardown regressions initially failed collection because
   `quiesce_comp2026_runtime` did not exist. The old `finally` block wrote
   quiescence before executor shutdown and after only a bounded worker join.
4. The old production image failed the artifact-boundary regression with:

   ```text
   unexpected non-runtime source: drone/missions/fm3.py
   ```

   The first default-deny rule set also stayed RED: a literal manifest diff found
   64 files instead of 13 because legacy-Docker directory negations re-admitted
   descendants. Hierarchical re-exclusion made the scratch context export exact
   before the second production rebuild.

The heartbeat ruling also received a mutation check. Temporarily changing the
existing transport-health limit from 60 seconds to 2 seconds produced:

```text
FAILED test_start_readiness_invalidates_when_current_dronekit_heartbeat_exceeds_existing_timeout
assert False is True  # current last_heartbeat=2.000001
```

Restoring the already configured `DroneControl(... heartbeat_timeout=60)`
interface returned the regression to GREEN.

### Live readiness correction

Process readiness, RUNNING, and first public-clock observation remain durable
facts. The other five start predicates no longer latch. Every main-loop refresh
queries and atomically replaces one locked snapshot containing:

- a genuine undelivered image/metadata pair from `RosFrameSource.ready`;
- current `RosLidar.get_distance()` success, including the 0.5 simulated-second
  age rule;
- current payload-service availability;
- current `vehicle.is_armable is True`; and
- current DroneKit transport health.

The heartbeat predicate does **not** create or retain a wall timestamp/window.
It directly reads DroneKit's current `last_heartbeat` value and compares it with
the exact existing 60-second `heartbeat_timeout` used by the nested
`DroneControl` connection. Mission freshness and all retained elapsed windows
remain simulation-time calculations. The test proves 2.000001 seconds is still
healthy under that existing interface, while 60.000001 is not.

Regression coverage first reaches ready, then independently invalidates on range
age 500,000,001 simulation nanoseconds, service loss, armability reversion,
current DroneKit heartbeat timeout, and consumption of the only genuine frame
pair.

### Fatal-attempt coordination

The first callback or mission failure now owns one reason. It synchronously
stops/wakes the gate, frame source, payload client, simulation clock, and event
emitter. Subsequent sensor callbacks skip input acceptance; subsequent nested
phase emission raises; terminal `mission-finished` is guarded by the same
failure lock.

Recovery is claimed once only after the original worker has terminated, so RTL,
LAND, and DISARM cannot compete with original mission flight logic. Repeated
failures and repeated recovery calls are idempotent. The focused regression
emits `FM1/STARTED`, injects a fatal image callback error, proves no
`FM1/COMPLETE` or terminal success can follow, retains only the first exact
reason, and observes exactly one recovery.

### Truthful teardown

`quiesce_comp2026_runtime` now performs the boundary in this order:

1. stop all attempt waits/output;
2. require the original worker to terminate;
3. shut down and join the ROS executor;
4. destroy the ROS node and close DroneKit;
5. emit final lifecycle output and write quiescence last.

A real-thread regression asserts both worker and executor are not alive inside
the finalization callback and verifies producer close precedes quiescence. A
stuck-worker regression writes exact runtime failure and proves no quiescence
marker is produced. Executor shutdown failure or a surviving executor thread is
handled by the same fail-without-quiescence result.

### Exact Docker closure and image

The root context now ignores the entire nested checkout first. It selectively
unignores parent directories, re-excludes their descendants, then admits only a
literal 13-file automatic-attempt closure:

```text
drone/auto_attempt.py
drone/common_types.py
drone/control/drone_control.py
drone/control/mission_info.py
drone/missions/fm1.py
drone/missions/fm2.py
drone/missions/utils.py
drone/mock_mission.py
drone/sensors/camera/_camera_manager.py
drone/sensors/camera/calibration.json
drone/sensors/camera/camera.py
drone/timebase.py
drone/utils/position_smoother.py
```

The production image matched that manifest byte-for-path with no diff. The lazy
original-function loader proved FM1=`drone.missions.fm1`,
FM2=`drone.missions.fm2`, and active FM3=`drone.mock_mission`; an explicit
`find_spec` check proved incomplete `drone.missions.fm3` is absent.

Final production build:

```text
$ docker build --target runtime -f companion/Dockerfile \
    -t drone-sim-companion-comp2026 .
Successfully built c740226e407a
Successfully tagged drone-sim-companion-comp2026:latest
```

Image identity and smoke:

```text
sha256:c740226e407aaad8ffdf70289f3209379d45ecf001e2a0f9688aa431accbb560
4.10.0 True dronekit drone.auto_attempt drone.missions.fm1
drone.missions.fm2 drone.mock_mission drone_sim_companion.runtime_node
future=1.0.0 dronekit=2.9.2 numpy=2.1.3 opencv=4.10.0.84
```

### Final GREEN evidence

```text
$ uv run pytest companion/tests/test_comp2026_host.py \
    companion/tests/test_runtime_node.py -q
40 passed

$ uv run pytest companion/tests -q
72 passed

$ uv run python -m compileall -q \
    companion/src/drone_sim_companion companion/src/sitecustomize.py
# exit 0

$ git diff --check
# exit 0
```

No epoch, timestamp rebase, activation barrier, pre-zero buffer, lockstep queue,
cross-service synchronization, retry, new service, or outer mission state
machine was introduced. The minor review note about applying the resolved
startup timeout to the nested DroneKit constructor remains deferred because it
was not required for these Important correctness fixes and changing the nested
constructor interface would broaden this round. The live seven-service attempt
also remains the next end-to-end integration check.
