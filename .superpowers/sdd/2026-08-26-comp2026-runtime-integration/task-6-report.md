# Task 6 Report: Independent `competition_v1` Physical Scoring

## Status

Implemented and verified from base `468b7fad28885aa15c890b2ccaff2a4e3f33c56b`.
The scorekeeper now selects a separate read-only `competition_v1` policy from
the resolved scenario while retaining byte-compatible `descent_v1` behavior.

## Files

Created:

- `scorekeeper/rules/competition_v1.json`
- `scorekeeper/src/drone_sim_scorekeeper/models.py`
- `scorekeeper/src/drone_sim_scorekeeper/competition.py`
- `scorekeeper/src/drone_sim_scorekeeper/competition_runtime.py`
- `scorekeeper/tests/test_competition_score.py`
- `scorekeeper/tests/test_competition_runtime.py`

Modified:

- `scorekeeper/src/drone_sim_scorekeeper/descent.py`
- `scorekeeper/src/drone_sim_scorekeeper/output.py`
- `scorekeeper/src/drone_sim_scorekeeper/runtime.py`
- `scorekeeper/src/drone_sim_scorekeeper/runtime_node.py`
- `scorekeeper/src/drone_sim_scorekeeper/self_test.py`
- `scorekeeper/src/drone_sim_scorekeeper/__init__.py`
- `scorekeeper/tests/test_descent_score.py`
- `scorekeeper/Dockerfile`
- `scorekeeper/EXTERNAL_INTERFACE.md`
- `scorekeeper/INTERNAL_INTERFACE.md`

Pre-existing untracked `SYSTEM_DIAGRAM.md` and `companion/comp2026/` were not
modified or staged.

## RED evidence

The initial required command failed exactly because the competition policy and
lifecycle did not exist:

```text
$ uv run pytest scorekeeper/tests/test_competition_score.py scorekeeper/tests/test_competition_runtime.py -q
ERROR scorekeeper/tests/test_competition_score.py
ModuleNotFoundError: No module named 'drone_sim_scorekeeper.competition'
ERROR scorekeeper/tests/test_competition_runtime.py
ModuleNotFoundError: No module named 'drone_sim_scorekeeper.competition'
```

The scorer-neutral model extraction also had an independently witnessed RED:

```text
ModuleNotFoundError: No module named 'drone_sim_scorekeeper.models'
```

Every self-review correction was reproduced before its minimal fix:

- next-tick recurrent physical attachment truth scored only 80 instead of 150;
- attachment truth two state ticks stale at release incorrectly scored 150;
- a non-OK in-attempt payload event was filtered out and a suffix scored 150;
- changed `competition_v1` release tolerance `0.151` was accepted;
- a `0.075001` m vertical pickup pose jump was accepted; and
- source readiness became true before the final payload and Home events arrived.

## GREEN evidence

Scorer-neutral extraction preserved descent behavior:

```text
$ uv run pytest scorekeeper/tests/test_descent_score.py scorekeeper/tests/test_output.py scorekeeper/tests/test_runtime.py -q
17 passed in 0.99s
```

Focused competition increments reached:

```text
20 passed in scorekeeper/tests/test_competition_score.py
7 passed in scorekeeper/tests/test_competition_runtime.py
```

Fresh final verification:

```text
$ git diff --check
$ uv run python -m py_compile scorekeeper/src/drone_sim_scorekeeper/*.py
$ uv run pytest scorekeeper/tests -q
56 passed in 3.61s
```

The perfect trace produces eight event IDs `0..7`, achieves `150/150`, and has
hand-derived cumulative point-component values
`20, 50, 60, 80, 95, 145, 150`. Independent failures cover missing FM1
completion, early/fast/off-position/below-height release, stale attachment,
rotated F2 bounds, unsettled speed, missing physical FM3 attach, XYZ pickup pose
jump, capacity overflow, deadline failure, out-of-order marker 4, non-OK event
history, and pre-start evidence.

## Mission-relative timing evidence

`CompetitionScorer.accept_mission_event()` stores `start_sim_time_ns` only from
the first current-run `FM1/STARTED` event. It never creates or changes a source
clock. Release stability compares the release event timestamp to the exact
window start; settling compares current payload timestamps to the saved eligible
window start; physical freshness compares event and adjacent state timestamps;
and Home evaluates:

```text
home_complete_sim_time - start_sim_time <= 600_000_000_000
```

The focused test starts the mission at absolute simulation timestamp
`900_000_000_000` and accepts Home at `+420_000_000_000`, even though the
absolute Home timestamp is well above 600 seconds. Home at
`+600_050_000_000` fails with `home_deadline_exceeded`. Pre-start physical and
event evidence earns no payload score. No public epoch, rebasing, activation
barrier, pre-zero queue, lockstep queue, or cross-service synchronization was
added.

## Image evidence

Fresh runtime build:

```text
$ docker build --target runtime -f scorekeeper/Dockerfile -t drone-sim-scorekeeper-competition .
Successfully built aa1d958bc572
Successfully tagged drone-sim-scorekeeper-competition:latest
```

Image identity:

```text
sha256:aa1d958bc5729f7f5aa3ff69092f9ff2bc0dc4edc60d12f0b944b5aecb6afa39
```

In-image smoke:

```text
competition_v1 150.0 /opt/drone_sim/scorekeeper/rules/competition_v1.json
```

## Self-review

- Read-only boundary: the competition ROS test asserts that
  `/simulation/score_events` is the sole publisher. There is no vehicle,
  electromagnet, Gazebo, service-client, actuator, pose, force, or physics
  mutation output.
- Independence: confirmed payload events are order evidence only. Points also
  require recurrent physical attachment/detachment truth, exact vehicle release
  windows, settled payload truth, complete rotated containment, and physical
  Home contact.
- Transport races: competition source readiness waits for ground truth, all
  three payload streams, payload event ID 4, and `HOME/COMPLETE` before the
  durable source marker may trigger scoring.
- Physical boundaries: official numeric thresholds and allocation are frozen
  under `competition_v1`; exact inclusive limits pass, values immediately above
  or below the required side fail.
- Compatibility: `descent_v1` retains its original imports, five event ordering,
  serialization documents, 100-point maximum, runtime branch, and QoS behavior.
- Lifecycle: results persist before publication, all eight publications flush
  before `score-finished`, and invalid Home/deadline evidence writes failure
  instead of completion.

## Concerns and deferred work

No Task 6 blocker remains. A real seven-service 150/150 preserved attempt still
depends on the planned companion host and final artifact validation tasks; that
end-to-end run is intentionally outside this focused physical scorer task. The
host Docker installation warns that the legacy builder is deprecated, but both
the real build and image smoke exited successfully.

## Fix Round 1

### Status and files

Resolved the five critical scoring-integrity findings on top of
`7636c4b013e85db2343f541dda8436212219bd69` without changing the ROS message
contracts or introducing a clock, epoch, barrier, queue, or synchronization
mechanism.

Modified:

- `scorekeeper/src/drone_sim_scorekeeper/competition.py`
- `scorekeeper/src/drone_sim_scorekeeper/competition_runtime.py`
- `scorekeeper/tests/test_competition_score.py`
- `scorekeeper/tests/test_competition_runtime.py`
- `scorekeeper/EXTERNAL_INTERFACE.md`
- `scorekeeper/INTERNAL_INTERFACE.md`

The pre-existing untracked `SYSTEM_DIAGRAM.md` and `companion/comp2026/` remain
unmodified and unstaged.

### RED evidence

The first focused run added one regression for each controller finding and
failed all seven selected cases:

```text
$ uv run pytest scorekeeper/tests/test_competition_score.py -q \
    -k 'prestart_vehicle or ground_truth_gap or payload_gap or outside_configured or settling_after_marker or settling_after_home or without_distinct_disarmed'
7 failed
```

The failures demonstrated that a pre-start L sample awarded 50 points, stream
gaps did not produce fail-closed diagnostics, an off-source FM3 pickup scored,
late payload 2 and payload 4 settlement backfilled checkpoints, and
`HOME/COMPLETE` without `HOME/DISARMED` still completed. After converting the
fixture to emit every vehicle and payload stream on the exact 50 ms grid, the
existing perfect trace also stayed RED until the new Home sequence was accepted:

```text
FAILED test_perfect_physical_attempt_scores_150_at_required_checkpoints
diagnostic='mission_sequence_invalid'
```

Additional boundary coverage was then added for marker 4 at the wrong WA
source, payload 3 settling after marker 4, physical landing after DISARMED, and
runtime source readiness without the distinct DISARMED tail event.

### GREEN evidence

Fresh focused scorer/runtime result:

```text
$ uv run pytest scorekeeper/tests/test_competition_score.py scorekeeper/tests/test_competition_runtime.py -q
38 passed in 9.51s
```

Fresh full scorekeeper result:

```text
$ uv run pytest scorekeeper/tests -q
67 passed in 11.16s
```

Fresh descent serialization/runtime compatibility result:

```text
$ uv run pytest scorekeeper/tests/test_descent_score.py scorekeeper/tests/test_output.py scorekeeper/tests/test_runtime.py -q
17 passed in 0.46s
```

`git diff --check` and Python byte-compilation also exited zero.

### Mission-relative timing and scoring integrity

- Vehicle and all three payload streams fail permanently on any post-start
  regression or interval other than exactly 50,000,000 ns. Continuity already
  received when `FM1/STARTED` arrives is checked at that point; later samples
  are checked incrementally.
- Physical vehicle/payload lookups have a hard lower bound at the saved
  `start_sim_time_ns`, and release windows cannot begin before it. Pre-start
  state and event history therefore cannot award any component.
- Marker 3 attachment requires its payload center inside configured WA; marker
  4 requires WM. Adjacent exact-grid states still independently enforce no
  pickup pose jump, while the complete streams make stale capacity carryover
  fail closed.
- Each settlement timestamp is consumed: payload 2 must settle before marker
  3 pickup, payload 3 before marker 4 pickup, and payload 4 before the earliest
  physical Home landing/DISARMED/COMPLETE boundary. Later evidence cannot
  backfill an earlier checkpoint.
- Valid completion requires Gazebo Home XY/contact/speed truth first, then
  strictly later current-run `HOME/DISARMED`, then strictly later
  `HOME/COMPLETE`. Runtime readiness waits for both Home tail events.
- The deadline remains exactly
  `home_complete_sim_time - start_sim_time <= 600_000_000_000`; the 900-second
  absolute source epoch test still accepts a +420-second mission and rejects
  +600.05 seconds.

### Image and smoke evidence

```text
$ docker build --target runtime -f scorekeeper/Dockerfile -t drone-sim-scorekeeper-competition .
Successfully built 08f261deac39
Successfully tagged drone-sim-scorekeeper-competition:latest
```

Image identity:

```text
sha256:08f261deac39ddb585a919e810acbad6bf45b9844ef362a7d20576809e099f4a
```

In-image frozen competition rule smoke:

```text
$ docker run --rm --entrypoint python3 drone-sim-scorekeeper-competition -c '<load competition_v1>'
competition_v1 150.0
```

Packaged ROS/descent compatibility smoke:

```text
$ docker run --rm drone-sim-scorekeeper-competition python3 -m drone_sim_scorekeeper.self_test
{"result": "ok", "ruleset_id": "descent_v1", "score": 100}
```

### Self-review and concerns

- Read-only authority remains intact: no command publisher, service client,
  actuator, electromagnet, Gazebo, pose, force, or physics mutation path was
  added.
- The first implementation compared settlement to the following phase start;
  self-review narrowed that to the exact required physical checkpoint: marker
  attach or the earliest Home landing/disarm/completion boundary.
- The initial continuous fixture made gap checking quadratic; the production
  acceptance path was reduced to an O(1) adjacent timestamp check, retaining a
  one-time history check when mission start arrives.
- `descent_v1` production files are untouched by this fix and its focused
  17-test byte/serialization/runtime suite remains green.
- No Task 6 blocker remains. Live 20 Hz behavior, Gazebo AGL semantics, and the
  controller's real DroneKit-observed disarm event remain explicit downstream
  live-acceptance items for Tasks 7, 8, and 10. Docker again emitted only the
  host legacy-builder deprecation warning.
