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
