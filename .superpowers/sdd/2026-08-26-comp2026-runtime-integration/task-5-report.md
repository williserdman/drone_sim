# Task 5 Report: Physical Payload Authority

## Status

Implemented the `competition_v1` electromagnet as the sole typed-request policy
gateway to the existing Gazebo detachable-joint coordinators. The preserved
`descent_v1` runtime remains inactive and unchanged in external behavior.

## Files

- Created `electromagnet/src/drone_sim_electromagnet/payload.py`
- Created `electromagnet/tests/test_payload.py`
- Modified `electromagnet/src/drone_sim_electromagnet/controller.py`
- Modified `electromagnet/src/drone_sim_electromagnet/runtime_node.py`
- Modified `electromagnet/tests/test_runtime_node.py`
- Modified `electromagnet/Dockerfile`
- Modified `electromagnet/pyproject.toml`
- Modified `electromagnet/EXTERNAL_INTERFACE.md`
- Modified `electromagnet/INTERNAL_INTERFACE.md`
- Modified `uv.lock` for the declared PyYAML runtime dependency

Unrelated untracked `SYSTEM_DIAGRAM.md` and `companion/comp2026/` were not
modified or staged.

## TDD evidence

Initial RED command:

```text
uv run pytest electromagnet/tests/test_payload.py electromagnet/tests/test_runtime_node.py -q
```

Observed collection failures were the intended missing seams:

```text
ModuleNotFoundError: No module named 'drone_sim_electromagnet.payload'
ImportError: cannot import name 'PayloadGateway'
2 errors in 0.49s
```

After the minimal pure authority, the pure-policy GREEN was:

```text
uv run pytest electromagnet/tests/test_payload.py -q
14 passed in 0.06s
```

A second focused RED caught acceptance of a nonmatching confirmed physical
state:

```text
FAILED test_gateway_rejects_nonmatching_physical_confirmation
assert (False, 'OK') == (False, 'PHYSICAL_CONFIRMATION_MISMATCH')
1 failed, 7 passed in 0.22s
```

The minimal mismatch-code fix produced:

```text
uv run pytest electromagnet/tests/test_runtime_node.py -q
8 passed in 0.15s
```

The implemented tests cover attach rejection codes, the inclusive 0.075 m
boundary, current-run and marker identity, pickup-zone identity, capacity,
release identity, command-ID replay/conflict, exact wire commands, strict result
parsing, confirmed response/event publication, timeout fail-closed behavior,
physical-state mismatch, and readiness dependencies.

## Runtime image evidence

The first real build reached the new image smoke import and failed because that
Docker build step had not sourced the generated ROS workspace:

```text
ModuleNotFoundError: No module named 'simulation_interfaces'
```

After sourcing `/opt/ros/jazzy/setup.bash` and `/ros_ws/install/setup.bash` in
that step, the required command succeeded:

```text
docker build --target runtime -f electromagnet/Dockerfile \
  -t drone-sim-electromagnet-payload .
Successfully built cb24756fecb3
Successfully tagged drone-sim-electromagnet-payload:latest
```

The smoke layer imports PyYAML, the three payload message types, the payload
service, and `drone_sim_electromagnet.runtime_node`. A container check also
confirmed generated service constants `ATTACH=1` and `RELEASE=2`.

## Self-review

- No pose, velocity, aircraft-control, scoring, or retry path was added.
- Each newly accepted request creates one marker-specific coordinator publish;
  completed identical requests replay the original response and sequence.
- Service success requires exact marker, command ID, confirmation status, and
  requested physical state. Timeout and mismatch leave attachment state and
  payload events unchanged.
- ROS uses `MultiThreadedExecutor(num_threads=4)` with a
  `ReentrantCallbackGroup`, so result subscriptions can progress during the
  service's bounded wall-time wait.
- Readiness requires current-run vehicle truth, all three payload states, all
  three exact result publishers, and the created service.
- Private aliases match Task 4 exactly: `payload_2`, `payload_3`, `payload_4`.
- The old inactive event, lifecycle signal handling, finalization marker, and
  protocol close remain in the `descent_v1` path.
- `git diff --check` and Python byte-compilation were clean before final
  verification.

## Concerns / deferred work

No Task 5 blocker remains. Full seven-service mission acceptance, scoring
interpretation of payload events, and companion integration belong to later
tasks. The five-second timeout intentionally does not retry; a late physical
result is ignored and recurrent Gazebo state remains the external truth.

## Fix Round 1

### Findings addressed

- Added one request-operation serialization boundary spanning validation,
  confirmation, event publication, and response caching. The mutex is separate
  from the fact/result mutex, so result subscriptions continue while a service
  waits. Concurrent exact duplicates now wait and replay the same response and
  sequence; conflicting reuse cannot cache over the original; distinct command
  IDs revalidate only after the previous operation completes.
- Removed the permanent post-result attachment freeze. A successful result
  updates only the requested payload's immediate fact, while later monotonic
  recurrent `PayloadState.attached` samples remain authoritative.
- Vehicle and per-payload timestamp regressions (including duplicate ticks) are
  ignored. Authorization requires vehicle plus IDs 2/3/4 at one common current
  simulation timestamp and otherwise returns `STALE_PHYSICAL_STATE`.
- More than one recurrent attached fact returns
  `INVALID_PHYSICAL_STATE`; inconsistent truth never maps to free capacity.

### RED evidence

Command:

```text
uv run pytest electromagnet/tests/test_runtime_node.py -q
```

Observed before the controller fix:

```text
6 failed, 9 passed in 0.47s
```

The failures proved that a duplicate returned before the original, two distinct
commands published concurrently, recurrent attachment stayed frozen after a
result, older timestamps overwrote newer facts, a mixed-timestamp world reached
the coordinator and timed out, and multiple attachments were treated as free
capacity and reached the coordinator. The conflict regression also requires the
conflicting caller to remain pending until the original response is cached.

### GREEN evidence

Focused runtime suite after the minimal controller change:

```text
uv run pytest electromagnet/tests/test_runtime_node.py -q
15 passed in 0.48s
```

Full electromagnet suite:

```text
uv run pytest electromagnet/tests -q
37 passed in 0.29s
```

Production image and smoke import:

```text
docker build --target runtime -f electromagnet/Dockerfile \
  -t drone-sim-electromagnet-payload .
Successfully built e700c85183ad
Successfully tagged drone-sim-electromagnet-payload:latest
```

### Files and self-review

- Modified `electromagnet/src/drone_sim_electromagnet/controller.py`
- Modified `electromagnet/tests/test_runtime_node.py`
- Modified `electromagnet/EXTERNAL_INTERFACE.md`
- Modified `electromagnet/INTERNAL_INTERFACE.md`
- Appended this report

Self-review confirmed the operation mutex is never acquired by state or result
callbacks, the five-wall-second timeout and no-retry behavior are unchanged,
coordinator confirmations still gate successful responses/events, and no pose,
scoring, aircraft-command, service-count, descent, or unrelated workspace path
was changed. No remaining fix-round concern was identified.
