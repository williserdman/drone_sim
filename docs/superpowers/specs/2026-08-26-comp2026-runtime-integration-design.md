# Comp2026 Runtime Integration Design

**Date:** 2026-08-26

**Status:** Approved design

**Path:** Architectural

## Objective

Integrate the original nested `companion/comp2026` repository into the current
seven-service drone simulation runtime. One normal `drone-sim start` invocation
must automatically run the complete physical competition attempt:

1. FM1 from Home to L;
2. FM2 from L to F2 with payload 2 release;
3. FM3 pickup and delivery of payload 3 from WA to F2;
4. FM3 pickup and delivery of payload 4 from WM to F2; and
5. return to Home, land, and disarm before 600 simulated seconds.

The acceptance outcome is a preserved, independently scored `150/150` run.
The implementation should remain a scrappy, reversible MVP and retain the
original mission code wherever a narrow adapter can make it usable.

## Constraints and non-goals

- Use the original nested repository at `companion/comp2026`, currently based
  on commit `b903edd`, rather than the later mission-runtime rewrite.
- Changes in the nested repository are limited to an automatic entry point and
  small dependency-injection or simulation-time seams that are necessary to run
  the original mission logic.
- Nested-repository commits remain local and are not pushed. Runtime provenance
  records the exact nested commit used by each run.
- Retain the current orchestration, lifecycle, clock, artifact, Gazebo,
  ArduPilot, electromagnet, and scorekeeper boundaries. Do not introduce a new
  service or container.
- Gazebo remains the sole authority for physical pose, contact, joints, and
  payload motion. The companion may request an action but may not directly edit
  Gazebo state.
- QGroundControl command handling, full-attempt retries, multiple vehicles,
  generalized competition infrastructure, dashboards, broad fault injection,
  and unrelated hardening are deferred.
- Camera output for this MVP is `640x480`, RGB, at 20 frames per simulated
  second. This matches the supplied scenario and the original calibration
  assumptions.
- A local workspace handoff includes the nested checkout. Remote reproduction
  of an unpublished nested integration commit remains unavailable until that
  commit is pushed or otherwise distributed; this constraint must be stated in
  handoff documentation rather than hidden.

## Mission and physical contract

Gazebo world XY coordinates are meters relative to Home:

| Point | X (m) | Y (m) | Purpose |
| --- | ---: | ---: | --- |
| H | 0.00 | 0.000 | Start, final landing |
| L | -91.44 | 0.000 | FM1 landing and FM2 start |
| F2 | -152.40 | 0.000 | Payload release zone |
| WA | -45.72 | -9.144 | Payload 3 pickup |
| WM | -45.72 | 9.144 | Payload 4 pickup |

Payload 2 is red and initially attached to the aircraft's physical hardpoint.
Payload 3 is yellow at WA. Payload 4 is blue at WM. All three payloads use the
original mission's ArUco IDs 2, 3, and 4 respectively.

FM1 takes off from H to 10 m AGL, flies to L, lands vertically, and disarms.
FM2 takes off from L to 10 m AGL, flies to F2, stabilizes, and releases payload
2. The cumulative expected score is 80. FM3 performs the same physical pickup
and delivery sequence first for payload 3, producing cumulative score 145, and
then for payload 4, producing cumulative score 150. The vehicle then returns to
H, lands vertically, and disarms.

At a pickup zone, acquisition occurs at 4.572 m AGL and requires five distinct,
fresh marker detections within 0.50 m. A physical attach request succeeds only
when the intended payload is grounded in the intended pickup zone, the aircraft
has free payload capacity, and the hardpoint is within 0.075 m of the payload's
attachment center. Re-reading one camera sample cannot count as multiple
detections. Attachment creates a physical joint and never teleports a payload.

At F2, release eligibility requires the aircraft to remain within 0.15 m of the
target with horizontal speed at or below 0.10 m/s continuously for two simulated
seconds at the required 10 m AGL release height. Release only detaches the joint;
the payload falls and settles under Gazebo physics. The scorekeeper awards the
drop only after the payload settles fully inside F2.

## Chosen approach

Use a thin simulation adapter host around the original mission code.

A small `Comp2026Host` inside the existing companion runtime imports the nested
repository and calls a new nested automatic-attempt entry point. That entry
point sequences the original FM1, FM2, and active FM3 behavior without driving
the QGC listener or recreating the mission in the outer runtime. The host injects
camera, downward range, payload-command, course, and clock adapters. Original
DroneKit flight commands continue to flow normally to ArduPilot SITL.

The mission sources are the existing `src/drone/missions/fm1.py`,
`src/drone/missions/fm2.py`, and the currently active FM3 behavior in
`src/drone/mock_mission.py`. The incomplete `src/drone/missions/fm3.py` is not
silently substituted. The automatic entry point may extract or parameterize the
active FM3 function narrowly so the yellow and blue cycles can be invoked in the
required order, but it must not replace that behavior with a new outer mission
state machine.

This approach preserves the original mission as the decision authority while
isolating hardware-only dependencies. It avoids both the wall-time and command
queue fragility of automatically driving the QGC listener and the excessive
scope of importing the previous mission-runtime rewrite.

## Service architecture

### Orchestration

The current controller still starts and supervises exactly seven services:
orchestration, artifacts, companion, ArduPilot SITL, Gazebo, electromagnet, and
scorekeeper. The default production profile selects the competition world,
`competition_v1` scoring, the automatic full attempt, and a 600-simulated-second
mission deadline. Existing readiness, terminal notification, quiescence, and
artifact-finalization protocols remain in force.

### Companion

The existing companion container remains the only companion service. Its outer
runtime continues to own lifecycle status, structured logging, mission-ready,
mission-command-delivered, mission-finished, and terminal acknowledgements.
`Comp2026Host` owns only adaptation and invocation of the nested mission.

The automatic mission executes in a worker thread. A ROS executor thread remains
available to receive `/clock`, image, range, ground-truth diagnostic, and payload
service updates while original blocking mission calls execute. Mission sleeps,
timeouts, freshness tests, hold windows, and the overall deadline wait against
the injected simulation clock. Wall time is used only for infrastructure liveness
and shutdown deadlines.

The original DroneKit controller connects to the current ArduPilot endpoint. The
course adapter converts configured H-relative Gazebo targets into the navigation
coordinates expected by DroneKit using the simulator's configured Home origin
and ENU frame. Gazebo world `x` is east and world `y` is north. For each target,
`east_m = x` and `north_m = y`; using WGS84 equatorial radius 6,378,137 m, the
adapter computes `latitude = home_lat + degrees(north_m / radius)` and
`longitude = home_lon + degrees(east_m / (radius * cos(home_lat)))`. Altitude is
supplied separately as AGL. Course coordinates have one configuration source and
are not duplicated as mission literals.

### Gazebo

Gazebo owns the competition course, zone visuals, aircraft and sensors, physical
payload models, contacts, detachable joints, pose history, and both cameras. It
publishes the ground truth required by payload policy and read-only scoring.

The competition aircraft extends the current flight-capable Iris with:

- a downward RGB camera at `640x480` and 20 Hz;
- a downward single-beam range sensor at 20 Hz, laterally offset so an attached
  payload does not occlude the ground measurement;
- a centered physical payload hardpoint with capacity one; and
- the minimum attach/detach coordinator required by Gazebo's detachable-joint
  mechanism.

The payload-2 initial pose matches the aircraft hardpoint rather than the
aircraft model origin. Payloads 3 and 4 start physically on WA and WM. Every
payload has collision, inertia, mass, color, and a top-facing ArUco texture.

### Electromagnet

The current inert placeholder becomes the payload policy and command gateway.
It accepts idempotent attach or release requests containing run identity,
payload ArUco ID, action, and command identity. For attachment it validates the
known payload, mission zone, grounded state, capacity, and physical center error
from current Gazebo facts. For release it validates that the requested payload is
currently attached. It then forwards an attach/detach command to Gazebo and
publishes the confirmed result and scenario event.

Rejection is explicit and has no physical side effect. The electromagnet cannot
set a model pose, award score, or command the aircraft.

### Scorekeeper

The scorekeeper adds a versioned `competition_v1` policy. It remains read-only
with respect to the vehicle, electromagnet, and Gazebo. It consumes ordered
simulation time, ground truth, and confirmed scenario events and independently
evaluates mission sequence, landing/disarm conditions, pickup outcomes, release
stability, payload settling, final Home landing, and the deadline.

Companion claims and electromagnet request acceptance are evidence but are not
sufficient to award physical score. Invalid, duplicate, stale-run, or out-of-order
events remain observable and award no score.

### Artifacts

The existing artifact pipeline records both complete 20-Hz image streams,
`/clock`, vehicle and payload ground truth, payload requests and results,
scenario events, score events, lifecycle events, Gazebo state, and all service
logs. The manifest snapshots resolved course/scenario/scoring configuration,
the parent revision, the exact nested revision, image digests, checksums, and
evidence references for every awarded score event.

## Competition scene and assets

The prior transfer project's generated `competition_mission.sdf` is not
self-contained: it references separate `model://` vehicle and payload assets.
The current repository therefore does not depend on that project's `.build`
directory and does not copy only the final world file. Its old `.build` payload
geometry also disagrees with its current scenario and generator, so those stale
outputs are not authoritative.

Port the narrowly required source material:

- `config/course.yaml` and `config/scenario.yaml` semantics;
- course pad and zone layout;
- payload geometry and ArUco texture generation;
- competition Iris sensor and hardpoint additions; and
- the minimum payload attach/detach Gazebo coordinator.

The current repository owns a trimmed deterministic asset generator plus the
generated world/model resources used by the runtime. Generated assets are
validated against configuration so a handoff can rebuild them without the
transfer workspace. The original transfer companion rewrite, referee framework,
dashboard, broad test suite, and unrelated scripts are not imported.

## End-to-end data flow

1. `drone-sim start` resolves configuration and launches the existing seven
   services. Recorders become ready before simulation advances.
2. Gazebo loads the competition world with payload 2 attached and payloads 3 and
   4 at WA and WM. ArduPilot establishes its normal flight-dynamics exchange.
3. The companion waits for current-run ArduPilot, `/clock`, camera, range, and
   payload-authority readiness before arming.
4. The host loads the configured course once, performs the frame conversion,
   and gives the automatic entry point its navigation targets and adapters.
5. The worker runs FM1, FM2, FM3 for marker 3, FM3 for marker 4, and return Home.
   Original DroneKit commands travel through ArduPilot to Gazebo.
6. Gazebo camera frames enter a latest-frame adapter with simulation timestamps.
   The original ArUco calculation consumes actual frames and only distinct fresh
   frames advance acquisition. Downward Gazebo range feeds the injected lidar
   interface.
7. Original pickup/release calls enter the payload adapter, which sends an
   idempotent request to electromagnet. Electromagnet validates current physical
   truth and asks Gazebo to change the detachable joint. The confirmed result
   returns to the companion and is independently visible to scoring.
8. The scorekeeper continuously derives ordered score transitions from physical
   truth. Artifacts record the same source topics and all service decisions.
9. After the final Home landing and disarm, the companion reports mission finish,
   scorekeeper commits the result, Gazebo freezes, recorders quiesce, and the
   orchestration protocol atomically finalizes the run bundle.

## Failure handling

- Before arming, missing or stale clock, ArduPilot exchange, camera, range,
  payload authority, or required Gazebo entities fails the attempt explicitly.
- Camera and range freshness is based on simulation timestamps. Paused
  simulation does not consume time or produce false progress.
- An invalid pickup never creates a joint or changes a payload pose. The reason
  is recorded and the mission makes a best-effort safe return Home.
- An invalid release, lost payload, phase timeout, or original mission exception
  fails the attempt without fabricated score and triggers best-effort
  RTL/land/disarm.
- One automatic full attempt is allowed. Only the original local landing
  correction behavior is retained; there is no full-attempt retry or QGC
  fallback.
- Unrecoverable service failure and the 600-simulated-second deadline use the
  current orchestration failure path. Every recoverable log and partial artifact
  is finalized under `FAILED` or `ABORTED` rather than discarded.
- Score input gaps, impossible ordering, duplicate events, or incomplete
  physical evidence cannot produce `COMPLETED` or maximum score.

## Focused verification

Automated tests cover only the new critical boundaries:

- course/scenario parsing and exact H-relative entity placement;
- generated world, models, marker textures, sensors, and payload plugin presence;
- Gazebo-to-navigation coordinate conversion;
- distinct camera-frame counting and simulation-time waits;
- payload request idempotency, capacity, identity, and off-center rejection;
- physical stability-window, settling, ordering, and score calculations; and
- automatic phase sequencing against adapter fakes.

These checks are necessary but do not prove the MVP. The authoritative
acceptance test is a real run started through the normal operator command:

```bash
drone-sim start
```

The preserved acceptance bundle must prove:

1. all seven services became ready and the configured competition entities and
   sensors existed before arming;
2. FM1 flew H to L, landed vertically, and disarmed;
3. FM2 physically released payload 2 at F2 after the required stability window,
   producing cumulative score 80;
4. FM3 physically acquired payload 3 at WA without teleportation and delivered
   it to F2, producing cumulative score 145;
5. FM3 did the same for payload 4 at WM, producing cumulative score 150;
6. the vehicle returned to H, landed, and disarmed before 600 simulated seconds;
7. payload pose history remained continuous, capacity never exceeded one, and
   every scored payload visibly settled inside F2; and
8. the terminal manifest reported `COMPLETED` and `150/150`, named both source
   revisions, and validated both videos, the ROS bag, Gazebo state, score evidence,
   configuration snapshots, and all seven service logs.

Both onboard and observer recordings are inspected as part of acceptance. A
passing unit suite, mocked mission, partial route, synthetic score, or merely
plausible final manifest is insufficient. The integration goal remains incomplete
until an actual preserved run satisfies every acceptance item.

The portable recording contract is encoder-independent: both streams must be
H.264, `yuv420p`, 20 FPS, 640x480, and contain the exact public-frame count.
Local/default execution uses `libx264`; an explicit GPU deployment may use
`h264_nvenc` only after a real hardware-encode preflight. Native Gazebo and SITL
clocking remain unchanged when public `/clock` fan-out is bounded to the same
50 ms evidence cadence.

## Deferred technical debt

- Restore optional QGC command-driven mission selection.
- Add policy for complete-attempt retries and recovery from transient pickup
  failure.
- Generalize payload authority beyond the three configured competition payloads.
- Expand fault injection, performance tuning, and broad regression coverage only
  when runtime evidence shows they block the critical path.
- Establish remote distribution for the local nested integration commit when the
  user authorizes a push or another publication mechanism.
