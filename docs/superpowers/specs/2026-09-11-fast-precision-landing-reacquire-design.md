# Fast precision-landing reacquisition design

## Purpose

Keep the competition mission's healthy precision descents at 0.50 m/s while
preventing noisy or stale camera estimates from driving large roll and pitch
commands. If target evidence becomes unreliable, the vehicle stops descending,
holds position, and tries to reacquire. It gets one bounded return to the search
hover before the pickup fails closed.

Completion requires a fresh, valid end-to-end run bundle with every payload
physically picked up and delivered, a 150/150 score, a completed manifest, and
usable flight recordings. Unit tests or a score alone do not complete the work.

## Evidence behind the change

The accepted physical run at
`runs/259863d9-c558-4102-ae34-fe6c31f5cf94` used ArduCopter 4.7 with
`LAND_SPD_MS=0.50` and `PLND_OPTIONS=4`. The later run at
`runs/8e7a6494-d824-474a-8b88-dfada27f755f` used `LAND_SPD_MS=0.10` and
`PLND_OPTIONS=0`, took about 82 simulated seconds longer in the comparable
landing phase, and missed payload 4 by 1.78 m.

The failed run did load the saved roll AutoTune gains. Vibration data did not
show a matching physical-vibration increase. During payload-4 target
acquisition, desired attitude reached 12.70 degrees roll and 17.61 degrees
pitch, while the accepted flight stayed below 0.34 degrees. The failed target
measurements also spread by roughly 0.31 m and 0.50 m on the two horizontal
axes. The camera view shook because the airframe followed those commands.

ArduPilot already ran with `PLND_STRICT=1`, `PLND_RET_MAX=4`, and
`PLND_TIMEOUT=4`. Those settings did not protect this flight because unstable
measurements continued to refresh the target before ArduPilot classified it as
lost. The fix therefore validates measurements before sending them and keeps
ArduPilot retry behavior as a backstop.

The missing fast-landing overlay is the confirmed deployment regression. The
available package does not contain raw image timestamps paired with every
detection, so it cannot prove why the longer precision loop eventually
diverged. This change records that evidence, rejects unsafe input, and preserves
each failed run for another diagnosis rather than pretending the remaining
cause is known.

## Scope and source baseline

Implementation starts from parent commit
`4132d7deb4d54e5b6e379e3ffc84f7c41b3abf3c` and nested Comp2026 commit
`54cdeffd36f6936036973e4a93bd38f78d62add2`. This is the most recent runnable
legacy automatic mission pairing. It uses pinned ArduCopter 4.7 commit
`1511f27194f1dcc3728270883047bdf022b3fd53`.

The independent `companion/comp2026` repository receives its own commit. The
parent image build must name that exact nested revision, and the run manifest
must record matching source and image provenance.

The unfinished QGC integration in the main checkout is outside this change. It
currently disables FM3 and cannot produce the required three-payload proof.
This change also does not alter the scorer, add a camera gimbal, fork ArduPilot,
retune the inner attitude loop, or impose a global vehicle angle limit.

## Landing behavior

The existing mission interface stays `pickup_sequence(...) -> bool`. Initial
acquisition remains at 4.572 m AGL and requires five distinct, fresh, centered
observations before entering LAND. Their projected earth positions must fit
within the same 0.20 m consistency radius; their coordinate-wise median becomes
the fixed target anchor for the landing attempt.

Each landing attempt follows these states:

1. **Tracking.** Enter LAND and send at most one `LANDING_TARGET` for each
   accepted camera frame. Healthy descent remains 0.50 m/s.
2. **Hold and reacquire.** If no acceptable observation arrives for 0.50
   simulated seconds, enter GUIDED, confirm the mode change, and repeatedly
   command one earth-fixed waypoint at the current horizontal position and
   relative altitude. Do not send landing targets while GUIDED. Continue camera
   and range sampling without descending.
3. **Resume.** Require five consecutive acceptable observations while holding.
   A rejected or missing observation resets the count. Confirm fresh range,
   re-enter LAND once, and resume the 0.50 m/s descent.
4. **Search-hover retry.** If hold/reacquire does not succeed within 5.0
   simulated seconds, return in GUIDED to 4.572 m AGL and run the existing
   target acquisition once more. A second landing attempt then uses the same
   rules.
5. **Failure.** Failure to confirm GUIDED, obtain fresh range, reacquire on the
   single retry, or touch down within the existing 60-second landing budget
   returns `False`. The caller performs its existing safe recovery. It never
   disarms or requests payload attachment for an unconfirmed touchdown.

The 60-second budget includes tracking, holds, and the one search-hover retry.
Touchdown confirmation, disarm confirmation, and payload attachment remain
ordered exactly as they are now.

## Measurement acceptance

The camera module adds one atomic observation interface that returns the marker
vector and the source frame timestamp together. Existing callers of
`vec_to_marker_3d` keep their current interface. Simulation capture must have a
bounded, non-blocking no-frame result so camera silence cannot prevent the
landing deadline or hold trigger from running.

A landing observation is acceptable only when all of these statements hold:

- The frame timestamp is an integer, strictly greater than the last consumed
  timestamp, not in the future, and no more than 0.25 simulated seconds old.
- The marker vector contains finite forward, right, and down values, and down
  is positive.
- Downward range is fresh under its existing 0.50-second rule.
- Projecting the body-frame marker vector through the current vehicle attitude
  and position places the stationary target no more than 0.20 m from the
  earth-fixed target anchor established by the five acquisition samples.
- While reacquiring, horizontal marker error is at most the existing 0.50 m
  centering tolerance.

A missing, duplicate, regressed, aged, malformed, or spatially inconsistent
observation is rejected and never sent to ArduPilot. One rejection does not
change flight mode. It only stops refreshing the last-acceptable time; the
0.50-second unhealthy window prevents a single dropped frame from causing a
mode switch.

The anchor is the median earth-fixed target location from the five acquisition
observations. The target is stationary, so this check catches cumulative drift
as well as one-frame jumps. It does not low-pass accepted positions or invent
new target measurements.

## Module seams

The change stays at three existing seams:

- `companion/comp2026/src/drone/sensors/camera/camera.py` owns the atomic marker
  observation returned to mission code.
- `companion/comp2026/src/drone/control/drone_control.py` owns one non-blocking,
  integer-coordinate GUIDED waypoint send. The mission reissues it every 0.20
  simulated seconds while holding.
- `companion/comp2026/src/drone/mock_mission.py` owns measurement acceptance,
  the internal landing states, the shared 60-second deadline, and the single
  acquisition retry. Callers still see only pickup success or failure.

`companion/src/drone_sim_companion/comp2026_host.py` already owns the simulated
camera queue and source timestamps. It receives only the smallest change needed
to expose a bounded capture to the nested camera adapter. No ROS message or
public mission-event schema changes.

## ArduPilot parameter profile

The image-baked overlay must contain and the DataFlash log must confirm:

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

`PLND_STRICT=2` prevents an eventual blind landing, and the shorter native
timeout limits descent if the companion stops sending altogether. The
companion normally changes to GUIDED first. `PLND_RET_MAX=1` bounds the separate
native fallback to one attempt; the mission-level search-hover retry remains
under companion control. `PLND_XY_DIST_MAX=0.50` prevents descent when
ArduPilot's own horizontal error exceeds the acquisition envelope. The existing
raw-sensor estimator, measured lag, camera orientation, and saved AutoTune gains
remain unchanged.

Before the first precision LAND command, the companion reads these parameters
from the connected flight controller and fails the pickup if any value differs
from the expected value. Integer-valued settings must match exactly. Fractional
settings use `math.isclose` with zero relative tolerance and `1e-3` absolute
tolerance. Parent configuration tests also assert the source overlay. Source
edits alone do not count because the overlay enters the runtime only after
rebuilding the SITL image.

## Diagnostics and artifacts

The companion writes parseable landing records to its existing runtime log as
`PRECISION_LANDING ` followed by one compact JSON object with sorted keys. A
record includes the state, transition reason, simulation time, target ID,
frame timestamp and age, AGL, marker forward/right/down, horizontal error,
anchor drift, retry number, and requested and observed flight mode. Record each
state transition and each rejected observation. Do not add a new ROS topic for
this change.

The completed run bundle must contain the normal DataFlash log, companion log,
score evidence, rosbag, and onboard and observer videos. The artifact validator
must accept the package. Physical mission outcome, score, and artifact validity
are reported separately.

## Testing and proof sequence

Development follows red-green-refactor for each behavior. Focused tests cover:

- fresh monotonic observations preserve uninterrupted LANDING_TARGET output;
- missing, duplicate, regressed, future, aged, non-finite, non-positive-down,
  and anchor-drift observations are never sent;
- the unhealthy timer uses simulation time and starts GUIDED hold only after
  0.50 seconds;
- ordering is LAND, confirmed GUIDED, repeated absolute hold; no landing target
  is sent while holding;
- five consecutive acceptable observations with fresh range cause exactly one
  return to LAND, while a bad observation resets the count;
- a failed hold returns once to 4.572 m acquisition, permits one more landing
  attempt, and then fails closed;
- camera silence cannot block past the shared deadline;
- failed mode confirmation, stale range, or expired 60-second budget prevents
  disarm and attachment;
- the exact parameter profile is parsed and enforced;
- existing acquisition, touchdown, payload, and mission-ordering tests remain
  green.

After focused and parent unit tests pass, rebuild the affected companion and
ArduPilot images with explicit source revisions. Run one fresh full 600-second
competition mission with seed 2026 and preserve the resulting package.

The goal is complete only when that fresh package proves all of the following:

- the manifest has terminal status `COMPLETED`;
- the score is exactly 150/150;
- payload IDs 2, 3, and 4 each have confirmed physical attachment, lift,
  release, and valid delivery placement;
- the vehicle returns home and lands;
- the bag and both flight videos pass the existing artifact checks and can be
  opened;
- the DataFlash parameter snapshot matches the profile above;
- the DataFlash parameter snapshot retains the promoted roll AutoTune gains
  from `ardupilot_sitl/params/descent.parm`;
- for DataFlash `ATT` samples whose nearest `PL` sample has `TAcq=1`, desired
  roll and pitch each remain within 5 degrees and actual roll and pitch each
  remain within 8 degrees;
- every hold or retry, if any, has a matching structured transition record and
  still finishes inside the mission window.

If the flight misses any item, retain the run unchanged, diagnose it from the
new evidence, and continue the goal. Do not weaken the scorer, artifact checks,
or acceptance limits to turn a failed run into a pass.

## Documentation impact

The same implementation change updates:

- `companion/README.md` for the landing health and recovery behavior;
- `ardupilot_sitl/README.md` for the exact fast-landing and retry guarantees;
- `docs/architecture.md` for the companion/ArduPilot ownership split;
- `docs/runbook.md` for image rebuild, focused checks, full-run command, and
  artifact inspection;
- `docs/handoff.md` with dated verification evidence and any remaining gap.

Exact parameter values stay linked to `ardupilot_sitl/params/descent.parm`
rather than copied into multiple current guides.
