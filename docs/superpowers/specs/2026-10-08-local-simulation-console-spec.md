# Local simulation console with QGroundControl flight control

Revised 2026-10-09. The user selected existing onboard missions started from
QGC. This revision awaits written-spec review; it does not describe an
implemented QGC command path. The earlier browser-controlled design and its
acceptance evidence remain in Git history at 9ff4e5f and in
[handoff](../../handoff.md).

## Intended outcome

Run one selected world and existing mission locally on this Mac, preserving
PR #3 ancestry, native arm64 Colima, native QGroundControl, interactive streamed
Linux Gazebo, and a separate live onboard camera. Preparation stays separate
from flight.

All **operator interaction with the aircraft** happens in QGC. The browser owns
world/preset selection, prerequisites, image builds, Prepare Simulation,
observation, evidence, Stop Simulation, and Run Again. The onboard companion
continues executing the selected mission through ArduPilot; Gazebo owns physics
and sensors. The browser never becomes another flight controller.

The user's confirmation selects existing onboard missions, with one QGC Start
for the complete selected preset. Preserve the automatic competition sequence
and configured diagnostic plans. QGC Plan waypoint authoring and separate
operator starts for each competition phase are outside this revision.

Also fix the diagnosed conflict between ordinary competition landing/recovery
and strict precision landing. Preserve scoring, sensor truth, prearm checks,
calibration provenance, timing, and artifact validity. Simulator verification
does not qualify a physical aircraft.

## Operator workflow

1. Select a complete supported world/preset tuple in the browser. Show the
   mission, calibration dependencies, and configured warmup/flight times.
2. Prepare starts the runtime and opens or focuses QGC. Physics, telemetry,
   and private camera frames advance while mission outputs remain blocked.
   Show concrete readiness reasons and warmup progress.
3. Prepare creates a fresh named QGC action file for this run. Select it in
   QGC's Fly View action settings; the browser shows its location and run ID.
   Do not assume QGC reloads an overwritten file automatically.
4. Inspect the views, then invoke **Start selected mission** in QGC.
   Automatic presets retain onboard mode/arm/takeoff. For **Operator Wait**,
   invoke Start first, then select GUIDED and arm through native QGC controls
   during the mission's explicit waiting step.
5. Use QGC for flight commands. **Abort and recover** cancels mission progression
   and invokes guarded onboard recovery. Native mode, arm, and LAND retain their
   flight-controller meanings. Native LAND obeys the current precision policy;
   it is not an alias for companion recovery.
6. **Stop simulation** in the browser finalizes evidence and removes resources.
   It can interrupt an airborne vehicle and never implies landing. Run Again
   waits for cleanup and creates a fresh run and action file.

Remove browser Start Mission and Abort Flight controls. Replace them with
readiness, the QGC action to use, and observed mission/recovery progress.
Reject their old HTTP and console-CLI endpoints for QGC-controlled runs;
hiding buttons is insufficient. Preserve unattended CLI/suite execution.

Browser focus and JavaScript timer cadence must not gate QGC commands. Prepare
still establishes actual desktop availability and advancing camera production.
Vehicle, sensor, and native-clock readiness remain mandatory at admission.
Backgrounding, closing, or reconnecting the browser cannot alter aircraft
authority. Stale-view indicators remain truthful; viewer loss does not kill
healthy flight services. This replaces the former browser-RFB-heartbeat Start gate.

## Chosen architecture

Use one companion-owned QGC mission-control module, with adapters for the
existing DroneKit and pymavlink transports. Its interface accepts decoded
commands and current readiness/vehicle observations, advances admission and
recovery, and publishes command responses/status. It owns identity validation,
attempt identity, retry behavior, and durable decisions across mission hosts.
Existing mission implementations retain flight execution.

Reuse validated patterns from the imported QGCCommandListener, attempt ledger,
acknowledgement helpers, and guarded output transactions. Its legacy FM1/FM2/FM3
interface is phase-by-phase and must not be reinterpreted as full-sequence Start.
The current simulator actually invokes run_auto_attempt; correct the nested
README's stale claim that it already installs the guarded QGC listener.

The real command route remains QGC → host byte relay → SITL SERIAL1 →
ArduPilot MAVLink routing → companion SERIAL0, with acknowledgements returning
over the same route. The host relay preserves frames and records interventions;
it never synthesizes a flight request or acknowledgement. Bench-prove this
routing before flight. If necessary, correct FC routing or use an explicit
transparent MAVLink router, rather than intercept commands on the desktop.

Alternatives rejected:

- Host-side interpretation would test a desktop-only command path.
- Standard mission-start commands conflate companion missions with uploaded
  ArduPilot AUTO missions.
- Legacy FM1 as full-sequence Start silently changes an existing contract.
- Static token-free actions cannot reject an old action after Run Again.

## Command contract and run identity

Use protocol qgc_actions_v1 with project commands:

| ID | Meaning |
| --- | --- |
| 31020 | Start the complete selected onboard mission |
| 31021 | Abort that mission and request guarded recovery |

These IDs are unassigned in the installed pinned ardupilotmega inventory and
distinct from the imported 31000–31015 registry. Validate the reservation against
all shipped dialects/handlers and retain it as executable data. They are project
commands, not standard QGC or ArduPilot actions.

Accept MAVLink 2 COMMAND_LONG addressed to system 1/component 191. Parameter 1
is the current nonzero attempt token; parameters 2–7 are zero. The token must
be an exactly representable float32 integer, at most 16,777,215, allocated from
a durable monotonic ledger. Never wrap, reuse a consumed generation, or silently
reset a damaged ledger. Packet sequence and confirmation count do not create a
new operation.

Pin GCS, companion, and FC identities separately. Intended setup is QGC 255/190,
companion 1/191, and FC 1/1; verify installed QGC's actual wire identity before
accepting that profile. Both companion adapters advertise the configured
identity. The current raw-pymavlink source 255 and synthetic competition
self-source are unsuitable substitutes for real QGC traffic. FC caches accept
only FC 1/1; dispatch QGC commands separately from telemetry filtering.

Retain launch_mode=operator and its dynamic epoch. Add a frozen operator_control
descriptor containing protocol kind, command IDs, token, distinct Start/Abort
request UUIDs, identities, and action-file path/hash. Render and hash action
bytes before calculating the configuration checksum, avoiding a digest cycle.
Include the action file in configuration evidence. Configurations without the
descriptor retain their historical meaning and bytes; unattended behavior stays
unchanged.

Preserve existing QGC user files/settings. Publish a run-named action file in its
configured save location and retain the immutable copy in the evidence bundle.
Document the exact selection step. Automate reload only after proving it on
installed QGC 5.0.8; do not discard an existing session's unsaved state to reload.

## Admission and recovery

Start requires the matching run/token, fresh healthy disarmed ground state,
matching calibration, advancing native time, and healthy required runtime
producers. An early command gets an actionable QGC rejection; it neither queues
future flight nor consumes the token. Premature native QGC arming never implies
mission admission.

Reserve an eligible Start once and durably write its prepared UUID through
OperatorProtocol. Return IN_PROGRESS while admission is pending. Reuse existing
request-bound companion readiness and Gazebo admission: one native 50 ms epoch,
immutable configuration/request/admission digests, matching RUNNING, then public
clock and mission output. Preserve Comp2026's first-command deadline and the
configured runner's execution-release semantics.

An accepted Start acknowledgement means admission, not completed flight.
Publish bounded progress and final command responses with readable QGC status.
Record source/target, command/token, request/admission identity, timestamps,
ACK outcome, and observed mission outcome as companion-owned evidence.
Passive host logs corroborate that record.

Equivalent retries replay the same pending/admitted result without another
worker, epoch, or arm. Reject wrong identities, stale tokens, malformed
parameters, unsupported messages, mismatched durable state, and terminal runs.
Revalidate readiness before admission. Failure after durable reservation needs
a fresh prepared run. Restart never silently resumes flight.

Abort has a separate immutable request identity under the same run token.
Cancel normal output once, then invoke the selected host's guarded recovery.
Repeats cannot restart recovery. Report observed landed/disarmed success or
failure through QGC and the read-only browser. Finalization wins over pending
commands and prevents further output.

## Aircraft authority

One mission/recovery owner issues onboard flight and payload commands. Keep
guards at the actual enqueue interface. Across all offered hosts, fresh
unexpected FC mode/arming changes cancel mission intent and future phases,
rather than provoking a command that restores the mission's preferred mode.
This includes QGC LAND or Pause. Continue monitoring and never automatically
resume when GUIDED reappears.

Distinguish expected active-operation transitions and Operator Wait from
intervention. Preserve FC failsafes and independent RC takeover proof.
A mode alone does not identify a pilot: use external/unknown authority unless
the onboard path has the trusted initiating command and matching transition.
Never fabricate RC evidence from a QGC action. Test pending-command races;
sampled telemetry is not a zero-latency takeover guarantee.

## Ordinary versus precision landing

Run 256fdc05-9cc4-44ee-a8d2-f0234ead717d reached L, selected LAND, and held
near 7.67 m. DataFlash recorded PrecLand failsafe, no acquired target,
PLND_ENABLED=1, PLND_STRICT=2, and PLND_ALT_MAX=8. Later Abort preserved the
same LAND state. Time and sensors remained live. Evidence was retained and
the run finalized ABORTED with complete cleanup.

Add one ordinary-landing operation shared by FM1, ordinary home landing, and
guarded recovery. Under valid command authority:

1. Verify the approved profile from fresh FC parameter observations.
2. Set only PLND_ENABLED=0 through the guarded transport and require a new
   source-matched readback before entering or continuing LAND.
3. Require observed touchdown; restore PLND_ENABLED=1 with new readback before
   phase success or later companion flight. Existing disarm confirmation remains.

Keep PLND_STRICT=2 and the remaining precision settings unchanged. Record all
deliberate parameter transitions. Use generation-stamped observations; an old
DroneKit cache value cannot prove a write. Missing/mismatched readback blocks
progress. Restore failure blocks later phases even after a physical landing.

Already-LAND recovery must use this operation while it owns authority, rather
than merely wait on strict hover. Preserve observed descent; do not climb away
from a landing merely to return home. On timeout while authority remains, make
a bounded verified restoration attempt before reporting failure. On pilot/FC
takeover, send no further parameter or mode commands, including unconditional
restoration; report the unrestored profile and block companion reentry.

FM3 and configured precision landing retain real target acquisition and strict
target-loss behavior. Revalidate the complete profile before precision entry.
Do not fabricate targets or globally relax strictness. Pinned Copter's command
handler ignores NAV_LAND's apparent per-command precision opt-out, so it cannot
substitute for the verified parameter transition.

Prove live PLND_ENABLED switching/readback on the pinned image before accepting
this fix. If that proof fails, keep the affected path unavailable and revise
the design. Fresh runs retain fresh SITL storage and the original preflight
profile verification.

## Verification and completion

- Exercise both real command adapters: source/target checks, malformed fields,
  early and stale actions, duplicate/concurrent requests, admission races,
  restart, Abort/finalization precedence, and absence of unauthorized outputs.
  Preserve legacy evidence and unattended tests.
- Bench-prove actual FC routing, installed native QGC action selection and ACK
  behavior, and old-action rejection after Run Again. A script impersonating
  QGC does not establish native application usability.
- Browser tests cover removal of aircraft actions, rejection by their old
  endpoints, clear readiness/recovery guidance, Prepare/Stop/Run Again,
  actual Gazebo interaction and camera freshness, and QGC operation while the
  browser is backgrounded or closed.
- Verify native QGC mode changes across mission hosts, Operator Wait, RC
  takeover, failsafes, pending output, and no mission reentry.
- Reproduce strict targetless LAND hold in pinned SITL, then prove ordinary
  descent, touchdown/disarm, and profile restoration. Cover stale readback,
  failed writes/restoration, already-LAND recovery, and takeover during the
  transition. Separately verify precision target loss still holds/fails.
- With matching source/images, execute configured descent through genuine QGC
  Start to independently accepted results, then prepare a fresh repeat.
  Exercise competition FM1 landing and subsequent progression, plus a separate
  QGC Abort flight that confirms landing/disarm. Report lifecycle, physical
  outcome, score, artifact validity, and cleanup separately.
- State the extent and result of full-competition/full-catalog testing. A partial
  flight is not a full mission acceptance claim.

Update affected module guides, architecture, runbook, handoff, and the live
ledger with implementation. Rebuild matching images for physical verification.
Preserve the original checkout, existing run evidence, and PR #3 ancestry.

## Source anchors

- [Operator protocol](../../../artifacts/src/artifacts/operator_protocol.py),
  [session owner](../../../orchestration/src/orchestration/operator_session.py),
  [companion supervision](../../../companion/src/drone_sim_companion/operator_control.py).
- [Runtime composition](../../../companion/src/drone_sim_companion/runtime_node.py),
  [QGC listener](../../../companion/comp2026/src/drone/control/listener.py),
  [mission supervisor](../../../companion/comp2026/src/drone/control/mission_supervisor.py),
  [flight operations](../../../companion/comp2026/src/drone/control/drone_control.py).
- [QGC custom actions](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/custom_actions/custom_actions.html)
  and [MAVLink command protocol](https://mavlink.io/en/services/command.html).
- [ArduPilot precision landing](https://ardupilot.org/copter/docs/precision-landing-and-loiter.html),
  pinned [LAND dispatch](https://github.com/ArduPilot/ardupilot/blob/1511f27194f1dcc3728270883047bdf022b3fd53/ArduCopter/mode.cpp),
  [command handler](https://github.com/ArduPilot/ardupilot/blob/1511f27194f1dcc3728270883047bdf022b3fd53/ArduCopter/GCS_MAVLink_Copter.cpp),
  [precision parameters](https://github.com/ArduPilot/ardupilot/blob/1511f27194f1dcc3728270883047bdf022b3fd53/libraries/AC_PrecLand/AC_PrecLand.cpp).
