# VFS 2027 configured mission design

Date: 2026-10-06. Status: conversational design approved; written spec awaiting
review. This describes intended behavior, not an implemented or verified mission.

## Objective and agreed scope

Run the VFS 2026-2027 flight mission through the existing `mission: configured`
tool runner. The default attempt handles all three payloads, targets C, retrieves
successful drops, delivers them while landed, and returns home. Each payload's
destination is configurable as A, B, or C. Judge checks are automatic.

Reuse the existing ArUco setup: dictionary `DICT_4X4_250`, payload IDs 2/3/4,
100 mm markers, downward camera, and guarded precision-landing behavior. Flight
decisions use camera/range observations and ArduPilot telemetry. Gazebo truth
belongs to physical validation and judging, not payload navigation.

Completion requires a fresh physical flight with all three default deliveries
and recoveries, an independently calculated score, and a valid recorded bundle.
Existing competition and diagnostic templates remain usable.

## Source and simulator assumptions

The governing source is the [VFS RFP version 1](https://vtol.org/files/dmfile/v1_vfs_dbvf_rfp_2026-2027.pdf),
dated 2026-09-21. Course geometry is on page 22, flight requirements on pages
23-29, scoring on pages 32-35, and autonomy on page 37. Do not import older
competition rules under the new scenario name.

The flight contract is H → D → H, supply drops from S into A/B/C, home inspection,
then recovery of eligible drops into L and return to H. Transit and supply
release require at least 30 ft AGL. Recovery delivery requires ground contact.
The active attempt lasts 600 seconds, excluding inspection; approved FM-3
takeoff has a separate 60-second limit. Payload containment uses final ground
contact, including the boundary. Payloads carry at least 3 lb of water plus tare.

The MVP uses a flat course and rigid, watertight container proxies. Terrain,
gusts, liquid motion, leakage, and breakage physics are deferred. The scenario's
canyon narrative does not add an obstacle course. ArUco markers are the agreed
simulation aid. GM-1 and human field conduct are outside the simulated flight.

## Existing implementation and chosen approach

The existing [plan schema](../../../companion/src/drone_sim_companion/mission_plan.py),
[runner](../../../companion/src/drone_sim_companion/configured_mission.py),
[operations](../../../companion/src/drone_sim_companion/operations.py), and
[runtime](../../../companion/src/drone_sim_companion/configured_runtime.py)
already validate and execute ordered tools. Each operation waits for observed
completion, has a simulation-time deadline, and owns flight output exclusively.

Extend those interfaces with reusable operations and limited conditional groups.
Do not introduce a separate `comp2027_auto` entry point. A whole-phase tool would
make the JSON shorter but hide the mission sequence in Python. A general
workflow language would add machinery the mission does not need.

Reuse camera processing and precision-landing policy from the imported mission
through the configured operation interface. Do not launch its automatic mission
worker alongside the configured runner. Preserve its original callers and tests.

## Configuration contract

Add `config/configured-vfs-2027-run.json`, selecting `mission: configured`, a
new `competition_2027_v1` scenario, world `competition_2027`, and vehicle
`iris_competition_2027`.
Keep course geometry, physical inventory, and mission steps separate using the
existing resolved, checksum-bound configuration mechanism.

The new template uses `mission_plan.schema_version: 2`. Version 1 retains its
current flat-step behavior. Version 2 accepts ordinary `tool`/`args` steps and
conditional groups with `when` and nonempty `steps`. The only predicates are:

| Predicate | Meaning |
| --- | --- |
| `{"any_payload_eligible": true}` | Enter FM-3 only when inspection approved at least one payload. |
| `{"payload_eligible": 2}` | Execute this complete retrieval sequence only for the named eligible payload. |

Permit the outer FM-3 group and one level of per-payload groups. A group cannot
contain both a tool call and child steps. No loops, expressions, variables, or
arbitrary code are added. All branches validate before resources open. An absent,
stale, or wrong-run inspection decision is an error, not a false predicate.
Skipped groups record their reason and emit no flight or payload commands.

The scenario declares one delivery target per payload, defaulting to
`{"2": "C", "3": "C", "4": "C"}`. Named navigation accepts a `payload_target`
reference so changing this map changes both drop navigation and retrieval search
without editing duplicated coordinates. Explicit waypoint references still
support H, D, L, and each payload's initial location in S. Resolution occurs
before flight, and the resolved plan and geometry are retained with the run.

## Added tools

Existing mode, arm, takeoff, waypoint, hold, and land tools retain their contracts.
New tools are registered in the same exported vocabulary and argument validation.

| Tool | Arguments and completion |
| --- | --- |
| `goto_course_waypoint` | Exactly one of `waypoint` or `payload_target`, plus `altitude_m` and optional tolerance. Resolve against the frozen course and pinned home; observe arrival and horizontal speed below 0.10 m/s for one second before completion. |
| `find_marker` | `aruco_id`. Observe the requested marker from the configured acquisition hover; finish only with a fresh, consistent camera-derived target. Timeout is a failure. |
| `precision_land` | `aruco_id`. Use that marker's observations and the guarded landing policy; finish after confirmed touchdown and disarm. A previous acquisition is not permission to use stale frames. |
| `attach_payload` | `aruco_id`. Require a landed, aligned aircraft and available payload; finish after physical attachment confirmation. |
| `release_payload` | `aruco_id`, `delivery_mode: airborne` or `grounded`. Check the mode's physical prerequisites, then wait for confirmed detachment. |
| `begin_phase` | `phase: FM1`, `FM2`, or `FM3`. Record ordered mission intent. This does not award points or establish physical success. |
| `check_phase` | `phase: FM1`, `FM2`, or `FM3`; FM1 also requires `checkpoint: D` or `H`. Await the correlated automatic judge decision. D approval permits return flight; H approval completes FM-1. |

`find_marker` is bounded stationary acquisition in this MVP. Configured flight
steps position the aircraft above the appropriate source or destination, descend
vertically to acquisition height, then invoke it. A missing or obscured marker
fails the operation; a course-wide search planner is deferred. Precision landing
retains its existing bounded hold/reacquire behavior and near-ground handoff.
The default uses 10 m transit/release and 4.572 m acquisition heights. Stop above
the target before descending. Larger horizontal repositioning requires climbing
back to transit height; local precision-landing corrections remain permitted.

Tool names do not hide arming or takeoff. The config explicitly returns to GUIDED,
arms, and takes off after a landing. Payload tools do not fly the vehicle.
Each operation remains responsive to clocks, aborts, telemetry, and finalization.
Sensor processing may use workers, but only the active operation issues flight
commands through the runtime's MAVLink adapter.

## Default sequence and failure behavior

| Stage | Configured sequence |
| --- | --- |
| FM-1 | Begin phase; take off from H; transit to D; land; check D; take off; return to H; land; check H. |
| FM-2 | Begin phase; for each explicitly listed ID 2/3/4, fly to its source, acquire marker, precision-land, attach, climb, transit to its selected target, and release airborne. Return H, land/disarm, and check phase. |
| Inspection | Judge waits for settled payload evidence, freezes eligibility, and publishes approval. With no eligible drops, finish the attempt at H with an honest partial result. |
| FM-3 | Inside the eligibility groups, reacquire each eligible marker near its configured target, land/attach, climb, transit to a separate placement location in L, land, and release grounded. Return H, land/disarm, and check phase. |

Use distinct configured release offsets within a shared target circle and distinct
delivery positions in L to reduce payload overlap. Offsets must fit the selected
target and the modeled footprint; they do not guarantee the actual drop result.
Retrieval always targets the observed marker, never the commanded drop point.

An operational failure stops later configured steps and remains a failed run even
if recovery lands successfully. Ordinary missed drops are scoring outcomes;
inspection can approve the remaining payloads. A lawful no-eligible-payload exit
is a completed partial attempt, but does not satisfy full-mission acceptance.

Attempt expiry cancels payload work and starts a bounded return-to-H recovery
using the same flight-output owner. Stale telemetry or an observed operator mode
change prevents forcing that recovery. A failsafe touchdown ends the attempt.
Recovery cannot erase a timeout, failure, penalty, or missing evidence.

## Automatic judge and clocks

Add a read-only judge evaluator to the scorekeeper runtime for the new profile.
It consumes ordered public vehicle/payload truth and phase requests. It publishes
typed, run-scoped judge decisions containing phase/checkpoint, request/event correlation,
decision timestamp, outcome, eligible IDs where applicable, and evidence
references. It never issues flight or payload commands. The companion consumes
decisions through `check_phase`; it does not consume exact payload truth poses.

For FM-1, verify each landing before its checkpoint approval, required flight
transitions, and the full aircraft footprint. For FM-2, require home
landing/disarm and settled payloads before
freezing eligibility. For FM-3, judge the final placements and return home.
Snapshot eligibility separately from point adjustments: an invalid low release
can lose drop points without making a physically contained payload disappear.

The companion publishes observed arming, mode, and failsafe evidence separately
from requested actions. The judge and scorer combine these telemetry facts with
physical truth. A `begin_phase` or successful command acknowledgment cannot prove
disarm, continuous autonomy, or absence of a failsafe. Record these facts in the
bag with timestamps and reject missing evidence needed for a verdict.

The judge and scorer reconstruct active time from physical takeoff and phase
evidence. Arming alone does not start it. Inspection stops active-time accounting;
FM-3 takeoff resumes it. The companion receives the same attempt timing facts for
deadline enforcement. Duplicate requests replay the same decision. Evidence gaps
or contradictory ordering fail judging rather than approving a phase.

Gazebo time, sensor publication, per-operation timeouts, and recording continue
during inspection. The default public recording window is 780 simulated seconds,
with the existing separate 90-second native warmup. Budget 30 public seconds for
initial readiness/takeoff, 30 for inspection, 60 for approval-to-takeoff, and 60
for terminal recovery, alongside the 600-second active limit. These are simulator
bounds; the flight must normally finish before the active limit. Exhausting any
bound preserves its failure evidence. Finalization remains the existing host
lifecycle, not a clock reset or a fabricated mission success.

## Physical world and payload policy

Generate a new flat S/D/L course with circular H/A/B/C markings. Use the RFP
diagram's dimensions where specified and record simulator choices for unspecified
positions in the course file. H is the ENU origin inside S; A/B/C are inside D.
Paint has no walls or raised collision boxes. The old F2 container walls must not
appear around these targets.

Default ENU layout, in metres, is a simulator interpretation of the schematic:

| Location | Center x, y | Geometry |
| --- | --- | --- |
| S / H | 0, 0 | S rectangle 15.24 × 19.812; H circle radius 2.286. |
| L | -30.48, 0 | Rectangle 15.24 × 19.812. |
| D | -91.44, 0 | Rectangle 15.24 × 19.812. |
| A / B / C | x=-86.106; y=3.048 / 0 / -3.048 | Circle radii 1.0668 / 0.762 / 0.4572. |

Initial payload locations are x=-3 with y=-3/0/3 for IDs 2/3/4. Their L delivery
locations use x=-30.48 and the same y values. Default release offsets from each
selected circle center are `[0, 0.28]`, `[-0.242, -0.14]`, and `[0.242, -0.14]`
respectively. `payload_target` resolves the selected center plus its offset;
camera acquisition must then locate the actual payload. These choices are frozen
in course/scenario configuration, not claimed as surveyed event coordinates.

All three payloads begin detached in S outside H. Preserve their IDs and marker
size. Model each as a 0.1524 m square container, 0.0762 m high, with 1.36077711 kg
water and 0.10 kg tare. These are simulator choices, not a container design
qualification. Generate matching inertia and adapt the pickup hardpoint/clearance
to the geometry. Keep contact, attachment, and vehicle lift physically simulated.

The vehicle carries one payload. The new electromagnet policy permits physically
valid pickups in S and later D, replacing the old immutable WA/WM assignments
only for this profile. The normal release tool enforces its selected delivery
mode; the independent scorer still detects invalid or unintended releases.
Ground contact includes support through an attached, grounded payload. Extend
the physical contact aggregation where necessary; the existing leg-only sensor
cannot alone decide whether a loaded landing permits release. Flight-controller
landing detection does not substitute for this physical evidence.

Judging derives a grounded support footprint from the known rigid-box geometry,
pose, and flat ground. Use the contacting face, edge, or point, not the payload
center or its full overhead projection. Boundary contact counts; suspended
overhang does not. Aircraft landing bounds include props and attachments using
a conservative envelope tied to the generated vehicle dimensions.

Failed drops are excluded by the frozen eligibility decision. Their physical
removal during inspection is deferred for the MVP; they remain collision objects
and visible markers, so this simplification must be recorded in the scenario and
run report. The default acceptance run requires three successful drops and does
not rely on this simplification.

## Scoring and evidence

Add a separate versioned rules file and scorer. The RFP flight formula is
`A + R * (BP + CP) + BA + CA + T`, before deductions, with a maximum of 380.
GM-1 contributes a separate 20 to the published 400 maximum; it is not earned or
silently assumed by this sim. Use the page 32-35 tables for the exact point,
weight, time, and deduction values rather than copying the legacy 150-point rules.

Persist every component, multiplier, adjustment, active-time interval, judge
decision, and supporting evidence reference. Physical placements, eligibility,
autonomy continuity, deadline violations, and failsafe landings remain separately
inspectable. Score invalid releases correctly even when testing them through a
synthetic trace instead of a normal guarded tool. Leakage/shatter and human
conduct deductions remain explicitly unmodeled, not certified absent.

Compute unloaded weight from generated airframe/attachments plus the heaviest
container tare, excluding water. Interpret adjacent weight brackets as continuous
half-open intervals at 10/12/14/16/18/20 lb. Preserve a nonnegative flight total.
All-C delivery and recovery need not earn the maximum time bonus to pass the
physical MVP.

Record the judge stream alongside mission events, payload events, physical truth,
and both videos. Extend scenario selection, manifest/ruleset validation, bag
validation, and acceptance inspection for the new profile. A valid partial score
is distinct from all-payload acceptance. Never relax clock continuity, timestamps,
artifact completeness, or source/image provenance to get a passing result.

## Changes and documentation ownership

| Module | Required change and guide |
| --- | --- |
| Companion | Extend schema, runner groups, operations, sensors and judge input; update [guide](../../../companion/README.md). Keep the executable tool contract beside the configured entry point. |
| Orchestration | Resolve the new scenario and version-2 plan, validate targets and referenced resources, preserve hashes; update [guide](../../../orchestration/README.md). |
| Gazebo and electromagnet | Generate aligned resources, expose physical facts, support repeated pickups and new inventory; update [Gazebo](../../../gazebo/README.md) and [electromagnet](../../../electromagnet/README.md) guides. |
| Scorekeeper and artifacts | Add judge decisions, rules, scoring, recording and acceptance; update [scorekeeper](../../../scorekeeper/README.md) and [artifacts](../../../artifacts/README.md) guides and required ROS schemas/QoS. |
| Shared guides and SITL | Update [architecture](../../architecture.md) for judge ownership/timing and [runbook](../../runbook.md) for build/run/inspection commands. Update [handoff](../../handoff.md) only with dated evidence. Check the [SITL guide](../../../ardupilot_sitl/README.md) if flight parameters or physical integration requirements change. |

This spec remains the pending design until implementation folds the relevant
contracts into those current guides and executable definitions. Remove the
superseded spec from active documentation then, preserving it in Git history.
No machine configuration changes are part of this design.

## Focused acceptance

1. Schema/runner tests prove version-1 compatibility, configurable target
   resolution, eager validation of guarded branches, truthful skip records, and
   failure/abort stopping later operations.
2. Tool tests prove fresh marker acquisition, guarded precision landing, confirmed
   attach/release, one command owner, and observed completion. Judge/scorer traces
   cover boundary containment, partial eligibility, clock pause/resume, approval
   timeout, invalid releases, and component/deduction arithmetic.
3. Asset checks prove initial unloaded state, mass/inertia, marker identity,
   painted target geometry, and agreement between resolved config and generated
   world. Check both an A/B target override and the default C selection without
   requiring separate full flights.
4. Rebuild affected images with matching source provenance. Run the configured
   default through all three C drops and recoveries. Demonstrate every pickup,
   landed delivery at L, and final home landing/disarm within the active budget.
5. Independently inspect the score and complete bundle, including onboard and
   observer videos, public-time continuity, judge evidence, and matching image
   provenance. Report physical outcome, score, and artifact validity separately.

Deferred work is limited to liquid/breakage physics, physical removal of failed
drops, wider marker search, manual judge/operator paths, and score optimization.
These limits must remain visible in the new profile's documentation and results.
