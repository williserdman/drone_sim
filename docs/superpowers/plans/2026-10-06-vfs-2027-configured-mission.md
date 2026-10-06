# VFS 2027 configured mission implementation plan

> **For agentic workers:** Execute task by task with `$implement` and TDD at the approved seams. Independent module tasks may run in parallel; perform the requested whole-change code review at the end. The user authorized execution on the current branch.

**Goal:** Fly and record the full configurable three-payload 2027 mission, default C, using the configured tool runner and automatic physical checks.

**Architecture:** Share frozen configuration and wire value contracts through the existing artifacts dependency. Extend the configured runner, keep one flight-output owner, and use a separate judge/scorer fed by ordered physical truth. Preserve the existing runtime lifecycle and evidence checks.

**Tech stack:** Python 3.12, ROS 2 Jazzy messages, Gazebo Harmonic, ArduPilot/PyMAVLink, OpenCV ArUco, Docker Compose, pytest.

**Spec:** [Approved design](../specs/2026-10-06-vfs-2027-configured-mission-design.md), approved for implementation on 2026-10-06.

## Global constraints

- Current branch, baseline `967aa87`; preserve other work and existing run evidence.
- `mission: configured`, scenario `competition_2027_v1`, world `competition_2027`, vehicle `iris_competition_2027`; template `config/configured-vfs-2027-run.json`.
- IDs 2/3/4, `DICT_4X4_250`, 100 mm markers; capacity one; C default with per-payload A/B/C overrides.
- 20 Hz public truth/camera; 600 active seconds; 780 public recording seconds; 90 native warmup seconds; inspection 30 seconds, approval-to-takeoff 60 seconds, recovery 60 seconds.
- Preserve version-1 plans and old scenario contracts. Judge truth never supplies exact positions to flight navigation. Report physical result, score, and artifact validity separately.

## Review focus

- Empty, stale, or wrong-run judge decisions must fail rather than silently skip recovery; test in task 2.
- Loaded touchdown may occur through an attached grounded payload; test in task 3.
- A valid command response does not establish pickup, landing, or scored placement; test in tasks 3-5.
- A marker observed before an operation cannot refresh its precision descent; test in task 4.
- A configured target override must affect world-independent navigation and score validation, with the exact resolved configuration retained; test in tasks 1 and 5.

## Task 1: Frozen configuration and tool-plan envelopes

**Files:** Create `artifacts/src/artifacts/vfs_contract.py`, `artifacts/tests/test_vfs_contract.py`, `config/course-2027.yaml`, `config/scenario-2027.yaml`. Modify `artifacts/pyproject.toml`, `uv.lock`, `orchestration/src/orchestration/config.py`, `config/run-template.schema.json`, `config/run.schema.json`, `orchestration/tests/test_config.py`, `orchestration/README.md`.

**Interfaces:** `VfsWaypoint(name, x, y, role, width=None, height=None, radius=None)` exposes `center_xy_m` and `contains(xy, margin=0)`. `VfsCourse.waypoints` maps names to waypoints and `.attempt` holds timing/heights. `VfsScenario` has typed `.camera`, `.range_sensor`, `.payload_interaction`, `.payload_geometry`, `.payloads`, `.targets`, and `payload(aruco_id)`. `VfsPayload` exposes `aruco_id`, `color`, `initial`, `delivery`, `release_offset_xy_m`; geometry exposes `size_m`, `water_mass_kg`, `tare_mass_kg`, `marker_size_m`, `total_mass_kg`. Functions: `load_vfs_course(path)`, `load_vfs_scenario(path)`, `resolve_waypoint_xy(course, scenario, *, waypoint=None, payload_target=None) -> tuple[float,float]`. Data is immutable; loaders accept the new profile only. Share schemas/data, not scoring or actuator logic.

- [ ] Write `test_vfs_target_override_changes_resolved_aimpoint`: default ID 2 resolves to C plus `[0,.28]`; changing only target to A resolves to A plus the same offset. Assert literal coordinates `(-86.106,-2.768)` and `(-86.106,3.328)`.
- [ ] Run `uv run --locked pytest artifacts/tests/test_vfs_contract.py -q`; observe failure because the new loader/resolver is absent.
- [ ] Implement typed loaders, exact default geometry/masses from the spec, target validation, and safe YAML dependencies; verify the test passes. Add one validation slice at a time for IDs, geometry, and immutable values.
- [ ] Add a version-2 config resolution test with a conditional group and the new source files; watch old version/profile validation fail. Implement envelope normalization, checksum/copy verification, 640×480 new-scenario recording, and required scenario bindings while preserving old validators.
- [ ] Run `uv run --locked pytest artifacts/tests/test_vfs_contract.py orchestration/tests/test_config.py -q` and typecheck the new contract. Update orchestration guide and commit this deliverable.

## Task 2: Validated conditional configured plans

**Files:** Modify `companion/src/drone_sim_companion/mission_plan.py`, `companion/src/drone_sim_companion/configured_mission.py`, `companion/tests/test_configured_mission.py`; create `companion/tests/test_vfs_plan.py`.

**Interfaces:** Frozen `Condition(kind, payload_id=None)`, `ConditionalGroup(when, steps)`, `PlanItem = MissionStep | ConditionalGroup`, and `MissionPlan(steps, schema_version=1)` retain existing callers. `ConfiguredMission(..., conditions=None, emit=None)` evaluates only frozen FM-2 eligibility through `conditions.evaluate(condition)`. Register the seven new tool argument shapes from the approved design. Existing `operations.start/status/tick` remains the runner interface.

- [ ] Write `test_v2_skips_ineligible_retrieval_without_commands`; a false payload predicate emits a skip and advances past all child operations. Run `uv run --locked pytest companion/tests/test_vfs_plan.py -q` and observe unsupported schema version failure.
- [ ] Implement version-2 parsing and a bounded cursor stack. Require group shape `{when,steps}`, eager call validation, maximum outer group plus one inner group, and known predicates/IDs. Preserve version-1 behavior.
- [ ] Add focused slices for selective eligibility, missing/foreign/stale decisions, invalid tools in skipped branches, mixed/empty/deep groups, and failure stopping subsequent commands.
- [ ] Run `uv run --locked pytest companion/tests/test_vfs_plan.py companion/tests/test_configured_mission.py -q`; typecheck changed plan/runner files. Root incorporates the contracts into the companion guide and commits with task 4.

## Task 3: Generated world and physical payload policy

**Files:** Modify `gazebo/scripts/prepare_competition_assets.py`, competition world selection in `gazebo/src/drone_sim_gazebo/worlds/api.py`, `server/process.py`, `runtime/{paths,children,entrypoint,runtime_node}.py`, `ros_adapter/{topics,node,aggregation}.py`; add/generated `gazebo/resources/{worlds,models}` 2027 resources. Modify `electromagnet/src/drone_sim_electromagnet/{payload,runtime_node,controller}.py`, focused asset/payload/aggregation tests, and both module READMEs.

**Interfaces:** Consume task-1 frozen config. Keep existing private payload/model/topic identities, but use distinct 2027 resource directories. New payload authority accepts each ID in S or D and preserves capacity/grounded/alignment checks. Public vehicle ground contact includes an attached grounded payload from the same epoch.

- [ ] Write an asset test asserting new profile world/model identities, no painted-zone collision walls, all initial joints detached, and payload mass `1.46077711`. Run the focused file and observe missing profile/resources failure.
- [ ] Add a 2027 generation profile, derive matching inertia and hardpoint clearance, generate assets, then pass the asset slice. Add selector/config slices for the new pair while preserving old pair tests.
- [ ] Write a payload-policy slice for ID 2 attach in S, confirmed detach, and subsequent attach in D. Watch current immutable pickup assignment reject it; implement allowed-region sets for the new profile.
- [ ] Write support aggregation slices for attached-grounded support and rejection of detached-grounded/attached-suspended support; implement epoch-aligned support without fabricating missing truth.
- [ ] Run focused Gazebo and electromagnet suites, typecheck new pure modules, update guides, and send evidence to root for commit. Do not rebuild shared image tags while the existing flight is active.

## Task 4: Configured flight, vision, payload, and judge operations

**Files:** Create `companion/src/drone_sim_companion/{configured_vision,precision_landing,vfs_operations}.py` where separation is needed; modify `operations.py`, `mission.py`, `mavlink_adapter.py`, `configured_runtime.py`, `runtime_node.py`. Add focused tests for operations, vision/precision, and MAVLink; update `companion/README.md` and Docker context/import closure if needed.

**Interfaces:** Consume tasks 1-2 and task-5 wire contracts. `DroneOperations` remains the sole flight owner and exposes `start`, `operation_status`, `tick`, `observe`, `abort`. Vision produces timestamped requested-ID observations only. Payload clients produce correlated confirmations only. Judge exchange produces typed decisions only. Extend telemetry for horizontal velocity, attitude, pinned home, precision parameters, and observed flight state.

- [ ] Test `goto_course_waypoint` through real operations and a fake transport: completing location alone is insufficient; speed under `0.10 m/s` for `1.0 s` is required. Watch unsupported tool fail, add symbolic resolution and dwell.
- [ ] Test stationary acquisition of five new consistent marker frames; repeated/stale/wrong-ID frames must not establish a target. Reuse existing camera calibration/transforms and requested ArUco ID detection; vision workers send no commands.
- [ ] Test precision health loss, GUIDED hold/reacquire, near-ground visibility loss, live-profile mismatch, touchdown/disarm, timeout, and abort one slice at a time. Adapt the existing policy to ticks: 0.50 s health, 5 s hold, 0.20 s refresh, five frames, 0.20 m anchor, 0.75 m floor, one search-hover retry, 60 s bound.
- [ ] Test physical attach/release correlation and airborne/grounded prerequisites; implement service adapters and actual completion. Test phase/checkpoint/request correlation and publish observed arming/mode/failsafe evidence independently from intent.
- [ ] Test active-attempt expiry cancels payload work and transfers the same owner to bounded home recovery; mode changes/stale state suppress forced commands. Run focused companion suites, typecheck new pure modules, update guide, and commit with task 2.

## Task 5: Automatic judge, scorer, and recorded acceptance

**Files:** Create `scorekeeper/src/drone_sim_scorekeeper/vfs.py`, `scorekeeper/rules/competition_2027_v1.json`, focused judge/scorer tests; modify scorekeeper runtime and guide. Add `ros_ws/src/simulation_interfaces/msg/{JudgeDecision,VehicleStatus}.msg` and CMake registration. Add independent 2027 evidence validation under `artifacts/src/artifacts`; modify runtime selection, bag topics/QoS/validation, score validation, acceptance inspection, focused artifacts tests, guide, and orchestration terminal scenario classification if needed.

**Interfaces:** `MissionEvent` with `state=CHECK_REQUESTED` carries JSON `request_id/checkpoint`; beginning phases uses ordered `STARTED`. Judge decisions are run/request/phase/checkpoint correlated and include approval, eligible IDs, timing facts, and evidence reference. Observed `VehicleStatus` contains armed/mode/failsafe and source timestamp. Pure `VfsJudge`/`VfsScorer` consume vehicle/payload samples plus observed status/events, and return decisions/results without flight output. Root freezes exact wire fields before implementation.

- [ ] Test settled circle-edge contact qualifies while suspended overhang does not; test FM1 D approval before return, FM2 no-success and partial eligibility, stale evidence refusal. Implement support geometry and automatic decisions.
- [ ] Test clock start at first physical takeoff, pause after FM2 home/disarm, resume on FM3 takeoff, and the 60 s approval limit. Implement an ordered timeline that the companion can consume for deadlines.
- [ ] Test a full autonomous C trace against independently worked totals: `365 + T` at multiplier 1.5; flight maximum 380, GM-1 absent. Add invalid low/airborne release, failsafe/contact/late-home penalties, and negative-total clamp slices. Implement versioned rules and persisted components/evidence.
- [ ] Test recording contract selects payload/judge/status streams for the new scenario and rejects missing/gapped/foreign evidence. Implement new-profile bag/manifest/score validation without weakening existing checks. Independently replay authoritative bag data for acceptance.
- [ ] Run focused scorekeeper/artifacts suites, typecheck new pure modules, update guides, and send evidence to root for commit.

## Task 6: End-to-end template, documentation, review, and flight

**Files:** Add `config/configured-vfs-2027-run.json` with explicit tool steps and eligibility groups. Update `README.md`, `docs/{architecture,runbook,handoff}.md`, build copies where required, and plan ledger. Preserve the approved spec in Git history when its implemented contracts replace it in current guides.

- [ ] Resolve and schema-check the full template and A/B target overrides; run focused runtime composition/contract tests. Verify all ordinary templates still resolve.
- [ ] Run typechecking, then the full host/nested test suite once; record pass/fail/skip counts and named failures. Run Compose configuration checks; commit source/docs with clean diff checks.
- [ ] Use the requested `code-review` skill from baseline `967aa87` against committed implementation. Run Standards and Spec agents in parallel; fix material findings with a focused failing regression first and report both axes.
- [ ] After the existing run ends, rebuild affected runtime images with the current committed revision. Run the new template, monitor status/logs, preserve every failed attempt, and diagnose failures before changing anything. After three failed fixes, stop and identify the doubtful assumption.
- [ ] Inspect actual three C drops, three landed L deliveries, home/disarm, score arithmetic, both videos, timestamp continuity, manifest, and image/source provenance. Record dated handoff evidence and commit. Mark the goal complete only when the full outcome passes.

## Execution notes

The spec review and `$implement` instruction authorize this plan's execution.
No per-task approval or review cycles are added. Plan progress, interface rulings,
commands, and evidence live in `.superpowers/sdd/2026-10-06-vfs-2027-configured-mission/progress.md`.
Project dependencies may change; machine configuration and unrelated running
services are outside this task. Keep source and image identity stable during flight.
