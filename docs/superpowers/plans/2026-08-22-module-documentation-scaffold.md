# Module Documentation Scaffold Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create a recursive, documentation-first module scaffold for the five initial drone-simulation subsystems and the repository root.

**Architecture:** Every module exposes one local seam in `INTERNAL_INTERFACE.md`, one cross-process seam in `EXTERNAL_INTERFACE.md`, and records its implementation sequence in `PLAN.md`. The root documents orchestration and relationships; child directories own subsystem-specific responsibilities without adding executable code or concrete ROS schemas.

**Tech Stack:** Markdown, ROS 2 Jazzy concepts, Ubuntu 24.04 LTS, MAVLink, ArduPilot-Gazebo adapter, Docker Compose concepts, POSIX shell verification

**Spec:** `docs/superpowers/specs/2026-08-22-module-interfaces-design.md`

## Global Constraints

- Target ROS 2 Jazzy on Ubuntu 24.04 LTS.
- Gazebo is authoritative for physics, sensors, camera output, and ground truth.
- Simulation time is authoritative for simulated behavior; wall time is restricted to infrastructure health and performance diagnostics.
- Camera output represents 20 frames per simulated second.
- ArduPilot and Gazebo operate in lockstep.
- Companion flight commands pass through ArduPilot and never manipulate Gazebo directly.
- The electromagnet module changes aircraft behavior through Gazebo physics.
- The scorekeeper is read-only with respect to aircraft control and physics.
- Cross-process communication uses documented ROS 2, MAVLink, or ArduPilot-Gazebo adapter seams.
- Same-process sibling imports use only the owning module's documented internal interface.
- This scaffold contains documentation only: no executable code, ROS packages, Dockerfiles, Compose files, or concrete ROS message definitions.
- Deferred details must be labeled `Deferred decision:` followed by the exact unresolved choice; do not use placeholder markers.

---

### Task 1: Root Orchestration Contract

**Files:**
- Create: `PLAN.md`
- Create: `INTERNAL_INTERFACE.md`
- Create: `EXTERNAL_INTERFACE.md`

**Interfaces:**
- Consumes: the approved design specification and the five child module names
- Produces: the repository-level lifecycle contract and the rules used by every child module

- [ ] **Step 1: Write the root plan**

Create `PLAN.md` with these sections and content:

```markdown
# Drone Simulation Plan

## Responsibility
Coordinate the lifecycle and configuration of deterministic simulation runs containing the companion, ArduPilot SITL, Gazebo, electromagnet, and scorekeeper modules.

## Non-responsibilities
- Flight-control decisions
- Physics or sensor simulation
- Mission decisions
- Scenario-effect implementation
- Score calculation

## Child modules
[A table listing each child and the responsibility assigned in the design spec.]

## Orchestration stages
1. Validate configuration and assign a unique run ID.
2. Start ROS 2 discovery and module processes through Docker Compose.
3. Wait for endpoint readiness rather than container-start status alone.
4. Start or reset Gazebo and publish simulation time.
5. Permit the run only after required modules report readiness.
6. Collect logs, diagnostics, scores, and run metadata.
7. Stop modules cleanly and preserve results.

## Acceptance criteria
[Repeat the eight verification conditions from the design spec in testable language.]

## Deferred decisions
[List the exact reset protocol, readiness protocol, DDS implementation, Compose topology, and result persistence format as deferred decisions.]
```

- [ ] **Step 2: Write the root internal interface**

Create `INTERNAL_INTERFACE.md` with sections `Purpose`, `Sibling modules`, `Allowed dependencies`, `Forbidden dependencies`, `Run identity`, and `Timing rules`. Include the complete top-level communication map from the spec. State that direct imports are permitted only for same-process siblings and only through a documented internal interface; the initial top-level modules are expected to be separate processes and therefore use their external interfaces.

- [ ] **Step 3: Write the root external interface**

Create `EXTERNAL_INTERFACE.md` with sections `Operator surface`, `Configuration inputs`, `Lifecycle operations`, `Observable outputs`, `Health semantics`, `Clock semantics`, `Failure behavior`, and `Deferred decisions`. Specify conceptual operations `start`, `reset`, `status`, `collect-results`, and `stop` without choosing a CLI syntax. Require `run_id` on run-scoped inputs and outputs, readiness checks for real endpoints, and wall-clock timeouts that never advance simulation state.

- [ ] **Step 4: Verify the root contract**

Run:

```bash
test -f PLAN.md
test -f INTERNAL_INTERFACE.md
test -f EXTERNAL_INTERFACE.md
rg -n "simulation time|Simulation time" PLAN.md INTERNAL_INTERFACE.md EXTERNAL_INTERFACE.md
rg -n "run_id|run ID" PLAN.md INTERNAL_INTERFACE.md EXTERNAL_INTERFACE.md
```

Expected: every command exits zero and the searches show explicit clock and run-identity rules.

- [ ] **Step 5: Commit the root contract**

```bash
git add PLAN.md INTERNAL_INTERFACE.md EXTERNAL_INTERFACE.md
git commit -m "docs: define root simulation orchestration"
```

### Task 2: Flight Stack Module Contracts

**Files:**
- Create: `companion/PLAN.md`
- Create: `companion/INTERNAL_INTERFACE.md`
- Create: `companion/EXTERNAL_INTERFACE.md`
- Create: `ardupilot_sitl/PLAN.md`
- Create: `ardupilot_sitl/INTERNAL_INTERFACE.md`
- Create: `ardupilot_sitl/EXTERNAL_INTERFACE.md`
- Create: `gazebo/PLAN.md`
- Create: `gazebo/INTERNAL_INTERFACE.md`
- Create: `gazebo/EXTERNAL_INTERFACE.md`

**Interfaces:**
- Consumes: root lifecycle rules; MAVLink; ROS 2 `/clock` and image transport; ArduPilot-Gazebo adapter
- Produces: documented seams for mission commands, telemetry, actuator exchange, sensors, camera data, physics effects, and ground truth

- [ ] **Step 1: Create the three module directories**

```bash
mkdir -p companion ardupilot_sitl gazebo
```

- [ ] **Step 2: Write the companion documents**

Create `companion/PLAN.md` with `Responsibility`, `Non-responsibilities`, `Inputs`, `Outputs`, `Implementation stages`, `Acceptance criteria`, and `Deferred decisions`. Its stages are: receive simulation-timestamped camera frames; perform frame-triggered vision; make mission decisions; issue MAVLink commands with causal frame metadata where the adapter permits; consume acknowledgements and telemetry; expose diagnostics.

Create `companion/INTERNAL_INTERFACE.md`. State that no same-process sibling interface exists in the initial scaffold. Reserve future language-native seams for mission, vision, and vehicle-adapter submodules, and require those submodules to preserve `run_id`, frame identity, and simulation timestamps.

Create `companion/EXTERNAL_INTERFACE.md` with:

- ROS 2 image input from Gazebo at 20 frames per simulated second;
- ROS 2 `/clock` consumption with `use_sim_time=true`;
- MAVLink command output to ArduPilot;
- MAVLink telemetry and acknowledgement input;
- rules for stale runs, duplicate frames, missing frames, and lost clock;
- a prohibition on direct Gazebo control;
- deferred exact topic names, QoS values, MAVLink endpoint configuration, and command-to-frame correlation encoding.

- [ ] **Step 3: Write the ArduPilot SITL documents**

Create `ardupilot_sitl/PLAN.md` with the standard responsibility sections. Its implementation stages are: configure SITL vehicle; establish the Gazebo adapter; establish companion MAVLink endpoints; prove lockstep progress; expose readiness and diagnostics; validate command acknowledgement and telemetry flow.

Create `ardupilot_sitl/INTERNAL_INTERFACE.md`. State that no language-native sibling interface exists initially because SITL is treated as an independently deployed flight-controller process.

Create `ardupilot_sitl/EXTERNAL_INTERFACE.md` documenting companion MAVLink commands, telemetry and acknowledgements, Gazebo actuator output, Gazebo sensor input, lockstep ordering, endpoint readiness, loss-of-peer behavior, and deferred ports and adapter version. Explicitly call it an ArduPilot SITL or simulated flight-controller module, not a Pixhawk simulator.

- [ ] **Step 4: Write the Gazebo documents**

Create `gazebo/PLAN.md` with the standard responsibility sections. Its implementation stages are: load the world and vehicle; establish the ArduPilot adapter; publish `/clock`; prove lockstep physics; configure a 20-Hz simulation-time camera; publish ground truth; accept validated physical-effect requests; expose collision and diagnostic state.

Create `gazebo/INTERNAL_INTERFACE.md`. State that no same-process sibling interface exists initially and that future world plugins may expose language-native internal seams without bypassing Gazebo's ownership of physical state.

Create `gazebo/EXTERNAL_INTERFACE.md` documenting:

- ArduPilot actuator input and sensor output through the established adapter;
- authoritative ROS 2 `/clock` output;
- ROS 2 camera output at 20 frames per simulated second;
- ROS 2 ground-truth output for scoring;
- ROS 2 physical-effect requests from the electromagnet module;
- run identity, reset semantics, ordering, stale-run rejection, paused-clock behavior, and failure behavior;
- deferred topic names, message schemas, QoS values, world/plugin selection, and adapter version.

- [ ] **Step 5: Verify the flight-stack contracts**

Run:

```bash
for module in companion ardupilot_sitl gazebo; do
  test -f "$module/PLAN.md"
  test -f "$module/INTERNAL_INTERFACE.md"
  test -f "$module/EXTERNAL_INTERFACE.md"
done
rg -n "20 frames per simulated second|20 Hz" companion gazebo
rg -n "lockstep" ardupilot_sitl gazebo
rg -n "MAVLink" companion ardupilot_sitl
rg -n "authoritative.*clock|/clock" gazebo
```

Expected: every command exits zero and every cross-module path appears in both participating module contracts.

- [ ] **Step 6: Commit the flight-stack contracts**

```bash
git add companion ardupilot_sitl gazebo
git commit -m "docs: define flight stack module contracts"
```

### Task 3: Scenario and Scoring Module Contracts

**Files:**
- Create: `electromagnet/PLAN.md`
- Create: `electromagnet/INTERNAL_INTERFACE.md`
- Create: `electromagnet/EXTERNAL_INTERFACE.md`
- Create: `scorekeeper/PLAN.md`
- Create: `scorekeeper/INTERNAL_INTERFACE.md`
- Create: `scorekeeper/EXTERNAL_INTERFACE.md`

**Interfaces:**
- Consumes: root lifecycle rules, Gazebo clock and ground truth
- Produces: documented physical-effect requests, scenario events, and read-only scoring outputs

- [ ] **Step 1: Create the two module directories**

```bash
mkdir -p electromagnet scorekeeper
```

- [ ] **Step 2: Write the electromagnet documents**

Create `electromagnet/PLAN.md` with the standard responsibility sections. Its implementation stages are: consume run configuration and simulation time; evaluate deterministic scenario conditions; publish idempotent activation/deactivation requests to Gazebo; publish matching scorekeeper events; expose diagnostics.

Create `electromagnet/INTERNAL_INTERFACE.md`. State that no same-process sibling interface exists initially. Reserve future internal seams for scenario policy and ROS 2 adapter modules, with policy expressed as deterministic transformations of simulation-timestamped input state.

Create `electromagnet/EXTERNAL_INTERFACE.md` documenting `/clock` consumption, Gazebo physical-effect request output, scorekeeper scenario-event output, `run_id`, `magnet_id`, desired state, event identity, simulation timestamp, idempotency, stale-run behavior, loss-of-clock behavior, and deferred topic names, schemas, QoS values, and effect parameters. Prohibit direct changes to ArduPilot or aircraft pose/state.

- [ ] **Step 3: Write the scorekeeper documents**

Create `scorekeeper/PLAN.md` with the standard responsibility sections. Its implementation stages are: consume run configuration; consume `/clock`, ground truth, and scenario events; correlate inputs by run and simulation time; calculate deterministic scores; publish or persist results; expose incomplete-run diagnostics.

Create `scorekeeper/INTERNAL_INTERFACE.md`. State that no same-process sibling interface exists initially. Reserve future pure scoring-policy seams that accept immutable event/state values and return score changes.

Create `scorekeeper/EXTERNAL_INTERFACE.md` documenting `/clock`, Gazebo ground truth, electromagnet scenario events, optional diagnostic-only ArduPilot telemetry, score-event/result output, ordering and duplicate handling, incomplete-run behavior, and deferred topic names, schemas, QoS values, and persistence format. Explicitly prohibit command, actuator, force, pose, and mode outputs.

- [ ] **Step 4: Verify scenario and scoring contracts**

Run:

```bash
for module in electromagnet scorekeeper; do
  test -f "$module/PLAN.md"
  test -f "$module/INTERNAL_INTERFACE.md"
  test -f "$module/EXTERNAL_INTERFACE.md"
done
rg -n "physical|force|effect" electromagnet
rg -n "read-only|prohibit|never" scorekeeper
rg -n "run_id|run ID" electromagnet scorekeeper
rg -n "simulation time|/clock" electromagnet scorekeeper
```

Expected: every command exits zero, the electromagnet affects the world only through Gazebo, and the scorekeeper exposes no control path.

- [ ] **Step 5: Commit the scenario contracts**

```bash
git add electromagnet scorekeeper
git commit -m "docs: define scenario and scoring contracts"
```

### Task 4: Cross-Contract Consistency Verification

**Files:**
- Modify only if verification reveals a contradiction: the affected `PLAN.md`, `INTERNAL_INTERFACE.md`, or `EXTERNAL_INTERFACE.md`

**Interfaces:**
- Consumes: all root and child documentation contracts
- Produces: a consistent scaffold matching every approved design constraint

- [ ] **Step 1: Verify the recursive file shape**

Run:

```bash
for module in . companion ardupilot_sitl gazebo electromagnet scorekeeper; do
  test -f "$module/PLAN.md"
  test -f "$module/INTERNAL_INTERFACE.md"
  test -f "$module/EXTERNAL_INTERFACE.md"
done
```

Expected: exit zero with 18 contract documents present.

- [ ] **Step 2: Verify forbidden placeholders and implementation artifacts**

Run:

```bash
marker_pattern='T''BD|TO''DO|PLACE''HOLDER'
if rg -n "$marker_pattern" --glob '*.md' .; then exit 1; fi
if find companion ardupilot_sitl gazebo electromagnet scorekeeper -type f ! -name '*.md' | grep -q .; then exit 1; fi
```

Expected: both commands exit zero with no matches.

- [ ] **Step 3: Audit every communication edge bidirectionally**

Read the producer and consumer `EXTERNAL_INTERFACE.md` files side by side for all nine rows in the design spec's communication map. Confirm mechanism, direction, clock, run identity, and failure behavior agree. Correct any contradiction in the module that owns the inaccurate statement.

- [ ] **Step 4: Verify architectural prohibitions**

Run:

```bash
rg -n "direct.*Gazebo|never.*Gazebo|prohibit.*Gazebo" companion
rg -n "direct.*ArduPilot|prohibit.*ArduPilot" electromagnet
rg -n "read-only|no.*control|prohibit.*command" scorekeeper
rg -n "wall time|wall-clock" ./*.md */*.md
```

Expected: the searches show explicit prohibitions and restrict wall time to health or performance concerns.

- [ ] **Step 5: Check Markdown and repository state**

Run:

```bash
git diff --check
git status --short
```

Expected: `git diff --check` exits zero. `git status --short` lists only intentional contract changes, or is empty if all task commits are complete.

- [ ] **Step 6: Commit consistency corrections if any were required**

```bash
git add PLAN.md INTERNAL_INTERFACE.md EXTERNAL_INTERFACE.md companion ardupilot_sitl gazebo electromagnet scorekeeper
git diff --cached --quiet || git commit -m "docs: align simulation module contracts"
```
