# Recursive Module Interfaces and Simulation Orchestration

## Purpose

Establish a documentation-first repository structure that allows the drone simulation modules to be designed and implemented independently. The structure must work recursively: a directory may represent a container, a library inside a container, or a smaller nested subsystem.

This phase defines responsibilities and communication seams. It does not implement containers, ROS 2 messages, flight behavior, physics, or scoring logic.

## Design Principles

1. Every module owns its contract next to its implementation.
2. Local, language-native communication is distinct from cross-process communication.
3. Callers use documented interfaces rather than a module's internal files.
4. Gazebo is authoritative for physical truth.
5. Simulation time is authoritative for simulated behavior.
6. Docker Compose manages lifecycle and configuration, not simulated communication.
7. Contracts remain language-neutral until implementation requires a language-specific definition.

## Recursive Module Shape

Every module directory initially contains:

```text
<module>/
├── PLAN.md
├── INTERNAL_INTERFACE.md
└── EXTERNAL_INTERFACE.md
```

The same shape may be introduced at deeper levels as modules are decomposed.

### `PLAN.md`

Defines:

- the module's responsibility and explicit non-responsibilities;
- dependencies and owned state;
- planned implementation stages;
- acceptance and verification criteria;
- unresolved decisions that must be settled before implementation.

### `INTERNAL_INTERFACE.md`

Defines the language-native interface available to sibling modules under the same parent when they execute in the same process. It records callable operations, exchanged types, invariants, ordering rules, errors, and ownership rules.

Sibling modules may import one another directly only through this documented interface. Implementation files are private to the owning module.

Modules that expose no local interface state that explicitly. An interface is not invented merely to make every document non-empty.

### `EXTERNAL_INTERFACE.md`

Defines communication that crosses a process or container seam. Depending on the module, it records:

- ROS 2 topics, services, and actions;
- MAVLink endpoints and message responsibilities;
- ArduPilot-Gazebo adapter behavior;
- network ports and discovery requirements;
- message fields and semantic invariants;
- QoS, ordering, and delivery expectations;
- simulation-time behavior;
- startup readiness and failure behavior.

The document describes the contract in the first phase. Concrete ROS 2 `.msg`, `.srv`, and `.action` packages will be placed according to build-system requirements in a later design step.

## Initial Repository Modules

```text
drone_sim/
├── PLAN.md
├── INTERNAL_INTERFACE.md
├── EXTERNAL_INTERFACE.md
├── companion/
├── ardupilot_sitl/
├── gazebo/
├── electromagnet/
└── scorekeeper/
```

The root is itself a module. Its internal interface describes how its immediate children relate. Its external interface describes how an operator, CI job, or surrounding system starts, configures, observes, and collects results from a simulation run.

### Companion

Owns mission, vision, and autonomy decisions. It consumes camera data and vehicle telemetry and sends flight commands through ArduPilot. It never manipulates Gazebo state directly.

### ArduPilot SITL

Owns estimation, navigation, and flight-control loops. It accepts MAVLink commands, emits MAVLink telemetry and acknowledgements, exchanges actuator and simulated sensor data with Gazebo, and participates in lockstep simulation.

### Gazebo

Owns physics, sensors, camera generation, collision state, external forces, and ground truth. It publishes the authoritative ROS 2 clock and advances simulated state in coordination with ArduPilot.

### Electromagnet

Owns scenario logic for electromagnet activation. It requests physical effects through Gazebo and publishes scenario events for scoring. It does not directly alter ArduPilot or aircraft state.

### Scorekeeper

Observes Gazebo ground truth and scenario events and calculates results. It is read-only with respect to aircraft control and physics.

## Communication Map

| Producer | Consumer | Mechanism | Purpose |
| --- | --- | --- | --- |
| Companion | ArduPilot SITL | MAVLink | Flight and mission commands |
| ArduPilot SITL | Companion | MAVLink | Telemetry, modes, and acknowledgements |
| ArduPilot SITL | Gazebo | ArduPilot-Gazebo adapter | Actuator outputs |
| Gazebo | ArduPilot SITL | ArduPilot-Gazebo adapter | Simulated sensors and dynamics |
| Gazebo | Companion | ROS 2 image transport | Camera frames at 20 Hz in simulation time |
| Gazebo | Scorekeeper | ROS 2 | Authoritative ground truth |
| Electromagnet | Gazebo | ROS 2 | Requests for external physical effects |
| Electromagnet | Scorekeeper | ROS 2 | Scenario events |
| Gazebo | Simulation-aware modules | ROS 2 `/clock` | Authoritative simulation time |

ArduPilot and Gazebo operate in lockstep. Docker Compose is not present in this table because it controls lifecycle rather than simulated events.

## Timing Contract

ROS 2 Jazzy on Ubuntu 24.04 LTS is the initial platform target. Every simulation-aware ROS 2 node enables `use_sim_time` and uses Gazebo's `/clock` for decisions affecting the simulated world.

The camera produces 20 frames per simulated second. A slow host may take more than one wall-clock second to calculate one simulated second without changing simulated frame spacing.

Simulation-relevant messages include at least:

- `run_id`;
- a simulation timestamp;
- a stable event or frame identifier when applicable.

Causally derived messages include source identifiers where useful, such as a mission command's `source_frame_id`.

Host wall time is permitted for profiling, health checks, resource monitoring, and detecting stalled infrastructure. It does not schedule simulated events or define modeled latency.

## Run Lifecycle and Orchestration

Docker Compose initially owns:

- startup ordering and health checks;
- ROS 2 discovery and container networking configuration;
- port assignments;
- run configuration and `run_id` distribution;
- reset and shutdown behavior;
- logging and result collection.

Readiness is distinct from container startup. A module is healthy only when its required internal process and communication endpoints are ready.

Reset begins a new run with a new `run_id`. Modules discard or safely ignore messages belonging to earlier runs. The exact reset protocol will be defined before executable orchestration is implemented.

## Failure Semantics

Each external interface must state its QoS and behavior for missing, late, duplicated, or out-of-order data. The initial common rules are:

- stale `run_id` data is ignored and reported diagnostically;
- loss of authoritative clock or lockstep progress prevents further simulated decisions;
- the scorekeeper records incomplete inputs but never attempts corrective control;
- mission commands are confirmed through ArduPilot acknowledgements where MAVLink supports them;
- lifecycle failures use wall-clock health monitoring but do not advance simulation state.

Module-specific plans must refine these rules before implementation.

## Initial Documentation Phase

The first implementation step creates the root and five module directories with the three standard documents. Each document will contain concrete responsibilities and known communication paths, while unresolved message schemas remain clearly labeled as proposed decisions.

This phase deliberately excludes:

- ROS 2 package scaffolding;
- `.msg`, `.srv`, or `.action` definitions;
- Dockerfiles and Compose configuration;
- executable Python, Go, or Rust modules;
- Gazebo worlds or plugins;
- ArduPilot configuration;
- mission and scoring algorithms.

## Verification

The documentation scaffold is complete when:

1. Every initial module has exactly one plan, one internal interface, and one external interface document.
2. Every known communication path has an owning producer and consumer.
3. Each external path identifies its mechanism and authoritative clock.
4. No module claims responsibility owned by another module.
5. The scorekeeper has no write path into flight control or physics.
6. The companion has no direct write path into Gazebo.
7. Wall-clock and simulation-time responsibilities are explicitly separated.
8. Unknown implementation details are identified as decisions rather than implied requirements.

## Deferred Decisions

The following decisions are intentionally deferred to focused implementation designs:

- ROS 2 package and workspace layout;
- exact topic, service, action, and message names;
- DDS implementation and discovery configuration;
- QoS values per data stream;
- language selection for each module;
- exact reset and readiness protocols;
- ArduPilot-Gazebo adapter version and configuration;
- result persistence format.
