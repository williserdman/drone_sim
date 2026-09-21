# Drone Simulation

Dockerized drone competition simulation built with ROS 2, Gazebo, ArduPilot SITL, and a companion mission controller.

The mission delivers three payloads, then returns home. Gazebo
provides physical truth; a separate scorer evaluates it; recordings and a
validated run bundle establish what actually happened.

## Start here

1. [Architecture and code map](docs/architecture.md): what each component owns,
   how one mission flows through the system, and where to make a change.
2. [Operator and developer runbook](docs/runbook.md): prerequisites, builds,
   running, finding videos, stopping safely, and testing.
3. [Human handoff / current status](docs/handoff.md): known failures, evidence,
   unfinished work, and the next useful tasks.

**Current caveat:** a physical three-payload flight and return home have been
verified, but the latest documented verification run failed later in the
recording window. A score of 150/150 alone does not mean the run passed.
See the [evidence and limitations](docs/handoff.md#verified-behavior-and-limits).

## First local check

From the repository root, with Python 3.12+ and `uv` available:

```bash
uv sync --locked
uv run --locked drone-sim --help
make test-unit
```

This checks the host tooling; it does **not** launch a flight or prove ROS/Gazebo
integration. The [runbook](docs/runbook.md) explains the remaining prerequisites.

Once the runtime images are ready, the normal flight command is:

```bash
uv run --locked drone-sim start --config config/default-run.json
```

It runs in the foreground and can take tens of minutes or longer. It does not
build images automatically. Do not start with a bare `docker compose up`.

## How to read this repository

- `config/` selects a run; `compose.yaml` connects its seven runtime services.
- Each of the seven modules has a concise README linked from the
  [module map](docs/architecture.md#where-to-read-or-change-code), covering its
  responsibilities, interfaces, entry points, tests, and constraints.
- `runs/<run_id>/` contains local generated evidence and is ignored by Git.
- Superseded plans and interface documents live in Git history, not the active
  documentation path. Dated verification notes remain evidence, not current specs.

## Contributing or adding a mission

For a fixed sequence, copy [configured-descent-run.json](config/configured-descent-run.json)
and edit its `mission_plan.steps`. The [companion guide](companion/README.md#configured-diagnostic-missions)
defines the seven tools; the [local workflow](docs/runbook.md#local-developer-workflow)
covers rebuilding changed code, running one template, and running the automatic set.

Read [AGENTS.md](AGENTS.md) for contribution rules, then the affected module
README. For a mission, start with [companion](companion/README.md): define its
behavior and success/failure conditions, implement and register it through the
existing selector, add a run template and focused tests, then follow the
[runbook](docs/runbook.md) to build and verify. Competition mission edits may
belong under `companion/comp2026`; read its local documentation before editing.

Update every affected module README and shared guide **in the same change**.
Report documentation impact (or why none is needed), tests, and any unverified
runtime behavior when handing off.

The Comp2026 mission source and its prior Git history are included in this
monorepo at `companion/comp2026`.
