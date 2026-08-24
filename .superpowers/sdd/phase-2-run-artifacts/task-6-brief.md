# Task 6: Implement the Host Run Controller and Operator CLI

## Context and base

- Worktree: `/home/willis/projects/drone_sim/.worktrees/phase-2-run-artifacts`
- Branch: `phase-2-run-artifacts`
- Base commit: `065cbad9e72901c86dcefaa6221c905f1395c1d4`
- Authoritative plan: `docs/superpowers/plans/phase-2-run-artifacts.md`, Task 6
- Frozen interfaces: root and `orchestration/{EXTERNAL_INTERFACE,INTERNAL_INTERFACE}.md`
- Tasks 1–5 are complete and independently approved. Consume their production seams; do not fork or duplicate them.
- Do not modify machine configuration, ROS messages, Compose runtime services, Dockerfiles, Gazebo, ArduPilot, mission/scoring, or `companion/comp2026`.

## Required files and scope

Create:

- `orchestration/src/orchestration/status_store.py`
- `orchestration/src/orchestration/_adapters/__init__.py`
- `orchestration/src/orchestration/_adapters/compose.py`
- `orchestration/src/orchestration/controller.py`
- `orchestration/src/orchestration/cli.py`
- `orchestration/src/orchestration/__main__.py`
- `orchestration/tests/test_status_store.py`
- `orchestration/tests/test_controller.py`
- `orchestration/tests/test_cli.py`

Modify only as required:

- `pyproject.toml`
- `uv.lock`
- `orchestration/pyproject.toml`
- `orchestration/src/orchestration/__init__.py`
- `artifacts/src/artifacts/__init__.py` only to deprecate/unexport the compatibility-only `build_manifest`; production must use `ArtifactSession.finalize(FinalizationInput)`.

Do not implement Task 7 runtime nodes or services in this task. Controller tests use injected fakes until Task 7 supplies the real profile.

## Exact public behavior

Expose `RunController.start`, `status`, `abort`, and `collect_results`, and install exactly these CLI commands:

```text
uv run drone-sim start --config PATH
uv run drone-sim status RUN_ID [--output-root PATH]
uv run drone-sim abort RUN_ID [--output-root PATH]
uv run drone-sim collect-results RUN_ID [--output-root PATH]
```

`start` is foreground/blocking and exits 0 COMPLETED, 1 FAILED, 130 ABORTED. Controlled usage/configuration failures exit 2 without traceback. `status`, `abort`, and `collect-results` each emit exactly one deterministic result JSON object to stdout. Foreground `start` emits deterministic `StructuredEvent` JSONL host events to stdout during execution and exactly one distinguishable final result JSON object last; it emits no human prose on stdout. `status`, `abort`, and `collect-results` communicate only through the run directory and may execute concurrently. Their default output root is `runs`; an explicit output root must be absolute. `collect-results` is strictly read-only and validates the existing manifest before returning its path.

Status output includes at least:

```json
{"run_id":"...","state":"RUNNING","reason":"","manifest_path":null}
```

## StatusStore and durable protocol

`StatusStore` owns only the host control/status files and allocation:

- exclusively create `runs/<uuid>/{.control,.status,configuration}` and reject any pre-existing run directory;
- atomically persist valid JSON using same-directory temp, file `fsync`, `os.replace`, and directory `fsync`, leaving no temp sibling;
- own `.status/operator-state.json`, `.control/finalize-request.json`, and `.control/terminal-committed.json`;
- read, but never rewrite, Task 7-owned `.status/{artifacts-ready,runtime-running,source-finished,runtime-failure,runtime-frozen,artifacts-final,terminal-notified}.json`;
- make `abort` an idempotent atomic request for ABORTED finalization; it must never invoke Compose stop/down;
- preserve the first observed terminal cause as primary. Later causes are diagnostics only and never replace it.

Validate run IDs and containment; reject symlinks, hard links, path escapes, non-regular protocol files, corrupt JSON, and unsafe pre-existing targets. Do not delete or truncate a run directory. Repeated cleanup is non-destructive.

## Compose adapter

`ComposeRuntime` is a policy-free injected subprocess boundary. Use shell-free argument arrays and a unique project name `drone-sim-<run-id-without-hyphens>`. Every Compose invocation uses explicit `--project-directory` and the environment:

```text
SIM_RUN_ID=<uuid>
SIM_RUN_DIRECTORY=<absolute run dir>
SIM_CONFIG_PATH=<absolute configuration/run.json>
SIM_PHASE2_PROFILE=1
```

Implement `up`, `logs`, `stop_services`, `down`, `ps`, and `image_digests`. `up` is bounded detached startup with exact trailing arguments `up --detach --no-build`. Each operation consumes the remaining shared monotonic deadline/timeout supplied by the controller and preserves merged subprocess output. Do not use shell execution, ambient Compose discovery, host networking, privileged mode, or Docker-socket access. `logs` validates the frozen Task 5 per-service argv and augments the actual subprocess call with the explicit project directory and environment; the controller supplies a one-argument closure that recomputes remaining time before every one of the seven capture calls. Do not change raw-log bytes or ordering semantics.

## Controller sequence and policy

For requested completion, assert this exact order:

```text
allocate -> snapshot config -> compose up -> wait artifacts-ready
-> wait source-finished -> request FINALIZING -> wait runtime-frozen
-> wait artifacts-final -> capture logs -> validate -> commit manifest
-> notify terminal -> wait terminal-notified -> compose down
```

Use the existing immutable `RunLifecycle`. Generate the UUID before Compose starts, allocate the run directory, then call `write_resolved_config`. `compose down` always runs in a bounded `finally` after any successful Compose start. Every path that started Compose enters finalization. Ctrl-C becomes requested ABORTED finalization and exit 130; do not let Task 5 cancellation semantics turn it into FAILED.

Use one absolute monotonic deadline per startup and one for the entire finalization phase. Retries/polls and every subprocess/adaptor call receive only remaining time; no step resets the budget. The overall `max_wall_seconds` also bounds running/stall observation. Polling wall time may only observe infrastructure files/process health; it must not schedule simulated events.

During STARTING/RUNNING, race expected success against runtime-failure, operator abort, child death, Ctrl-C, clock/source stall, and the shared deadline. First observed cause wins and is durably persisted. Required failure tests: startup deadline, recorder failure, clock stall, Docker-log capture failure, validation failure, Ctrl-C, and finalization deadline.

Task 7 writes `.status/runtime-running.json` immediately after publishing `RUNNING` for the first valid clock. Validate its exact run ID, fixed state, and nonnegative integer simulation timestamp, then update operator state. Test abort racing this signal; do not infer RUNNING from elapsed wall time or readiness alone.

Terminal semantics:

- completion + fully valid artifacts -> COMPLETED;
- requested completion + invalid/missing artifact or log-capture failure -> commit a FAILED manifest first, then apply `FINALIZATION_FAILED`;
- requested abort remains ABORTED even if artifacts validate;
- any terminal path preserves partial/surviving artifacts and attempts a manifest;
- write `.control/terminal-committed.json` only after authoritative manifest commit;
- no required artifact may be modified after commit;
- wait for `.status/terminal-notified.json`, then teardown in bounded `finally`.

Reserve `min(5 seconds, finalization_wall_seconds / 5)` inside the single finalization deadline for teardown. All prior finalization work uses the shortened work deadline; `compose down` receives the actual remaining total budget. Test exhaustion before teardown and require the down attempt still occurs.

## Host structured events and Task 5 seam

Append each host operator event both to stdout and descriptor-safely to fixed `logs/orchestration-host.jsonl.partial` using the existing `StructuredEvent` six-field schema and exact run/module ownership (`module="orchestration"`). Use timezone-aware UTC wall timestamps and simulation timestamp `null` when no authoritative simulation stamp is available. Do not invent another logging schema.

Invoke `DockerLogCapture(..., host_events=True)` before manifest finalization. Its frozen behavior validates the host partial and merges orchestration events by normalized UTC wall timestamp, Compose-before-host tie, then source line order. On `DockerLogCaptureError`, consume its immutable result and downgrade requested completion even if all final filenames exist. Preserve Docker raw streams exactly; never append host bytes to them.

## Manifest/provenance

Use only `ArtifactSession.finalize(FinalizationInput)` for production finalization. Remove `build_manifest` from the artifacts public export surface while retaining internal compatibility only if existing tests require it.

Before manifest commit, gather source revision/dirty state using read-only Git commands, Compose image digests using `docker image inspect` behind the adapter, configuration checksum records from the resolved snapshot, timing, scoring fields if present, and evidence paths. Missing/invalid provenance must fail closed or be represented explicitly according to the existing manifest types; do not fabricate revisions/digests/scores. Ensure a FAILED/ABORTED bundle remains diagnosable.

Strictly parse `.status/artifacts-final.json`: its exact top-level keys are `run_id`, `complete`, and `records`. `records` is a three-element list containing each of `video/onboard.mp4`, `video/observer.mp4`, and `rosbag` exactly once by `relative_path`; every record has exactly `relative_path`, `status` (`valid|missing|invalid`), `detail`, `size_bytes`, `sha256`, and a nonempty `semantic` object. Missing/invalid records still identify their validator/kind and observed failure in `semantic`. A valid completion requires all three records `valid`, non-null size/checksum agreement, and aggregate `complete=true` consistent with those records. Build specialized `ArtifactSession` validators for these three paths that recompute host-safe file/tree size and checksum and require exact report agreement. Missing/malformed/extra/duplicate records, stale hashes, status mismatch, and corrupt video/bag reports downgrade completion; never use the default presence-only validators for recorder outputs. The host does not rerun FFmpeg/ROS semantic tooling.

## Workspace and executable

Add `artifacts` and `orchestration` as uv workspace members at the root, make the root dev project depend on both workspace packages, make orchestration depend on artifacts, and register:

```toml
[project.scripts]
drone-sim = "orchestration.cli:main"
```

Regenerate `uv.lock`. The executable must work through installed workspace dependencies, not pytest-only `pythonpath`.

## TDD, verification, and report

Follow Superpowers TDD and systematic debugging. Persist genuine RED tests before production implementation. Test at minimum:

- exclusive allocation, path safety, atomic/no-temp status writes, corrupt/pre-existing controls;
- exact CLI parse/output/exit codes and concurrent status/abort behavior;
- exact controller order for COMPLETED plus every named failure/abort/deadline path;
- durable runtime-running validation and abort racing that signal;
- first-cause-wins races and later diagnostic retention;
- shared monotonic deadline consumption without reset;
- bounded teardown in `finally`, including adapter exceptions;
- exact Compose arrays/project/env/timeouts and merged output;
- detached `up --detach --no-build` and decreasing timeout recomputation across all seven log calls;
- host structured logging and `host_events=True` log capture;
- `DockerLogCaptureError` terminal downgrade;
- manifest-before-terminal-notification and no post-commit artifact writes;
- strict artifacts-final record/hash agreement, including corrupt video, corrupt bag, stale digest, and missing record cases;
- read-only validated `collect-results`;
- installed `uv run drone-sim --help` lists exactly four commands.

Run at least:

```bash
uv run pytest orchestration/tests/test_status_store.py orchestration/tests/test_controller.py orchestration/tests/test_cli.py -v
uv run pytest orchestration/tests -v
uv run pytest artifacts/tests orchestration/tests tests/contracts -v
uv run pytest -v
uv run drone-sim --help
uv run python -m compileall -q orchestration/src orchestration/tests
git diff --check
```

Write `.superpowers/sdd/phase-2-run-artifacts/task-6-report.md` containing RED/GREEN evidence, implementation/files/self-review, exact test counts, command help evidence, and concerns for Task 7. Commit scoped work as `feat: add simulation operator lifecycle`, leave the worktree clean, and report the commit hash.

## Judgment constraints

- Keep adapters deep and policy-free; lifecycle policy belongs only in `RunController`.
- Favor immutable result/domain values and injected clocks/runners over broad mocks.
- Do not weaken the frozen lifecycle, manifest, structured-log, or filesystem safety contracts to simplify Task 6.
- Do not claim Phase 2 is runnable until Tasks 7–8 pass real Compose integration.
- Follow the global no-AI-attribution rule.
