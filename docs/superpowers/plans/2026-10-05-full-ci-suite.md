# Full mission CI suite implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. The user's MVP instructions prohibit unsolicited code-review cycles; preserve that override whichever execution method is selected.

**Goal:** Run every checked-in flight setup through one local command and a manual workstation CI job, with accepted all-axis calibration, fresh reload validation, and honest per-mission results.

**Architecture:** Extend the existing calibration importer, mission hosts and independent acceptance. A small orchestration suite runner sequences the existing `RunController`; its Docker/build boundary and reports remain separate from flight control. GitHub Actions invokes the same command on this workstation and publishes compact diagnostics.

**Tech Stack:** Python 3.12+, locked uv workspace, pytest, Docker Compose, ROS 2 Jazzy, Gazebo, pinned ArduCopter 4.7, PyMAVLink and GitHub Actions.

**Spec:** [Approved design](../specs/2026-10-05-full-ci-suite-design.md), approved 2026-10-05. Runtime baseline is the calibration/moving-pad branch underlying spec commit `482b5fa`.

## Global constraints

| Constraint | Requirement |
| --- | --- |
| Catalog | All 11 spec entries, in order. Calibration and fresh reload are gates; attempt the remaining nine sequentially after gates pass. Individual mission failure does not skip later independent cases. |
| Outcomes | Physical behavior, raw score, lifecycle, independent acceptance and teardown remain separate. Any required failed, blocked or unrun case makes the suite fail. |
| Provenance | Clean committed source; build seven Phase 3 images once per suite; freeze source and image identities; fresh run directory, Compose project and SITL storage per case; no build or source edits during flights. |
| Parameters | Same accepted 15-gain artifact and source manifest for every consumer; base then scenario overlay then gains; complete matching effective readback before readiness/commands. Roll-only diagnostic writes stay within its run. |
| Evidence and provider | Preserve existing bundles, including failed/partial runs. Manual Actions only, this workstation, one suite at a time across checkouts/output roots; full recordings local, summaries/logs uploaded. Update `/home/willis/SETUP_REPLICATION.md` with machine changes, without secrets. |

Preserve imported Comp2026 history, licenses and unrelated work. No new dependency,
mission DSL, scoring threshold change, calibration cache or automatic trigger.
Use focused checks. After three failed fixes, stop and name the doubtful assumption.
During a flight, keep progress in ignored scratch and commentary; do not update
tracked plan checkboxes until teardown finishes.

## Review focus

These are test targets, not a request to initiate a separate review cycle.

| Condition | Expected behavior | Owning task |
| --- | --- | --- |
| Changed consumer dynamics, overlay or missing/mismatched live gain | Reject compatibility or keep all commands blocked; preserve legacy inspection. | 2, 3 |
| Process completed but score/evidence fails | Case and suite fail; later independent cases still execute. | 4, 7 |
| Temporary template relocates course/calibration inputs | Original inputs, plan, seed and timing survive with absolute references. | 6 |
| Operator ACK without observed state, premature readiness or wrong run | No premature arm/takeoff; failed or timed-out case retains its evidence. | 5 |
| Second suite or interruption with active containers | Common lock prevents overlap; abort/finalize current run, retain partial report, release lock only after bounded cleanup. | 7 |

## File and interface map

| Unit | Files and responsibility |
| --- | --- |
| Comp2026 packaging | `.dockerignore`, new companion `comp2026_smoke.py`; admit/import the automatic mission's real dependency closure. |
| Calibration profile/import | Existing `artifacts/calibration.py`, orchestration `calibration.py`/`config.py`; identify compatible physical variants and freeze exact consumer inputs. |
| Runtime parameter gate | New companion `calibration_gate.py`, existing configured/runtime hosts; share value matching and block flight until readback. |
| Mission acceptance | Existing artifacts `acceptance.py`, `calibration.py`, new `diagnostic_acceptance.py`; keep score replay, use mission-specific evidence and safe diagnostic criteria. |
| External operator | New companion `operator_wait.py`, one narrow runtime status, native SITL SERIAL1 and an auxiliary Compose service; preserve the original wait plan. |
| Suite | New orchestration `suite.py` for catalog/templates/sequencing/results, `_adapters/suite.py` for build/provenance/lock; existing CLI/controller handle actual runs. |
| CI | New `.github/workflows/mission-suite.yml`; same suite command, persistent output root, summaries and compact uploads. |

Before editing, read the affected module READMEs and current architecture/runbook.
Tasks 1–5 provide independently testable repairs. Task 6 defines the suite data
interfaces, task 7 connects the runner, task 8 provisions CI, task 9 proves it.
Before task 1, use the git-worktrees skill to create `feat/manual-ci-suite` from
the approved calibration/spec/plan branch. Preserve the existing moving-pad PR,
checkout and evidence. Main alone does not yet contain the calibration changes.

## Task 1: Repair and smoke-test Comp2026 image packaging

**Files:** Modify `.dockerignore`, `companion/README.md`, `docs/runbook.md`; create `companion/src/drone_sim_companion/comp2026_smoke.py`, `companion/tests/test_comp2026_smoke.py`.

**Interfaces:** `comp2026_smoke.main(argv: Sequence[str] | None = None) -> int`, runnable with `python3 -m drone_sim_companion.comp2026_smoke`. It imports `drone.control.drone_control`, camera/range support and `drone.auto_attempt._original_mission_functions()` without constructing hardware or a flight controller. Zero means all original FM1/FM2/FM3 callables imported.

- [ ] **Step 1: Add the focused import test.** Use the actual tracked Comp2026 source on the host import path; do not construct hardware. Test `test_comp2026_smoke_imports_full_auto_closure` and `test_comp2026_smoke_reports_missing_runtime_module`. The latter supplies a missing-module import fault and asserts a nonzero result naming that module. The real image check remains required.

```python
def test_comp2026_smoke_imports_full_auto_closure(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / "companion/comp2026/src"))
    assert comp2026_smoke.main([]) == 0
```
- [ ] **Step 2: Run the test and capture its expected failure.** `uv run --locked pytest companion/tests/test_comp2026_smoke.py -q` must initially fail because the entry point is absent.
- [ ] **Step 3: Admit the runtime import closure and implement the entry point.** Add `flight_state.py`, `mission_supervisor.py`, `stability.py`, and the required LiDAR package/parents to the existing allowlist; follow transitive imports for any further necessary files. Keep unrelated nested source excluded. Document the smoke command.
- [ ] **Step 4: Verify unit behavior and the real image.** Run the focused test, then build `companion-runtime` with `SIM_COMP2026_REVISION=$(git rev-parse HEAD)` and run `docker run --rm drone-sim-companion-runtime:phase3 python3 -m drone_sim_companion.comp2026_smoke`. Both must exit zero. This import check is not flight evidence.
- [ ] **Step 5: Commit only the packaging/smoke/doc files.** Commit message `Fix Comp2026 runtime import closure`.

## Task 2: Bind calibrated consumers to compatible aircraft and overlays

**Files:** Modify `orchestration/src/orchestration/{calibration,config,controller}.py`, `artifacts/src/artifacts/calibration.py`, `config/{run-template,run}.schema.json`, `config/calibration-validation-run.json`, `orchestration/tests/{test_calibration,test_config}.py`, `tests/contracts/test_calibration_interfaces.py`, orchestration/artifacts READMEs and `docs/architecture.md`.

**Interfaces:**

```python
# artifacts.calibration
def airframe_fingerprint(model_xml: bytes) -> str: ...
# orchestration.calibration, preserve existing optional test seams
def build_source_calibration_profile(project_directory, *, image_digests=None) -> dict: ...
def freeze_calibration_import(source_run_directory, *, project_directory,
    consumer_vehicle="iris_flight", consumer_scenario="descent_v1",
    inspector=None, image_digests=None, artifact_reader=None,
    baseline_reader=None) -> CalibrationImport: ...
```

Change `resolve_run_config`'s injected importer to receive `(source_path,
consumer_vehicle, consumer_scenario)` after template validation. Keep exact-byte
copying in `write_resolved_config`. The outer calibration JSON stays version 1;
the new nested profile declares `schema_version: 2`, `id: competition-airframe-v2`.
Add optional template/resolved `calibration_validation: bool`, default false,
and set it true only in `calibration-validation-run.json`. True requires a
configured calibrated consumer. This explicit frozen role separates reload-only
checks from ordinary gain consumption; do not infer it from every import.

- [ ] **Step 1: Add focused compatibility tests.** Parametrize supported consumer selectors `configured`, `controlled_descent`, `hover_roll`, `autotune_roll`, `comp2026_auto`. `test_changed_consumer_physics_is_incompatible` verifies three stock vehicle fingerprints agree, but changing rotor force, inertia, sensor or hardpoint rejects. `test_moving_overlay_binds_effective_baseline` verifies overlay settings override source baseline while gains stay last. Preserve the existing source/artifact checksum assertions and legacy unloaded-profile test.

```python
def test_source_profile_binds_all_compatible_vehicle_models():
    profile = build_source_calibration_profile(ROOT, image_digests=IMAGES)
    assert profile["schema_version"] == 2
    assert set(profile["vehicle_models_sha256"]) == {
        "iris_flight", "iris_moving_pad", "iris_competition"}
    assert len(profile["airframe_sha256"]) == 64
```
- [ ] **Step 2: Run tests and confirm supported legacy consumers currently fail.** `uv run --locked pytest orchestration/tests/test_calibration.py orchestration/tests/test_config.py tests/contracts/test_calibration_interfaces.py -q`.
- [ ] **Step 3: Implement versioned family and consumer binding.** Canonicalize the generated vehicle XML after normalizing only its outer model name and removing `gz::sim::systems::PosePublisher` plus the declared `libcwru_payload_command_coordinator.so` / `libdrone_sim_detachable_joint_system.so` plugins. Retain all physical links, joints, sensors and motor/controller settings; unknown plugins remain part of the fingerprint. Require all three stock variants to share `airframe_sha256`; record their exact model digests in `vehicle_models_sha256`. In the imported profile record `consumer_vehicle`, `consumer_model_resource`, `consumer_model_sha256`, ordered `overlay_parameter_sha256s` and `effective_baseline_parameters`; the effective set includes existing mission-specific precision-readiness values as well as preserved source settings. Preserve source `baseline_parameters` and original manifest/artifact bytes. Compare family/base/firmware/SITL-Gazebo images before import. Keep the original `competition-unloaded-v1` path for legacy `iris_flight` only. Update actual schemas, not a parallel validator.
- [ ] **Step 4: Run focused checks including SITL load order.** Repeat step 2 plus `uv run --locked pytest ardupilot_sitl/tests/test_config.py -q`. Gain files still load after the base/moving overlay; unsupported aircraft, mismatches and altered bytes reject. Document the compatibility boundary.
- [ ] **Step 5: Commit explicit changed files.** Commit message `Extend calibration compatibility to suite aircraft variants`.

## Task 3: Gate every mission host on effective parameter readback

**Files:** Create `companion/src/drone_sim_companion/calibration_gate.py`, `companion/tests/test_calibration_gate.py`; modify companion `configured_runtime.py`, `runtime_node.py`, `autotune.py` only for declared-write evidence, tests `test_configured_live_loop.py`/`test_runtime_node.py`, companion README and architecture.

**Interfaces:**

```python
def calibration_parameters(config) -> dict[str, float]: ...
class CalibrationGate:
    def __init__(self, expected: Mapping[str, float]) -> None: ...
    def observe(self, name: str, value: float) -> None: ...
    # ready: bool; failure: str | None
    def snapshot(self) -> dict[str, float]: ...
```

`calibration_parameters` uses v2 `effective_baseline_parameters` plus gains,
falling back to v1 source baseline. Retain the old configured helper as a wrapper
if existing callers/tests use it. Gate comparisons use `rel_tol=1e-5`,
`abs_tol=1e-7`; preserve precision-setting comparisons at their stricter tolerance.

- [ ] **Step 1: Add gate and host tests.** The pure test observes every expected value except one and asserts `not ready`; a complete matching set unlocks; one mismatch records failure. Extend existing live-loop/runtime harnesses with `test_each_consumer_blocks_commands_without_complete_readback`, parametrized for controlled descent, hover, roll tune and Comp2026: missing/mismatched readback yields zero mode/arm/takeoff commands. The matching case emits one `calibration_parameters_verified` event with `stage=pre_arm` and the full set before the first command.

```python
def test_gate_requires_every_effective_parameter():
    gate = CalibrationGate({"ATC_RAT_RLL_P": 0.1, "ATC_RATE_FF_ENAB": 1.0})
    gate.observe("ATC_RAT_RLL_P", 0.1)
    assert not gate.ready
    gate.observe("ATC_RATE_FF_ENAB", 1.0)
    assert gate.ready and gate.failure is None
    assert set(gate.snapshot()) == {"ATC_RAT_RLL_P", "ATC_RATE_FF_ENAB"}
```
- [ ] **Step 2: Confirm the new tests fail.** `uv run --locked pytest companion/tests/test_calibration_gate.py companion/tests/test_configured_live_loop.py companion/tests/test_runtime_node.py -q`.
- [ ] **Step 3: Apply the gate to existing host entry points.** Reuse the single complete parameter-list request for MAVLink hosts and mission-local DroneKit readback where already used. Gate `_run_controlled_descent`, `_run_autotune_roll` including hover, `_run_comp2026`, and configured readiness without changing mission behavior. Preserve passive readiness so warmup cannot deadlock. The roll mission records imported verification before its declared seed/retune writes, and saves its own five-field artifact without replacing the shared source.
- [ ] **Step 4: Verify the focused gate/mission checks.** Repeat step 2 plus `companion/tests/test_configured_runtime.py`, `companion/tests/test_autotune.py`, `companion/tests/test_hover.py`, `companion/tests/test_mavlink_adapter.py`. Matching values unlock; missing values expire under existing bounds; no unverified host commands execute. Update companion constraints/readiness docs in this change.
- [ ] **Step 5: Commit the gate, affected hosts/tests/docs.** Commit message `Verify calibrated parameters before every suite mission`.

## Task 4: Apply mission-specific independent acceptance

**Files:** Modify artifacts `acceptance.py`/`calibration.py`, create `diagnostic_acceptance.py` and `artifacts/tests/test_diagnostic_acceptance.py`; extend `artifacts/tests/{test_calibration_acceptance,test_acceptance,test_session}.py`, artifacts README and architecture. Add a narrow allowlist consistency assertion to `tests/contracts/test_calibration_interfaces.py`.

**Interfaces:** Public `inspect_phase3_via_container(...)` and `inspect_phase3_bundle(...)` keep their existing signatures. Mission/scenario/frozen plan determine their required evidence. New diagnostic helpers:

```python
def validate_diagnostic_logs(mission: str, companion: Sequence[Mapping]) -> None: ...
def validate_diagnostic_landing(rule_results: Sequence[Mapping]) -> None: ...
def validate_roll_gain_artifact(run_directory: Path) -> dict[str, float]: ...
```

Place the roll parser/validator in artifacts so the artifacts image never needs
to import the companion package. Bind its five-name allowlist to the existing
companion `ROLL_GAIN_PARAMETERS` with the focused contract assertion.

- [ ] **Step 1: Add acceptance tests with the exact diagnostic policy.** A replay-validated raw descent result with only XY precision missing is 60/100 and passes diagnostic landing; a result missing `safe_preimpact_speed` or `stable_contact` fails regardless of other points. Test hover phases plus five continuous seconds of stable samples inside the real 10 s ALT_HOLD. Test ordered roll success/save, terminal phase, exactly five positive artifact values with I=P and one matching DataFlash save. Test that a calibrated moving/competition plan does not require the reload hover, and that post-arm readback cannot satisfy the gate.

```python
def test_diagnostic_landing_does_not_require_uncommanded_xy_precision():
    rows = [{"rule_id": name, "passed": True} for name in (
        "airborne_then_contact", "safe_preimpact_speed", "stable_contact")]
    rows.append({"rule_id": "touchdown_precision", "passed": False})
    assert validate_diagnostic_landing(rows) is None
    rows[1]["passed"] = False
    with pytest.raises(BundleAcceptanceError):
        validate_diagnostic_landing(rows)
```
- [ ] **Step 2: Confirm failures in focused acceptance tests.** `uv run --locked pytest artifacts/tests/test_diagnostic_acceptance.py artifacts/tests/test_calibration_acceptance.py -q`.
- [ ] **Step 3: Separate checks while retaining independent replay.** Universal import checks validate copied source identity, consumer model/resources, effective values and mission-aware pre-arm order. For v2 imports, only frozen `calibration_validation: true` requires the configured 5 m/10 s sequence; preserve the original stricter validation interpretation for unmarked legacy v1 imports. Branch diagnostic grammar by mission; preserve configured/controlled-descent checks. After existing ground-truth score replay succeeds, require the three named safe landing rules for diagnostics, maximum score for the other suite contracts. Do not make calibration-input presence force maximum diagnostic score. Roll parsing preserves its existing header/format and compares to native saved parameters. Retain v1 profile inspection and checksum-bound historical rule replay.
- [ ] **Step 4: Run affected acceptance/replay checks.** Run tests from step 2 plus `artifacts/tests/test_acceptance.py`, `artifacts/tests/test_calibration.py`, `artifacts/tests/test_score_validation.py`, `artifacts/tests/test_session.py`, `tests/contracts/test_calibration_interfaces.py`. Tests must still reject hard impact, unstable contact, wrong source/model, missing save and invalid evidence. No scoring rule file changes. Update artifacts/architecture documentation.
- [ ] **Step 5: Commit the acceptance/tests/docs change.** Commit message `Validate suite diagnostics against their actual mission contracts`.

## Task 5: Exercise the external operator wait through native SERIAL1

**Files:** Modify SITL `config.py`/`test_config.py`, `compose.yaml`, artifacts `runtime_status.py`/`test_runtime_status.py`, companion `configured_runtime.py`/`pyproject.toml`/`test_configured_host.py`; create companion `operator_wait.py`/`test_operator_wait.py`; modify orchestration `controller.py`/`_adapters/compose.py`/`test_controller.py`, extend artifacts `test_session.py` and acceptance tests. Update SITL, companion, orchestration, artifacts READMEs plus architecture/runbook.

**Interfaces:**

```python
# RuntimeConfig: operator_mavlink_port: int = 5762
# Narrow typed status, using existing RuntimeStatus conventions:
OperatorWaitStartedStatus(run_id, operation_id, sim_timestamp_ns)
# wire name operator-wait-started; fixed tool=wait_for_state, state=running
class OperatorWaitActor:
    def __init__(self, vehicle: MavlinkAdapter, emit: Callable) -> None: ...
    def observe(self, telemetry: Telemetry) -> None: ...
    def tick(self, timestamp_ns: int) -> None: ...
def main() -> int: ...  # entry point drone-sim-operator-wait
```

Extend `RunController.start(config_path, *, auxiliary_services: Sequence[str] = ())`
and Compose construction with the same argument; allow only empty or
`("operator-wait-runtime",)`. Extend service-health checks to include that sidecar.
Ordinary launches still have exactly seven owners and unchanged defaults.

- [ ] **Step 1: Add operator ordering/topology tests.** Assert primary 5760 and separate `--serial1 tcp:0.0.0.0:5762`; no host-published port. `test_actor_requires_same_run_started_wait` covers RUNNING, execution readiness, started wait and fresh heartbeat with zero early commands. `test_actor_requires_observed_guided_after_ack` rejects ARM after ACK alone; accepted ARM ACK also needs observed arming. Stale/incorrect state or rejected ACK fails. Assert PASSIVE remains alive and finalization prevents new commands. Controller tests require normal seven-service health versus the explicit eight-service operator case.

```python
def test_operator_port_preserves_the_primary_endpoint(tmp_path):
    config = RuntimeConfig(RUN_ID, tmp_path.resolve(), gazebo_host="127.0.0.1")
    argv = config.argv
    assert argv[argv.index("--serial0") + 1] == "tcp:0.0.0.0:5760"
    assert argv[argv.index("--serial1") + 1] == "tcp:0.0.0.0:5762"
```
- [ ] **Step 2: Confirm the focused tests fail.** Run `ardupilot_sitl/tests/test_config.py`, `artifacts/tests/test_runtime_status.py`, `companion/tests/test_operator_wait.py`, `companion/tests/test_configured_host.py`, and the new auxiliary-service cases in `orchestration/tests/test_controller.py` using `uv run --locked pytest ... -q`.
- [ ] **Step 3: Implement the endpoint, one readiness status and bounded actor.** Publish `OperatorWaitStartedStatus` once after the first wait operation actually starts. The actor reads typed same-run statuses, subscribes read-only to public `/clock`, uses its own distinct GCS sysid on 5762, and reuses `MavlinkAdapter.send/poll` for normal GUIDED/ARM commands. Stay passive/alive afterward until finalization. A profile `operator-wait` reuses the companion image. Write ordered run-correlated evidence to `logs/docker/operator.jsonl`, which existing optional artifact discovery includes. Stop the auxiliary service after runtime-frozen and before host manifest validation; close its log before hashing. Do not add an eighth quiescence owner. Independent acceptance verifies operator ordering against the configured wait and pre-arm gate.
- [ ] **Step 4: Verify focused checks and resolved topology.** Repeat step 2 plus `artifacts/tests/test_session.py` and operator acceptance tests. Assert frozen -> sidecar stop -> artifact final -> manifest -> down ordering and operator-log checksum. `docker compose --profile phase3 --profile operator-wait config` must resolve the private endpoint/actor. The live dual-connection proof belongs to task 9's operator case.
- [ ] **Step 5: Commit endpoint/actor/integration/tests/docs together.** Commit message `Run the operator-wait scenario through a separate MAVLink endpoint`.

## Task 6: Define the complete catalog and suite report

**Files:** Create `config/ci-suite.json`, `orchestration/src/orchestration/suite.py`, `orchestration/tests/test_suite.py`, `tests/contracts/test_suite_catalog.py`; update orchestration README and link the catalog from the runbook.

**Interfaces:**

```python
@dataclass(frozen=True)
class SuiteCase:
    name: str
    template: Path
    acceptance: str
def load_suite_catalog(path: Path, *, project_directory: Path) -> tuple[SuiteCase, ...]: ...
def prepare_suite_template(case: SuiteCase, *, suite_directory: Path,
    output_root: Path, calibration_source: Path | None) -> Path: ...
def write_suite_report(suite_directory: Path, report: Mapping[str, object]) -> None: ...
```

Report schema version 1 has `suite_id`, `state`, `reason`, `started_at`,
`finished_at`, frozen `source_revisions`/`image_digests`, `catalog_sha256`,
`template_sha256s`,
calibration/reload run IDs and ordered `cases`. Case fields are `name`, `template`,
`acceptance_contract`, `status`, `run_id`, `lifecycle`, `reason`, `physical_outcome`,
`score`, `artifact_acceptance`, `teardown_diagnostics` and relative `bundle_path`.
Unknown run/physical/score facts are null. `artifact_acceptance` contains its
accepted flag, reason and independent report when available. Case status is
`passed`, `failed`, `blocked` or `unrun`; initial entries are unrun.
Set physical outcome to `LANDED` only when independently established by the
mission's accepted physical contract; otherwise leave it null. A companion
success log or completed process alone cannot supply that fact.

Use case names `calibration`, `reload-validation`, `configured-descent`,
`operator-wait`, `controlled-descent`, `hover-roll`, `autotune-roll`, `moving-pad`,
`stationary-pad`, `competition-slow`, `competition-realtime` in that order.

- [ ] **Step 1: Add catalog/template/report tests.** Assert all 11 spec templates exactly once, gates first, and fixed valid contract names. A newly added top-level flight config missing from the catalog fails coverage. `test_prepared_competition_template_preserves_relative_inputs` checks absolute course/scenario/calibration references and unchanged mission plan, seed, timing and recording after relocation. Assert JSON and Markdown distinguish failed lifecycle, 60/100 accepted diagnostic and invalid artifacts, with links to original bundles.

```python
def test_catalog_includes_every_setup_and_both_gates_first():
    cases = load_suite_catalog(ROOT / "config/ci-suite.json", project_directory=ROOT)
    assert len(cases) == len({case.template for case in cases}) == 11
    assert [case.acceptance for case in cases[:2]] == ["calibration", "reload_validation"]
    assert cases[-1].template == ROOT / "config/realtime-run.json"
```
- [ ] **Step 2: Confirm missing catalog/interfaces fail.** `uv run --locked pytest orchestration/tests/test_suite.py tests/contracts/test_suite_catalog.py -q`.
- [ ] **Step 3: Implement only the catalog, preparation and report functions.** Populate names/contracts from the approved table; resolve project-relative templates and template-relative input paths before copying. Only override output root/calibration source. Write each temporary configuration to `suites/SUITE_ID/configuration/CASE_NAME.json`. Publish report JSON atomically and derive Markdown from it. Keep run bundles under `OUTPUT_ROOT/RUN_ID`, with no video/bag copies.
- [ ] **Step 4: Verify the focused tests and links.** Repeat step 2; check the catalog resolves every real template and points to existing rules. Update the orchestration/runbook descriptions, marking the CLI as not available until task 7.
- [ ] **Step 5: Commit catalog/report/tests/docs.** Commit message `Define the complete mission suite catalog and reports`.

## Task 7: Execute the suite through the existing run controller

**Files:** Extend orchestration `suite.py`, create `_adapters/suite.py`, `orchestration/tests/test_suite_runtime.py`; modify CLI, controller allocation seam and provenance helper as needed, `orchestration/tests/{test_suite,test_cli,test_controller}.py`, orchestration README, README, architecture and runbook.

**Interfaces:**

```python
@dataclass(frozen=True)
class FrozenSuiteRuntime:
    source_revisions: tuple[SourceRevision, ...]
    image_digests: tuple[ImageDigest, ...]
class SuiteRuntime:
    def __init__(self, *, project_directory: Path) -> None: ...
    def lock(self) -> ContextManager[None]: ...
    def build_and_freeze(self) -> FrozenSuiteRuntime: ...
    def assert_unchanged(self, frozen: FrozenSuiteRuntime) -> None: ...
    def inspect_run(self, case: SuiteCase, run_directory: Path,
        frozen: FrozenSuiteRuntime) -> BundleAcceptanceReport: ...
@dataclass(frozen=True)
class SuiteResult:
    suite_id: str
    state: str
    report_path: Path
    exit_code: int
    def to_dict(self) -> dict[str, object]: ...
class SuiteRunner:
    def __init__(self, *, project_directory: Path, controller: RunController,
        runtime: SuiteRuntime | None = None, event_stream: TextIO) -> None: ...
    def run(self, catalog_path: Path, *, output_root: Path) -> SuiteResult: ...
```

Expose `RunController.source_revisions(deadline: float) -> tuple[SourceRevision, ...]` using its current implementation
and `RunController.start(..., on_allocated: Callable[[str, Path], None] | None = None)`
alongside task 5's auxiliary argument. Allocation callback records the active ID
before Compose starts. Restore prior signal handlers after suite execution.
Suite states are `PASSED`, `FAILED`, `SETUP_FAILED`, `INTERRUPTED`, mapping to
exits 0, 1, 2, 130. `to_dict()` writes `result_type: suite_result`, `suite_id`,
`state`, `report_path` as a string, and `exit_code`; print it last in CLI output.

- [ ] **Step 1: Add focused runner/CLI tests using injected controller/runtime fakes.** A successful path launches 11 unique IDs and builds once. Failed calibration launches one case; failed reload launches two; both block later cases. Failed case 6 still reaches case 11. `test_completed_run_with_invalid_artifacts_fails_suite` asserts nonzero overall exit despite completed lifecycle. Changed image/source or unconfirmed teardown blocks remaining launches. `test_startup_interruption_aborts_only_current_allocated_run` preserves report/unrun cases and waits for teardown. `test_suite_lock_spans_different_output_roots` contends on the same lock. Assert CLI final JSON maps passed/failed/setup/interrupted exits to 0/1/2/130.

Define test helper `make_suite(tmp_path, *, failed_case=None)` returning runner,
fake controller and fake runtime. The controller records prepared filename stems
in `started_cases`; the runtime records `build_calls` and independently failed
inspection for the requested case. Other cases return valid frozen evidence.

```python
def test_individual_failure_does_not_skip_remaining_cases(tmp_path):
    runner, controller, runtime = make_suite(tmp_path, failed_case="hover-roll")
    result = runner.run(ROOT / "config/ci-suite.json", output_root=tmp_path / "runs")
    assert len(controller.started_cases) == 11
    assert controller.started_cases[-1] == "competition-realtime"
    assert runtime.build_calls == 1
    assert result.exit_code == 1
```
- [ ] **Step 2: Confirm new runner behavior fails.** `uv run --locked pytest orchestration/tests/test_suite.py orchestration/tests/test_suite_runtime.py orchestration/tests/test_cli.py -q`.
- [ ] **Step 3: Implement the shared runner and production boundary.** Add `suite --config` with default `config/ci-suite.json` and optional absolute `--output-root`. Acquire `/tmp/drone-sim-suite-UID.lock` nonblocking before build, independent of checkout/output root. Use shell-free bounded subprocess calls for the documented seven-image build with exact source revision and a 3,600 s build limit; run task 1's smoke then freeze existing typed source/image records. Start cases through the same controller, passing auxiliary actor only for `operator_wait`; feed the accepted source to all consumers. Use task 4 inspection with the frozen identity and proper rule file. Read persisted teardown diagnostics after return; unconfirmed cleanup blocks later launches. Write report after allocation and each case, preserving separate physical/lifecycle/score/acceptance facts. On signals, request the existing abort for the active ID and permit bounded finalization; stop scheduling. Never kill unrelated projects or release the lock while current cleanup still runs.
- [ ] **Step 4: Verify runner/CLI integration with focused checks.** Repeat step 2 plus changed controller cases. Confirm help lists `suite` and existing commands retain their exit/result behavior. Failed preflight produces a failed setup report without launching; image IDs remain exact across manifests; gate source copies are byte-identical. Update all local commands and current status to reflect implementation but not yet a full flight pass.
- [ ] **Step 5: Commit runnable suite, tests and docs.** Commit message `Run all mission setups with calibration gates and independent reports`.

## Task 8: Add the manual workflow and register this workstation

**Files:** Create `.github/workflows/mission-suite.yml`, `tests/contracts/test_ci_workflow.py`; update runbook/README/dated handoff and `/home/willis/SETUP_REPLICATION.md`. No new runner-management framework or bootstrap script.

**Interfaces:** Workflow filename `mission-suite.yml`; one `suite` job, `workflow_dispatch` only; `runs-on: [self-hosted, linux, x64, drone-sim]`; fixed concurrency group `drone-sim-mission-suite`, `cancel-in-progress: false`. Reuse task 7's CLI. Runner account `willis`, installation `/home/willis/actions-runner-drone-sim`, persistent evidence `/home/willis/projects/drone_sim/runs/ci/GITHUB_RUN_ID-GITHUB_RUN_ATTEMPT`.

- [ ] **Step 1: Add a focused workflow contract test.** Assert manual-only trigger, one workstation job, same suite command/catalog, fixed noncancelling concurrency, sufficient job timeout and `always()` report publication. Do not set the long suite step's separate timeout, whose provider maximum is 360 minutes. Upload paths admit this dispatch's reports, manifests, score results and logs, excluding videos/bags and other dispatches. Use the existing locked PyYAML dependency, handling its YAML boolean `on` convention; no new dependency.

```python
def test_workflow_is_manual_and_does_not_cancel_a_running_suite():
    workflow = yaml.safe_load((ROOT / ".github/workflows/mission-suite.yml").read_text())
    assert set(workflow.get("on", workflow.get(True))) == {"workflow_dispatch"}
    assert workflow["concurrency"]["cancel-in-progress"] is False
    assert workflow["jobs"]["suite"]["timeout-minutes"] == 1440
```
- [ ] **Step 2: Confirm absent workflow fails.** `uv run --locked pytest tests/contracts/test_ci_workflow.py -q`.
- [ ] **Step 3: Write the thin workflow and configure the runner.** Pin official checkout/upload actions to verified releases/SHAs at execution. Use existing `/home/willis/.local/bin/uv`, `uv sync --locked`, then the complete suite with this dispatch's absolute output root. Preserve the suite exit through console capture; publish report.md in the job summary and upload compact allowlisted diagnostics with `always()`. Use `contents: read`, no checkout-persisted credentials. Set `timeout-minutes: 1440`: current catalog has 70,200 s run deadlines plus 3,600 s finalization, leaving room for a bounded build, 11 inspections and upload. Install/register GitHub's official Linux x64 runner as `willis`; use a temporary registration token without echoing or storing it in artifacts/docs. Install its service and label `drone-sim`; update machine setup replication with audit/verification details in this same task. Do not alter Codex environment variables or restart its daemon for this runner.
- [ ] **Step 4: Verify focused contract and runner health.** Repeat step 2; resolve the workflow configuration, check service active and `gh api repos/williserdman/drone_sim/actions/runners` reports the matching online label. Record that job execution is unverified until dispatch. Provider docs permit self-hosted jobs up to five days; do not impose the GitHub-hosted six-hour limit. See [Actions execution limits](https://docs.github.com/en/actions/reference/limits).
- [ ] **Step 5: Commit workflow/tests/operator docs.** Commit message `Add manual workstation mission-suite workflow`. Preserve the machine guide locally without secrets; it is outside this repository.

## Task 9: Demonstrate the full sweep and provider flow

**Files:** Update dated `docs/handoff.md`, runbook verification notes and affected module READMEs only if execution discovers a required behavior fix. Evidence remains ignored under run/suite directories.

**Interfaces:** The same `drone-sim suite` command and reports from tasks 6–8.
No alternative sweep script, handcrafted success report or new scoring policy.

- [ ] **Step 1: Commit the final runtime source and verify the relevant checks.** Run the focused tests listed in the changed tasks, `git diff --check`, and documentation links. Confirm clean source and no active flight/build. Continue in the task-1 isolated CI checkout; preserve prior evidence.
- [ ] **Step 2: Run one complete local suite.** `uv run --locked drone-sim suite --config config/ci-suite.json`. Inspect its final report and each bundle independently through the runner. Target timing totals about 4 h 40 min before build/startup/finalization; actual duration is measured, not guaranteed. Gates must release all nine consumers for an all-setup demonstration.
- [ ] **Step 3: Resolve any required failures without erasing evidence.** For the operator case prove simultaneous primary/operator connections, wait/readback/GUIDED/ARM order, companion-owned remaining plan, 100/100, checksummed operator log and removed containers. Both moving/static cases need continued deck support; both competitions need 150/150. Diagnostics require their safe physical criteria. If roll tuning still lands unsafely, stop calling it passing; retain its failed bundle and make a focused native-LAND/save fix based on the existing all-axis sequence, with a failing mission-state test first. Rebuild from committed changed source and rerun affected behavior; rerun the complete catalog when necessary to establish a final common-build pass. After three failed fixes, stop and name the doubtful assumption.
- [ ] **Step 4: Exercise the manual workflow once its definition is on main.** First make the complete local implementation/report reviewable in a PR; merge requires the user's instruction and is not pre-authorized by this plan. GitHub cannot dispatch a new workflow absent from the default branch. See [manual dispatch requirements](https://docs.github.com/en/actions/how-tos/manage-workflow-runs/manually-run-a-workflow). After it is present, run `gh workflow run mission-suite.yml --ref BRANCH`, select/watch that dispatch, verify the same full catalog and retained recordings, summary/upload, and correct terminal result. Before this step is possible, report Actions dispatch as unrun, not passing. No temporary automatic trigger is a substitute.
- [ ] **Step 5: Commit the dated verification/handoff update.** Record source/image identity, suite/run IDs, physical outcomes, raw scores, independent acceptance, failed/blocked/unrun cases and local video paths. Separate local sweep and GitHub dispatch evidence. Keep all failed attempts. Once delivered, remove this superseded plan from active docs while retaining Git history and current guides. No unsolicited review cycle or broad repeat tests.

## Execution handoff and deferred work

Approved 2026-10-05; executing on `feat/manual-ci-suite`. Recommend native
execution with bounded independent delegation, following the user's MVP rule
against unsolicited review cycles. The explicit `$implement` request adds one
final code review. Use `superpowers:executing-plans` for root implementation;
delegation does not authorize speculative refactoring or hardening.

Defer cache/promotion, parallel jobs, scheduled/PR triggers, video hosting,
automatic retention, general operator tooling and performance optimization.
Workflow availability on main is the provider dependency, not a reason to delay
making the local suite, workflow patch and evidence concrete.

Provider references: [manual dispatch and default-branch requirement](https://docs.github.com/en/actions/how-tos/manage-workflow-runs/manually-run-a-workflow),
[official runner setup](https://docs.github.com/en/actions/how-tos/manage-runners/self-hosted-runners/add-runners),
[Actions execution limits](https://docs.github.com/en/actions/reference/limits),
[job and step timeout syntax](https://docs.github.com/en/actions/reference/workflows-and-actions/workflow-syntax#jobsjob_idtimeout-minutes).
