# CI calibration implementation plan

Status 2026-10-02: implementation delivered, final two-run acceptance incomplete.
Calibration passed at 100/100; the fresh-process consumer landed but scored 80/100
because stable contact included impact motion. The approved versioned settling
policy is implemented; recorded replay and fresh validation remain pending.
See [current evidence and limits](../../handoff.md). Retain this plan until the
final acceptance requirement is resolved.

> Execution: use `superpowers:executing-plans` for integration, with bounded
> subagents for independent model and scoring changes. No review cycles unless
> requested, following AGENTS.md. Check off each step after its evidence exists.

**Goal:** Produce accepted roll/pitch/yaw gains, reload them into fresh SITL, and
pass a configured hover/landing validation on the shared competition airframe.

**Architecture:** Companion controls tuning and native landing. Scorekeeper checks
physical landing; artifacts verifies saved gains and provenance. Orchestration
freezes accepted calibration input; SITL loads it and companion verifies readback.

**Tech stack:** Existing Python/pytest, SDF generator, MAVLink, ROS 2, Gazebo and
pinned ArduPilot containers. Reuse `pymavlink==2.4.49` in the artifacts image
for independent DataFlash validation; no new service.

**Spec:** [CI calibration design](../../architecture.md#ci-calibration-design).
This plan implements delivery steps 1–2. Automated whole-suite dependencies and
Comp2026 calibration consumption are delivery step 3, after this milestone.

## Constraints

- Preserve historical recordings, existing rule identities, and unrelated dirty
  README/runbook/handoff edits. Work in `design/moving-pad-landing`.
- One competition airframe, with mission-specific payloads. Calibration is
  unloaded. Keep current vehicle IDs, public topics, camera recording contracts,
  scoped contacts, and ArduPilot firmware revision.
- Use `AUTOTUNE_AXES=7`, normal prearm checks, native LOITER/LAND, and observed
  completion. Recovery never changes a failed result into success.
- Export only the 15 tuned values; preserve yaw D and other untuned baseline
  settings. Freeze provenance and verify gains before a dependent mission arms.
- Run focused checks and a fresh calibration/validation pair. Broader mission
  compatibility remains unverified until those missions are flown with the gains.

## Failure cases to prove

| Input or condition | Required result | Task |
| --- | --- | --- |
| Only roll or pitch completes | No accepted calibration artifact | 2–3 |
| Auxiliary command ACKs but gains never activate | No successful LAND/save transition | 3 |
| Unrelated parameter writes or old DataFlash records | Reject mixed/stale saved gains | 2–4 |
| Changed source bundle, aircraft, firmware, or parameter bytes | Reject import before launch | 5 |
| Loaded gains differ, or yaw D is validly zero | Block mismatches; allow preserved zero yaw D | 2, 5 |

## 1. Generate one physical airframe

**Files:** `gazebo/scripts/prepare_competition_assets.py`; new
`gazebo/scripts/templates/iris_competition_vehicle.sdf`; generated
`gazebo/resources/models/{iris_flight,iris_moving_pad,iris_competition}/model.sdf`
and `model.config`; `gazebo/tests/test_{competition_assets,flight_resources,moving_pad_resources}.py`;
`gazebo/README.md`.

**Interface:** `build_vehicle(source_root: Path, scenario: ScenarioConfig, *,
name: str, payloads: tuple[Payload, ...]) -> ET.Element`. Consume the canonical
motor/IMU template and existing `iris_phase3` geometry. Produce matching physical
bodies under the existing three names; retain variant-specific publishers outside
the common physical construction so the diagnostic world does not gain a second
pose publisher.

- [ ] Add generator tests comparing body/rotor inertials, motor limits, sensor
  hardware and hardpoint across all outputs: empty mass 1.66001 kg, motor force
  limits ±3.4, original joint/leg collision names. Only competition gets payload
  coordinator/joint plugins, with payload 2 initially attached and 3/4 detached.
- [ ] Run the three named test files and observe the present mass/limit mismatch.
- [ ] Implement the common builder, regenerate deterministic assets, and update
  existing assertions that moving-pad has no hardpoint. Keep world URIs and
  recording-source topics unchanged; no world geometry or sensor calibration edit.
- [ ] Rerun those tests plus `gazebo/tests/test_world_resources.py`; regenerate
  twice and require no second diff. Document the common physical profile.
- [ ] Commit only this task's files: `Unify scenario aircraft dynamics`.

Check command: `uv run --locked pytest gazebo/tests/test_competition_assets.py gazebo/tests/test_flight_resources.py gazebo/tests/test_moving_pad_resources.py gazebo/tests/test_world_resources.py -q`.
The new `test_all_variants_share_unloaded_physics` must compare each generated
physical signature to competition's unloaded signature. All selected tests pass.

## 2. Define the all-axis parameter artifact

**Files:** new `artifacts/src/artifacts/calibration.py` and
`artifacts/tests/test_calibration.py`; `artifacts/pyproject.toml`, `uv.lock`;
`companion/src/drone_sim_companion/autotune.py`;
`companion/tests/test_autotune.py`; affected artifacts/companion READMEs.

**Interfaces:** `validate_calibration_parameters(values: Mapping[str, float]) ->
dict[str, float]`; `read_calibration_parameters(path: Path) -> tuple[str, dict[str, float]]`;
`write_calibration_parameters(run_directory: Path, run_id: str, values:
Mapping[str, float]) -> Path`. Output `ardupilot_sitl/autotune.parm`, with a
versioned header, canonical run UUID, axis mask 7, and deterministic parameter order.
Also provide `read_saved_calibration_parameters(run_directory: Path) ->
dict[str, float]` in the artifacts module, consumed by companion and acceptance.
Keep the old roll-only parser and artifact readable for historical bundles.

- [ ] Add real round-trip/rejection tests for exactly five keys per axis:
  RLL/PIT rate P/I/D, angle P and acceleration; YAW rate P/I/FLTE, angle P and
  acceleration. Require finite positive tuned values, roll/pitch I=P and yaw
  I=0.1×P using `math.isclose(rel_tol=1e-5, abs_tol=1e-7)` for parameter comparisons.
  Yaw D is a baseline/readback check,
  not a sixteenth tuned output.
- [ ] Run the new tests and observe missing implementation failures.
- [ ] Implement canonical parsing/writing and all-axis DataFlash extraction.
  Require all-axis saved status/event and a complete associated parameter-save
  epoch. Do not collect arbitrary latest values across different epochs or assume
  separate axis saves have identical microsecond timestamps without evidence.
  Compare saved values to explicit post-disarm PARAM_VALUE readback.
  Reuse the existing pinned MAVLink dependency in artifacts and refresh `uv.lock`.
- [ ] Rerun the artifact tests and `companion/tests/test_autotune.py`, including
  old roll-artifact cases; document the format beside its entry point.
- [ ] Commit: `Add validated all-axis calibration artifacts`.

Check command: `uv run --locked pytest artifacts/tests/test_calibration.py companion/tests/test_autotune.py -q`.
Name the rejection cases `test_rejects_partial_axes`,
`test_rejects_mixed_save_epochs` and `test_preserves_zero_baseline_yaw_d`.
Each invalid case raises `ValueError`; valid round trips preserve all 15 values.

## 3. Fly AutoTune and native landing

**Files:** `companion/src/drone_sim_companion/{autotune,runtime_node}.py`;
`companion/tests/test_{autotune,runtime_node}.py`; new
`config/autotune-run.json`; companion README and runbook.

**Interface:** Add mission `autotune` alongside historical `autotune_roll`.
Extend the existing pure observation/action driver with auxiliary-command ACKs,
fresh position/velocity/attitude samples and parameter replies. New phases cover
LOITER before tuning, LOITER after success, gain activation, settling, LAND, and
post-disarm verification. Shared runtime plumbing keeps `hover_roll` working.

- [ ] Test the real phase sequence and negative branches: GUIDED → LOITER →
  AUTOTUNE → LOITER → aux function 180/HIGH → settle → LAND → disarm/save.
  Neither `AutoTune: Success` nor ACK alone completes gain activation. Require
  `AutoTune: Pilot Testing gains for Roll Pitch Yaw(E)` and matching readback;
  require `AutoTune: Saved gains for Roll Pitch Yaw(E)` before export.
- [ ] Observe failures, then implement. Require live `ATC_RATE_FF_ENAB=1` and
  axes-mask readback. Neutral sticks during tune/settle; clear overrides for LAND.
  Settle for two continuous simulation seconds with horizontal speed ≤0.2 m/s,
  vertical speed ≤0.2 m/s and roll/pitch within 5°, using samples no older than
  0.5 s. Invalid or stale samples reset the settle window.
- [ ] Bound mode/ACK waits to 10 s, settling to 20 s, and landing to 45 s.
  Reserve the final 60 s of the public window for landing/failure handling.
  Initial template: 600 s public, 90 s warmup, target RTF 0.25; wall limits
  7200/900/300 s for overall/startup/finalization. These are budgets, not runtime
  estimates. Deadline failure stays failed even if native LAND recovery succeeds.
- [ ] Rerun focused companion/runtime tests and test the real action adapter's
  MAVLink encoding, ACK/status ordering and post-disarm reads. Update mission
  documentation; keep private Gazebo truth out of flight decisions.
- [ ] Commit: `Land and save all-axis AutoTune gains natively`.

Check command: `uv run --locked pytest companion/tests/test_autotune.py companion/tests/test_runtime_node.py -q`.
`test_aux_ack_without_testing_evidence_cannot_complete` must emit no successful
save/export action. `test_native_land_saves_all_axes` must emit one export only
after disarm, saved status and matching parameter evidence. All selected tests pass.

## 4. Accept calibration for its actual purpose

**Files:** new `scorekeeper/rules/calibration_v1.json`, pure
`scorekeeper/src/drone_sim_scorekeeper/calibration.py`, `calibration_runtime.py`;
`scorekeeper/src/drone_sim_scorekeeper/runtime_node.py`; new
`artifacts/src/artifacts/calibration_score_validation.py`; existing artifacts
`acceptance.py`, `score_validation.py`, `runtime_configuration.py`, `session.py`,
`_adapters/rosbag.py`; orchestration score-result dispatch; focused tests in these
modules and `tests/contracts/test_calibration_interfaces.py`;
scorekeeper/artifacts READMEs.

**Interfaces:** `CalibrationScorer` follows existing scorer `accept/finalize`
contracts over `GroundTruthSample`. Physical rules are airborne/contact 20,
safe preimpact speed 40, stable contact 40; copy the approved descent thresholds
without a position-radius rule. Calibration artifact acceptance is an additional
required gate, not points awarded for a companion success message.
`validate_calibration_artifact(run_directory: Path) -> dict[str, float]` in
`artifacts/calibration.py` rejects a wrong mission/ruleset, incomplete-axis
evidence, missing disarm, an uninventoried artifact, or mismatched saved/readback
values. It returns the validated 15 gains for acceptance and import.

- [ ] Add two distinguishing fixtures: landing far from origin at 0.5 m/s passes
  calibration while failing descent precision; landing at 1.052 m/s fails
  calibration's speed rule. Missing contact/grid evidence fails closed.
- [ ] Observe failures, then wire the new ruleset through every runtime and
  independent bag/score validator. Rotate recorded body velocity to world for
  calibration vertical-speed checks; independently test a tilted touchdown.
  Leave historical `descent_v1`/moving-pad replay semantics unchanged.
- [ ] Make log validation select the actual mission. AutoTune requires all-axis
  tune/testing/save and landed/disarmed evidence; configured validation requires
  the ordered successful configured operations. Retain controlled-descent's
  existing command/event checks. Verify exported gains against saved DataFlash
  evidence and parameter readback, and inventory the new artifact with its hash.
- [ ] Run focused scorer, recording-contract, score-validation and acceptance
  tests, including old-rule fixtures. Confirm max-score acceptance rejects an
  unsafe landing even when gains were saved. Update module guides.
- [ ] Commit: `Add independent calibration acceptance`.

Check command: `uv run --locked pytest scorekeeper/tests/test_calibration.py scorekeeper/tests/test_calibration_runtime.py scorekeeper/tests/test_descent_score.py artifacts/tests/test_calibration_score_validation.py artifacts/tests/test_score_validation.py artifacts/tests/test_acceptance.py artifacts/tests/test_rosbag_adapter.py tests/contracts/test_calibration_interfaces.py -q`.
The new physical fixtures assert scores 100 and 60 respectively. ROS-only skipped
cases remain unverified until the rebuilt artifacts container runs acceptance.

## 5. Reload accepted gains and prove the first milestone

**Files:** orchestration `config.py`/`controller.py`, both run-configuration
schemas `config/run-template.schema.json` and `config/run.schema.json`, and their
tests; artifacts `calibration.py`/inventory validation;
ArduPilot `config.py`/`runtime_node.py` and tests; companion
`configured_runtime.py`/`runtime_node.py` and configured-host tests; new
`config/calibration-validation-run.json`; affected READMEs, runbook and handoff.

**Interfaces:** Optional template `calibration: {"source_run_directory": "..."}`
is resolved relative to the template. Only `mission: configured` consumes this
input in the first milestone; reject other consumers explicitly. Extend frozen
RunConfig using the existing immutable-JSON pattern. Before Compose starts, call
`inspect_phase3_via_container(..., require_maximum_score=True)` on the source and
freeze its run/manifest/parameter digests, airframe physical-profile identity,
base parameter hashes, firmware revision and image identities.
The resolved object stores `schema_version: 1`, `source_run_id`,
`source_manifest_sha256`, `source_artifact_sha256`, `gains`, and `profile`;
the external source path is absent. Parameter hashes cover the base and each
scenario overlay in load order; compare preserved baseline settings after
overlay resolution, allowing unrelated mission-specific settings to differ.
For this pair, both templates use the diagnostic world and generated `iris_flight`.
The profile binds `id: competition-unloaded-v1`, that model's SHA-256 from
`resolve_world().resource_sha256s`, the base parameter SHA-256, the revision in
`ardupilot_sitl/provenance/ardupilot.json`, and Gazebo/SITL image digests. Hash its
sorted compact JSON as `profile_sha256`; require exact compatibility before
launch. Preserve the source's full image provenance separately. Generalizing
imports to other vehicle names belongs to delivery step 3.

- [ ] Test changed source bytes, wrong aircraft/firmware/base identity, missing
  acceptance, unknown parameter keys and overlay precedence before implementing.
  A rejected source must never call Compose up. Preserve read-only old configs.
- [ ] Copy the accepted parameters and source manifest into
  `configuration/calibration.parm` and `configuration/calibration-manifest.json`;
  bind their hashes and source identity into resolved configuration and manifest
  inventory. Check again before copying to close the validation/copy race.
  SITL loads base, scenario overlay, then calibration; no tracked parameter edits.
- [ ] Extend `ConfiguredHost`'s existing parameter gate with the 15 gains and
  preserved baseline settings. Request fresh PARAM_VALUE replies and block the
  arm operation until they match; missing/mismatched replies fail with a deadline.
  Test this at the real host/operation seam, including normal moving-profile gates.
- [ ] Add the validation plan: GUIDED, arm, takeoff 5 m, hold 10 s, native land.
  Require the existing descent score 100/100 and independent acceptance. During
  that hold, independent recorded truth must show at least five continuous seconds
  within 0.5 m of commanded altitude, horizontal/vertical speed ≤0.2 m/s, and
  roll/pitch within 5°. A configured hold completing alone cannot pass this gate.
  Add `test_validation_rejects_unstable_hover_despite_safe_landing`. Document
  how to copy the template and set its source-run field. Run focused checks for
  all touched modules, then commit matching runtime source and rebuild all seven
  images, retaining previous image digests and run evidence.
- [ ] Fly and independently accept one new calibration and its fresh validation
  run. Check live gains, saved gains, reload gains and hashes agree. Preserve both
  run IDs and report physical outcome, score and artifact acceptance separately.
  Record any failures as failures; do not change thresholds to obtain a pass.

Check command: `uv run --locked pytest orchestration/tests/test_config.py orchestration/tests/test_controller.py ardupilot_sitl/tests/test_config.py ardupilot_sitl/tests/test_runtime_node.py companion/tests/test_configured_host.py companion/tests/test_configured_live_loop.py artifacts/tests/test_calibration.py artifacts/tests/test_acceptance.py -q`.
Verify mismatched readback emits zero arm commands and the calibration overlay is
last in SITL's `--defaults` argument. After committing runtime source, build with
`SIM_COMP2026_REVISION=$(git rev-parse HEAD) docker compose --profile phase3 build`.
Launch with `uv run --locked drone-sim start --config config/autotune-run.json`;
then copy the validation template, set its source directory to that run, and launch
the copy with the same CLI. Run the runbook's independent acceptance command with
`calibration_v1.json` and `descent_v1.json` respectively. Expect both reports to
pass with matching captured source/image provenance and no active run resources.

## Completion and deferred work

The first milestone is complete only with that accepted two-run chain. A shared
airframe changes other scenarios; record their status as unrun under the new
profile, not inherited passes. Full-suite dependency execution, Comp2026 startup
readback integration, its known image-packaging failure, CI-provider setup,
caching and parallel scheduling remain follow-up work. Replace superseded current
guide text and remove this working plan from the active docs after execution,
retaining it in Git history.
