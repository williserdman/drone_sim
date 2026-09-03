from __future__ import annotations

import ast
from dataclasses import FrozenInstanceError
import math
from pathlib import Path

import pytest

from drone_sim_gazebo.ros_adapter import AdapterSummary
from drone_sim_gazebo.runtime import (
    ActivateOutput,
    AdapterCompleted,
    ArtifactsReady,
    BeginFinalization,
    ChildExited,
    EndpointTimeout,
    FinalizationRequested,
    GazeboReady,
    PublishGazeboReady,
    RequestSteps,
    RunStateEvent,
    RuntimeModel,
    RuntimeModelError,
    ServerStopFailed,
    ServerStopped,
    SetPaused,
    StopServer,
    WriteQuiescence,
    WriteRuntimeFailure,
    WriteSourceFinished,
)
from drone_sim_gazebo.server import NativeArtifactSummary


RUN_ID = "00000000-0000-4000-8000-000000000505"
STALE_RUN_ID = "00000000-0000-4000-8000-000000000999"
INTERVAL_NS = 50_000_000


def _summary(frames: int = 2, **changes: object) -> AdapterSummary:
    values: dict[str, object] = {
        "onboard_frames": frames,
        "observer_frames": frames,
        "paired_frames": frames,
        "ground_truth_samples": frames,
        "first_sim_timestamp_ns": INTERVAL_NS if frames else None,
        "last_sim_timestamp_ns": frames * INTERVAL_NS if frames else None,
    }
    values.update(changes)
    return AdapterSummary(**values)  # type: ignore[arg-type]


def _native_summary(
    tmp_path: Path,
    *,
    run_id: str = RUN_ID,
    returncode: int = 0,
    graceful: bool = True,
):
    run = tmp_path / run_id
    state = run / "gazebo/state/state.tlog.zst"
    log = run / "gazebo/server.log"
    state.parent.mkdir(parents=True, exist_ok=True)
    log.write_bytes(b"log")
    state.write_bytes(b"state")
    return NativeArtifactSummary(log, state, returncode, graceful)


def _running_model(*, expected_frames: int = 2) -> RuntimeModel:
    model = RuntimeModel(run_id=RUN_ID, expected_frames=expected_frames)
    assert model.accept(ArtifactsReady(RUN_ID)) == ()
    assert model.accept(GazeboReady(RUN_ID)) == (PublishGazeboReady(),)
    assert model.accept(RunStateEvent(RUN_ID, "READY")) == (SetPaused(False),)
    assert model.accept(RunStateEvent(RUN_ID, "RUNNING")) == (ActivateOutput(),)
    return model


@pytest.mark.parametrize("first", [ArtifactsReady, GazeboReady])
def test_ready_unpauses_private_warmup_and_running_only_activates_public_output(first):
    model = RuntimeModel(run_id=RUN_ID, expected_frames=40)
    second = GazeboReady if first is ArtifactsReady else ArtifactsReady

    assert model.accept(first(RUN_ID)) == ()
    assert model.accept(first(RUN_ID)) == ()
    assert model.accept(second(RUN_ID)) == (PublishGazeboReady(),)
    assert model.accept(second(RUN_ID)) == ()
    assert model.accept(RunStateEvent(RUN_ID, "READY")) == (SetPaused(False),)
    assert model.accept(RunStateEvent(RUN_ID, "READY")) == ()
    running_actions = model.accept(RunStateEvent(RUN_ID, "RUNNING"))
    assert running_actions == (ActivateOutput(),)
    assert not any(isinstance(action, SetPaused) for action in running_actions)
    assert model.accept(RunStateEvent(RUN_ID, "RUNNING")) == ()


def test_stale_run_facts_cannot_advance_readiness_completion_or_quiescence(tmp_path: Path):
    model = RuntimeModel(run_id=RUN_ID, expected_frames=2)
    stale_native = _native_summary(tmp_path)

    assert model.accept(ArtifactsReady(STALE_RUN_ID)) == ()
    assert model.accept(GazeboReady(STALE_RUN_ID)) == ()
    assert model.accept(RunStateEvent(STALE_RUN_ID, "READY")) == ()
    assert model.accept(AdapterCompleted(STALE_RUN_ID, _summary())) == ()
    assert model.accept(ServerStopped(STALE_RUN_ID, stale_native)) == ()
    assert model.accept(ArtifactsReady(RUN_ID)) == ()
    assert model.accept(GazeboReady(RUN_ID)) == (PublishGazeboReady(),)


def test_premature_running_fails_once_and_never_releases_progress_later():
    model = RuntimeModel(run_id=RUN_ID, expected_frames=2)

    actions = model.accept(RunStateEvent(RUN_ID, "RUNNING"))

    assert actions == (
        WriteRuntimeFailure(
            "RUNNING received before the controlled readiness step",
            ("gazebo/server.log.partial",),
        ),
        BeginFinalization(
            "FAILED",
            "RUNNING received before the controlled readiness step",
        ),
    )
    assert model.accept(ArtifactsReady(RUN_ID)) == ()
    assert model.accept(GazeboReady(RUN_ID)) == ()
    assert model.accept(AdapterCompleted(RUN_ID, _summary())) == ()
    assert model.accept(FinalizationRequested(RUN_ID, "COMPLETED", "late success", 30.0)) == (
        StopServer(30.0),
    )


def test_exact_frozen_adapter_completion_pauses_before_source_finished():
    model = _running_model(expected_frames=2)

    actions = model.accept(AdapterCompleted(RUN_ID, _summary()))

    assert actions == (
        SetPaused(True),
        WriteSourceFinished(100_000_000),
    )
    assert model.accept(AdapterCompleted(RUN_ID, _summary())) == ()


@pytest.mark.parametrize(
    ("summary", "reason_fragment"),
    [
        (_summary(3), "exceeds"),
        (_summary(2, observer_frames=1), "aligned"),
        (_summary(2, ground_truth_samples=1), "aligned"),
        (_summary(2, first_sim_timestamp_ns=1), "timestamp"),
        (_summary(2, last_sim_timestamp_ns=99_999_999), "timestamp"),
        (_summary(1), "exactly 2"),
    ],
    ids=["overrun", "camera-mismatch", "truth-mismatch", "wrong-first", "wrong-last", "partial"],
)
def test_invalid_or_incomplete_adapter_summary_fails_closed(summary: AdapterSummary, reason_fragment: str):
    model = _running_model(expected_frames=2)

    actions = model.accept(AdapterCompleted(RUN_ID, summary))

    assert isinstance(actions[0], WriteRuntimeFailure)
    assert reason_fragment in actions[0].reason
    assert actions[1] == SetPaused(True)
    assert isinstance(actions[2], BeginFinalization)
    assert model.accept(AdapterCompleted(RUN_ID, _summary())) == ()


def test_child_exit_is_first_failure_and_later_timeout_cannot_replace_it():
    model = _running_model()

    actions = model.accept(ChildExited(RUN_ID, "server", 17))

    assert actions == (
        WriteRuntimeFailure(
            "runtime child server exited unexpectedly with return code 17",
            ("gazebo/server.log.partial", "gazebo/state"),
        ),
        SetPaused(True),
        BeginFinalization(
            "FAILED",
            "runtime child server exited unexpectedly with return code 17",
        ),
    )
    assert model.accept(EndpointTimeout(RUN_ID, "world-control")) == ()
    assert model.accept(ChildExited(RUN_ID, "adapter", 99)) == ()


@pytest.mark.parametrize("child_name", ["server", "bridge", "image_bridge", "adapter"])
def test_each_canonical_child_identity_is_preserved_in_first_failure(child_name: str):
    model = RuntimeModel(run_id=RUN_ID, expected_frames=2)

    actions = model.accept(ChildExited(RUN_ID, child_name, -9))

    assert isinstance(actions[0], WriteRuntimeFailure)
    assert actions[0].reason == (
        f"runtime child {child_name} exited unexpectedly with return code -9"
    )
    assert isinstance(actions[-1], BeginFinalization)


@pytest.mark.parametrize(
    "child_name",
    ["", "Bridge", "image-bridge", "_adapter", "a" * 65, "bridge/path"],
)
def test_child_identity_rejects_noncanonical_or_unbounded_names(child_name: str):
    with pytest.raises(ValueError, match="child_name"):
        ChildExited(RUN_ID, child_name, 1)


def test_endpoint_timeout_is_an_infrastructure_fact_not_simulation_progress():
    model = RuntimeModel(run_id=RUN_ID, expected_frames=2)

    actions = model.accept(EndpointTimeout(RUN_ID, "camera/onboard"))

    assert actions == (
        WriteRuntimeFailure(
            "Gazebo endpoint readiness timed out: camera/onboard",
            ("gazebo/server.log.partial",),
        ),
        BeginFinalization(
            "FAILED",
            "Gazebo endpoint readiness timed out: camera/onboard",
        ),
    )
    assert all(not isinstance(action, (RequestSteps, WriteSourceFinished)) for action in actions)


def test_finalizing_lifecycle_preempts_completion_then_request_uses_one_absolute_deadline():
    model = _running_model()

    assert model.accept(RunStateEvent(RUN_ID, "FINALIZING")) == (SetPaused(True),)
    assert model.accept(AdapterCompleted(RUN_ID, _summary())) == ()
    assert model.accept(FinalizationRequested(RUN_ID, "ABORTED", "operator stop", 42.5)) == (
        BeginFinalization("ABORTED", "operator stop"),
        StopServer(42.5),
    )
    assert model.accept(FinalizationRequested(RUN_ID, "ABORTED", "operator stop", 42.5)) == ()


def test_direct_finalization_pauses_before_beginning_and_stopping_server():
    model = _running_model()

    assert model.accept(FinalizationRequested(RUN_ID, "FAILED", "recorder fault", 50.0)) == (
        SetPaused(True),
        BeginFinalization("FAILED", "recorder fault"),
        StopServer(50.0),
    )


def test_completed_finalization_fails_closed_before_exact_source_finished():
    model = RuntimeModel(run_id=RUN_ID, expected_frames=2)

    actions = model.accept(
        FinalizationRequested(RUN_ID, "COMPLETED", "controller complete", 50.0)
    )

    reason = "COMPLETED finalization requested before exact source-finished"
    assert actions == (
        WriteRuntimeFailure(reason, ("gazebo/server.log.partial",)),
        BeginFinalization("FAILED", reason),
        StopServer(50.0),
    )
    assert all(
        not isinstance(action, BeginFinalization)
        or action.requested_terminal != "COMPLETED"
        for action in actions
    )


def test_completed_finalization_requires_exact_adapter_summary_then_succeeds():
    model = _running_model()
    assert model.accept(AdapterCompleted(RUN_ID, _summary())) == (
        SetPaused(True),
        WriteSourceFinished(100_000_000),
    )

    assert model.accept(
        FinalizationRequested(RUN_ID, "COMPLETED", "complete", 50.0)
    ) == (
        BeginFinalization("COMPLETED", "complete"),
        StopServer(50.0),
    )


def test_partial_adapter_failure_cannot_be_repaired_by_completed_finalization():
    model = _running_model()
    first = model.accept(AdapterCompleted(RUN_ID, _summary(1)))
    assert isinstance(first[0], WriteRuntimeFailure)
    assert first[-1].requested_terminal == "FAILED"  # type: ignore[union-attr]

    assert model.accept(
        FinalizationRequested(RUN_ID, "COMPLETED", "late success", 50.0)
    ) == (StopServer(50.0),)


def test_server_stop_failure_is_typed_first_failure_and_forbids_quiescence(
    tmp_path: Path,
):
    model = _running_model()
    assert model.accept(
        FinalizationRequested(RUN_ID, "FAILED", "controller fault", 50.0)
    ) == (
        SetPaused(True),
        BeginFinalization("FAILED", "controller fault"),
        StopServer(50.0),
    )

    actions = model.accept(
        ServerStopFailed(
            RUN_ID,
            "native state.tlog validation failed",
            ("gazebo/server.log.partial", "gazebo/state"),
        )
    )

    assert actions == (
        WriteRuntimeFailure(
            "native state.tlog validation failed",
            ("gazebo/server.log.partial", "gazebo/state"),
        ),
    )
    assert model.accept(ServerStopped(RUN_ID, _native_summary(tmp_path))) == ()
    assert model.accept(
        ServerStopFailed(RUN_ID, "later failure", ("gazebo/server.log",))
    ) == ()


def test_prior_runtime_failure_still_allows_valid_native_quiescence(tmp_path: Path):
    model = _running_model()
    model.accept(ChildExited(RUN_ID, "bridge", 17))
    assert model.accept(
        FinalizationRequested(RUN_ID, "FAILED", "child failed", 50.0)
    ) == (StopServer(50.0),)
    native = _native_summary(tmp_path)

    assert model.accept(ServerStopped(RUN_ID, native)) == (WriteQuiescence(native),)


def test_wrong_run_native_summary_latches_stop_failure_and_blocks_repair(
    tmp_path: Path,
):
    model = RuntimeModel(run_id=RUN_ID, expected_frames=2)
    model.accept(FinalizationRequested(RUN_ID, "ABORTED", "operator stop", 30.0))
    wrong = _native_summary(tmp_path, run_id=STALE_RUN_ID)

    actions = model.accept(ServerStopped(RUN_ID, wrong))

    assert actions == (
        WriteRuntimeFailure(
            "native artifact summary belongs to another run",
            ("gazebo/server.log.partial", "gazebo/state"),
        ),
    )
    assert model.accept(ServerStopped(RUN_ID, wrong)) == ()
    assert model.accept(ServerStopped(RUN_ID, _native_summary(tmp_path))) == ()


@pytest.mark.parametrize("tamper", ["state-path", "graceful-value"])
def test_malformed_native_summary_latches_stop_failure(
    tmp_path: Path,
    tamper: str,
):
    model = RuntimeModel(run_id=RUN_ID, expected_frames=2)
    model.accept(FinalizationRequested(RUN_ID, "ABORTED", "operator stop", 30.0))
    native = _native_summary(tmp_path)
    if tamper == "state-path":
        object.__setattr__(
            native,
            "state_log_path",
            native.state_log_path.with_name("replacement.tlog"),
        )
    else:
        object.__setattr__(native, "graceful", 1)

    actions = model.accept(ServerStopped(RUN_ID, native))

    assert actions == (
        WriteRuntimeFailure(
            "native artifact summary is malformed",
            ("gazebo/server.log.partial", "gazebo/state"),
        ),
    )
    assert model.accept(ServerStopped(RUN_ID, _native_summary(tmp_path))) == ()


def test_prior_first_failure_remains_diagnostic_but_bad_native_evidence_blocks_repair(
    tmp_path: Path,
):
    model = _running_model()
    first = model.accept(ChildExited(RUN_ID, "bridge", 17))
    model.accept(FinalizationRequested(RUN_ID, "FAILED", "child failed", 30.0))

    assert model.accept(
        ServerStopped(RUN_ID, _native_summary(tmp_path, run_id=STALE_RUN_ID))
    ) == ()
    assert model.accept(ServerStopped(RUN_ID, _native_summary(tmp_path))) == ()
    assert first[0] == WriteRuntimeFailure(
        "runtime child bridge exited unexpectedly with return code 17",
        ("gazebo/server.log.partial", "gazebo/state"),
    )


def test_premature_server_stopped_evidence_cannot_be_repaired_later(tmp_path: Path):
    model = RuntimeModel(run_id=RUN_ID, expected_frames=2)
    native = _native_summary(tmp_path)

    first = model.accept(ServerStopped(RUN_ID, native))
    assert isinstance(first[0], WriteRuntimeFailure)
    assert model.accept(
        FinalizationRequested(RUN_ID, "FAILED", "invalid stop order", 30.0)
    ) == (StopServer(30.0),)
    assert model.accept(ServerStopped(RUN_ID, native)) == ()


def test_quiescence_requires_stop_request_and_valid_native_summary(tmp_path: Path):
    native = _native_summary(tmp_path, returncode=-9, graceful=False)
    model = RuntimeModel(run_id=RUN_ID, expected_frames=2)

    premature = model.accept(ServerStopped(RUN_ID, native))
    assert isinstance(premature[0], WriteRuntimeFailure)
    assert all(not isinstance(action, WriteQuiescence) for action in premature)

    model = RuntimeModel(run_id=RUN_ID, expected_frames=2)
    model.accept(FinalizationRequested(RUN_ID, "ABORTED", "operator stop", 30.0))
    assert model.accept(ServerStopped(RUN_ID, native)) == (WriteQuiescence(native),)
    assert model.accept(ServerStopped(RUN_ID, native)) == ()


def test_post_quiescence_freeze_rejects_new_input_but_terminal_duplicates_are_idempotent(tmp_path: Path):
    native = _native_summary(tmp_path)
    model = RuntimeModel(run_id=RUN_ID, expected_frames=2)
    model.accept(FinalizationRequested(RUN_ID, "ABORTED", "operator stop", 30.0))
    model.accept(ServerStopped(RUN_ID, native))

    assert model.accept(ServerStopped(RUN_ID, native)) == ()
    with pytest.raises(RuntimeModelError, match="frozen"):
        model.accept(
            ServerStopped(
                RUN_ID,
                _native_summary(tmp_path, returncode=-9, graceful=False),
            )
        )
    assert model.accept(RunStateEvent(RUN_ID, "FINALIZING")) == ()
    assert model.accept(RunStateEvent(RUN_ID, "ABORTED")) == ()
    with pytest.raises(RuntimeModelError, match="frozen"):
        model.accept(ArtifactsReady(RUN_ID))


@pytest.mark.parametrize(
    "factory",
    [
        lambda: RuntimeModel(run_id="bad", expected_frames=2),
        lambda: RuntimeModel(run_id=RUN_ID, expected_frames=True),
        lambda: RuntimeModel(run_id=RUN_ID, expected_frames=0),
        lambda: ChildExited(RUN_ID, "server", True),
        lambda: ServerStopFailed(RUN_ID, "reason", ("/absolute",)),
        lambda: FinalizationRequested(RUN_ID, "FAILED", "reason", math.inf),
        lambda: FinalizationRequested(RUN_ID, "COMPLETED", "", 10.0),
        lambda: RequestSteps(True),
        lambda: StopServer(math.nan),
        lambda: WriteRuntimeFailure("reason", ("/absolute",)),
        lambda: WriteRuntimeFailure("reason", ("../escape",)),
    ],
)
def test_runtime_values_reject_noncanonical_numbers_terminal_data_and_paths(factory):
    with pytest.raises((TypeError, ValueError)):
        factory()


def test_runtime_actions_events_and_native_summary_are_frozen(tmp_path: Path):
    values = [
        ActivateOutput(),
        PublishGazeboReady(),
        RequestSteps(1),
        SetPaused(True),
        WriteSourceFinished(100_000_000),
        WriteRuntimeFailure("reason", ("gazebo/server.log.partial",)),
        BeginFinalization("FAILED", "reason"),
        StopServer(30.0),
        WriteQuiescence(_native_summary(tmp_path)),
        ArtifactsReady(RUN_ID),
        AdapterCompleted(RUN_ID, _summary()),
    ]
    for value in values:
        with pytest.raises(FrozenInstanceError):
            value.extra = True  # type: ignore[attr-defined]


def test_runtime_model_source_has_no_ros_gazebo_binding_or_wall_clock_dependency():
    source = Path(__file__).parents[1] / "src/drone_sim_gazebo/runtime/model.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    imported = {
        alias.name.split(".")[0]
        for node in ast.walk(tree)
        if isinstance(node, (ast.Import, ast.ImportFrom))
        for alias in node.names
    }
    assert imported.isdisjoint({"rclpy", "gz", "time", "datetime"})
