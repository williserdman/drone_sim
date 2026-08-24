from __future__ import annotations

from dataclasses import replace
from datetime import datetime, timedelta, timezone
import hashlib
import io
import json
import os
from pathlib import Path
from types import SimpleNamespace
from uuid import UUID

import pytest

from artifacts import (
    DockerLogCaptureError,
    DockerLogCaptureResult,
    DockerLogCommandResult,
    ImageDigest,
)
from artifacts.manifest import REQUIRED_ARTIFACT_PATHS
from orchestration._adapters.compose import ComposeCommandResult, ComposeRuntime
from orchestration.controller import RunController
from orchestration.status_store import OperatorStatus, StatusStore


RUN_ID = "00000000-0000-4000-8000-000000000606"
FIXED_UUID = UUID(RUN_ID)
SHA_A = "a" * 64
MODULES = (
    "orchestration",
    "artifacts",
    "companion",
    "ardupilot_sitl",
    "gazebo",
    "electromagnet",
    "scorekeeper",
)


def _template(tmp_path: Path, **updates) -> Path:
    document = {
        "world": "competition",
        "vehicle": "iris",
        "mission": "descent",
        "scenario": "maximum_score",
        "output_root": str((tmp_path / "runs").resolve()),
        "max_wall_seconds": 30,
        "startup_wall_seconds": 10,
        "finalization_wall_seconds": 10,
        "recording": {
            "width_px": 320,
            "height_px": 240,
            "fps": 20,
            "encoding": "rgb8",
        },
    }
    document.update(updates)
    path = tmp_path / "template.json"
    path.write_text(json.dumps(document), encoding="utf-8")
    return path


def _write_json(path: Path, document: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, sort_keys=True), encoding="utf-8")


def _tree_facts(path: Path) -> tuple[int, str]:
    rows = []
    size = 0
    for candidate in sorted(value for value in path.rglob("*") if value.is_file()):
        payload = candidate.read_bytes()
        rows.append(
            (
                candidate.relative_to(path).as_posix(),
                len(payload),
                hashlib.sha256(payload).hexdigest(),
            )
        )
        size += len(payload)
    digest = hashlib.sha256()
    for relative, length, file_digest in rows:
        digest.update(f"{relative}\0{length}\0{file_digest}\n".encode("utf-8"))
    return size, digest.hexdigest()


def _file_facts(path: Path) -> tuple[int, str]:
    payload = path.read_bytes()
    return len(payload), hashlib.sha256(payload).hexdigest()


def _complete_runtime_outputs(run_directory: Path, *, report_mutator=None) -> None:
    payloads = {
        "gazebo/server.log": b"fixture gazebo log",
        "video/onboard.mp4": b"valid onboard h264 fixture",
        "video/observer.mp4": b"valid observer h264 fixture",
        "scoring/events.jsonl": b'{"event":"landed"}\n',
        "scoring/result.json": b'{"achieved_score":100,"maximum_available_score":100}',
    }
    for relative_path, payload in payloads.items():
        target = run_directory / relative_path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(payload)
    (run_directory / "gazebo/state").mkdir(parents=True, exist_ok=True)
    (run_directory / "gazebo/state/state.json").write_bytes(b"{}")
    (run_directory / "rosbag").mkdir(parents=True, exist_ok=True)
    (run_directory / "rosbag/metadata.yaml").write_bytes(b"storage_identifier: mcap")
    (run_directory / "rosbag/data.mcap").write_bytes(b"valid synthetic mcap")

    records = []
    for relative_path in ("video/onboard.mp4", "video/observer.mp4", "rosbag"):
        target = run_directory / relative_path
        size, sha256 = _tree_facts(target) if target.is_dir() else _file_facts(target)
        semantic = (
            {"readable": True, "storage_id": "mcap", "topic_count": 10}
            if relative_path == "rosbag"
            else {"playable": True, "codec": "h264", "fps": 20, "frame_count": 40}
        )
        records.append(
            {
                "relative_path": relative_path,
                "status": "valid",
                "detail": "semantic validation passed",
                "size_bytes": size,
                "sha256": sha256,
                "semantic": semantic,
            }
        )
    report = {"run_id": RUN_ID, "complete": True, "records": records}
    if report_mutator is not None:
        report_mutator(report)
    _write_json(run_directory / ".status/artifacts-final.json", report)


class FakeClock:
    def __init__(self):
        self.value = 100.0
        self.timeouts: list[float] = []

    def monotonic(self):
        return self.value

    def sleep(self, seconds):
        self.timeouts.append(seconds)
        self.value += seconds


class TraceStore(StatusStore):
    def __init__(self, output_root, trace):
        super().__init__(output_root)
        self.trace = trace

    def allocate(self, run_id):
        self.trace.append("allocate")
        return super().allocate(run_id)

    def read_runtime_status(self, run_id, name):
        value = super().read_runtime_status(run_id, name)
        if value is not None and name in {
            "artifacts-ready",
            "source-finished",
            "runtime-frozen",
            "artifacts-final",
            "terminal-notified",
        }:
            marker = f"wait {name}"
            if marker not in self.trace:
                self.trace.append(marker)
        return value

    def request_finalization(self, run_id, requested_terminal, reason):
        self.trace.append("request FINALIZING")
        return super().request_finalization(run_id, requested_terminal, reason)

    def write_terminal_committed(self, run_id, document):
        assert (self.run_directory(run_id) / "manifest.json").is_file()
        self.trace.append("notify terminal")
        return super().write_terminal_committed(run_id, document)


class FakeCompose:
    project_name = "drone-sim-" + RUN_ID.replace("-", "")

    def __init__(
        self,
        run_directory: Path,
        trace: list[str],
        *,
        statuses: tuple[str, ...] = (
            "artifacts-ready",
            "runtime-running",
            "source-finished",
            "runtime-frozen",
            "terminal-notified",
        ),
        report_mutator=None,
        up_error: Exception | None = None,
        down_error: Exception | None = None,
        interrupt_sleep=None,
    ):
        self.run_directory = run_directory
        self.trace = trace
        self.statuses = statuses
        self.report_mutator = report_mutator
        self.up_error = up_error
        self.down_error = down_error
        self.down_timeouts: list[float] = []
        self.log_timeouts: list[float] = []

    def up(self, timeout):
        self.trace.append("compose up")
        if self.up_error:
            raise self.up_error
        _complete_runtime_outputs(self.run_directory, report_mutator=self.report_mutator)
        documents = {
            "artifacts-ready": {"run_id": RUN_ID, "ready": True},
            "runtime-running": {"run_id": RUN_ID, "state": "RUNNING", "sim_timestamp_ns": 0},
            "source-finished": {"run_id": RUN_ID, "finished": True, "sim_timestamp_ns": 2_000_000_000},
            "runtime-frozen": {"run_id": RUN_ID, "frozen": True},
            "terminal-notified": {"run_id": RUN_ID, "notified": True},
        }
        for name in self.statuses:
            _write_json(self.run_directory / f".status/{name}.json", documents[name])
        return ComposeCommandResult(0, b"started")

    def ps(self, timeout):
        return ComposeCommandResult(0, b'[{"Service":"synthetic-gazebo","State":"running"}]')

    def logs(self, command, timeout):
        self.log_timeouts.append(timeout)
        return DockerLogCommandResult(0, b"")

    def image_digests(self, timeout):
        return (ImageDigest("phase2", SHA_A),)

    def down(self, timeout):
        self.trace.append("compose down")
        self.down_timeouts.append(timeout)
        if self.down_error:
            raise self.down_error
        return ComposeCommandResult(0, b"removed")

    def stop_services(self, services, timeout):
        return ComposeCommandResult(0, b"stopped")


class FakeCapture:
    def __init__(self, *, run_directory, trace, command_runner, host_events, fail=False, **kwargs):
        assert host_events is True
        self.run_directory = Path(run_directory)
        self.trace = trace
        self.command_runner = command_runner
        self.fail = fail

    def capture(self):
        self.trace.append("capture logs")
        for module in MODULES:
            target = self.run_directory / f"logs/{module}.jsonl"
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(
                json.dumps(
                    {
                        "run_id": RUN_ID,
                        "module": module,
                        "severity": "INFO",
                        "event": "captured",
                        "sim_timestamp": None,
                        "wall_timestamp": "2026-08-24T00:00:00Z",
                        "fields": {},
                    },
                    separators=(",", ":"),
                    sort_keys=True,
                )
                + "\n",
                encoding="utf-8",
            )
        if self.fail:
            raise DockerLogCaptureError(
                DockerLogCaptureResult(
                    False, (), (), (), ("synthetic-gazebo",), (), (), ()
                )
            )
        return DockerLogCaptureResult(True, (), tuple(f"logs/{m}.jsonl" for m in MODULES), (), (), ())


def _controller(
    tmp_path: Path,
    *,
    statuses=None,
    report_mutator=None,
    capture_failure=False,
    up_error=None,
    down_error=None,
    clock=None,
    sleep=None,
    artifact_session_factory=None,
):
    trace: list[str] = []
    clock = clock or FakeClock()
    compose_holder = {}

    def store_factory(root):
        return TraceStore(root, trace)

    def compose_factory(config, run_directory):
        compose = FakeCompose(
            run_directory,
            trace,
            statuses=FakeCompose.__init__.__kwdefaults__["statuses"] if statuses is None else statuses,
            report_mutator=report_mutator,
            up_error=up_error,
            down_error=down_error,
        )
        compose_holder["value"] = compose
        return compose

    def capture_factory(**kwargs):
        return FakeCapture(
            **kwargs,
            trace=trace,
            fail=capture_failure,
        )

    class TracedSession:
        def __init__(self, *args, **kwargs):
            from artifacts import ArtifactSession

            self.delegate = ArtifactSession(*args, **kwargs)

        def finalize(self, request):
            trace.append("validate")
            result = self.delegate.finalize(request)
            trace.append("commit manifest")
            return result

    def traced_config_writer(run_directory, config):
        from orchestration.config import write_resolved_config

        result = write_resolved_config(run_directory, config)
        trace.append("snapshot config")
        return result

    controller = RunController(
        project_directory=Path(__file__).parents[2],
        status_store_factory=store_factory,
        compose_factory=compose_factory,
        log_capture_factory=capture_factory,
        artifact_session_factory=artifact_session_factory or TracedSession,
        config_writer=traced_config_writer,
        uuid_factory=lambda: FIXED_UUID,
        monotonic=clock.monotonic,
        sleep=sleep or clock.sleep,
        utcnow=lambda: datetime(2026, 8, 24, tzinfo=timezone.utc)
        + timedelta(seconds=clock.monotonic() - 100.0),
        event_stream=io.StringIO(),
        poll_interval=1.0,
    )
    return controller, trace, clock, compose_holder


def test_compose_runtime_uses_exact_detached_arrays_environment_and_merged_output(tmp_path):
    calls = []

    def runner(command, *, env, timeout):
        calls.append((command, env, timeout))
        return SimpleNamespace(returncode=0, stdout=b"stdout\nstderr\n")

    project = tmp_path.resolve()
    run_directory = (tmp_path / "runs" / RUN_ID).resolve()
    config_path = run_directory / "configuration/run.json"
    runtime = ComposeRuntime(
        project_directory=project,
        run_id=RUN_ID,
        run_directory=run_directory,
        config_path=config_path,
        runner=runner,
        base_environment={"PATH": "/bin"},
    )

    result = runtime.up(timeout=9.5)

    assert result == ComposeCommandResult(0, b"stdout\nstderr\n")
    command, environment, timeout = calls[0]
    assert command == [
        "docker",
        "compose",
        "--project-directory",
        str(project),
        "-p",
        "drone-sim-00000000000040008000000000000606",
        "up",
        "--detach",
        "--no-build",
    ]
    assert environment == {
        "PATH": "/bin",
        "SIM_CONFIG_PATH": str(config_path),
        "SIM_PHASE2_PROFILE": "1",
        "SIM_RUN_DIRECTORY": str(run_directory),
        "SIM_RUN_ID": RUN_ID,
    }
    assert timeout == 9.5


def test_compose_log_runner_validates_frozen_command_and_augments_project_directory(tmp_path):
    calls = []

    def runner(command, *, env, timeout):
        calls.append((command, timeout))
        return SimpleNamespace(returncode=7, stdout=b"exact raw bytes\x80")

    runtime = ComposeRuntime(
        project_directory=tmp_path.resolve(),
        run_id=RUN_ID,
        run_directory=(tmp_path / "run").resolve(),
        config_path=(tmp_path / "run/configuration/run.json").resolve(),
        runner=runner,
        base_environment={},
    )
    frozen = [
        "docker",
        "compose",
        "-p",
        runtime.project_name,
        "logs",
        "--no-color",
        "--no-log-prefix",
        "synthetic-gazebo",
    ]

    result = runtime.logs(frozen, timeout=4.25)

    assert result == DockerLogCommandResult(7, b"exact raw bytes\x80")
    assert calls == [
        (
            [
                "docker",
                "compose",
                "--project-directory",
                str(tmp_path.resolve()),
                "-p",
                runtime.project_name,
                "logs",
                "--no-color",
                "--no-log-prefix",
                "synthetic-gazebo",
            ],
            4.25,
        )
    ]
    with pytest.raises(ValueError, match="frozen"):
        runtime.logs(["docker", "compose", "logs", "--follow"], timeout=1)


def test_compose_image_digest_lookup_shares_one_decreasing_timeout(tmp_path):
    calls = []
    values = iter([10.0, 10.0, 11.5])

    def monotonic():
        return next(values)

    def runner(command, *, env, timeout):
        calls.append((command, timeout))
        output = (
            b"image-a\nimage-b\n"
            if command[-2:] == ["config", "--images"]
            else ("sha256:" + "a" * 64 + "\nsha256:" + "b" * 64 + "\n").encode()
        )
        return SimpleNamespace(returncode=0, stdout=output)

    runtime = ComposeRuntime(
        project_directory=tmp_path.resolve(),
        run_id=RUN_ID,
        run_directory=(tmp_path / "run").resolve(),
        config_path=(tmp_path / "run/configuration/run.json").resolve(),
        runner=runner,
        base_environment={},
        monotonic=monotonic,
    )

    assert runtime.image_digests(5) == (
        ImageDigest("image-a", "a" * 64),
        ImageDigest("image-b", "b" * 64),
    )
    assert [timeout for _command, timeout in calls] == [5.0, 3.5]


def test_log_capture_wrapper_recomputes_remaining_timeout_for_all_seven_services(tmp_path):
    clock = FakeClock()

    class ExercisingCapture(FakeCapture):
        def capture(self):
            for index, (service, _module) in enumerate(_OWNERSHIP_FOR_TEST):
                command = [
                    "docker",
                    "compose",
                    "-p",
                    FakeCompose.project_name,
                    "logs",
                    "--no-color",
                    "--no-log-prefix",
                    service,
                ]
                self.command_runner(command)
            return super().capture()

    _OWNERSHIP_FOR_TEST = (
        ("orchestration-runtime", "orchestration"),
        ("artifacts-runtime", "artifacts"),
        ("synthetic-companion", "companion"),
        ("synthetic-ardupilot-sitl", "ardupilot_sitl"),
        ("synthetic-gazebo", "gazebo"),
        ("synthetic-electromagnet", "electromagnet"),
        ("synthetic-scorekeeper", "scorekeeper"),
    )

    trace: list[str] = []
    captured_compose = {}

    class AdvancingCompose(FakeCompose):
        def logs(self, command, timeout):
            self.log_timeouts.append(timeout)
            clock.value += 0.25
            return DockerLogCommandResult(0, b"")

    def compose_factory(config, run_directory):
        value = AdvancingCompose(run_directory, trace)
        captured_compose["value"] = value
        return value

    def capture_factory(**kwargs):
        return ExercisingCapture(**kwargs, trace=trace)

    controller = RunController(
        project_directory=Path(__file__).parents[2],
        status_store_factory=lambda root: TraceStore(root, trace),
        compose_factory=compose_factory,
        log_capture_factory=capture_factory,
        artifact_session_factory=lambda *args, **kwargs: __import__(
            "artifacts"
        ).ArtifactSession(*args, **kwargs),
        uuid_factory=lambda: FIXED_UUID,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
        utcnow=lambda: datetime(2026, 8, 24, tzinfo=timezone.utc),
        event_stream=io.StringIO(),
    )
    controller.start(_template(tmp_path))

    timeouts = captured_compose["value"].log_timeouts
    assert len(timeouts) == 7
    assert all(later < earlier for earlier, later in zip(timeouts, timeouts[1:]))


def test_git_provenance_commands_consume_one_shared_remaining_deadline(tmp_path):
    values = iter([10.0, 11.0])
    calls = []

    def monotonic():
        return next(values)

    def runner(command, **kwargs):
        calls.append((command, kwargs["timeout"]))
        output = b"abc123\n" if command[-2:] == ["rev-parse", "HEAD"] else b" M file\n"
        return SimpleNamespace(returncode=0, stdout=output)

    controller = RunController(
        project_directory=tmp_path,
        monotonic=monotonic,
        source_runner=runner,
    )

    assert controller._source_revisions(15.0) == (
        __import__("artifacts").SourceRevision("drone_sim", "abc123", True),
    )
    assert [timeout for _command, timeout in calls] == [5.0, 4.0]


def test_completed_controller_executes_frozen_order_commits_manifest_then_tears_down(tmp_path):
    controller, trace, _clock, holder = _controller(tmp_path)

    result = controller.start(_template(tmp_path))

    assert result.state == "COMPLETED"
    assert result.manifest_path == "manifest.json"
    assert trace == [
        "allocate",
        "snapshot config",
        "compose up",
        "wait artifacts-ready",
        "wait source-finished",
        "request FINALIZING",
        "wait runtime-frozen",
        "wait artifacts-final",
        "capture logs",
        "validate",
        "commit manifest",
        "notify terminal",
        "wait terminal-notified",
        "compose down",
    ]
    run_directory = tmp_path / "runs" / RUN_ID
    manifest = json.loads((run_directory / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["terminal_status"] == "COMPLETED"
    assert manifest["source_revisions"]
    assert manifest["image_digests"] == [{"name": "phase2", "digest": SHA_A}]
    assert holder["value"].down_timeouts[0] > 0

    host_lines = (run_directory / "logs/orchestration-host.jsonl.partial").read_text().splitlines()
    assert host_lines
    assert all(
        set(json.loads(line))
        == {"run_id", "module", "severity", "event", "sim_timestamp", "wall_timestamp", "fields"}
        and json.loads(line)["run_id"] == RUN_ID
        and json.loads(line)["module"] == "orchestration"
        for line in host_lines
    )


def test_runtime_running_signal_is_validated_before_operator_reports_running(tmp_path):
    controller, _trace, _clock, _holder = _controller(tmp_path)
    run_directory = tmp_path / "runs" / RUN_ID
    # A ready file never permits RUNNING without Task 7's durable first-clock signal.
    result = controller.start(
        _template(tmp_path, startup_wall_seconds=1),
    )
    assert result.state == "COMPLETED"
    state = json.loads((run_directory / ".status/operator-state.json").read_text())
    assert state["state"] == "COMPLETED"


@pytest.mark.parametrize(
    "bad_document",
    [
        {"run_id": RUN_ID, "state": "READY", "sim_timestamp_ns": 0},
        {"run_id": RUN_ID, "state": "RUNNING", "sim_timestamp_ns": -1},
        {"run_id": RUN_ID, "state": "RUNNING", "sim_timestamp_ns": True},
    ],
)
def test_invalid_runtime_running_signal_fails_closed(tmp_path, bad_document):
    controller, _trace, _clock, _holder = _controller(tmp_path)
    original = FakeCompose.up

    def corrupt_up(self, timeout):
        result = original(self, timeout)
        _write_json(self.run_directory / ".status/runtime-running.json", bad_document)
        return result

    FakeCompose.up = corrupt_up
    try:
        result = controller.start(_template(tmp_path))
    finally:
        FakeCompose.up = original

    assert result.state == "FAILED"
    assert "runtime-running" in result.reason


def test_readiness_without_runtime_running_never_infers_simulation_progress(tmp_path):
    controller, _trace, _clock, _holder = _controller(
        tmp_path,
        statuses=("artifacts-ready", "runtime-frozen", "terminal-notified"),
    )

    result = controller.start(
        _template(tmp_path, max_wall_seconds=2, finalization_wall_seconds=5)
    )

    assert result.state == "FAILED"
    assert result.reason == "clock_source_stall"


def test_abort_requested_immediately_after_running_signal_wins_completion(tmp_path):
    trace: list[str] = []

    class AbortingStore(TraceStore):
        def write_operator_status(self, status):
            path = super().write_operator_status(status)
            if status.state == "RUNNING":
                self.request_finalization(RUN_ID, "ABORTED", "operator_abort")
            return path

    clock = FakeClock()
    compose_holder = {}

    def compose_factory(config, run_directory):
        compose = FakeCompose(run_directory, trace)
        compose_holder["value"] = compose
        return compose

    controller = RunController(
        project_directory=Path(__file__).parents[2],
        status_store_factory=lambda root: AbortingStore(root, trace),
        compose_factory=compose_factory,
        log_capture_factory=lambda **kwargs: FakeCapture(**kwargs, trace=trace),
        uuid_factory=lambda: FIXED_UUID,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
        utcnow=lambda: datetime(2026, 8, 24, tzinfo=timezone.utc),
        event_stream=io.StringIO(),
    )

    result = controller.start(_template(tmp_path))

    assert result.state == "ABORTED"
    assert result.reason == "operator_abort"


def test_child_exit_race_enters_failed_finalization(tmp_path):
    controller, _trace, _clock, _holder = _controller(
        tmp_path,
        statuses=("runtime-frozen", "terminal-notified"),
    )
    original = FakeCompose.ps

    def exited(self, timeout):
        return ComposeCommandResult(
            0, b'[{"Service":"synthetic-gazebo","State":"exited"}]'
        )

    FakeCompose.ps = exited
    try:
        result = controller.start(_template(tmp_path))
    finally:
        FakeCompose.ps = original

    assert result.state == "FAILED"
    assert result.reason == "compose_child_exited"


def test_first_observed_abort_wins_runtime_failure_race_and_later_cause_is_diagnostic(tmp_path):
    controller, _trace, _clock, _holder = _controller(
        tmp_path,
        statuses=("artifacts-ready", "runtime-running", "runtime-frozen", "terminal-notified"),
    )
    original = FakeCompose.up

    def raced_up(self, timeout):
        result = original(self, timeout)
        store = StatusStore((tmp_path / "runs").resolve())
        store.request_finalization(RUN_ID, "ABORTED", "operator_abort")
        _write_json(
            self.run_directory / ".status/runtime-failure.json",
            {"run_id": RUN_ID, "module": "artifacts", "reason": "recorder_failed"},
        )
        return result

    FakeCompose.up = raced_up
    try:
        result = controller.start(_template(tmp_path))
    finally:
        FakeCompose.up = original

    assert result.state == "ABORTED"
    status = controller.status(RUN_ID, (tmp_path / "runs").resolve())
    assert status.reason == "operator_abort"
    document = json.loads(
        (tmp_path / "runs" / RUN_ID / ".status/operator-state.json").read_text()
    )
    assert document["primary_cause"]["reason"] == "operator_abort"
    assert {item["reason"] for item in document["diagnostics"]} >= {"recorder_failed"}


@pytest.mark.parametrize(
    ("case", "statuses", "capture_failure", "expected_reason"),
    [
        ("startup_deadline", ("runtime-frozen", "terminal-notified"), False, "startup_deadline"),
        (
            "clock_stall",
            ("artifacts-ready", "runtime-running", "runtime-frozen", "terminal-notified"),
            False,
            "clock_source_stall",
        ),
        (
            "log_capture",
            ("artifacts-ready", "runtime-running", "source-finished", "runtime-frozen", "terminal-notified"),
            True,
            "docker_log_capture_failed",
        ),
        (
            "finalization_deadline",
            ("artifacts-ready", "runtime-running", "source-finished", "terminal-notified"),
            False,
            "finalization_deadline",
        ),
    ],
)
def test_named_failures_commit_failed_manifest_and_teardown(
    tmp_path, case, statuses, capture_failure, expected_reason
):
    controller, trace, clock, holder = _controller(
        tmp_path,
        statuses=statuses,
        capture_failure=capture_failure,
    )
    template = _template(
        tmp_path,
        startup_wall_seconds=2,
        max_wall_seconds=4,
        finalization_wall_seconds=5,
    )

    result = controller.start(template)

    assert result.state == "FAILED"
    assert expected_reason in result.reason
    assert trace[-1] == "compose down"
    assert holder["value"].down_timeouts
    manifest_path = tmp_path / "runs" / RUN_ID / "manifest.json"
    assert manifest_path.is_file()
    assert json.loads(manifest_path.read_text())["terminal_status"] == "FAILED"


def test_ctrl_c_requests_aborted_finalization_and_returns_130_semantics(tmp_path):
    calls = 0

    def interrupting_sleep(_seconds):
        nonlocal calls
        calls += 1
        raise KeyboardInterrupt

    controller, trace, _clock, _holder = _controller(
        tmp_path,
        statuses=("runtime-frozen", "terminal-notified"),
        sleep=interrupting_sleep,
    )

    result = controller.start(_template(tmp_path))

    assert calls == 1
    assert result.state == "ABORTED"
    assert "operator_interrupt" in result.reason
    assert trace[-1] == "compose down"
    manifest = json.loads((tmp_path / "runs" / RUN_ID / "manifest.json").read_text())
    assert manifest["terminal_status"] == "ABORTED"


def test_recorder_failure_enters_failed_finalization(tmp_path):
    controller, _trace, _clock, _holder = _controller(
        tmp_path,
        statuses=("artifacts-ready", "runtime-running", "runtime-frozen", "terminal-notified"),
    )
    original = FakeCompose.up

    def failing_up(self, timeout):
        result = original(self, timeout)
        _write_json(
            self.run_directory / ".status/runtime-failure.json",
            {"run_id": RUN_ID, "module": "artifacts", "reason": "observer_encoder_failed"},
        )
        return result

    FakeCompose.up = failing_up
    try:
        result = controller.start(_template(tmp_path))
    finally:
        FakeCompose.up = original

    assert result.state == "FAILED"
    assert result.reason == "observer_encoder_failed"


def test_validation_exception_still_runs_bounded_teardown_without_false_manifest(tmp_path):
    class BrokenSession:
        def __init__(self, *args, **kwargs):
            pass

        def finalize(self, request):
            raise ValueError("semantic validation exploded")

    controller, trace, _clock, holder = _controller(
        tmp_path, artifact_session_factory=BrokenSession
    )

    result = controller.start(_template(tmp_path))

    assert result.state == "FAILED"
    assert "semantic validation exploded" in result.reason
    assert trace[-1] == "compose down"
    assert holder["value"].down_timeouts
    assert not (tmp_path / "runs" / RUN_ID / "manifest.json").exists()
    assert not (
        tmp_path / "runs" / RUN_ID / ".control/terminal-committed.json"
    ).exists()


def test_compose_up_exception_still_attempts_finalization_and_bounded_down(tmp_path):
    controller, trace, _clock, holder = _controller(
        tmp_path, up_error=RuntimeError("docker unavailable")
    )

    result = controller.start(_template(tmp_path))

    assert result.state == "FAILED"
    assert result.manifest_path == "manifest.json"
    assert trace[-1] == "compose down"
    assert holder["value"].down_timeouts


def test_compose_factory_exception_still_commits_failed_diagnostic_manifest(tmp_path):
    clock = FakeClock()

    def broken_factory(config, run_directory):
        raise RuntimeError("adapter construction failed")

    controller = RunController(
        project_directory=Path(__file__).parents[2],
        compose_factory=broken_factory,
        uuid_factory=lambda: FIXED_UUID,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
        utcnow=lambda: datetime(2026, 8, 24, tzinfo=timezone.utc),
        event_stream=io.StringIO(),
    )

    result = controller.start(_template(tmp_path))

    assert result.state == "FAILED"
    assert result.manifest_path == "manifest.json"
    manifest = json.loads(
        (tmp_path / "runs" / RUN_ID / "manifest.json").read_text()
    )
    assert manifest["terminal_status"] == "FAILED"
    assert "compose_factory" in manifest["reason"]


def test_teardown_exception_is_retained_as_later_operator_diagnostic(tmp_path):
    controller, _trace, _clock, _holder = _controller(
        tmp_path, down_error=RuntimeError("daemon disappeared")
    )

    result = controller.start(_template(tmp_path))

    assert result.state == "COMPLETED"
    document = json.loads(
        (tmp_path / "runs" / RUN_ID / ".status/operator-state.json").read_text()
    )
    assert any(item["kind"] == "teardown" for item in document["diagnostics"])


@pytest.mark.parametrize(
    ("mutation", "reason_fragment"),
    [
        (lambda report: report["records"].pop(), "missing"),
        (
            lambda report: report["records"][0].update({"sha256": "b" * 64}),
            "checksum",
        ),
        (
            lambda report: (
                report.update({"complete": False}),
                report["records"][0].update(
                    {"status": "invalid", "detail": "corrupt video"}
                ),
            ),
            "corrupt video",
        ),
        (
            lambda report: (
                report.update({"complete": False}),
                report["records"][2].update(
                    {"status": "invalid", "detail": "corrupt bag"}
                ),
            ),
            "corrupt bag",
        ),
    ],
)
def test_strict_artifacts_final_report_downgrades_completion(tmp_path, mutation, reason_fragment):
    controller, _trace, _clock, _holder = _controller(
        tmp_path, report_mutator=mutation
    )

    result = controller.start(_template(tmp_path))

    assert result.state == "FAILED"
    assert reason_fragment in result.reason
    manifest = json.loads((tmp_path / "runs" / RUN_ID / "manifest.json").read_text())
    assert manifest["terminal_status"] == "FAILED"


def test_abort_command_is_idempotent_and_never_constructs_compose(tmp_path):
    output_root = (tmp_path / "runs").resolve()
    store = StatusStore(output_root)
    store.allocate(RUN_ID)
    store.write_operator_status(OperatorStatus(RUN_ID, "RUNNING", "", None))
    constructed = 0

    def forbidden_compose(*args, **kwargs):
        nonlocal constructed
        constructed += 1
        raise AssertionError("abort must not construct Compose")

    controller = RunController(compose_factory=forbidden_compose)

    first = controller.abort(RUN_ID, output_root)
    second = controller.abort(RUN_ID, output_root)

    assert first.state == second.state == "RUNNING"
    assert constructed == 0
    request = json.loads(
        (output_root / RUN_ID / ".control/finalize-request.json").read_text()
    )
    assert request["requested_terminal"] == "ABORTED"


def test_collect_results_validates_manifest_and_is_strictly_read_only(tmp_path):
    controller, _trace, _clock, _holder = _controller(tmp_path)
    started = controller.start(_template(tmp_path))
    run_directory = tmp_path / "runs" / RUN_ID
    before = {
        path.relative_to(run_directory).as_posix(): (path.stat().st_mtime_ns, path.read_bytes())
        for path in run_directory.rglob("*")
        if path.is_file()
    }

    collected = controller.collect_results(RUN_ID, (tmp_path / "runs").resolve())

    after = {
        path.relative_to(run_directory).as_posix(): (path.stat().st_mtime_ns, path.read_bytes())
        for path in run_directory.rglob("*")
        if path.is_file()
    }
    assert collected == started
    assert before == after

    manifest = run_directory / "manifest.json"
    document = json.loads(manifest.read_text())
    document["run_id"] = "wrong"
    manifest.chmod(0o644)
    manifest.write_text(json.dumps(document))
    with pytest.raises(Exception, match="run_id"):
        controller.collect_results(RUN_ID, (tmp_path / "runs").resolve())
