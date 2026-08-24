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
from orchestration.controller import ControllerError, RunController, RunResult, TerminalCause
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
SERVICES = (
    "orchestration-runtime",
    "artifacts-runtime",
    "synthetic-companion",
    "synthetic-ardupilot-sitl",
    "synthetic-gazebo",
    "synthetic-electromagnet",
    "synthetic-scorekeeper",
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


def _complete_runtime_outputs(
    run_directory: Path, *, report_mutator=None, score_mutator=None
) -> None:
    scoring_checksum = "c" * 64
    payloads = {
        "gazebo/server.log": b"fixture gazebo log",
        "video/onboard.mp4": b"valid onboard h264 fixture",
        "video/observer.mp4": b"valid observer h264 fixture",
        "scoring/events.jsonl": b'{"event":"landed"}\n',
        "scoring/result.json": json.dumps(
            {
                "achieved_score": 100,
                "maximum_available_score": 100,
                "scoring_checksum": scoring_checksum,
                "evidence_paths": ["scoring/events.jsonl"],
            },
            separators=(",", ":"),
            sort_keys=True,
        ).encode(),
    }
    for relative_path, payload in payloads.items():
        target = run_directory / relative_path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(payload)
    if score_mutator is not None:
        score_mutator(run_directory / "scoring/result.json")
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


def test_score_metadata_consumes_declared_checksum_and_safe_evidence_paths(tmp_path):
    run_directory = tmp_path / "run"
    result = run_directory / "scoring/result.json"
    result.parent.mkdir(parents=True)
    result.write_text(
        json.dumps(
            {
                "achieved_score": 0.0,
                "maximum_available_score": 0.0,
                "scoring_checksum": "d" * 64,
                "evidence_paths": ["scoring/events.jsonl#event-0"],
            }
        ),
        encoding="utf-8",
    )

    assert RunController()._score_metadata(run_directory) == (
        0.0,
        0.0,
        "d" * 64,
        ("scoring/events.jsonl#event-0",),
    )


@pytest.mark.parametrize(
    "updates",
    [
        {"scoring_checksum": "D" * 64},
        {"scoring_checksum": "d" * 63},
        {"evidence_paths": ["../outside"]},
        {"evidence_paths": ["/absolute"]},
        {"evidence_paths": ["scoring/events.jsonl", "scoring/events.jsonl"]},
        {"evidence_paths": [{}]},
        {"achieved_score": None},
        {"maximum_available_score": float("inf")},
    ],
)
def test_score_metadata_rejects_invalid_provenance_document(tmp_path, updates):
    run_directory = tmp_path / "run"
    result = run_directory / "scoring/result.json"
    result.parent.mkdir(parents=True)
    document = {
        "achieved_score": 0.0,
        "maximum_available_score": 0.0,
        "scoring_checksum": "d" * 64,
        "evidence_paths": ["scoring/events.jsonl"],
    }
    document.update(updates)
    result.write_text(json.dumps(document), encoding="utf-8")

    assert RunController()._score_metadata(run_directory) == (None, None, None, ())


@pytest.mark.parametrize("unsafe_kind", ["parent-symlink", "file-symlink", "hardlink"])
def test_score_metadata_rejects_out_of_bundle_or_multiply_linked_provenance(
    tmp_path, unsafe_kind
):
    run_directory = tmp_path / "run"
    outside = tmp_path / "outside"
    outside.mkdir()
    payload = json.dumps(
        {
            "achieved_score": 0.0,
            "maximum_available_score": 0.0,
            "scoring_checksum": "d" * 64,
            "evidence_paths": ["scoring/events.jsonl"],
        }
    )
    outside_result = outside / "result.json"
    outside_result.write_text(payload, encoding="utf-8")
    scoring = run_directory / "scoring"
    run_directory.mkdir()
    if unsafe_kind == "parent-symlink":
        scoring.symlink_to(outside, target_is_directory=True)
    else:
        scoring.mkdir()
        result = scoring / "result.json"
        if unsafe_kind == "file-symlink":
            result.symlink_to(outside_result)
        else:
            os.link(outside_result, result)

    assert RunController()._score_metadata(run_directory) == (None, None, None, ())


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

    def read_runtime_status(self, run_id, name, deadline_check=None):
        value = super().read_runtime_status(run_id, name, deadline_check)
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
        score_mutator=None,
        up_error: Exception | None = None,
        down_error: Exception | None = None,
        interrupt_sleep=None,
    ):
        self.run_directory = run_directory
        self.trace = trace
        self.statuses = statuses
        self.report_mutator = report_mutator
        self.score_mutator = score_mutator
        self.up_error = up_error
        self.down_error = down_error
        self.down_timeouts: list[float] = []
        self.log_timeouts: list[float] = []

    def up(self, timeout):
        self.trace.append("compose up")
        if self.up_error:
            raise self.up_error
        _complete_runtime_outputs(
            self.run_directory,
            report_mutator=self.report_mutator,
            score_mutator=self.score_mutator,
        )
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
        return ComposeCommandResult(
            0,
            json.dumps(
                [
                    {"Service": service, "State": "running", "Health": "healthy"}
                    for service in SERVICES
                ]
            ).encode(),
        )

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
    score_mutator=None,
    capture_failure=False,
    up_error=None,
    down_error=None,
    clock=None,
    sleep=None,
    artifact_session_factory=None,
    event_stream=None,
    store_type=TraceStore,
):
    trace: list[str] = []
    clock = clock or FakeClock()
    compose_holder = {}

    def store_factory(root):
        return store_type(root, trace)

    def compose_factory(config, run_directory):
        compose = FakeCompose(
            run_directory,
            trace,
            statuses=FakeCompose.__init__.__kwdefaults__["statuses"] if statuses is None else statuses,
            report_mutator=report_mutator,
            score_mutator=score_mutator,
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

        def finalize_with_result(self, request):
            trace.append("validate")
            result = self.delegate.finalize_with_result(request)
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
        event_stream=event_stream or io.StringIO(),
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
        base_environment={
            "PATH": "/bin",
            "COMPOSE_FILE": "/tmp/hostile.yaml",
            "COMPOSE_ENV_FILES": "/tmp/hostile.env",
            "COMPOSE_DISABLE_ENV_FILE": "0",
            "COMPOSE_PATH_SEPARATOR": ";",
            "COMPOSE_PROFILES": "hostile",
            "COMPOSE_PROJECT_NAME": "hostile",
            "COMPOSE_PROJECT_DIR": "/tmp/hostile-project",
            "COMPOSE_PROJECT_DIRECTORY": "/tmp/hostile-project-directory",
            "DOCKER_HOST": "tcp://docker.example:2376",
            "DOCKER_TLS_VERIFY": "1",
            "DOCKER_CERT_PATH": "/certs",
            "DOCKER_CONTEXT": "remote",
        },
    )

    result = runtime.up(timeout=9.5)

    assert result == ComposeCommandResult(0, b"stdout\nstderr\n")
    command, environment, timeout = calls[0]
    assert command == [
        "docker",
        "compose",
        "--file",
        str(project / "compose.yaml"),
        "--project-directory",
        str(project),
        "-p",
        "drone-sim-00000000000040008000000000000606",
        "up",
        "--detach",
        "--no-build",
    ]
    assert environment == {
        "COMPOSE_DISABLE_ENV_FILE": "1",
        "COMPOSE_PROFILES": "phase2",
        "DOCKER_CERT_PATH": "/certs",
        "DOCKER_CONTEXT": "remote",
        "DOCKER_HOST": "tcp://docker.example:2376",
        "DOCKER_TLS_VERIFY": "1",
        "PATH": "/bin",
        "SIM_CONFIG_PATH": str(config_path),
        "SIM_PHASE2_PROFILE": "1",
        "SIM_RUN_DIRECTORY": str(run_directory),
        "SIM_RUN_ID": RUN_ID,
    }
    assert timeout == 9.5


def test_non_phase2_recording_geometry_is_rejected_before_compose_construction(tmp_path):
    template = _template(tmp_path)
    document = json.loads(template.read_text(encoding="utf-8"))
    document["recording"] = {
        **document["recording"],
        "width_px": 640,
        "height_px": 480,
    }
    template.write_text(json.dumps(document), encoding="utf-8")
    compose_calls = []

    def compose_factory(config, run_directory):
        compose_calls.append((config, run_directory))
        raise AssertionError("Compose must not be constructed")

    controller = RunController(
        project_directory=Path(__file__).parents[2],
        compose_factory=compose_factory,
        uuid_factory=lambda: FIXED_UUID,
        event_stream=io.StringIO(),
    )

    with pytest.raises(ControllerError, match="dimensions"):
        controller.start(template)
    assert compose_calls == []


def test_compose_profile_environment_and_exact_ps_argv_cover_every_operation(tmp_path):
    calls = []

    def runner(command, *, env, timeout):
        calls.append((command, env.copy()))
        if command[-2:] == ["config", "--images"]:
            output = b"phase2-image\n"
        elif command[:3] == ["docker", "image", "inspect"]:
            output = ("sha256:" + "a" * 64 + "\n").encode()
        else:
            output = b"[]"
        return SimpleNamespace(returncode=0, stdout=output)

    runtime = ComposeRuntime(
        project_directory=tmp_path.resolve(),
        run_id=RUN_ID,
        run_directory=(tmp_path / "run").resolve(),
        config_path=(tmp_path / "run/configuration/run.json").resolve(),
        runner=runner,
        base_environment={},
    )
    runtime.up(3)
    runtime.ps(3)
    runtime.logs(
        [
            "docker", "compose", "-p", runtime.project_name, "logs",
            "--no-color", "--no-log-prefix", "synthetic-gazebo",
        ],
        3,
    )
    runtime.image_digests(3)
    runtime.down(3)

    assert all(environment["COMPOSE_PROFILES"] == "phase2" for _, environment in calls)
    assert calls[1][0][-4:] == ["ps", "--all", "--format", "json"]


@pytest.mark.parametrize(
    ("rows", "reason"),
    [
        ([{"Service": service, "State": "running"} for service in SERVICES[:-1]], "compose_child_set_invalid"),
        ([{"Service": service, "State": "running"} for service in (*SERVICES, "extra")], "compose_child_set_invalid"),
        ([{"Service": service, "State": "running"} for service in (*SERVICES[:-1], SERVICES[0])], "compose_child_set_invalid"),
        ([{"Service": service, "State": "running"} for service in SERVICES[:-1]] + [{"Service": SERVICES[-1], "State": "exited"}], "compose_child_exited"),
        ([{"State": "running"}], "compose_ps_invalid"),
    ],
)
def test_compose_health_requires_exact_seven_unique_running_services(rows, reason):
    cause = RunController._ps_cause(ComposeCommandResult(0, json.dumps(rows).encode()))

    assert cause is not None
    assert cause.reason == reason


def test_compose_health_accepts_real_compose_ndjson_ps_output():
    output = b"\n".join(
        json.dumps(
            {"Service": service, "State": "running", "Health": "healthy"}
        ).encode()
        for service in SERVICES
    )

    assert RunController._ps_cause(ComposeCommandResult(0, output)) is None


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
                "--file",
                str(tmp_path.resolve() / "compose.yaml"),
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
    values = iter([10.0, 10.0, 11.0, 11.0, 11.0])
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

    assert not (run_directory / "logs/orchestration-host.jsonl.partial").exists()
    host_lines = (run_directory / "logs/orchestration.jsonl").read_text().splitlines()
    assert host_lines
    assert all(
        set(json.loads(line))
        == {"run_id", "module", "severity", "event", "sim_timestamp", "wall_timestamp", "fields"}
        and json.loads(line)["run_id"] == RUN_ID
        and json.loads(line)["module"] == "orchestration"
        for line in host_lines
    )


@pytest.mark.parametrize(
    "score_mutator",
    [
        lambda path: path.unlink(),
        lambda path: path.write_bytes(b"{malformed"),
    ],
    ids=("absent", "malformed"),
)
def test_completed_request_fails_closed_without_valid_scoring_provenance(
    tmp_path, score_mutator
):
    controller, _trace, _clock, _holder = _controller(
        tmp_path, score_mutator=score_mutator
    )

    result = controller.start(_template(tmp_path))

    manifest = json.loads(
        (tmp_path / "runs" / RUN_ID / "manifest.json").read_text(encoding="utf-8")
    )
    assert result.state == "FAILED"
    assert result.reason == "scoring_provenance_invalid"
    assert manifest["terminal_status"] == "FAILED"
    assert manifest["reason"] == "scoring_provenance_invalid"


def test_invalid_scoring_provenance_never_changes_requested_failure(tmp_path):
    controller, _trace, _clock, _holder = _controller(
        tmp_path,
        statuses=(
            "artifacts-ready",
            "runtime-running",
            "runtime-frozen",
            "terminal-notified",
        ),
        score_mutator=lambda path: path.write_bytes(b"{malformed"),
    )

    result = controller.start(
        _template(tmp_path, max_wall_seconds=2, finalization_wall_seconds=5)
    )

    manifest = json.loads(
        (tmp_path / "runs" / RUN_ID / "manifest.json").read_text(encoding="utf-8")
    )
    assert result.state == "FAILED"
    assert result.reason == "clock_source_stall"
    assert manifest["terminal_status"] == "FAILED"
    assert manifest["reason"] == "clock_source_stall"


def test_invalid_scoring_provenance_never_changes_requested_abort(tmp_path):
    class AbortingStore(TraceStore):
        def write_operator_status(self, status):
            path = super().write_operator_status(status)
            if status.state == "RUNNING":
                self.request_finalization(RUN_ID, "ABORTED", "operator_abort")
            return path

    controller, _trace, _clock, _holder = _controller(
        tmp_path,
        store_type=AbortingStore,
        score_mutator=lambda path: path.write_bytes(b"{malformed"),
    )

    result = controller.start(_template(tmp_path))

    manifest = json.loads(
        (tmp_path / "runs" / RUN_ID / "manifest.json").read_text(encoding="utf-8")
    )
    assert result.state == "ABORTED"
    assert result.reason == "operator_abort"
    assert manifest["terminal_status"] == "ABORTED"
    assert manifest["reason"] == "operator_abort"


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


def test_missing_artifacts_final_expires_before_log_capture_or_mutable_bag_hashing(
    tmp_path, monkeypatch
):
    capture_checks = []

    class DeadlineGuardedCapture:
        def __init__(self, *, deadline_check, **_kwargs):
            self.deadline_check = deadline_check

        def capture(self):
            capture_checks.append("entered")
            with pytest.raises(TimeoutError, match="finalization_deadline"):
                self.deadline_check()
            capture_checks.append("blocked")
            raise TimeoutError("finalization_deadline")

    def forbidden_validation(*_args, **_kwargs):
        raise AssertionError("expired work may not inspect or hash mutable artifacts")

    monkeypatch.setattr("artifacts.session.validate_regular_file", forbidden_validation)
    monkeypatch.setattr("artifacts.session.validate_tree", forbidden_validation)
    real_up = FakeCompose.up

    def stubborn_runtime_up(self, timeout):
        result = real_up(self, timeout)
        (self.run_directory / ".status/artifacts-final.json").unlink()
        return result

    monkeypatch.setattr(FakeCompose, "up", stubborn_runtime_up)

    controller, _trace, _clock, holder = _controller(
        tmp_path,
        statuses=(
            "artifacts-ready",
            "runtime-running",
            "source-finished",
            "runtime-frozen",
            "terminal-notified",
        ),
    )
    controller.log_capture_factory = DeadlineGuardedCapture

    result = controller.start(
        _template(tmp_path, finalization_wall_seconds=5, max_wall_seconds=4)
    )

    assert result.state == "FAILED"
    assert capture_checks == ["entered", "blocked"]
    manifest = json.loads(
        (tmp_path / "runs" / RUN_ID / "manifest.json").read_text(encoding="utf-8")
    )
    assert manifest["terminal_status"] == "FAILED"
    required = {
        record["relative_path"]: record
        for record in manifest["artifacts"]
        if record["relative_path"] in REQUIRED_ARTIFACT_PATHS
    }
    assert set(required) == set(REQUIRED_ARTIFACT_PATHS)
    assert all(record["validation"] == "invalid" for record in required.values())
    assert all(record["sha256"] is None for record in required.values())
    assert all(record["size_bytes"] is None for record in required.values())
    assert holder["value"].down_timeouts


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

        def finalize_with_result(self, request):
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


def test_terminal_commit_write_failure_cannot_override_committed_manifest(tmp_path):
    class FailingTerminalStore(TraceStore):
        def write_terminal_committed(self, run_id, document):
            raise OSError("control volume became read-only")

    controller, _trace, _clock, _holder = _controller(
        tmp_path, store_type=FailingTerminalStore
    )

    result = controller.start(_template(tmp_path))

    manifest = json.loads((tmp_path / "runs" / RUN_ID / "manifest.json").read_text())
    status = json.loads(
        (tmp_path / "runs" / RUN_ID / ".status/operator-state.json").read_text()
    )
    assert result.state == manifest["terminal_status"] == status["state"] == "COMPLETED"
    assert any(item["kind"] == "terminal_notification" for item in status["diagnostics"])


def test_terminal_commit_write_failure_cannot_override_aborted_manifest(tmp_path):
    class AbortingFailingTerminalStore(TraceStore):
        def write_operator_status(self, status):
            path = super().write_operator_status(status)
            if status.state == "RUNNING":
                self.request_finalization(RUN_ID, "ABORTED", "operator_abort")
            return path

        def write_terminal_committed(self, run_id, document):
            raise OSError("control volume became read-only")

    controller, _trace, _clock, _holder = _controller(
        tmp_path, store_type=AbortingFailingTerminalStore
    )

    result = controller.start(_template(tmp_path))

    manifest = json.loads((tmp_path / "runs" / RUN_ID / "manifest.json").read_text())
    assert result.state == manifest["terminal_status"] == "ABORTED"
    assert result.reason == manifest["reason"] == "operator_abort"


def test_terminal_notification_read_failure_cannot_override_committed_manifest(tmp_path):
    class FailingNotificationStore(TraceStore):
        def read_runtime_status(self, run_id, name, deadline_check=None):
            if name == "terminal-notified":
                raise OSError("status volume became unreadable")
            return super().read_runtime_status(run_id, name, deadline_check)

    controller, _trace, _clock, _holder = _controller(
        tmp_path, store_type=FailingNotificationStore
    )

    result = controller.start(_template(tmp_path))

    manifest = json.loads((tmp_path / "runs" / RUN_ID / "manifest.json").read_text())
    assert result.state == manifest["terminal_status"] == "COMPLETED"
    status = json.loads(
        (tmp_path / "runs" / RUN_ID / ".status/operator-state.json").read_text()
    )
    assert any(item["kind"] == "terminal_notification" for item in status["diagnostics"])


def test_post_commit_manifest_reread_failure_cannot_override_typed_authority(tmp_path):
    class FailingPostCommitReadStore(TraceStore):
        def validated_manifest_result(self, run_id, deadline_check=None):
            raise TimeoutError("post_commit_deadline")

    controller, _trace, _clock, _holder = _controller(
        tmp_path, store_type=FailingPostCommitReadStore
    )

    result = controller.start(_template(tmp_path))

    manifest = json.loads((tmp_path / "runs" / RUN_ID / "manifest.json").read_text())
    status = json.loads(
        (tmp_path / "runs" / RUN_ID / ".status/operator-state.json").read_text()
    )
    assert result.state == manifest["terminal_status"] == status["state"] == "COMPLETED"
    assert result.reason == manifest["reason"] == "mission_complete"
    assert any(item["kind"] == "manifest_verification" for item in status["diagnostics"])


def test_post_manifest_operator_status_failure_returns_committed_result_and_commands_prefer_it(
    tmp_path,
):
    class FailingTerminalOperatorStore(TraceStore):
        def write_operator_status(self, status):
            if status.manifest_path == "manifest.json":
                raise OSError("operator status volume became read-only")
            return super().write_operator_status(status)

    controller, _trace, _clock, _holder = _controller(
        tmp_path,
        store_type=FailingTerminalOperatorStore,
    )

    started = controller.start(_template(tmp_path))
    manifest_path = tmp_path / "runs" / RUN_ID / "manifest.json"
    immutable = manifest_path.read_bytes()
    expected = RunResult(RUN_ID, "COMPLETED", "mission_complete", "manifest.json")

    assert started == expected
    assert controller.status(RUN_ID, tmp_path / "runs") == expected
    assert controller.abort(RUN_ID, tmp_path / "runs") == expected
    assert controller.collect_results(RUN_ID, tmp_path / "runs") == expected
    assert manifest_path.read_bytes() == immutable


def test_wait_rejects_runtime_status_that_crosses_deadline_and_passes_checker():
    clock = FakeClock()
    received = []

    class CrossingStore:
        def read_runtime_status(self, run_id, name, deadline_check=None):
            received.append(deadline_check)
            clock.value += 10
            return {"run_id": run_id, "frozen": True}

    controller = RunController(
        monotonic=clock.monotonic,
        sleep=clock.sleep,
        event_stream=io.StringIO(),
    )
    deadline = clock.monotonic() + 5
    checker = controller._deadline_check(deadline)

    document, cause = controller._wait_for(
        CrossingStore(),
        RUN_ID,
        FakeCompose(Path("/tmp/unused"), []),
        "runtime-frozen",
        deadline,
        TerminalCause("finalization_deadline", "finalization_deadline"),
        observe_causes=False,
        deadline_check=checker,
    )

    assert document is None
    assert cause is not None and cause.reason == "finalization_deadline"
    assert received == [checker]


def test_terminal_notification_crossing_manifest_deadline_is_diagnostic_only(tmp_path):
    clock = FakeClock()
    received = []
    runtime_failure_checks = []

    class CrossingNotificationStore(TraceStore):
        def read_runtime_status(self, run_id, name, deadline_check=None):
            if name == "terminal-notified":
                received.append(deadline_check)
                clock.value += 100
                return {"run_id": run_id, "notified": True}
            if name == "runtime-failure":
                runtime_failure_checks.append(deadline_check)
            return super().read_runtime_status(run_id, name, deadline_check)

    controller, _trace, _clock, holder = _controller(
        tmp_path,
        clock=clock,
        store_type=CrossingNotificationStore,
    )

    result = controller.start(_template(tmp_path))

    assert result.state == "COMPLETED"
    assert received and callable(received[0])
    assert runtime_failure_checks and all(callable(item) for item in runtime_failure_checks)
    assert holder["value"].down_timeouts == [0.0]
    status = json.loads(
        (tmp_path / "runs" / RUN_ID / ".status/operator-state.json").read_text()
    )
    assert any(
        item["reason"] == "terminal_notification_deadline"
        for item in status["diagnostics"]
    )


def test_broken_stdout_during_second_event_preserves_manifest_and_teardown(tmp_path):
    class BrokenSecondWrite(io.StringIO):
        def __init__(self):
            super().__init__()
            self.calls = 0

        def write(self, value):
            self.calls += 1
            if self.calls == 2:
                raise BrokenPipeError("operator disconnected")
            return super().write(value)

    controller, trace, _clock, _holder = _controller(
        tmp_path, event_stream=BrokenSecondWrite()
    )

    result = controller.start(_template(tmp_path))

    assert result.state == "COMPLETED"
    assert (tmp_path / "runs" / RUN_ID / "manifest.json").is_file()
    assert trace[-1] == "compose down"
    status = json.loads(
        (tmp_path / "runs" / RUN_ID / ".status/operator-state.json").read_text()
    )
    assert any(item["kind"] == "observability" for item in status["diagnostics"])


def test_host_partial_append_failure_is_diagnostic_and_finalization_continues(
    tmp_path, monkeypatch
):
    real_write = os.write
    host_writes = 0

    def failing_write(descriptor, payload):
        nonlocal host_writes
        try:
            target = os.readlink(f"/proc/self/fd/{descriptor}")
        except OSError:
            target = ""
        if target.endswith("orchestration-host.jsonl.partial"):
            host_writes += 1
            if host_writes == 2:
                raise OSError("host event volume full")
        return real_write(descriptor, payload)

    monkeypatch.setattr(os, "write", failing_write)
    controller, trace, _clock, _holder = _controller(tmp_path)

    result = controller.start(_template(tmp_path))

    assert result.state == "COMPLETED"
    assert (tmp_path / "runs" / RUN_ID / "manifest.json").is_file()
    assert trace[-1] == "compose down"
    status = json.loads(
        (tmp_path / "runs" / RUN_ID / ".status/operator-state.json").read_text()
    )
    assert any(item["kind"] == "observability" for item in status["diagnostics"])


def test_host_event_close_failure_is_diagnostic_and_finalization_continues(
    tmp_path, monkeypatch
):
    real_fsync = os.fsync
    host_fsyncs = 0

    def failing_fsync(descriptor):
        nonlocal host_fsyncs
        try:
            target = os.readlink(f"/proc/self/fd/{descriptor}")
        except OSError:
            target = ""
        if target.endswith("orchestration-host.jsonl.partial"):
            host_fsyncs += 1
            if host_fsyncs == 4:
                raise OSError("host event fsync failed")
        return real_fsync(descriptor)

    monkeypatch.setattr(os, "fsync", failing_fsync)
    controller, trace, _clock, _holder = _controller(tmp_path)

    result = controller.start(_template(tmp_path))

    assert result.state == "COMPLETED"
    assert trace[-1] == "compose down"
    status = json.loads(
        (tmp_path / "runs" / RUN_ID / ".status/operator-state.json").read_text()
    )
    assert any(item["kind"] == "observability" for item in status["diagnostics"])


def test_host_event_descriptor_close_failure_does_not_skip_remaining_cleanup(
    tmp_path, monkeypatch
):
    real_close = os.close
    failed = False

    def failing_close(descriptor):
        nonlocal failed
        try:
            target = os.readlink(f"/proc/self/fd/{descriptor}")
        except OSError:
            target = ""
        if not failed and target.endswith("orchestration-host.jsonl.partial"):
            failed = True
            raise OSError("host event close failed")
        return real_close(descriptor)

    monkeypatch.setattr(os, "close", failing_close)
    controller, trace, _clock, _holder = _controller(tmp_path)

    result = controller.start(_template(tmp_path))

    assert result.state == "COMPLETED"
    assert trace[-1] == "compose down"
    run_directory = str(tmp_path / "runs" / RUN_ID)
    leaked = []
    for entry in Path("/proc/self/fd").iterdir():
        try:
            target = os.readlink(entry)
        except OSError:
            continue
        if target.startswith(run_directory):
            leaked.append(target)
    assert leaked == [
        f"{run_directory}/logs/orchestration-host.jsonl.partial (deleted)"
    ]


def test_session_deadline_exhaustion_cannot_return_completed(tmp_path):
    clock = FakeClock()

    class AdvancingSession:
        def __init__(self, *args, deadline_check=None, **kwargs):
            self.deadline_check = deadline_check

        def finalize_with_result(self, request):
            clock.value += 1000
            if self.deadline_check is None:
                raise RuntimeError("deadline seam missing")
            self.deadline_check()
            raise AssertionError("deadline check did not stop finalization")

    controller, trace, _clock, holder = _controller(
        tmp_path, clock=clock, artifact_session_factory=AdvancingSession
    )

    result = controller.start(_template(tmp_path))

    assert result.state == "FAILED"
    assert "finalization_deadline" in result.reason
    assert trace[-1] == "compose down"
    assert holder["value"].down_timeouts == [0.0]
    assert not (tmp_path / "runs" / RUN_ID / "manifest.json").exists()


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
