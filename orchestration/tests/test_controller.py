from __future__ import annotations

from dataclasses import replace
from datetime import datetime, timedelta, timezone
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace
from uuid import UUID

import pytest
import yaml

import orchestration.config as config_module
from artifacts import (
    DockerLogCaptureError,
    DockerLogCaptureResult,
    DockerLogCommandResult,
    ImageDigest,
    SourceRevision,
)
from artifacts.manifest import REQUIRED_ARTIFACT_PATHS
from artifacts.runtime_status import (
    ArduPilotReadyStatus,
    ArtifactFinalRecord,
    ArtifactsFinalStatus,
    ArtifactsReadyStatus,
    CompanionReadyStatus,
    FlightExchange,
    GazeboReadyStatus,
    MissionFinishedStatus,
    MissionReadyStatus,
    RuntimeFailureStatus,
    RuntimeFrozenStatus,
    RuntimeRunningStatus,
    ScoreFinishedStatus,
    SourceFinishedStatus,
    TerminalNotifiedStatus,
    status_document,
    status_name,
)
from artifacts.validation import ValidationStatus
from orchestration._adapters.compose import ComposeCommandResult, ComposeRuntime
from orchestration.controller import ControllerError, RunController, RunResult, TerminalCause
from orchestration.status_store import OperatorStatus, ProtocolFileError, StatusStore
from orchestration.config import QGCSources, resolve_run_config


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
PHASE3_SERVICES = (
    "orchestration-runtime",
    "artifacts-runtime",
    "companion-runtime",
    "ardupilot-sitl",
    "gazebo-runtime",
    "electromagnet-runtime",
    "scorekeeper-runtime",
)
FLIGHT_EXCHANGE = FlightExchange(True, 1, 1, 0, 0, 1, 0, 0, 0)


def _topology(profile: str):
    ownership = tuple(
        zip(PHASE3_SERVICES if profile == "phase3" else SERVICES, MODULES, strict=True)
    )
    return config_module.RuntimeTopology(profile=profile, ownership=ownership)


def _qgc_sources(
    *,
    deployment_profile: bytes = b'{"profile":"alpha"}',
    origin: object = None,
) -> QGCSources:
    if origin is None:
        origin = {
            "latitude_deg": 52.1,
            "longitude_deg": 13.2,
            "amsl_m": 48.25,
            "heading_deg": 123,
        }
    return QGCSources(
        deployment_profile=deployment_profile,
        listener_session=b"{}",
        qgc_actions=b"{}",
        runtime_policy=json.dumps({"simulator_launch_origin": origin}).encode(),
    )


def _qgc_state(tmp_path: Path, qgc: QGCSources) -> tuple[Path, Path]:
    root = (tmp_path / "attempt-state").resolve()
    state = root / qgc.attempt_state_id
    state.mkdir(parents=True)
    (state / "attempt-ledger.json").write_bytes(b'{"used":false}\n')
    (state / "attempt-ledger.json.lock").write_bytes(b"")
    return root, state


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


def _comp2026_template(tmp_path: Path, **updates) -> Path:
    source_root = Path(__file__).parents[2] / "config"
    (tmp_path / "course.yaml").write_bytes((source_root / "course.yaml").read_bytes())
    (tmp_path / "scenario.yaml").write_bytes(
        (source_root / "scenario.yaml").read_bytes()
    )
    qgc_sources = {
        "deployment_profile": "deployment-profile-source.json",
        "listener_session": "listener-session-source.json",
        "qgc_actions": "qgc-actions-source.json",
        "runtime_policy": "qgc-runtime-source.json",
    }
    for field, name in qgc_sources.items():
        (tmp_path / name).write_text(json.dumps({"fixture": field}), encoding="utf-8")
    return _template(
        tmp_path,
        world="competition_mission",
        vehicle="iris_competition",
        mission="comp2026_auto",
        scenario="competition_v1",
        recording={
            "width_px": 640,
            "height_px": 480,
            "fps": 20,
            "encoding": "rgb8",
        },
        competition={"course": "course.yaml", "scenario": "scenario.yaml"},
        qgc=qgc_sources,
        **updates,
    )


def _search_delivery_template(tmp_path: Path, **updates) -> Path:
    source_root = Path(__file__).parents[2] / "config"
    course_name = "course-search-delivery.yaml"
    scenario_name = "scenario-search-delivery.yaml"
    (tmp_path / course_name).write_bytes((source_root / course_name).read_bytes())
    (tmp_path / scenario_name).write_bytes((source_root / scenario_name).read_bytes())
    return _template(
        tmp_path,
        world="search_delivery",
        vehicle="iris_search_delivery",
        mission="configured",
        scenario="search_delivery_v1",
        recording={
            "width_px": 640,
            "height_px": 480,
            "observer_width_px": 1280,
            "observer_height_px": 960,
            "fps": 20,
            "encoding": "rgb8",
        },
        runtime_profile="phase3",
        simulation={
            "seed": 2026,
            "duration_sim_seconds": 240.0,
            "public_epoch_native_sim_seconds": 90.0,
            "target_real_time_factor": 1.0,
        },
        competition={"course": course_name, "scenario": scenario_name},
        mission_plan={
            "schema_version": 1,
            "steps": [{"tool": "land", "args": {}}],
        },
        **updates,
    )


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
    scoring_checksum = hashlib.sha256(
        (Path(__file__).parents[2] / "scorekeeper/rules/descent_v1.json").read_bytes()
    ).hexdigest()
    rule_ids = (
        "airborne_then_contact",
        "touchdown_precision",
        "safe_preimpact_speed",
        "stable_contact",
    )
    available_points = (20.0, 40.0, 20.0, 20.0)
    awarded_points = (20.0, 40.0, 0.0, 0.0)
    events = [
        {
            "run_id": RUN_ID,
            "sim_timestamp_ns": 2_000_000_000,
            "event_id": index,
            "event_type": f"descent.{rule_id}",
            "value": awarded,
            "evidence_ref": f"scoring/events.jsonl#event-{index}",
        }
        for index, (rule_id, awarded) in enumerate(
            zip(rule_ids, awarded_points, strict=True)
        )
    ]
    events.append(
        {
            "run_id": RUN_ID,
            "sim_timestamp_ns": 2_000_000_000,
            "event_id": 4,
            "event_type": "score.finalized",
            "value": 60.0,
            "evidence_ref": "scoring/events.jsonl#event-4",
        }
    )
    payloads = {
        "gazebo/server.log": b"fixture gazebo log",
        "video/onboard.mp4": b"valid onboard h264 fixture",
        "video/observer.mp4": b"valid observer h264 fixture",
        "scoring/events.jsonl": "".join(
            json.dumps(event, sort_keys=True) + "\n" for event in events
        ).encode(),
        "scoring/result.json": json.dumps(
            {
                "run_id": RUN_ID,
                "ruleset_id": "descent_v1",
                "complete": True,
                "achieved_score": 60.0,
                "maximum_available_score": 100.0,
                "scoring_checksum": scoring_checksum,
                "evidence_paths": [
                    *(f"scoring/events.jsonl#event-{index}" for index in range(5)),
                    "rosbag#/simulation/ground_truth",
                ],
                "rule_results": [
                    {
                        "rule_id": rule_id,
                        "passed": awarded > 0,
                        "awarded_points": awarded,
                        "available_points": available,
                        "evidence_ref": (
                            f"rosbag#/simulation/ground_truth:{rule_id}"
                        ),
                    }
                    for rule_id, awarded, available in zip(
                        rule_ids, awarded_points, available_points, strict=True
                    )
                ],
                "diagnostic": None,
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
    state_source = run_directory / "gazebo/state/state.source"
    state_source.write_bytes(b"native state")
    subprocess.run(
        (
            "zstd",
            "-3",
            "--quiet",
            str(state_source),
            "-o",
            str(run_directory / "gazebo/state/state.tlog.zst"),
        ),
        check=True,
    )
    state_source.unlink()
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
            ArtifactFinalRecord(
                relative_path,
                ValidationStatus.VALID,
                "semantic validation passed",
                size,
                sha256,
                semantic,
            )
        )
    report = status_document(ArtifactsFinalStatus(RUN_ID, tuple(records)))
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

    def read_runtime_status(self, run_id, status_type, deadline_check=None):
        value = super().read_runtime_status(run_id, status_type, deadline_check)
        if value is not None and status_type in {
            ArtifactsReadyStatus,
            GazeboReadyStatus,
            ArduPilotReadyStatus,
            CompanionReadyStatus,
            MissionReadyStatus,
            RuntimeRunningStatus,
            SourceFinishedStatus,
            MissionFinishedStatus,
            ScoreFinishedStatus,
            RuntimeFrozenStatus,
            ArtifactsFinalStatus,
            TerminalNotifiedStatus,
        }:
            marker = f"wait {status_name(status_type)}"
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
        statuses: tuple[type, ...] = (
            ArtifactsReadyStatus,
            RuntimeRunningStatus,
            SourceFinishedStatus,
            RuntimeFrozenStatus,
            TerminalNotifiedStatus,
        ),
        report_mutator=None,
        score_mutator=None,
        runtime_mutator=None,
        up_error: Exception | None = None,
        down_error: Exception | None = None,
        interrupt_sleep=None,
        services=SERVICES,
    ):
        self.run_directory = run_directory
        self.trace = trace
        self.statuses = statuses
        self.report_mutator = report_mutator
        self.score_mutator = score_mutator
        self.runtime_mutator = runtime_mutator
        self.up_error = up_error
        self.down_error = down_error
        self.down_timeouts: list[float] = []
        self.log_timeouts: list[float] = []
        self.services = services

    def up(self, timeout):
        self.trace.append("compose up")
        if self.up_error:
            raise self.up_error
        _complete_runtime_outputs(
            self.run_directory,
            report_mutator=self.report_mutator,
            score_mutator=self.score_mutator,
        )
        if self.runtime_mutator is not None:
            self.runtime_mutator(self.run_directory)
        values = {
            ArtifactsReadyStatus: ArtifactsReadyStatus(RUN_ID),
            GazeboReadyStatus: GazeboReadyStatus(RUN_ID, FLIGHT_EXCHANGE),
            ArduPilotReadyStatus: ArduPilotReadyStatus(RUN_ID),
            CompanionReadyStatus: CompanionReadyStatus(RUN_ID),
            MissionReadyStatus: MissionReadyStatus(RUN_ID),
            RuntimeRunningStatus: RuntimeRunningStatus(RUN_ID, 0),
            SourceFinishedStatus: SourceFinishedStatus(RUN_ID, 2_000_000_000),
            MissionFinishedStatus: MissionFinishedStatus(RUN_ID, 1_500_000_000),
            ScoreFinishedStatus: ScoreFinishedStatus(RUN_ID, 2_000_000_000),
            RuntimeFrozenStatus: RuntimeFrozenStatus(RUN_ID),
            TerminalNotifiedStatus: TerminalNotifiedStatus(RUN_ID),
        }
        for status_type in self.statuses:
            _write_json(
                self.run_directory / f".status/{status_name(status_type)}.json",
                status_document(values[status_type]),
            )
        return ComposeCommandResult(0, b"started")

    def ps(self, timeout):
        return ComposeCommandResult(
            0,
            json.dumps(
                [
                    {"Service": service, "State": "running", "Health": "healthy"}
                    for service in self.services
                ]
            ).encode(),
        )

    def logs(self, command, timeout):
        self.log_timeouts.append(timeout)
        return DockerLogCommandResult(0, b"")

    def image_digests(self, timeout):
        return (ImageDigest("phase2", SHA_A),)

    def bind_source_revisions(self, revisions, timeout):
        self.bound_source_revisions = tuple(revisions)
        return ComposeCommandResult(0, b"source revisions bound")

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
    runtime_mutator=None,
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
            runtime_mutator=runtime_mutator,
            up_error=up_error,
            down_error=down_error,
            services=(PHASE3_SERVICES if config.runtime_profile == "phase3" else SERVICES),
        )
        compose_holder["value"] = compose
        return compose

    def capture_factory(**kwargs):
        compose_holder["log_ownership"] = kwargs["ownership"]
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
        topology=_topology("phase2"),
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
        "SIM_LAUNCH_ORIGIN_JSON": (
            '{"amsl_m":0.0,"heading_deg":0.0,'
            '"latitude_deg":37.4003371,"longitude_deg":-122.0800351}'
        ),
        "SIM_PHASE2_PROFILE": "1",
        "SIM_RUN_DIRECTORY": str(run_directory),
        "SIM_RUN_ID": RUN_ID,
    }
    assert timeout == 9.5


def test_compose_runtime_gpu_overlay_uses_only_the_owned_gpu_file(tmp_path):
    calls = []

    def runner(command, *, env, timeout):
        calls.append((command, env, timeout))
        return SimpleNamespace(returncode=0, stdout=b"")

    project = tmp_path.resolve()
    run_directory = (tmp_path / "runs" / RUN_ID).resolve()
    runtime = ComposeRuntime(
        project_directory=project,
        run_id=RUN_ID,
        run_directory=run_directory,
        config_path=run_directory / "configuration/run.json",
        topology=_topology("phase3"),
        runner=runner,
        base_environment={"PATH": "/bin", "SIM_COMPOSE_OVERLAY": "gpu"},
    )

    runtime.up(timeout=9.5)

    command, environment, _timeout = calls[0]
    assert command[:7] == [
        "docker",
        "compose",
        "--file",
        str(project / "compose.yaml"),
        "--file",
        str(project / "compose.gpu.yaml"),
        "--project-directory",
    ]
    assert "SIM_COMPOSE_OVERLAY" not in environment


def test_qgc_compose_uses_owned_overlay_state_and_canonical_origin(tmp_path):
    calls = []

    def runner(command, *, env, timeout):
        calls.append((command, env.copy(), timeout))
        return SimpleNamespace(returncode=0, stdout=b"")

    qgc = _qgc_sources()
    state_root, _state = _qgc_state(tmp_path, qgc)
    project = tmp_path.resolve()
    run_directory = (tmp_path / "runs" / RUN_ID).resolve()
    runtime = ComposeRuntime(
        project_directory=project,
        run_id=RUN_ID,
        run_directory=run_directory,
        config_path=run_directory / "configuration/run.json",
        topology=_topology("phase3"),
        qgc=qgc,
        runner=runner,
        base_environment={
            "PATH": "/bin",
            "SIM_COMPOSE_OVERLAY": "gpu",
            "DOCKER_HOST": "tcp://hostile.example:2376",
            "DOCKER_CONTEXT": "hostile-saved-context",
            "DOCKER_TLS": "1",
            "DOCKER_TLS_VERIFY": "1",
            "DOCKER_CERT_PATH": "/hostile/certs",
            "SIM_QGC_ATTEMPT_STATE_DIRECTORY": "/tmp/hostile-state",
            "SIM_LAUNCH_ORIGIN_JSON": '{"latitude_deg":0}',
        },
        test_only_qgc_state_root=state_root,
    )

    runtime.up(4.5)

    production_state = (
        Path("/var/lib/drone-sim/comp2026-attempt-state") / qgc.attempt_state_id
    )
    assert calls == [
        (
            [
                "docker",
                "--host",
                "unix:///var/run/docker.sock",
                "compose",
                "--file",
                str(project / "compose.yaml"),
                "--file",
                str(project / "compose.gpu.yaml"),
                "--file",
                str(project / "compose.qgc.yaml"),
                "--project-directory",
                str(project),
                "-p",
                "drone-sim-00000000000040008000000000000606",
                "up",
                "--detach",
                "--no-build",
            ],
            {
                "COMPOSE_DISABLE_ENV_FILE": "1",
                "COMPOSE_PROFILES": "phase3",
                "PATH": "/bin",
                "SIM_CONFIG_PATH": str(run_directory / "configuration/run.json"),
                "SIM_LAUNCH_ORIGIN_JSON": (
                    '{"amsl_m":48.25,"heading_deg":123.0,'
                    '"latitude_deg":52.1,"longitude_deg":13.2}'
                ),
                "SIM_QGC_ATTEMPT_STATE_DIRECTORY": str(production_state),
                "SIM_RUN_DIRECTORY": str(run_directory),
                "SIM_RUN_ID": RUN_ID,
            },
            4.5,
        )
    ]


def test_qgc_direct_image_inspect_is_pinned_to_the_local_daemon(tmp_path):
    calls = []
    qgc = _qgc_sources()
    state_root, _state = _qgc_state(tmp_path, qgc)

    def runner(command, *, env, timeout):
        calls.append((command, env.copy()))
        return SimpleNamespace(returncode=0, stdout=("b" * 40 + "\n").encode())

    runtime = ComposeRuntime(
        project_directory=tmp_path.resolve(),
        run_id=RUN_ID,
        run_directory=(tmp_path / "run").resolve(),
        config_path=(tmp_path / "run/configuration/run.json").resolve(),
        topology=_topology("phase3"),
        qgc=qgc,
        runner=runner,
        base_environment={"DOCKER_CONTEXT": "persisted-remote"},
        test_only_qgc_state_root=state_root,
    )

    runtime.bind_source_revisions(
        (
            SourceRevision("drone_sim", "a" * 40, True),
            SourceRevision("comp2026", "b" * 40, True),
        ),
        1,
    )

    command, environment = calls[0]
    assert command[:5] == [
        "docker",
        "--host",
        "unix:///var/run/docker.sock",
        "image",
        "inspect",
    ]
    assert "DOCKER_CONTEXT" not in environment


def test_non_qgc_compose_replaces_ambient_origin_with_diagnostic_origin(tmp_path):
    calls = []
    run_directory = (tmp_path / "run").resolve()
    runtime = ComposeRuntime(
        project_directory=tmp_path.resolve(),
        run_id=RUN_ID,
        run_directory=run_directory,
        config_path=run_directory / "configuration/run.json",
        topology=_topology("phase3"),
        runner=lambda command, *, env, timeout: calls.append((command, env.copy()))
        or SimpleNamespace(returncode=0, stdout=b""),
        base_environment={
            "SIM_QGC_ATTEMPT_STATE_DIRECTORY": "/ambient/state",
            "SIM_LAUNCH_ORIGIN_JSON": '{"ambient":true}',
        },
    )

    runtime.up(1)

    command, environment = calls[0]
    assert environment["SIM_QGC_ATTEMPT_STATE_DIRECTORY"] == "/ambient/state"
    assert environment["SIM_LAUNCH_ORIGIN_JSON"] == (
        '{"amsl_m":0.0,"heading_deg":0.0,'
        '"latitude_deg":37.4003371,"longitude_deg":-122.0800351}'
    )
    assert str(tmp_path.resolve() / "compose.qgc.yaml") not in command


@pytest.mark.parametrize(
    ("template_name", "mission"),
    [
        ("vertical-descent-run.json", "controlled_descent"),
        ("autotune-roll-run.json", "autotune_roll"),
        ("hover-roll-run.json", "hover_roll"),
    ],
)
def test_controller_gives_every_diagnostic_mission_the_frozen_launch_origin(
    tmp_path, template_name, mission
):
    project = Path(__file__).parents[2]
    config = resolve_run_config(
        project / "config" / template_name,
        run_id_factory=lambda: FIXED_UUID,
    )
    controller = RunController(project_directory=project)

    runtime = controller._default_compose(config, tmp_path.resolve())

    assert config.mission == mission
    assert config.qgc is None
    assert runtime.environment["SIM_LAUNCH_ORIGIN_JSON"] == (
        '{"amsl_m":0.0,"heading_deg":0.0,'
        '"latitude_deg":37.4003371,"longitude_deg":-122.0800351}'
    )


def test_qgc_state_path_depends_on_profile_digest_not_run_id(tmp_path):
    environments = []
    qgc_a = _qgc_sources(deployment_profile=b'{"profile":"same"}')
    qgc_b = _qgc_sources(deployment_profile=b'{"profile":"different"}')
    state_root = (tmp_path / "attempt-state").resolve()
    for qgc in (qgc_a, qgc_b):
        state = state_root / qgc.attempt_state_id
        state.mkdir(parents=True, exist_ok=True)
        (state / "attempt-ledger.json").write_bytes(b"{}")
        (state / "attempt-ledger.json.lock").write_bytes(b"")

    def runner(command, *, env, timeout):
        environments.append(env.copy())
        return SimpleNamespace(returncode=0, stdout=b"")

    for run_id, qgc in (
        (RUN_ID, qgc_a),
        ("00000000-0000-4000-8000-000000000607", qgc_a),
        ("00000000-0000-4000-8000-000000000608", qgc_b),
    ):
        run_directory = (tmp_path / "runs" / run_id).resolve()
        ComposeRuntime(
            project_directory=tmp_path.resolve(),
            run_id=run_id,
            run_directory=run_directory,
            config_path=run_directory / "configuration/run.json",
            topology=_topology("phase3"),
            qgc=qgc,
            runner=runner,
            base_environment={},
            test_only_qgc_state_root=state_root,
        ).up(1)

    paths = [env["SIM_QGC_ATTEMPT_STATE_DIRECTORY"] for env in environments]
    assert paths[0] == paths[1]
    assert paths[0] != paths[2]
    assert RUN_ID not in paths[0]


@pytest.mark.parametrize(
    "origin",
    [
        {},
        {"latitude_deg": 0, "longitude_deg": 0, "amsl_m": 0, "heading_deg": 0, "x": 1},
        {"latitude_deg": True, "longitude_deg": 0, "amsl_m": 0, "heading_deg": 0},
        {"latitude_deg": 0, "longitude_deg": 0, "amsl_m": 10**1000, "heading_deg": 0},
        {"latitude_deg": 91, "longitude_deg": 0, "amsl_m": 0, "heading_deg": 0},
        {"latitude_deg": 0, "longitude_deg": -181, "amsl_m": 0, "heading_deg": 0},
        {"latitude_deg": 0, "longitude_deg": 0, "amsl_m": 0, "heading_deg": 360},
    ],
)
def test_qgc_compose_rejects_malformed_launch_origin_before_runner(tmp_path, origin):
    calls = []
    qgc = _qgc_sources(origin=origin)
    state_root, _state = _qgc_state(tmp_path, qgc)

    with pytest.raises(ValueError, match="simulator launch origin"):
        ComposeRuntime(
            project_directory=tmp_path.resolve(),
            run_id=RUN_ID,
            run_directory=(tmp_path / "run").resolve(),
            config_path=(tmp_path / "run/configuration/run.json").resolve(),
            topology=_topology("phase3"),
            qgc=qgc,
            runner=lambda *args, **kwargs: calls.append((args, kwargs)),
            base_environment={},
            test_only_qgc_state_root=state_root,
        )
    assert calls == []


@pytest.mark.parametrize(
    "unsafe",
    [
        "missing",
        "missing-lock",
        "extra",
        "ledger-symlink",
        "ledger-fifo",
        "lock-symlink",
        "state-symlink",
        "root-symlink",
    ],
)
def test_qgc_compose_rejects_unsafe_or_missing_state_before_runner(tmp_path, unsafe):
    calls = []
    qgc = _qgc_sources()
    state_root, state = _qgc_state(tmp_path, qgc)
    if unsafe == "missing":
        (state / "attempt-ledger.json").unlink()
    elif unsafe == "missing-lock":
        (state / "attempt-ledger.json.lock").unlink()
    elif unsafe == "extra":
        (state / ".attempt-ledger.json.tmp").write_bytes(b"partial")
    elif unsafe == "ledger-symlink":
        ledger = state / "attempt-ledger.json"
        ledger.unlink()
        ledger.symlink_to(tmp_path / "outside-ledger")
    elif unsafe == "ledger-fifo":
        ledger = state / "attempt-ledger.json"
        ledger.unlink()
        os.mkfifo(ledger)
    elif unsafe == "lock-symlink":
        lock = state / "attempt-ledger.json.lock"
        lock.unlink()
        lock.symlink_to(tmp_path / "outside-lock")
    elif unsafe == "state-symlink":
        moved = state_root / "real-state"
        state.rename(moved)
        state.symlink_to(moved, target_is_directory=True)
    else:
        real_root = tmp_path / "real-root"
        state_root.rename(real_root)
        state_root.symlink_to(real_root, target_is_directory=True)

    runtime = ComposeRuntime(
        project_directory=tmp_path.resolve(),
        run_id=RUN_ID,
        run_directory=(tmp_path / "run").resolve(),
        config_path=(tmp_path / "run/configuration/run.json").resolve(),
        topology=_topology("phase3"),
        qgc=qgc,
        runner=lambda *args, **kwargs: calls.append((args, kwargs)),
        base_environment={},
        test_only_qgc_state_root=state_root,
    )

    with pytest.raises(ValueError, match="attempt state"):
        runtime.up(1)
    assert calls == []


@pytest.mark.parametrize("alias", ["//tmp/qgc-state", "/tmp/qgc-state\0suffix"])
def test_qgc_compose_rejects_noncanonical_test_state_root(tmp_path, alias):
    qgc = _qgc_sources()

    with pytest.raises(ValueError, match="attempt state root"):
        ComposeRuntime(
            project_directory=tmp_path.resolve(),
            run_id=RUN_ID,
            run_directory=(tmp_path / "run").resolve(),
            config_path=(tmp_path / "run/configuration/run.json").resolve(),
            topology=_topology("phase3"),
            qgc=qgc,
            base_environment={},
            test_only_qgc_state_root=alias,
        )


def test_qgc_compose_state_validation_does_not_read_or_mutate_files(tmp_path, monkeypatch):
    calls = []
    qgc = _qgc_sources()
    state_root, state = _qgc_state(tmp_path, qgc)
    lock = state / "attempt-ledger.json.lock"
    lock.write_bytes(b"locked")
    before = {
        path.name: (path.read_bytes(), path.stat().st_mtime_ns)
        for path in state.iterdir()
    }
    monkeypatch.setattr(os, "read", lambda *args: (_ for _ in ()).throw(AssertionError("read")))

    runtime = ComposeRuntime(
        project_directory=tmp_path.resolve(),
        run_id=RUN_ID,
        run_directory=(tmp_path / "run").resolve(),
        config_path=(tmp_path / "run/configuration/run.json").resolve(),
        topology=_topology("phase3"),
        qgc=qgc,
        runner=lambda command, *, env, timeout: calls.append(command)
        or SimpleNamespace(returncode=0, stdout=b""),
        base_environment={},
        test_only_qgc_state_root=state_root,
    )
    runtime.up(1)

    after = {
        path.name: (path.read_bytes(), path.stat().st_mtime_ns)
        for path in state.iterdir()
    }
    assert after == before
    assert calls


def test_qgc_state_changes_after_up_do_not_block_observation_or_teardown(tmp_path):
    calls = []
    qgc = _qgc_sources()
    state_root, state = _qgc_state(tmp_path, qgc)

    def runner(command, *, env, timeout):
        calls.append(command)
        if command[-2:] == ["config", "--images"]:
            output = b"phase3-image\n"
        elif command[3:5] == ["image", "inspect"]:
            output = ("sha256:" + "a" * 64 + "\n").encode()
        else:
            output = b"[]"
        return SimpleNamespace(returncode=0, stdout=output)

    runtime = ComposeRuntime(
        project_directory=tmp_path.resolve(),
        run_id=RUN_ID,
        run_directory=(tmp_path / "run").resolve(),
        config_path=(tmp_path / "run/configuration/run.json").resolve(),
        topology=_topology("phase3"),
        qgc=qgc,
        runner=runner,
        base_environment={},
        test_only_qgc_state_root=state_root,
    )
    runtime.up(1)

    (state / ".attempt-ledger.json.tmp").write_bytes(b"partial")
    (state / "attempt-ledger.json").unlink()
    replacement = state_root / "replaced-state"
    state.rename(replacement)
    state.symlink_to(replacement, target_is_directory=True)

    runtime.ps(1)
    runtime.logs(
        [
            "docker",
            "compose",
            "-p",
            runtime.project_name,
            "logs",
            "--no-color",
            "--no-log-prefix",
            "gazebo-runtime",
        ],
        1,
    )
    runtime.stop_services(["companion-runtime"], 1)
    runtime.down(1)
    assert runtime.image_digests(1) == (ImageDigest("phase3-image", "a" * 64),)

    assert [command[-1] for command in calls] == [
        "--no-build",
        "json",
        "gazebo-runtime",
        "companion-runtime",
        "--remove-orphans",
        "--images",
        "phase3-image",
    ]
    assert all(
        command[:3] == ["docker", "--host", "unix:///var/run/docker.sock"]
        for command in calls
    )


def test_qgc_overlay_gives_state_only_to_companion_and_origin_only_to_sitl():
    overlay = yaml.safe_load(
        (Path(__file__).parents[2] / "compose.qgc.yaml").read_text(encoding="utf-8")
    )

    assert set(overlay) == {"services"}
    assert set(overlay["services"]) == {"companion-runtime", "ardupilot-sitl"}
    companion = overlay["services"]["companion-runtime"]
    sitl = overlay["services"]["ardupilot-sitl"]
    assert companion == {
        "volumes": [
            {
                "type": "bind",
                "source": "${SIM_QGC_ATTEMPT_STATE_DIRECTORY:?QGC attempt state is required}",
                "target": "${SIM_QGC_ATTEMPT_STATE_DIRECTORY:?QGC attempt state is required}",
                "read_only": False,
                "bind": {"create_host_path": False},
            }
        ]
    }
    assert sitl == {
        "environment": {
            "SIM_LAUNCH_ORIGIN_JSON": "${SIM_LAUNCH_ORIGIN_JSON:?QGC launch origin is required}"
        }
    }


def test_base_compose_allows_direct_config_without_origin_only_for_ardupilot_sitl():
    document = yaml.safe_load(
        (Path(__file__).parents[2] / "compose.yaml").read_text(encoding="utf-8")
    )
    services = document["services"]

    assert services["ardupilot-sitl"]["environment"]["SIM_LAUNCH_ORIGIN_JSON"] == (
        "${SIM_LAUNCH_ORIGIN_JSON:-}"
    )
    assert all(
        "SIM_LAUNCH_ORIGIN_JSON" not in service.get("environment", {})
        for name, service in services.items()
        if name != "ardupilot-sitl"
    )


def test_phase2_compose_command_keeps_the_diagnostic_origin_out_of_services(tmp_path):
    document = yaml.safe_load(
        (Path(__file__).parents[2] / "compose.yaml").read_text(encoding="utf-8")
    )
    active_services = [
        service
        for service in document["services"].values()
        if "phase2" in service.get("profiles", [])
    ]

    assert active_services
    assert all(
        "SIM_LAUNCH_ORIGIN_JSON" not in service.get("environment", {})
        for service in active_services
    )


def test_compose_runtime_rejects_unknown_overlay(tmp_path):
    run_directory = (tmp_path / "runs" / RUN_ID).resolve()

    with pytest.raises(ValueError, match="SIM_COMPOSE_OVERLAY"):
        ComposeRuntime(
            project_directory=tmp_path.resolve(),
            run_id=RUN_ID,
            run_directory=run_directory,
            config_path=run_directory / "configuration/run.json",
            topology=_topology("phase3"),
            base_environment={"SIM_COMPOSE_OVERLAY": "../../hostile.yaml"},
        )


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
        topology=_topology("phase3"),
        runner=runner,
        base_environment={},
    )
    runtime.up(3)
    runtime.ps(3)
    runtime.logs(
        [
            "docker", "compose", "-p", runtime.project_name, "logs",
            "--no-color", "--no-log-prefix", "gazebo-runtime",
        ],
        3,
    )
    runtime.image_digests(3)
    runtime.stop_services(PHASE3_SERVICES, 3)
    runtime.down(3)

    assert all(environment["COMPOSE_PROFILES"] == "phase3" for _, environment in calls)
    assert calls[1][0][-4:] == ["ps", "--all", "--format", "json"]
    with pytest.raises(ValueError, match="selected topology"):
        runtime.stop_services(["synthetic-gazebo"], 3)


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
    cause = RunController._ps_cause(
        ComposeCommandResult(0, json.dumps(rows).encode()),
        _topology("phase2"),
    )

    assert cause is not None
    assert cause.reason == reason


@pytest.mark.parametrize(
    ("profile", "services"),
    [("phase2", SERVICES), ("phase3", PHASE3_SERVICES)],
)
def test_compose_health_accepts_selected_topology_ndjson_ps_output(profile, services):
    output = b"\n".join(
        json.dumps(
            {"Service": service, "State": "running", "Health": "healthy"}
        ).encode()
        for service in services
    )

    assert RunController._ps_cause(
        ComposeCommandResult(0, output),
        _topology(profile),
    ) is None


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
        topology=_topology("phase2"),
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
        topology=_topology("phase2"),
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


def test_phase3_controller_uses_phase3_ownership_for_health_logs_and_images(tmp_path):
    controller, trace, _clock, holder = _controller(
        tmp_path,
        statuses=(
            ArtifactsReadyStatus,
            GazeboReadyStatus,
            RuntimeRunningStatus,
            ArduPilotReadyStatus,
            CompanionReadyStatus,
            MissionReadyStatus,
            SourceFinishedStatus,
            MissionFinishedStatus,
            ScoreFinishedStatus,
            RuntimeFrozenStatus,
            TerminalNotifiedStatus,
        ),
    )

    result = controller.start(
        _template(
            tmp_path,
            runtime_profile="phase3",
            simulation={
                "seed": 9,
                "duration_sim_seconds": 2.0,
                "public_epoch_native_sim_seconds": 90.0,
                "target_real_time_factor": 0.1,
            },
        )
    )

    assert result.state == "COMPLETED"
    assert trace.index("wait artifacts-ready") < trace.index("wait gazebo-ready")
    assert trace.index("wait gazebo-ready") < trace.index("wait ardupilot-ready")
    assert trace.index("wait ardupilot-ready") < trace.index("wait companion-ready")
    assert trace.index("wait companion-ready") < trace.index("wait mission-ready")
    assert trace.index("wait mission-ready") < trace.index("wait runtime-running")
    assert trace.index("wait source-finished") < trace.index("wait mission-finished")
    assert trace.index("wait mission-finished") < trace.index("wait score-finished")
    assert trace.index("wait score-finished") < trace.index("request FINALIZING")
    assert holder["value"].services == PHASE3_SERVICES
    assert holder["log_ownership"] == _topology("phase3").ownership


def test_comp2026_controller_waits_for_runtime_before_mission_ready(tmp_path):
    controller, trace, _clock, _holder = _controller(
        tmp_path,
        statuses=(
            ArtifactsReadyStatus,
            GazeboReadyStatus,
            ArduPilotReadyStatus,
            CompanionReadyStatus,
            RuntimeRunningStatus,
            MissionReadyStatus,
            SourceFinishedStatus,
            MissionFinishedStatus,
            ScoreFinishedStatus,
            RuntimeFrozenStatus,
            TerminalNotifiedStatus,
        ),
    )

    result = controller.start(
        _comp2026_template(
            tmp_path,
            runtime_profile="phase3",
            simulation={
                "seed": 9,
                "duration_sim_seconds": 2.0,
                "public_epoch_native_sim_seconds": 90.0,
                "target_real_time_factor": 0.1,
            },
        )
    )

    assert result.state == "COMPLETED"
    assert trace.index("wait ardupilot-ready") < trace.index("wait companion-ready")
    assert trace.index("wait companion-ready") < trace.index("wait runtime-running")
    assert trace.index("wait runtime-running") < trace.index("wait mission-ready")
    assert trace.index("wait mission-ready") < trace.index("wait source-finished")


def test_search_delivery_uses_scenario_score_provenance(tmp_path):
    def replace_descent_details(path: Path) -> None:
        document = json.loads(path.read_text(encoding="utf-8"))
        document["ruleset_id"] = "search_delivery_v1"
        document["rule_results"] = []
        path.write_text(json.dumps(document), encoding="utf-8")

    controller, _trace, _clock, _holder = _controller(
        tmp_path,
        score_mutator=replace_descent_details,
        statuses=(
            ArtifactsReadyStatus,
            GazeboReadyStatus,
            ArduPilotReadyStatus,
            CompanionReadyStatus,
            RuntimeRunningStatus,
            MissionReadyStatus,
            SourceFinishedStatus,
            MissionFinishedStatus,
            ScoreFinishedStatus,
            RuntimeFrozenStatus,
            TerminalNotifiedStatus,
        ),
    )

    result = controller.start(_search_delivery_template(tmp_path))

    assert result.state == "COMPLETED"


@pytest.mark.parametrize(
    ("statuses", "reason"),
    [
        (
            (
                ArtifactsReadyStatus,
                GazeboReadyStatus,
                ArduPilotReadyStatus,
                CompanionReadyStatus,
                RuntimeFrozenStatus,
                TerminalNotifiedStatus,
            ),
            "clock_source_stall",
        ),
        (
            (
                ArtifactsReadyStatus,
                GazeboReadyStatus,
                ArduPilotReadyStatus,
                CompanionReadyStatus,
                RuntimeRunningStatus,
                RuntimeFrozenStatus,
                TerminalNotifiedStatus,
            ),
            "mission_readiness_stall",
        ),
    ],
    ids=(RuntimeRunningStatus, MissionReadyStatus),
)
def test_comp2026_timeout_identifies_the_next_missing_status(tmp_path, statuses, reason):
    controller, _trace, _clock, _holder = _controller(tmp_path, statuses=statuses)

    result = controller.start(
        _comp2026_template(
            tmp_path,
            runtime_profile="phase3",
            max_wall_seconds=2,
            finalization_wall_seconds=5,
            simulation={
                "seed": 9,
                "duration_sim_seconds": 2.0,
                "public_epoch_native_sim_seconds": 90.0,
                "target_real_time_factor": 0.1,
            },
        )
    )

    assert result.state == "FAILED"
    assert result.reason == reason


def test_phase3_completed_run_rejects_score_for_another_run(tmp_path):
    def replace_score_run_id(path: Path) -> None:
        document = json.loads(path.read_text(encoding="utf-8"))
        document["run_id"] = "11111111-1111-4111-8111-111111111111"
        path.write_text(json.dumps(document), encoding="utf-8")

    controller, _trace, _clock, _holder = _controller(
        tmp_path,
        score_mutator=replace_score_run_id,
        statuses=(
            ArtifactsReadyStatus,
            GazeboReadyStatus,
            RuntimeRunningStatus,
            ArduPilotReadyStatus,
            CompanionReadyStatus,
            MissionReadyStatus,
            SourceFinishedStatus,
            MissionFinishedStatus,
            ScoreFinishedStatus,
            RuntimeFrozenStatus,
            TerminalNotifiedStatus,
        ),
    )

    result = controller.start(
        _template(
            tmp_path,
            runtime_profile="phase3",
            simulation={
                "seed": 9,
                "duration_sim_seconds": 2.0,
                "public_epoch_native_sim_seconds": 90.0,
                "target_real_time_factor": 0.1,
            },
        )
    )

    assert result.state == "FAILED"
    assert result.reason == "scoring_provenance_invalid"


def test_phase3_completed_run_requires_native_state_tlog(tmp_path):
    def remove_native_state(run: Path) -> None:
        (run / "gazebo/state/state.tlog.zst").unlink()
        (run / "gazebo/state/state.json").write_bytes(b"{}")

    controller, _trace, _clock, _holder = _controller(
        tmp_path,
        runtime_mutator=remove_native_state,
        statuses=(
            ArtifactsReadyStatus,
            GazeboReadyStatus,
            RuntimeRunningStatus,
            ArduPilotReadyStatus,
            CompanionReadyStatus,
            MissionReadyStatus,
            SourceFinishedStatus,
            MissionFinishedStatus,
            ScoreFinishedStatus,
            RuntimeFrozenStatus,
            TerminalNotifiedStatus,
        ),
    )

    result = controller.start(
        _template(
            tmp_path,
            runtime_profile="phase3",
            simulation={
                "seed": 9,
                "duration_sim_seconds": 2.0,
                "public_epoch_native_sim_seconds": 90.0,
                "target_real_time_factor": 0.1,
            },
        )
    )

    assert result.state == "FAILED"
    manifest = json.loads(
        (tmp_path / "runs" / RUN_ID / "manifest.json").read_text(encoding="utf-8")
    )
    state = next(
        record for record in manifest["artifacts"]
        if record["relative_path"] == "gazebo/state"
    )
    assert state["validation"] == "invalid"
    assert state["detail"] == "gazebo state requires valid state.tlog.zst"


def test_phase2_preserves_synthetic_state_and_score_completion_contract(tmp_path):
    def restore_synthetic_state(run: Path) -> None:
        (run / "gazebo/state/state.tlog.zst").unlink()
        (run / "gazebo/state/state.json").write_bytes(b"{}")

    def restore_synthetic_score(path: Path) -> None:
        document = json.loads(path.read_text(encoding="utf-8"))
        path.write_text(
            json.dumps(
                {
                    "achieved_score": document["achieved_score"],
                    "maximum_available_score": document["maximum_available_score"],
                    "scoring_checksum": document["scoring_checksum"],
                    "evidence_paths": document["evidence_paths"],
                }
            ),
            encoding="utf-8",
        )

    controller, _trace, _clock, _holder = _controller(
        tmp_path,
        runtime_mutator=restore_synthetic_state,
        score_mutator=restore_synthetic_score,
    )

    result = controller.start(_template(tmp_path))

    assert result.state == "COMPLETED"


def test_git_provenance_commands_consume_one_shared_remaining_deadline(tmp_path):
    now = [10.0]
    calls = []

    def monotonic():
        return now[0]

    def runner(command, **kwargs):
        calls.append((command, kwargs["timeout"]))
        nested = command[2] == str(tmp_path / "companion/comp2026")
        if command[-2:] == ["rev-parse", "HEAD"]:
            output = (b"b" * 40 + b"\n") if nested else (b"a" * 40 + b"\n")
        else:
            output = b"?? docs/\n" if nested else b""
        now[0] += 0.5
        return SimpleNamespace(returncode=0, stdout=output)

    controller = RunController(
        project_directory=tmp_path,
        monotonic=monotonic,
        source_runner=runner,
    )

    assert controller._source_revisions(15.0) == (
        SourceRevision("drone_sim", "a" * 40, False),
        SourceRevision("comp2026", "b" * 40, True),
    )
    assert [timeout for _command, timeout in calls] == [5.0, 4.5, 4.0, 3.5]
    assert [command[2] for command, _timeout in calls] == [
        str(tmp_path),
        str(tmp_path),
        str(tmp_path / "companion/comp2026"),
        str(tmp_path / "companion/comp2026"),
    ]


def test_phase3_compose_binds_nested_revision_label_before_launch(tmp_path):
    calls = []

    def runner(command, *, env, timeout):
        calls.append((command, env.copy(), timeout))
        return SimpleNamespace(returncode=0, stdout=("b" * 40 + "\n").encode())

    runtime = ComposeRuntime(
        project_directory=tmp_path.resolve(),
        run_id=RUN_ID,
        run_directory=(tmp_path / "run").resolve(),
        config_path=(tmp_path / "run/configuration/run.json").resolve(),
        topology=_topology("phase3"),
        runner=runner,
        base_environment={},
    )

    result = runtime.bind_source_revisions(
        (
            SourceRevision("drone_sim", "a" * 40, True),
            SourceRevision("comp2026", "b" * 40, True),
        ),
        4.0,
    )

    assert result.returncode == 0
    assert calls == [
        (
            [
                "docker",
                "image",
                "inspect",
                "--format",
                '{{ index .Config.Labels "org.opencontainers.image.comp2026.revision" }}',
                "drone-sim-companion-runtime:phase3",
            ],
            {
                "COMPOSE_DISABLE_ENV_FILE": "1",
                "COMPOSE_PROFILES": "phase3",
                "SIM_COMP2026_REVISION": "b" * 40,
                "SIM_CONFIG_PATH": str((tmp_path / "run/configuration/run.json").resolve()),
                "SIM_LAUNCH_ORIGIN_JSON": (
                    '{"amsl_m":0.0,"heading_deg":0.0,'
                    '"latitude_deg":37.4003371,"longitude_deg":-122.0800351}'
                ),
                "SIM_RUN_DIRECTORY": str((tmp_path / "run").resolve()),
                "SIM_RUN_ID": RUN_ID,
            },
            4.0,
        )
    ]


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
            "wait runtime-running",
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


def test_malformed_source_finished_fails_without_trusting_its_timestamp(
    tmp_path, monkeypatch
):
    real_up = FakeCompose.up

    def malformed_source_up(self, timeout):
        result = real_up(self, timeout)
        _write_json(
            self.run_directory / ".status/source-finished.json",
            {
                "run_id": RUN_ID,
                "finished": False,
                "sim_timestamp_ns": 2_000_000_000,
            },
        )
        return result

    monkeypatch.setattr(FakeCompose, "up", malformed_source_up)
    controller, _trace, _clock, _holder = _controller(tmp_path)

    result = controller.start(_template(tmp_path))

    manifest = json.loads(
        (tmp_path / "runs" / RUN_ID / "manifest.json").read_text(encoding="utf-8")
    )
    assert result.state == "FAILED"
    assert "source-finished" in result.reason
    assert "invalid schema" in result.reason
    assert manifest["terminal_status"] == "FAILED"
    assert manifest["reason"] == result.reason
    assert manifest["simulation_timing"] == {
        "start_ns": None,
        "end_ns": None,
        "duration_ns": None,
    }


def test_malformed_runtime_frozen_stops_before_log_capture_or_artifact_validation(
    tmp_path, monkeypatch
):
    entered = []
    real_up = FakeCompose.up

    def malformed_frozen_up(self, timeout):
        result = real_up(self, timeout)
        _write_json(
            self.run_directory / ".status/runtime-frozen.json",
            {"run_id": RUN_ID, "frozen": False},
        )
        return result

    def capture_factory(**_kwargs):
        entered.append("log capture")
        raise AssertionError("log capture must not start after malformed runtime-frozen")

    def artifact_validator(*_args, **_kwargs):
        entered.append("artifact validation")
        raise AssertionError("artifacts must not be inspected before runtime freezes")

    monkeypatch.setattr(FakeCompose, "up", malformed_frozen_up)
    monkeypatch.setattr(
        "orchestration.controller.validate_regular_file", artifact_validator
    )
    monkeypatch.setattr("orchestration.controller.validate_tree", artifact_validator)
    controller, _trace, _clock, _holder = _controller(tmp_path)
    controller.log_capture_factory = capture_factory

    with pytest.raises(ProtocolFileError, match="runtime-frozen.*invalid schema"):
        controller.start(_template(tmp_path))

    assert entered == []


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
            ArtifactsReadyStatus,
            RuntimeRunningStatus,
            RuntimeFrozenStatus,
            TerminalNotifiedStatus,
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
        statuses=(ArtifactsReadyStatus, RuntimeFrozenStatus, TerminalNotifiedStatus),
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
        statuses=(RuntimeFrozenStatus, TerminalNotifiedStatus),
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


def test_one_compose_ps_timeout_is_retried_within_the_startup_deadline(tmp_path):
    controller, _trace, _clock, _holder = _controller(tmp_path)
    original = FakeCompose.ps
    timeouts: list[float] = []

    def timeout_once(self, timeout):
        timeouts.append(timeout)
        if len(timeouts) == 1:
            raise subprocess.TimeoutExpired(["docker", "compose", "ps"], timeout)
        return original(self, timeout)

    FakeCompose.ps = timeout_once
    try:
        result = controller.start(_template(tmp_path))
    finally:
        FakeCompose.ps = original

    assert result.state == "COMPLETED", result.reason
    assert len(timeouts) >= 2
    assert timeouts[0] < 45


def test_first_observed_abort_wins_runtime_failure_race_and_later_cause_is_diagnostic(tmp_path):
    controller, _trace, _clock, _holder = _controller(
        tmp_path,
        statuses=(
            ArtifactsReadyStatus,
            RuntimeRunningStatus,
            RuntimeFrozenStatus,
            TerminalNotifiedStatus,
        ),
    )
    original = FakeCompose.up

    def raced_up(self, timeout):
        result = original(self, timeout)
        store = StatusStore((tmp_path / "runs").resolve())
        store.request_finalization(RUN_ID, "ABORTED", "operator_abort")
        _write_json(
            self.run_directory / ".status/runtime-failure.json",
            status_document(
                RuntimeFailureStatus(RUN_ID, "artifacts", "recorder_failed", ())
            ),
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
        ("startup_deadline", (RuntimeFrozenStatus, TerminalNotifiedStatus), False, "startup_deadline"),
        (
            "clock_stall",
            (ArtifactsReadyStatus, RuntimeRunningStatus, RuntimeFrozenStatus, TerminalNotifiedStatus),
            False,
            "clock_source_stall",
        ),
        (
            "log_capture",
            (
                ArtifactsReadyStatus,
                RuntimeRunningStatus,
                SourceFinishedStatus,
                RuntimeFrozenStatus,
                TerminalNotifiedStatus,
            ),
            True,
            "docker_log_capture_failed",
        ),
        (
            "finalization_deadline",
            (
                ArtifactsReadyStatus,
                RuntimeRunningStatus,
                SourceFinishedStatus,
                TerminalNotifiedStatus,
            ),
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
            ArtifactsReadyStatus,
            RuntimeRunningStatus,
            SourceFinishedStatus,
            RuntimeFrozenStatus,
            TerminalNotifiedStatus,
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
        statuses=(RuntimeFrozenStatus, TerminalNotifiedStatus),
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
        statuses=(
            ArtifactsReadyStatus,
            RuntimeRunningStatus,
            RuntimeFrozenStatus,
            TerminalNotifiedStatus,
        ),
    )
    original = FakeCompose.up

    def failing_up(self, timeout):
        result = original(self, timeout)
        _write_json(
            self.run_directory / ".status/runtime-failure.json",
            status_document(
                RuntimeFailureStatus(
                    RUN_ID, "artifacts", "observer_encoder_failed", ()
                )
            ),
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


def test_compose_up_failure_preserves_docker_error(tmp_path, monkeypatch):
    stream = io.StringIO()
    controller, trace, _clock, _holder = _controller(tmp_path, event_stream=stream)
    monkeypatch.setattr(
        FakeCompose, "up", lambda self, timeout: ComposeCommandResult(
            1, b"conflicting options: port exposing and the container type network mode"
        )
    )
    result = controller.start(_template(tmp_path))
    assert result.reason == "compose_up_failed"
    assert trace[-1] == "compose down"
    assert "conflicting options: port exposing" in stream.getvalue()


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
        def read_runtime_status(self, run_id, status_type, deadline_check=None):
            if status_type is TerminalNotifiedStatus:
                raise OSError("status volume became unreadable")
            return super().read_runtime_status(run_id, status_type, deadline_check)

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
        def read_runtime_status(self, run_id, status_type, deadline_check=None):
            assert status_type is RuntimeFrozenStatus
            received.append(deadline_check)
            clock.value += 10
            return RuntimeFrozenStatus(run_id)

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
        _topology("phase2"),
        RuntimeFrozenStatus,
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
        def read_runtime_status(self, run_id, status_type, deadline_check=None):
            if status_type is TerminalNotifiedStatus:
                received.append(deadline_check)
                clock.value += 100
                return TerminalNotifiedStatus(run_id)
            if status_type is RuntimeFailureStatus:
                runtime_failure_checks.append(deadline_check)
            return super().read_runtime_status(run_id, status_type, deadline_check)

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
        (
            lambda report: report["records"][0].update({"sha256": "b" * 64}),
            "checksum",
        ),
        (
            lambda report: report["records"][0].update(
                {"size_bytes": report["records"][0]["size_bytes"] + 1}
            ),
            "size mismatch",
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
