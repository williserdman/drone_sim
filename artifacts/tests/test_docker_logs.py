"""Host-side Docker log capture contract tests."""

from __future__ import annotations

from dataclasses import FrozenInstanceError
import json
import os
from pathlib import Path
import subprocess

import pytest

from artifacts import (
    DockerLogCapture,
    DockerLogCaptureError,
    DockerLogCommandResult,
)


RUN_ID = "00000000-0000-4000-8000-000000000555"
MODULES = (
    "orchestration",
    "artifacts",
    "companion",
    "ardupilot_sitl",
    "gazebo",
    "electromagnet",
    "scorekeeper",
)
OWNERSHIP = tuple((f"service-{module}", module) for module in MODULES)


def _event(
    module: str,
    *,
    event: str = "ready",
    wall_timestamp: str = "2026-08-24T11:30:00+02:00",
    sim_timestamp: object = 1.25,
    fields: object | None = None,
) -> bytes:
    payload = {
        "run_id": RUN_ID,
        "module": module,
        "severity": "INFO",
        "event": event,
        "sim_timestamp": sim_timestamp,
        "wall_timestamp": wall_timestamp,
        "fields": {"source": module} if fields is None else fields,
    }
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


class FakeRunner:
    def __init__(self, results: dict[str, DockerLogCommandResult]) -> None:
        self.results = results
        self.calls: list[list[str]] = []

    def __call__(self, command: list[str]) -> DockerLogCommandResult:
        self.calls.append(command)
        return self.results[command[-1]]


def _valid_results() -> dict[str, DockerLogCommandResult]:
    return {
        service: DockerLogCommandResult(0, _event(module) + b"\n")
        for service, module in OWNERSHIP
    }


def _capture(
    tmp_path: Path,
    results: dict[str, DockerLogCommandResult] | None = None,
    *,
    ownership=OWNERSHIP,
    project="phase2-run",
    host_events=False,
):
    runner = FakeRunner(_valid_results() if results is None else results)
    options = {"host_events": True} if host_events else {}
    capture = DockerLogCapture(
        run_directory=tmp_path,
        project_name=project,
        ownership=ownership,
        run_id=RUN_ID,
        command_runner=runner,
        **options,
    )
    return capture, runner


def test_capture_invokes_one_exact_argument_array_per_service_in_mapping_order(tmp_path):
    """A shell or aggregate Compose call could corrupt ownership attribution."""
    capture, runner = _capture(tmp_path)

    result = capture.capture()

    assert result.succeeded is True
    assert runner.calls == [
        [
            "docker",
            "compose",
            "-p",
            "phase2-run",
            "logs",
            "--no-color",
            "--no-log-prefix",
            service,
        ]
        for service, _module in OWNERSHIP
    ]


def test_capture_checks_deadline_during_per_line_partitioning(tmp_path):
    results = _valid_results()
    service, module = OWNERSHIP[0]
    results[service] = DockerLogCommandResult(
        0, b"\n".join(_event(module, event=f"event-{index}") for index in range(20))
    )
    checks = 0

    def deadline_check():
        nonlocal checks
        checks += 1
        if checks == 15:
            raise TimeoutError("finalization_deadline")

    runner = FakeRunner(results)
    capture = DockerLogCapture(
        run_directory=tmp_path,
        project_name="phase2-run",
        ownership=OWNERSHIP,
        run_id=RUN_ID,
        command_runner=runner,
        deadline_check=deadline_check,
    )

    with pytest.raises(DockerLogCaptureError, match="finalization_deadline") as raised:
        capture.capture()

    assert raised.value.result.succeeded is False


def test_capture_preserves_each_returned_byte_exactly(tmp_path):
    """Decoding or line normalization would destroy raw diagnostic evidence."""
    arbitrary = b"third party\n\ninvalid: \xff\xfe\nlast line without newline"
    results = _valid_results()
    first_service, first_module = OWNERSHIP[0]
    results[first_service] = DockerLogCommandResult(
        0, arbitrary + b"\n" + _event(first_module)
    )
    capture, _runner = _capture(tmp_path, results)

    capture.capture()

    assert (tmp_path / f"logs/docker/{first_service}.log").read_bytes() == results[
        first_service
    ].output


def test_capture_keeps_unclaimed_json_and_non_json_only_in_raw_log(tmp_path):
    """Third-party JSON must not be reclassified as an owned event."""
    service, module = OWNERSHIP[0]
    raw_only = b'plain text\n{"message":"third party","nested":{"run_id":"not top-level"}}\n42\n'
    results = _valid_results()
    results[service] = DockerLogCommandResult(0, raw_only + _event(module) + b"\n")
    capture, _runner = _capture(tmp_path, results)

    capture.capture()

    structured = (tmp_path / f"logs/{module}.jsonl").read_bytes()
    assert structured.count(b"\n") == 1
    assert b"third party" not in structured
    assert (tmp_path / f"logs/docker/{service}.log").read_bytes().startswith(raw_only)


def test_capture_routes_valid_events_in_source_line_order_through_canonical_schema(tmp_path):
    """Routing out of order or bypassing canonical UTC serialization would corrupt logs."""
    service, module = OWNERSHIP[1]
    results = _valid_results()
    results[service] = DockerLogCommandResult(
        0,
        _event(module, event="first", sim_timestamp=None)
        + b"\n"
        + _event(module, event="second", wall_timestamp="2026-08-24T09:30:01Z")
        + b"\n",
    )
    capture, _runner = _capture(tmp_path, results)

    result = capture.capture()

    assert result.structured_paths == tuple(f"logs/{module}.jsonl" for module in MODULES)
    lines = (tmp_path / f"logs/{module}.jsonl").read_text(encoding="utf-8").splitlines()
    assert [json.loads(line)["event"] for line in lines] == ["first", "second"]
    assert json.loads(lines[0]) == {
        "event": "first",
        "fields": {"source": "artifacts"},
        "module": "artifacts",
        "run_id": RUN_ID,
        "severity": "INFO",
        "sim_timestamp": None,
        "wall_timestamp": "2026-08-24T09:30:00Z",
    }


def test_capture_accepts_finite_fractional_numbers_nested_in_fields(tmp_path):
    """Parsing valid JSON fractions into an unserializable type would reject owned events."""
    service, module = OWNERSHIP[0]
    results = _valid_results()
    results[service] = DockerLogCommandResult(
        0,
        _event(
            module,
            fields={"measurement": 1.5, "nested": {"samples": [0.25, 2]}},
        )
        + b"\n",
    )
    capture, _runner = _capture(tmp_path, results)

    capture.capture()

    payload = json.loads((tmp_path / f"logs/{module}.jsonl").read_text())
    assert payload["fields"] == {
        "measurement": 1.5,
        "nested": {"samples": [0.25, 2]},
    }


@pytest.mark.parametrize(
    ("malformed", "detail"),
    [
        (
            b'{"run_id":"wrong","module":"orchestration","severity":"INFO","event":"bad","sim_timestamp":0,"wall_timestamp":"2026-08-24T09:30:00Z","fields":{}}',
            "run_id",
        ),
        (
            b'{"run_id":"' + RUN_ID.encode() + b'","module":"unknown","severity":"INFO","event":"bad","sim_timestamp":0,"wall_timestamp":"2026-08-24T09:30:00Z","fields":{}}',
            "module",
        ),
        (
            b'{"run_id":"' + RUN_ID.encode() + b'","module":"orchestration","severity":"INFO","event":"bad","sim_timestamp":0,"wall_timestamp":"2026-08-24T09:30:00Z"}',
            "top-level",
        ),
        (
            b'{"run_id":"' + RUN_ID.encode() + b'","module":"orchestration","severity":"INFO","event":"bad","sim_timestamp":0,"wall_timestamp":"2026-08-24T09:30:00Z","fields":{},"extra":1}',
            "top-level",
        ),
        (_event("orchestration").replace(b'"severity":"INFO"', b'"severity":""'), "severity"),
        (_event("orchestration").replace(b'"event":"ready"', b'"event":7'), "event"),
        (_event("orchestration").replace(b'"sim_timestamp":1.25', b'"sim_timestamp":true'), "sim_timestamp"),
        (_event("orchestration").replace(b'"sim_timestamp":1.25', b'"sim_timestamp":NaN'), "finite"),
        (_event("orchestration").replace(b'"sim_timestamp":1.25', b'"sim_timestamp":Infinity'), "finite"),
        (_event("orchestration", wall_timestamp="2026-08-24T09:30:00"), "timezone-aware"),
        (_event("orchestration", wall_timestamp="2026-08-24x09:30:00+00:00"), "profile"),
        (_event("orchestration", wall_timestamp="2026-08-24T09:30:00.1234567Z"), "profile"),
        (_event("orchestration", wall_timestamp="2026-08-24T09:30:00+00:00:30"), "profile"),
        (_event("orchestration", wall_timestamp="not-a-time"), "wall_timestamp"),
        (_event("orchestration", fields=[]), "fields"),
        (_event("orchestration", fields={"nested": [float("inf")]}), "finite"),
        (_event("orchestration", fields={"run_id": "collision"}), "common field"),
        (
            b'{"run_id":"' + RUN_ID.encode() + b'","run_id":"other","module":"orchestration","severity":"INFO","event":"bad","sim_timestamp":0,"wall_timestamp":"2026-08-24T09:30:00Z","fields":{}}',
            "duplicate",
        ),
        (
            b'{"run_id":"' + RUN_ID.encode() + b'","module":"orchestration","severity":"INFO","event":"bad","sim_timestamp":0,"wall_timestamp":"2026-08-24T09:30:00Z","fields":{"nested":{"key":1,"key":2}}}',
            "duplicate",
        ),
        (
            b'{"run_id":"' + RUN_ID.encode() + b'","module":"orchestration","severity":"INFO","event":"bad","sim_timestamp":0,"wall_timestamp":"2026-08-24T09:30:00Z","fields":{"invalid_unicode":"\\ud800"}}',
            "surrogate",
        ),
    ],
)
def test_malformed_attempted_event_fails_closed_but_preserves_all_raw_logs(
    tmp_path, malformed, detail
):
    """Silently dropping a claimed common event would hide owned-process corruption."""
    service, module = OWNERSHIP[0]
    results = _valid_results()
    results[service] = DockerLogCommandResult(
        0, _event(module) + b"\n" + malformed + b"\n"
    )
    capture, runner = _capture(tmp_path, results)

    with pytest.raises(DockerLogCaptureError) as raised:
        capture.capture()

    assert len(runner.calls) == 7
    assert any(detail in diagnostic.detail for diagnostic in raised.value.result.diagnostics)
    for captured_service, _captured_module in OWNERSHIP:
        assert (tmp_path / f"logs/docker/{captured_service}.log").read_bytes() == results[
            captured_service
        ].output
    assert not any((tmp_path / f"logs/{name}.jsonl").exists() for name in MODULES)
    assert list((tmp_path / "logs").glob("*.partial")) == []


def test_event_from_wrong_owning_service_fails_closed(tmp_path):
    """Trusting an event's self-declared module would permit cross-module spoofing."""
    service, _module = OWNERSHIP[0]
    results = _valid_results()
    results[service] = DockerLogCommandResult(0, _event("artifacts") + b"\n")
    capture, _runner = _capture(tmp_path, results)

    with pytest.raises(DockerLogCaptureError) as raised:
        capture.capture()

    assert any("ownership" in item.detail for item in raised.value.result.diagnostics)


@pytest.mark.parametrize(
    ("resource_line", "detail"),
    [
        (
            b'{"run_id":"'
            + RUN_ID.encode()
            + b'","module":"orchestration","severity":"INFO","event":"deep","sim_timestamp":0,"wall_timestamp":"2026-08-24T09:30:00Z","fields":{"nested":'
            + b"[" * 1_100
            + b"0"
            + b"]" * 1_100
            + b"}}",
            "resource",
        ),
        (
            b'{"run_id":"'
            + RUN_ID.encode()
            + b'","module":"orchestration","severity":"INFO","event":"huge","sim_timestamp":0,"wall_timestamp":"2026-08-24T09:30:00Z","fields":{"integer":'
            + b"9" * 5_000
            + b"}}",
            "integer",
        ),
    ],
    ids=("deep-nesting", "huge-integer"),
)
def test_json_resource_failures_are_typed_and_publish_every_raw_stream(
    tmp_path, resource_line, detail
):
    """Parser resource limits must not escape before recoverable raw publication."""
    service, module = OWNERSHIP[0]
    results = _valid_results()
    results[service] = DockerLogCommandResult(
        0,
        _event(module) + b"\n" + resource_line + b"\n",
    )
    capture, runner = _capture(tmp_path, results)

    with pytest.raises(DockerLogCaptureError) as raised:
        capture.capture()

    assert len(runner.calls) == 7
    assert any(detail in item.detail for item in raised.value.result.diagnostics)
    assert raised.value.result.raw_paths == tuple(
        f"logs/docker/{captured_service}.log"
        for captured_service, _captured_module in OWNERSHIP
    )
    for captured_service, _captured_module in OWNERSHIP:
        assert (tmp_path / f"logs/docker/{captured_service}.log").read_bytes() == results[
            captured_service
        ].output
    assert not any((tmp_path / f"logs/{name}.jsonl").exists() for name in MODULES)


def test_missing_required_module_stream_fails_with_sorted_explicit_modules(tmp_path):
    """Publishing an empty required module file would mask a silent process."""
    missing_service, _module = OWNERSHIP[-1]
    results = _valid_results()
    results[missing_service] = DockerLogCommandResult(0, b"third party only\n")
    capture, _runner = _capture(tmp_path, results)

    with pytest.raises(DockerLogCaptureError) as raised:
        capture.capture()

    assert raised.value.result.missing_modules == ("scorekeeper",)
    assert "scorekeeper" in raised.value.result.diagnostics[-1].detail
    assert not (tmp_path / "logs/scorekeeper.jsonl").exists()


def test_nonzero_command_is_typed_failure_after_all_services_and_retains_output(tmp_path):
    """A failing service must not suppress its bytes or later service diagnostics."""
    failed_service, failed_module = OWNERSHIP[2]
    results = _valid_results()
    results[failed_service] = DockerLogCommandResult(
        23, b"compose diagnostic\n" + _event(failed_module) + b"\n"
    )
    capture, runner = _capture(tmp_path, results)

    with pytest.raises(DockerLogCaptureError) as raised:
        capture.capture()

    assert len(runner.calls) == 7
    assert raised.value.result.command_failures == (failed_service,)
    assert (tmp_path / f"logs/docker/{failed_service}.log").read_bytes() == results[
        failed_service
    ].output
    assert not (tmp_path / "logs/orchestration.jsonl").exists()


def test_runner_exception_is_typed_and_does_not_stop_later_captures(tmp_path):
    """An adapter exception must not abort collection of independent raw evidence."""
    calls = []
    later_service = OWNERSHIP[-1][0]

    def runner(command):
        calls.append(command)
        if command[-1] == OWNERSHIP[0][0]:
            raise OSError("docker unavailable")
        module = dict(OWNERSHIP)[command[-1]]
        return DockerLogCommandResult(0, _event(module) + b"\n")

    capture = DockerLogCapture(tmp_path, "phase2-run", OWNERSHIP, RUN_ID, runner)

    with pytest.raises(DockerLogCaptureError) as raised:
        capture.capture()

    assert calls[-1][-1] == later_service
    assert raised.value.result.command_failures == (OWNERSHIP[0][0],)
    assert (tmp_path / f"logs/docker/{OWNERSHIP[0][0]}.log").read_bytes() == b""


@pytest.mark.parametrize(
    ("exception", "expected"),
    [
        (
            subprocess.CalledProcessError(7, ["docker"], stderr=b"stderr-only"),
            b"stderr-only",
        ),
        (
            subprocess.CalledProcessError(
                7,
                ["docker"],
                output=b"stdout",
                stderr=b"stderr",
            ),
            b"stdoutstderr",
        ),
        (
            subprocess.CalledProcessError(
                7,
                ["docker"],
                output="snowman: ☃\n",
                stderr="error: é",
            ),
            "snowman: ☃\nerror: é".encode(),
        ),
    ],
)
def test_runner_exception_preserves_normalized_stdout_and_stderr_once(
    tmp_path, exception, expected
):
    """Discarding stderr or duplicating stdout would corrupt command evidence."""
    service = OWNERSHIP[0][0]
    calls = []

    def runner(command):
        calls.append(command)
        if command[-1] == service:
            raise exception
        module = dict(OWNERSHIP)[command[-1]]
        return DockerLogCommandResult(0, _event(module) + b"\n")

    capture = DockerLogCapture(tmp_path, "phase2-run", OWNERSHIP, RUN_ID, runner)

    with pytest.raises(DockerLogCaptureError):
        capture.capture()

    assert len(calls) == 7
    assert (tmp_path / f"logs/docker/{service}.log").read_bytes() == expected


@pytest.mark.parametrize(
    ("project", "service"),
    [
        ("-project", "valid"),
        ("Project", "valid"),
        ("project/name", "valid"),
        ("valid", "-service"),
        ("valid", "../service"),
        ("valid", "service/name"),
        ("valid", "service\x00name"),
    ],
)
def test_unsafe_identifiers_are_rejected_before_invoking_docker(
    tmp_path, project, service
):
    """Option-like or path-bearing names must not alter commands or file targets."""
    ownership = ((service, "orchestration"),) + OWNERSHIP[1:]
    capture, runner = _capture(tmp_path, ownership=ownership, project=project)

    with pytest.raises(ValueError, match="identifier"):
        capture.capture()

    assert runner.calls == []


@pytest.mark.parametrize(
    "ownership",
    [
        OWNERSHIP[:-1],
        OWNERSHIP + (("extra-service", "unknown"),),
        OWNERSHIP + ((OWNERSHIP[0][0], "scorekeeper"),),
        OWNERSHIP + (("other-service", "scorekeeper"),),
    ],
)
def test_ownership_must_be_an_exact_one_to_one_cover_before_invoking_docker(
    tmp_path, ownership
):
    """Ambiguous or incomplete ownership makes source attribution impossible."""
    capture, runner = _capture(tmp_path, ownership=ownership)

    with pytest.raises(ValueError, match="ownership"):
        capture.capture()

    assert runner.calls == []


def test_run_directory_must_be_absolute(tmp_path, monkeypatch):
    """Resolving a relative root later could redirect durable output."""
    monkeypatch.chdir(tmp_path)
    runner = FakeRunner(_valid_results())
    capture = DockerLogCapture(Path("relative"), "phase2-run", OWNERSHIP, RUN_ID, runner)

    with pytest.raises(ValueError, match="absolute"):
        capture.capture()

    assert runner.calls == []


def test_symlinked_run_or_log_directory_is_rejected_before_invoking_docker(tmp_path):
    """Following a directory symlink could publish evidence outside the allocated run."""
    real_root = tmp_path / "real"
    real_root.mkdir()
    linked_root = tmp_path / "linked"
    linked_root.symlink_to(real_root, target_is_directory=True)
    capture, runner = _capture(linked_root)

    with pytest.raises(DockerLogCaptureError, match="run directory"):
        capture.capture()

    assert runner.calls == []


def test_symlinked_logs_directory_rejection_closes_retained_run_descriptor(tmp_path):
    """A setup rejection must not leak the run-root descriptor into the controller."""
    outside = tmp_path / "outside"
    outside.mkdir()
    (tmp_path / "logs").symlink_to(outside, target_is_directory=True)
    capture, runner = _capture(tmp_path)
    before = len(list(Path("/proc/self/fd").iterdir()))

    with pytest.raises(DockerLogCaptureError, match="logs"):
        capture.capture()

    after = len(list(Path("/proc/self/fd").iterdir()))
    assert after == before
    assert runner.calls == []


@pytest.mark.parametrize("collision", ["final", "partial", "symlink", "hardlink", "fifo"])
def test_preexisting_raw_targets_are_rejected_without_clobber(tmp_path, collision):
    """Existing raw paths belong to prior or foreign writers and must remain untouched."""
    docker = tmp_path / "logs/docker"
    docker.mkdir(parents=True)
    service = OWNERSHIP[0][0]
    target = docker / f"{service}.log"
    if collision == "partial":
        target = target.with_name(target.name + ".partial")
        target.write_bytes(b"before")
    elif collision == "symlink":
        outside = tmp_path / "outside"
        outside.write_bytes(b"outside")
        target.symlink_to(outside)
    elif collision == "hardlink":
        outside = tmp_path / "outside"
        outside.write_bytes(b"outside")
        os.link(outside, target)
    elif collision == "fifo":
        os.mkfifo(target)
    else:
        target.write_bytes(b"before")
    before = target.lstat()
    capture, runner = _capture(tmp_path)

    with pytest.raises(DockerLogCaptureError, match="preexisting"):
        capture.capture()

    after = target.lstat()
    assert (before.st_dev, before.st_ino, before.st_mode) == (
        after.st_dev,
        after.st_ino,
        after.st_mode,
    )
    assert runner.calls == []


def test_preexisting_structured_target_is_rejected_before_capture(tmp_path):
    """A prior structured stream must never be appended to or replaced."""
    logs = tmp_path / "logs"
    logs.mkdir()
    target = logs / "gazebo.jsonl"
    target.write_bytes(b"before")
    capture, runner = _capture(tmp_path)

    with pytest.raises(DockerLogCaptureError, match="preexisting"):
        capture.capture()

    assert target.read_bytes() == b"before"
    assert runner.calls == []


def _write_host_events(tmp_path: Path, contents: bytes) -> Path:
    path = tmp_path / "logs/orchestration-host.jsonl.partial"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(contents)
    return path


def test_requested_host_events_merge_chronologically_with_stable_source_ties(tmp_path):
    """Nondeterministic host/Compose merge order would make orchestration evidence unstable."""
    service, module = OWNERSHIP[0]
    results = _valid_results()
    results[service] = DockerLogCommandResult(
        0,
        _event(
            module,
            event="compose-late",
            wall_timestamp="2026-08-24T09:30:02Z",
        )
        + b"\n"
        + _event(
            module,
            event="compose-early",
            wall_timestamp="2026-08-24T09:30:00Z",
        )
        + b"\n",
    )
    host_contents = (
        _event(
            "orchestration",
            event="host-tie",
            wall_timestamp="2026-08-24T11:30:02+02:00",
        )
        + b"\n"
        + _event(
            "orchestration",
            event="host-middle",
            wall_timestamp="2026-08-24T09:30:01Z",
        )
        + b"\n"
    )
    host_path = _write_host_events(tmp_path, host_contents)
    before = host_path.stat()
    capture, _runner = _capture(tmp_path, results, host_events=True)

    result = capture.capture()

    events = [
        json.loads(line)["event"]
        for line in (tmp_path / "logs/orchestration.jsonl").read_text().splitlines()
    ]
    assert events == ["compose-early", "host-middle", "compose-late", "host-tie"]
    assert result.recovery_paths == ("logs/orchestration-host.jsonl.partial",)
    assert host_path.read_bytes() == host_contents
    after = host_path.stat()
    assert (before.st_dev, before.st_ino, before.st_mode, before.st_nlink) == (
        after.st_dev,
        after.st_ino,
        after.st_mode,
        after.st_nlink,
    )


def test_requested_host_stream_can_supply_orchestration_module_coverage(tmp_path):
    """Host-owned orchestration events must count without weakening service ownership."""
    service, _module = OWNERSHIP[0]
    results = _valid_results()
    results[service] = DockerLogCommandResult(0, b"Compose progress only\n")
    _write_host_events(tmp_path, _event("orchestration", event="operator-start") + b"\n")
    capture, _runner = _capture(tmp_path, results, host_events=True)

    result = capture.capture()

    assert result.missing_modules == ()
    assert json.loads((tmp_path / "logs/orchestration.jsonl").read_text())["event"] == "operator-start"


@pytest.mark.parametrize(
    "host_contents",
    [
        b"plain host text\n",
        b'{"message":"unclaimed host JSON"}\n',
        _event("artifacts") + b"\n",
        _event("orchestration").replace(RUN_ID.encode(), b"wrong-run") + b"\n",
        b'{"run_id":"' + RUN_ID.encode() + b'"}\n',
    ],
)
def test_requested_host_stream_is_structured_only_and_preserved_on_failure(
    tmp_path, host_contents
):
    """Unclaimed or misattributed host lines must fail without becoming Docker raw output."""
    host_path = _write_host_events(tmp_path, host_contents)
    capture, runner = _capture(tmp_path, host_events=True)

    with pytest.raises(DockerLogCaptureError) as raised:
        capture.capture()

    assert len(runner.calls) == 7
    assert raised.value.result.recovery_paths == (
        "logs/orchestration-host.jsonl.partial",
    )
    assert host_path.read_bytes() == host_contents
    assert all((tmp_path / path).is_file() for path in raised.value.result.raw_paths)
    assert not any((tmp_path / f"logs/{module}.jsonl").exists() for module in MODULES)


@pytest.mark.parametrize("unsafe", ["missing", "symlink", "hardlink"])
def test_requested_missing_or_unsafe_host_stream_is_typed_after_raw_capture(
    tmp_path, unsafe
):
    """A requested host source must be an existing single-link regular file."""
    host_path = tmp_path / "logs/orchestration-host.jsonl.partial"
    host_path.parent.mkdir(parents=True)
    outside = tmp_path / "outside-host"
    if unsafe == "symlink":
        outside.write_bytes(_event("orchestration") + b"\n")
        host_path.symlink_to(outside)
    elif unsafe == "hardlink":
        outside.write_bytes(_event("orchestration") + b"\n")
        os.link(outside, host_path)
    capture, runner = _capture(tmp_path, host_events=True)

    with pytest.raises(DockerLogCaptureError) as raised:
        capture.capture()

    assert len(runner.calls) == 7
    assert len(raised.value.result.raw_paths) == 7
    assert any("host" in item.detail for item in raised.value.result.diagnostics)
    assert not any((tmp_path / f"logs/{module}.jsonl").exists() for module in MODULES)


def test_unrequested_host_stream_is_not_required(tmp_path):
    """Callers that do not request host events retain the original capture contract."""
    capture, _runner = _capture(tmp_path, host_events=False)

    result = capture.capture()

    assert result.succeeded is True
    assert result.recovery_paths == ()


def test_candidates_are_file_fsynced_then_published_no_clobber_and_directory_fsynced(
    tmp_path, monkeypatch
):
    """Skipping either durability barrier could report files lost after a crash."""
    events = []
    real_fsync = os.fsync
    real_link = os.link

    def recording_fsync(fd):
        kind = "directory" if os.path.isdir(f"/proc/self/fd/{fd}") else "file"
        events.append(f"fsync-{kind}")
        return real_fsync(fd)

    def recording_link(source, target, **kwargs):
        events.append(f"link-{target}")
        return real_link(source, target, **kwargs)

    monkeypatch.setattr(os, "fsync", recording_fsync)
    monkeypatch.setattr(os, "link", recording_link)
    capture, _runner = _capture(tmp_path)

    capture.capture()

    assert events.count("fsync-file") >= 14
    for service, _module in OWNERSHIP:
        link_index = events.index(f"link-{service}.log")
        assert "fsync-file" in events[:link_index]
    first_structured_link = min(
        events.index(f"link-{module}.jsonl") for module in MODULES
    )
    assert events[:first_structured_link].count("fsync-file") >= 14
    assert events[-1] == "fsync-directory"
    assert not list((tmp_path / "logs").rglob("*.partial"))


def test_structured_publication_failure_reports_partial_set_without_false_success(
    tmp_path, monkeypatch
):
    """POSIX multi-file publication failure must expose any already-linked subset."""
    real_link = os.link

    def fail_on_companion(source, target, **kwargs):
        if target == "companion.jsonl":
            raise OSError("injected structured publication failure")
        return real_link(source, target, **kwargs)

    monkeypatch.setattr(os, "link", fail_on_companion)
    capture, _runner = _capture(tmp_path)

    with pytest.raises(DockerLogCaptureError) as raised:
        capture.capture()

    result = raised.value.result
    assert result.succeeded is False
    assert result.structured_paths == (
        "logs/orchestration.jsonl",
        "logs/artifacts.jsonl",
    )
    assert any("publication" in item.detail for item in result.diagnostics)
    for service, _module in OWNERSHIP:
        assert (tmp_path / f"logs/docker/{service}.log").is_file()
    assert not (tmp_path / "logs/companion.jsonl").exists()
    assert not list((tmp_path / "logs").rglob("*.partial"))


def test_failure_after_structured_link_reports_the_linked_file_as_partial_publication(
    tmp_path, monkeypatch
):
    """A failure after link must not omit a structured file already visible by name."""
    real_unlink = os.unlink
    injected = False

    def fail_first_companion_partial_unlink(path, **kwargs):
        nonlocal injected
        if path == "companion.jsonl.partial" and not injected:
            injected = True
            raise OSError("injected post-link cleanup failure")
        return real_unlink(path, **kwargs)

    monkeypatch.setattr(os, "unlink", fail_first_companion_partial_unlink)
    capture, _runner = _capture(tmp_path)

    with pytest.raises(DockerLogCaptureError) as raised:
        capture.capture()

    assert raised.value.result.structured_paths == (
        "logs/orchestration.jsonl",
        "logs/artifacts.jsonl",
        "logs/companion.jsonl",
    )
    assert (tmp_path / "logs/companion.jsonl").is_file()
    assert not (tmp_path / "logs/companion.jsonl.partial").exists()


def test_failure_after_raw_link_never_returns_false_success(tmp_path, monkeypatch):
    """A visible raw file does not erase a failed no-clobber publication step."""
    service = OWNERSHIP[0][0]
    partial_name = f"{service}.log.partial"
    real_unlink = os.unlink
    injected = False

    def fail_first_raw_partial_unlink(path, **kwargs):
        nonlocal injected
        if path == partial_name and not injected:
            injected = True
            raise OSError("injected raw post-link failure")
        return real_unlink(path, **kwargs)

    monkeypatch.setattr(os, "unlink", fail_first_raw_partial_unlink)
    capture, _runner = _capture(tmp_path)

    with pytest.raises(DockerLogCaptureError) as raised:
        capture.capture()

    assert f"logs/docker/{service}.log" in raised.value.result.raw_paths
    assert any("raw publication failed" in item.detail for item in raised.value.result.diagnostics)
    assert not any((tmp_path / f"logs/{module}.jsonl").exists() for module in MODULES)


def test_result_diagnostics_and_command_results_are_immutable(tmp_path):
    """Controller decisions must not depend on mutable post-capture diagnostics."""
    capture, _runner = _capture(tmp_path)
    result = capture.capture()

    with pytest.raises(FrozenInstanceError):
        result.succeeded = False
    with pytest.raises(FrozenInstanceError):
        DockerLogCommandResult(0, b"").returncode = 1


def test_raw_creation_failure_reports_cleanup_unlink_leftover_before_tracking(
    tmp_path, monkeypatch
):
    """A factory-owned partial must remain visible in the immutable failure result."""
    service = OWNERSHIP[0][0]
    partial_name = f"{service}.log.partial"
    real_open = os.open
    real_fsync = os.fsync
    real_unlink = os.unlink
    target_fd = None
    injected_create_failure = False

    def record_target(path, flags, mode=0o777, *, dir_fd=None):
        nonlocal target_fd
        fd = real_open(path, flags, mode, dir_fd=dir_fd)
        if path == partial_name:
            target_fd = fd
        return fd

    def fail_target_file_fsync(fd):
        nonlocal injected_create_failure
        if fd == target_fd and not injected_create_failure:
            injected_create_failure = True
            raise OSError("injected raw candidate creation failure")
        return real_fsync(fd)

    def fail_target_cleanup_unlink(path, **kwargs):
        if path == partial_name:
            raise OSError("injected raw creation cleanup unlink failure")
        return real_unlink(path, **kwargs)

    monkeypatch.setattr(os, "open", record_target)
    monkeypatch.setattr(os, "fsync", fail_target_file_fsync)
    monkeypatch.setattr(os, "unlink", fail_target_cleanup_unlink)
    capture, _runner = _capture(tmp_path)

    with pytest.raises(DockerLogCaptureError) as raised:
        capture.capture()

    result = raised.value.result
    expected = f"logs/docker/{partial_name}"
    assert result.leftover_partials == (expected,)
    assert any("cleanup unlink" in item.detail for item in result.diagnostics)
    assert (tmp_path / expected).is_file()


@pytest.mark.parametrize(
    "control_flow",
    [KeyboardInterrupt(), SystemExit(130)],
    ids=("keyboard-interrupt", "system-exit"),
)
def test_candidate_creation_cleanup_preserves_control_flow_baseexception(
    tmp_path, monkeypatch, control_flow
):
    """Cleanup must not convert cancellation or process exit into a capture error."""
    service = OWNERSHIP[0][0]
    partial_name = f"{service}.log.partial"
    real_open = os.open
    real_fsync = os.fsync
    real_unlink = os.unlink
    target_fd = None
    injected = False
    cleanup_unlinked = False
    cleanup_fsynced = False

    def record_target(path, flags, mode=0o777, *, dir_fd=None):
        nonlocal target_fd
        fd = real_open(path, flags, mode, dir_fd=dir_fd)
        if path == partial_name:
            target_fd = fd
        return fd

    def interrupt_target_fsync(fd):
        nonlocal injected, cleanup_fsynced
        if fd == target_fd and not injected:
            injected = True
            raise control_flow
        result = real_fsync(fd)
        if cleanup_unlinked:
            cleanup_fsynced = True
        return result

    def mark_cleanup_unlink(path, **kwargs):
        nonlocal cleanup_unlinked
        result = real_unlink(path, **kwargs)
        if path == partial_name:
            cleanup_unlinked = True
        return result

    monkeypatch.setattr(os, "open", record_target)
    monkeypatch.setattr(os, "fsync", interrupt_target_fsync)
    monkeypatch.setattr(os, "unlink", mark_cleanup_unlink)
    capture, runner = _capture(tmp_path)

    with pytest.raises(type(control_flow)) as raised:
        capture.capture()

    assert raised.value is control_flow
    assert len(runner.calls) == 7
    assert cleanup_unlinked is True
    assert cleanup_fsynced is True
    assert not (tmp_path / f"logs/docker/{partial_name}").exists()
    assert target_fd is not None
    with pytest.raises(OSError):
        os.fstat(target_fd)


def test_structured_creation_cleanup_directory_fsync_failure_is_reported(
    tmp_path, monkeypatch
):
    """Factory cleanup is not durable until its parent directory is fsynced."""
    partial_name = "orchestration.jsonl.partial"
    real_open = os.open
    real_fchmod = os.fchmod
    real_fsync = os.fsync
    real_unlink = os.unlink
    target_fd = None
    cleanup_unlinked = False
    injected_cleanup_fsync = False

    def record_target(path, flags, mode=0o777, *, dir_fd=None):
        nonlocal target_fd
        fd = real_open(path, flags, mode, dir_fd=dir_fd)
        if path == partial_name:
            target_fd = fd
        return fd

    def fail_target_fchmod(fd, mode):
        if fd == target_fd:
            raise OSError("injected structured candidate creation failure")
        return real_fchmod(fd, mode)

    def mark_cleanup_unlink(path, **kwargs):
        nonlocal cleanup_unlinked
        result = real_unlink(path, **kwargs)
        if path == partial_name:
            cleanup_unlinked = True
        return result

    def fail_cleanup_fsync(fd):
        nonlocal injected_cleanup_fsync
        if cleanup_unlinked and not injected_cleanup_fsync:
            injected_cleanup_fsync = True
            raise OSError("injected structured creation cleanup fsync failure")
        return real_fsync(fd)

    monkeypatch.setattr(os, "open", record_target)
    monkeypatch.setattr(os, "fchmod", fail_target_fchmod)
    monkeypatch.setattr(os, "unlink", mark_cleanup_unlink)
    monkeypatch.setattr(os, "fsync", fail_cleanup_fsync)
    capture, _runner = _capture(tmp_path)

    with pytest.raises(DockerLogCaptureError) as raised:
        capture.capture()

    result = raised.value.result
    assert any("cleanup fsync" in item.detail for item in result.diagnostics)
    assert result.leftover_partials == ()
    assert not (tmp_path / f"logs/{partial_name}").exists()


def test_publish_fsync_failure_after_partial_unlink_has_no_false_missing_diagnostic(
    tmp_path, monkeypatch
):
    """An absent partial after successful unlink is already clean, not an inspection error."""
    service = OWNERSHIP[0][0]
    partial_name = f"{service}.log.partial"
    real_fsync = os.fsync
    real_unlink = os.unlink
    published_partial_removed = False
    injected_publish_fsync = False

    def mark_publish_unlink(path, **kwargs):
        nonlocal published_partial_removed
        result = real_unlink(path, **kwargs)
        if path == partial_name:
            published_partial_removed = True
        return result

    def fail_publish_fsync(fd):
        nonlocal injected_publish_fsync
        if published_partial_removed and not injected_publish_fsync:
            injected_publish_fsync = True
            raise OSError("injected post-unlink publication fsync failure")
        return real_fsync(fd)

    monkeypatch.setattr(os, "unlink", mark_publish_unlink)
    monkeypatch.setattr(os, "fsync", fail_publish_fsync)
    capture, _runner = _capture(tmp_path)

    with pytest.raises(DockerLogCaptureError) as raised:
        capture.capture()

    result = raised.value.result
    assert f"logs/docker/{service}.log" in result.raw_paths
    assert not any("ownership inspection" in item.detail for item in result.diagnostics)
    assert result.leftover_partials == ()


def test_partial_cleanup_unlink_failure_is_reported_with_exact_leftover(
    tmp_path, monkeypatch
):
    """Suppressing cleanup unlink failure would falsely claim no partial remains."""
    real_link = os.link
    real_unlink = os.unlink

    def fail_companion_publication(source, target, **kwargs):
        if target == "companion.jsonl":
            raise OSError("injected publication failure")
        return real_link(source, target, **kwargs)

    def fail_gazebo_partial_cleanup(path, **kwargs):
        if path == "gazebo.jsonl.partial":
            raise OSError("injected cleanup unlink failure")
        return real_unlink(path, **kwargs)

    monkeypatch.setattr(os, "link", fail_companion_publication)
    monkeypatch.setattr(os, "unlink", fail_gazebo_partial_cleanup)
    capture, _runner = _capture(tmp_path)

    with pytest.raises(DockerLogCaptureError) as raised:
        capture.capture()

    assert "logs/gazebo.jsonl.partial" in raised.value.result.leftover_partials
    assert any("cleanup unlink" in item.detail for item in raised.value.result.diagnostics)
    assert (tmp_path / "logs/gazebo.jsonl.partial").is_file()


def test_partial_cleanup_directory_fsync_failure_is_reported(tmp_path, monkeypatch):
    """A removed name is not durably cleaned until its parent directory fsync succeeds."""
    real_link = os.link
    real_unlink = os.unlink
    real_fsync = os.fsync
    cleanup_started = False
    injected = False

    def fail_companion_publication(source, target, **kwargs):
        if target == "companion.jsonl":
            raise OSError("injected publication failure")
        return real_link(source, target, **kwargs)

    def mark_cleanup(path, **kwargs):
        nonlocal cleanup_started
        result = real_unlink(path, **kwargs)
        if path == "gazebo.jsonl.partial":
            cleanup_started = True
        return result

    def fail_cleanup_fsync(fd):
        nonlocal injected
        if cleanup_started and not injected:
            injected = True
            raise OSError("injected cleanup fsync failure")
        return real_fsync(fd)

    monkeypatch.setattr(os, "link", fail_companion_publication)
    monkeypatch.setattr(os, "unlink", mark_cleanup)
    monkeypatch.setattr(os, "fsync", fail_cleanup_fsync)
    capture, _runner = _capture(tmp_path)

    with pytest.raises(DockerLogCaptureError) as raised:
        capture.capture()

    assert any("cleanup fsync" in item.detail for item in raised.value.result.diagnostics)


def test_final_candidate_descriptor_close_failure_downgrades_success(
    tmp_path, monkeypatch
):
    """A final result cannot be successful while an owned descriptor failed to close."""
    real_open = os.open
    real_close = os.close
    target_fd = None
    injected = False

    def record_orchestration_candidate(path, flags, mode=0o777, *, dir_fd=None):
        nonlocal target_fd
        fd = real_open(path, flags, mode, dir_fd=dir_fd)
        if path == "orchestration.jsonl.partial":
            target_fd = fd
        return fd

    def fail_target_close(fd):
        nonlocal injected
        if fd == target_fd and not injected:
            injected = True
            raise OSError("injected candidate close failure")
        return real_close(fd)

    monkeypatch.setattr(os, "open", record_orchestration_candidate)
    monkeypatch.setattr(os, "close", fail_target_close)
    capture, _runner = _capture(tmp_path)
    try:
        with pytest.raises(DockerLogCaptureError) as raised:
            capture.capture()

        assert raised.value.result.succeeded is False
        assert any("cleanup close" in item.detail for item in raised.value.result.diagnostics)
        assert raised.value.result.leftover_partials == ()
    finally:
        if target_fd is not None:
            try:
                real_close(target_fd)
            except OSError:
                pass
