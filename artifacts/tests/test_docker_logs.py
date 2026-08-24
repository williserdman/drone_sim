"""Host-side Docker log capture contract tests."""

from __future__ import annotations

from dataclasses import FrozenInstanceError
import json
import os
from pathlib import Path

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
):
    runner = FakeRunner(_valid_results() if results is None else results)
    capture = DockerLogCapture(
        run_directory=tmp_path,
        project_name=project,
        ownership=ownership,
        run_id=RUN_ID,
        command_runner=runner,
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
