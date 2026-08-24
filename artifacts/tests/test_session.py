from datetime import datetime, timezone
import json
import os

import pytest

from artifacts.manifest import (
    ConfigurationRecord,
    ImageDigest,
    REQUIRED_ARTIFACT_PATHS,
    SourceRevision,
)
from artifacts.session import (
    ArtifactSession,
    FinalizationConflict,
    FinalizationInput,
    FinalizationResult,
)
from artifacts.validation import ValidationResult, ValidationStatus


SHA_A = "a" * 64
SHA_B = "b" * 64


def _write(run_dir, relative_path, contents=b"artifact"):
    path = run_dir / relative_path
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(contents)
    return path


def _complete_run_directory(run_dir):
    for relative_path in REQUIRED_ARTIFACT_PATHS:
        if relative_path in {"configuration", "gazebo/state", "rosbag"}:
            _write(run_dir, f"{relative_path}/content.bin")
        else:
            _write(run_dir, relative_path)


def _finalization_input(**changes):
    values = {
        "run_id": "run-7",
        "requested_terminal": "COMPLETED",
        "reason": "mission_complete",
        "sim_start_ns": 10,
        "sim_end_ns": 35,
        "wall_started_at": datetime(2026, 8, 23, 10, 0, tzinfo=timezone.utc),
        "wall_ended_at": datetime(2026, 8, 23, 10, 0, 2, 500_000, tzinfo=timezone.utc),
        "source_revisions": (SourceRevision("drone_sim", "abc123", True),),
        "image_digests": (ImageDigest("artifacts", SHA_A),),
        "configuration_records": (ConfigurationRecord("configuration/run.json", SHA_B),),
        "achieved_score": 85.5,
        "maximum_available_score": 100.0,
        "scoring_checksum": SHA_A,
        "evidence_paths": ("scoring/events.jsonl#event-12",),
    }
    values.update(changes)
    return FinalizationInput(**values)


def test_finalize_writes_expanded_schema_valid_manifest(tmp_path):
    _complete_run_directory(tmp_path)

    path = ArtifactSession(tmp_path).finalize(_finalization_input())
    manifest = json.loads(path.read_text(encoding="utf-8"))

    assert manifest["schema_version"] == 1
    assert manifest["terminal_status"] == "COMPLETED"
    assert manifest["simulation_timing"] == {
        "start_ns": 10,
        "end_ns": 35,
        "duration_ns": 25,
    }
    assert manifest["wall_timing"] == {
        "started_at": "2026-08-23T10:00:00+00:00",
        "ended_at": "2026-08-23T10:00:02.500000+00:00",
        "duration_seconds": 2.5,
    }
    assert manifest["source_revisions"] == [
        {"name": "drone_sim", "revision": "abc123", "dirty": True}
    ]
    assert manifest["image_digests"] == [{"name": "artifacts", "digest": SHA_A}]
    assert manifest["configurations"] == [
        {"relative_path": "configuration/run.json", "sha256": SHA_B}
    ]
    assert manifest["incomplete_paths"] == []
    assert manifest["scoring"] == {
        "achieved_score": 85.5,
        "maximum_available_score": 100.0,
        "scoring_checksum": SHA_A,
        "evidence_paths": ["scoring/events.jsonl#event-12"],
    }
    assert all(record["validation"] == "valid" for record in manifest["artifacts"])
    assert all(record["detail"] for record in manifest["artifacts"])


def test_finalize_deadline_check_stops_hashing_and_commits_timeout_inventory(tmp_path):
    _complete_run_directory(tmp_path)
    checks = 0

    def deadline_check():
        nonlocal checks
        checks += 1
        if checks == 5:
            raise TimeoutError("finalization_deadline")

    path = ArtifactSession(
        tmp_path,
        deadline_check=deadline_check,
        commit_deadline_check=lambda: None,
    ).finalize(_finalization_input())

    assert checks == 5
    manifest = json.loads(path.read_text())
    assert manifest["terminal_status"] == "FAILED"
    assert any("deadline" in record["detail"] for record in manifest["artifacts"])


def test_requested_completion_downgrades_to_failed_when_required_path_is_missing(tmp_path):
    _complete_run_directory(tmp_path)
    (tmp_path / "video/observer.mp4").unlink()

    path = ArtifactSession(tmp_path).finalize(_finalization_input())
    manifest = json.loads(path.read_text(encoding="utf-8"))

    assert manifest["terminal_status"] == "FAILED"
    assert manifest["incomplete_paths"] == ["video/observer.mp4"]


@pytest.mark.parametrize(
    ("requested_terminal", "remove_path", "expected_terminal"),
    [
        ("COMPLETED", None, "COMPLETED"),
        ("COMPLETED", "video/observer.mp4", "FAILED"),
        ("ABORTED", None, "ABORTED"),
    ],
)
def test_finalize_with_result_returns_immutable_committed_authority_and_is_idempotent(
    tmp_path, requested_terminal, remove_path, expected_terminal
):
    _complete_run_directory(tmp_path)
    if remove_path is not None:
        (tmp_path / remove_path).unlink()
    session = ArtifactSession(tmp_path)
    request = _finalization_input(requested_terminal=requested_terminal)

    first = session.finalize_with_result(request)
    second = session.finalize_with_result(request)

    assert first == second
    assert first.path == tmp_path / "manifest.json"
    assert first.run_id == "run-7"
    assert first.terminal_status == expected_terminal
    assert first.reason == "mission_complete"
    with pytest.raises(AttributeError):
        first.terminal_status = "FAILED"


def test_validator_registry_uses_required_path_override_for_semantic_validation(tmp_path):
    _complete_run_directory(tmp_path)

    def reject_unplayable_video(run_directory, relative_path):
        return ValidationResult(
            ValidationStatus.INVALID, None, None, "video is not playable"
        )

    path = ArtifactSession(
        tmp_path,
        validators={"video/onboard.mp4": reject_unplayable_video},
    ).finalize(_finalization_input())
    manifest = json.loads(path.read_text(encoding="utf-8"))
    records = {record["relative_path"]: record for record in manifest["artifacts"]}

    assert manifest["terminal_status"] == "FAILED"
    assert records["video/onboard.mp4"]["detail"] == "video is not playable"
    assert manifest["incomplete_paths"] == ["video/onboard.mp4"]


@pytest.mark.parametrize("requested_terminal", ["FAILED", "ABORTED"])
def test_requested_failure_or_abort_never_upgrades(tmp_path, requested_terminal):
    _complete_run_directory(tmp_path)

    path = ArtifactSession(tmp_path).finalize(
        _finalization_input(requested_terminal=requested_terminal)
    )

    assert json.loads(path.read_text(encoding="utf-8"))["terminal_status"] == requested_terminal


def test_identical_finalization_returns_existing_identical_bytes(tmp_path):
    _complete_run_directory(tmp_path)
    session = ArtifactSession(tmp_path)
    request = _finalization_input()

    first = session.finalize(request)
    original = first.read_bytes()
    second = session.finalize(request)

    assert second == first
    assert second.read_bytes() == original


def test_conflicting_second_finalization_raises_and_preserves_first_manifest(tmp_path):
    _complete_run_directory(tmp_path)
    session = ArtifactSession(tmp_path)
    first = session.finalize(_finalization_input())
    original = first.read_bytes()

    with pytest.raises(FinalizationConflict, match="already finalized"):
        session.finalize(_finalization_input(reason="different"))

    assert first.read_bytes() == original


def test_finalize_fsyncs_parent_directory_after_no_clobber_publication(
    tmp_path, monkeypatch
):
    _complete_run_directory(tmp_path)
    events = []
    real_link = os.link
    real_fsync = os.fsync

    def recording_link(source, target, **kwargs):
        real_link(source, target, **kwargs)
        events.append("link")

    def recording_fsync(fd):
        real_fsync(fd)
        event = "directory-fsync" if os.path.isdir(f"/proc/self/fd/{fd}") else "file-fsync"
        events.append(event)

    monkeypatch.setattr(os, "link", recording_link)
    monkeypatch.setattr(os, "fsync", recording_fsync)

    ArtifactSession(tmp_path).finalize(_finalization_input())

    assert events[-2:] == ["link", "directory-fsync"]


def test_deadline_expiry_inside_link_returns_durable_typed_publication_authority(
    tmp_path, monkeypatch
):
    """A cooperative timeout after the hard link must not hide the commit."""
    _complete_run_directory(tmp_path)
    real_link = os.link
    real_fsync = os.fsync
    published = False
    directory_fsynced = False

    def expiring_link(source, target, **kwargs):
        nonlocal published
        real_link(source, target, **kwargs)
        published = True
        deadline_check()

    def deadline_check():
        if published:
            raise TimeoutError("finalization_deadline")

    def recording_fsync(descriptor):
        nonlocal directory_fsynced
        if published and os.path.isdir(f"/proc/self/fd/{descriptor}"):
            directory_fsynced = True
        return real_fsync(descriptor)

    monkeypatch.setattr(os, "link", expiring_link)
    monkeypatch.setattr(os, "fsync", recording_fsync)

    result = ArtifactSession(
        tmp_path,
        commit_deadline_check=deadline_check,
    ).finalize_with_result(_finalization_input())

    assert isinstance(result, FinalizationResult)
    assert result.path == tmp_path / "manifest.json"
    assert result.terminal_status == "COMPLETED"
    assert directory_fsynced is True
    assert json.loads(result.path.read_text())["reason"] == result.reason


def test_identical_existing_manifest_remains_authoritative_after_link_deadline(
    tmp_path, monkeypatch
):
    _complete_run_directory(tmp_path)
    request = _finalization_input()
    first = ArtifactSession(tmp_path).finalize_with_result(request)
    real_link = os.link
    link_attempted = False

    def existing_link(source, target, **kwargs):
        nonlocal link_attempted
        try:
            return real_link(source, target, **kwargs)
        finally:
            link_attempted = True

    def deadline_check():
        if link_attempted:
            raise TimeoutError("finalization_deadline")

    monkeypatch.setattr(os, "link", existing_link)

    second = ArtifactSession(
        tmp_path,
        commit_deadline_check=deadline_check,
    ).finalize_with_result(request)

    assert second == first


def test_finalize_cleans_collision_safe_temporary_after_publication_error(
    tmp_path, monkeypatch
):
    _complete_run_directory(tmp_path)

    def fail_link(source, target, **kwargs):
        raise OSError("publication failed")

    monkeypatch.setattr(os, "link", fail_link)

    with pytest.raises(OSError, match="publication failed"):
        ArtifactSession(tmp_path).finalize(_finalization_input())

    assert not (tmp_path / "manifest.json").exists()
    assert list(tmp_path.glob(".manifest.json.*.tmp")) == []


def test_inventory_includes_optional_diagnostics_without_treating_them_as_required(tmp_path):
    _complete_run_directory(tmp_path)
    (tmp_path / "video/observer.mp4").unlink()
    _write(tmp_path, "logs/docker/gazebo.log", b"raw diagnostics")
    _write(tmp_path, "video/observer.mp4.partial", b"partial movie")
    _write(tmp_path, ".control/finalize-request.json", b"mutable")
    _write(tmp_path, ".status/artifacts-final.json", b"mutable")
    _write(tmp_path, "manifest.json.partial", b"old temp")

    path = ArtifactSession(tmp_path).finalize(
        _finalization_input(requested_terminal="FAILED")
    )
    manifest = json.loads(path.read_text(encoding="utf-8"))
    records = {record["relative_path"]: record for record in manifest["artifacts"]}

    assert "logs/docker/gazebo.log" in records
    assert "video/observer.mp4.partial" in records
    assert "manifest.json.partial" in records
    assert "manifest.json" not in records
    assert not any(path.startswith(".control/") for path in records)
    assert not any(path.startswith(".status/") for path in records)
    assert records["video/observer.mp4"].get("validation") == "missing"
    assert manifest["incomplete_paths"] == ["video/observer.mp4"]


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"sim_start_ns": -1}, "simulation"),
        ({"sim_start_ns": 2, "sim_end_ns": 1}, "simulation"),
        ({"sim_start_ns": None, "sim_end_ns": 1}, "simulation"),
        ({"wall_started_at": datetime(2026, 1, 1)}, "timezone-aware"),
        (
            {
                "wall_ended_at": datetime(2026, 8, 23, 9, 59, tzinfo=timezone.utc),
            },
            "wall",
        ),
        ({"scoring_checksum": "A" * 64}, "scoring_checksum"),
        ({"image_digests": (ImageDigest("artifacts", "short"),)}, "digest"),
        (
            {
                "source_revisions": (
                    SourceRevision("repo", "one", False),
                    SourceRevision("repo", "two", False),
                )
            },
            "unique",
        ),
        (
            {
                "configuration_records": (
                    ConfigurationRecord("configuration/run.json", SHA_A),
                    ConfigurationRecord("configuration/run.json", SHA_B),
                )
            },
            "unique",
        ),
        ({"evidence_paths": ("../outside#event",)}, "evidence"),
    ],
)
def test_finalize_rejects_invalid_metadata_before_writing_manifest(tmp_path, changes, message):
    _complete_run_directory(tmp_path)

    with pytest.raises(ValueError, match=message):
        ArtifactSession(tmp_path).finalize(_finalization_input(**changes))

    assert not (tmp_path / "manifest.json").exists()


def test_finalize_accepts_absent_simulation_interval(tmp_path):
    _complete_run_directory(tmp_path)

    path = ArtifactSession(tmp_path).finalize(
        _finalization_input(sim_start_ns=None, sim_end_ns=None)
    )

    assert json.loads(path.read_text(encoding="utf-8"))["simulation_timing"] == {
        "start_ns": None,
        "end_ns": None,
        "duration_ns": None,
    }
