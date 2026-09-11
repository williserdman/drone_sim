from dataclasses import replace
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
from pathlib import Path
import threading

import pytest
from jsonschema import Draft202012Validator
from jsonschema.exceptions import ValidationError

import artifacts.manifest as manifest_module
from artifacts.manifest import (
    ArtifactRecord,
    ConfigurationRecord,
    FinalizationConflict,
    REQUIRED_ARTIFACT_PATHS,
    build_manifest,
    canonical_manifest_bytes,
    is_manifest_relative_path,
    validate_manifest,
    write_manifest_atomic,
)


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        ("configuration/run.json", True),
        ("video/onboard.mp4", True),
        ("rosbag", True),
        ("", False),
        (".", False),
        ("/absolute", False),
        ("../escape", False),
        ("a/../escape", False),
        (r"logs/docker/bad\name.log", False),
    ],
)
def test_manifest_relative_path_uses_portable_posix_grammar(value, expected):
    predicate = getattr(manifest_module, "is_manifest_relative_path", None)

    assert predicate is not None, "shared manifest path predicate is missing"
    assert predicate(value) is expected


def _write(run_dir, relative_path, contents="artifact"):
    path = run_dir / relative_path
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(contents, encoding="utf-8")
    return path


def _complete_run_directory(run_dir):
    for relative_path in REQUIRED_ARTIFACT_PATHS:
        if relative_path in {"configuration", "gazebo/state", "rosbag"}:
            _write(run_dir, f"{relative_path}/content.bin")
        else:
            _write(run_dir, relative_path)


def test_build_manifest_records_file_checksum_relative_path_size_and_validation(tmp_path):
    """A changed file must produce a changed checksum and byte count."""
    _complete_run_directory(tmp_path)
    _write(tmp_path, "video/onboard.mp4", "movie")

    manifest = build_manifest(tmp_path, "run-7", "COMPLETED", "mission_complete")
    onboard = next(record for record in manifest.artifacts if record.relative_path == "video/onboard.mp4")

    assert onboard.relative_path == "video/onboard.mp4"
    assert onboard.size_bytes == 5
    assert onboard.sha256 == hashlib.sha256(b"movie").hexdigest()
    assert onboard.validation == "valid"
    assert onboard.detail == "valid regular file"


def test_build_manifest_classifies_required_paths_as_valid_missing_or_invalid(tmp_path):
    """A directory at a required file path must not be reported as valid."""
    _complete_run_directory(tmp_path)
    (tmp_path / "video/observer.mp4").unlink()
    (tmp_path / "video/observer.mp4").mkdir()
    (tmp_path / "rosbag/content.bin").unlink()
    (tmp_path / "rosbag").rmdir()

    manifest = build_manifest(tmp_path, "run-7", "FAILED", "recorder_failed")
    records = {record.relative_path: record for record in manifest.artifacts}

    assert records["video/onboard.mp4"].validation == "valid"
    assert records["rosbag"].validation == "missing"
    assert records["video/observer.mp4"].validation == "invalid"


def test_completed_manifest_downgrades_when_a_required_artifact_is_missing(tmp_path):
    """Accepting incomplete completed runs would violate bundle completeness."""
    manifest = build_manifest(tmp_path, "run-7", "COMPLETED", "mission_complete")

    assert manifest.terminal_status == "FAILED"
    assert manifest.incomplete_paths == REQUIRED_ARTIFACT_PATHS


@pytest.mark.parametrize("terminal_status", ["FAILED", "ABORTED"])
def test_non_completed_manifest_retains_missing_artifact_records(tmp_path, terminal_status):
    """Dropping missing records would make partial runs non-diagnosable."""
    manifest = build_manifest(tmp_path, "run-7", terminal_status, "interrupted")

    assert len(manifest.artifacts) == len(REQUIRED_ARTIFACT_PATHS)
    assert all(record.validation == "missing" for record in manifest.artifacts)


def test_write_manifest_atomic_publishes_manifest_without_a_temporary_sibling(tmp_path):
    """Leaving the temporary file behind would make finalization non-atomic."""
    _complete_run_directory(tmp_path)
    manifest = build_manifest(tmp_path, "run-7", "COMPLETED", "mission_complete")

    manifest_path = write_manifest_atomic(tmp_path, manifest)

    assert manifest_path == tmp_path / "manifest.json"
    assert json.loads(manifest_path.read_text(encoding="utf-8"))["run_id"] == "run-7"
    assert not (tmp_path / "manifest.json.tmp").exists()


def test_concurrent_conflicting_finalizations_publish_one_winner_without_clobber(
    tmp_path, monkeypatch
):
    _complete_run_directory(tmp_path)
    first = build_manifest(tmp_path, "run-7", "COMPLETED", "first")
    second = replace(first, reason="second")
    start_gate = threading.Barrier(2)
    check_gate = threading.Barrier(2)
    check_counts: dict[int, int] = {}
    check_lock = threading.Lock()
    target = tmp_path / "manifest.json"
    real_exists = Path.exists

    def synchronized_exists(path):
        result = real_exists(path)
        if path == target:
            thread_id = threading.get_ident()
            with check_lock:
                check_counts[thread_id] = check_counts.get(thread_id, 0) + 1
                count = check_counts[thread_id]
            if count == 2:
                check_gate.wait(timeout=5)
        return result

    monkeypatch.setattr(Path, "exists", synchronized_exists)

    def finalize(manifest):
        start_gate.wait(timeout=5)
        try:
            write_manifest_atomic(tmp_path, manifest)
        except FinalizationConflict:
            return "conflict", canonical_manifest_bytes(manifest)
        return "returned", canonical_manifest_bytes(manifest)

    with ThreadPoolExecutor(max_workers=2) as executor:
        outcomes = list(executor.map(finalize, (first, second)))

    assert sorted(outcome for outcome, _ in outcomes) == ["conflict", "returned"]
    winner = next(payload for outcome, payload in outcomes if outcome == "returned")
    assert target.read_bytes() == winner
    assert list(tmp_path.glob(".manifest.json.*.tmp")) == []


def test_manifest_round_trips_scoring_summary_and_evidence_paths(tmp_path):
    """Omitting score metadata would prevent audit of an awarded result."""
    _complete_run_directory(tmp_path)
    manifest = build_manifest(
        tmp_path,
        "run-7",
        "COMPLETED",
        "mission_complete",
        achieved_score=85.5,
        maximum_available_score=100.0,
        scoring_checksum="a" * 64,
        evidence_paths=("scoring/events.jsonl#12", "video/observer.mp4#frame-42"),
    )

    data = manifest.to_dict()

    assert data["scoring"] == {
        "achieved_score": 85.5,
        "maximum_available_score": 100.0,
        "scoring_checksum": "a" * 64,
        "evidence_paths": ["scoring/events.jsonl#12", "video/observer.mp4#frame-42"],
    }


@pytest.mark.parametrize(
    ("field_name", "value"),
    [
        ("achieved_score", math.nan),
        ("achieved_score", math.inf),
        ("achieved_score", -math.inf),
        ("maximum_available_score", math.nan),
        ("maximum_available_score", math.inf),
        ("maximum_available_score", -math.inf),
    ],
)
def test_build_manifest_rejects_non_finite_score_values(tmp_path, field_name, value):
    """A non-finite score would make the persisted manifest invalid JSON."""
    with pytest.raises(ValueError, match=field_name):
        build_manifest(
            tmp_path,
            "run-7",
            "FAILED",
            "scoring_failed",
            **{field_name: value},
        )


@pytest.mark.parametrize("field_name", ["achieved_score", "maximum_available_score"])
def test_write_manifest_atomic_refuses_non_standard_json_numbers(tmp_path, field_name):
    """The persistence boundary must reject invalid manifests from any caller."""
    manifest = build_manifest(tmp_path, "run-7", "FAILED", "scoring_failed")
    invalid_manifest = replace(manifest, **{field_name: math.nan})

    with pytest.raises(ValueError, match="JSON compliant"):
        write_manifest_atomic(tmp_path, invalid_manifest)

    assert not (tmp_path / "manifest.json").exists()
    assert not (tmp_path / "manifest.json.tmp").exists()


def test_manifest_schema_accepts_a_completed_manifest(tmp_path):
    """A schema drift that rejects persisted manifests must be detected."""
    _complete_run_directory(tmp_path)
    manifest = build_manifest(tmp_path, "run-7", "COMPLETED", "mission_complete")
    schema_path = Path(__file__).parents[1] / "schemas/manifest.schema.json"
    schema = json.loads(schema_path.read_text(encoding="utf-8"))

    Draft202012Validator.check_schema(schema)
    Draft202012Validator(schema).validate(manifest.to_dict())


@pytest.mark.parametrize(
    ("field_name", "value", "path", "expected"),
    [
        ("configuration", "./", "./", False),
        ("configuration", "././", "././", False),
        ("configuration", ".//.", ".//.", False),
        ("configuration", "...", "...", True),
        ("configuration", "\nlogs", "\nlogs", True),
        ("configuration", "logs/a\nb", "logs/a\nb", True),
        ("configuration", "logs/end\n", "logs/end\n", True),
        ("configuration", ".\n", ".\n", True),
        ("configuration", "logs/a\nb\\bad", "logs/a\nb\\bad", False),
        ("configuration", "logs/a\n/../escape", "logs/a\n/../escape", False),
        ("evidence", ".//#selector", ".//.", False),
        ("evidence", "...#selector", "...", True),
        ("evidence", "\nlogs#selector", "\nlogs", True),
        ("evidence", "logs/a\nb#selector", "logs/a\nb", True),
        ("evidence", "logs/end\n#selector", "logs/end\n", True),
        ("evidence", ".\n#selector", ".\n", True),
        (
            "evidence",
            r"scoring/events.jsonl#bad\anchor",
            "scoring/events.jsonl",
            True,
        ),
    ],
)
def test_manifest_schema_matches_python_path_contract(
    tmp_path, field_name, value, path, expected
):
    manifest = build_manifest(tmp_path, "run-7", "FAILED", "recording_failed")
    if field_name == "configuration":
        candidate = replace(
            manifest,
            configurations=(ConfigurationRecord(value, "a" * 64),),
        )
    else:
        candidate = replace(manifest, evidence_paths=(value,))
    schema_path = Path(__file__).parents[1] / "schemas/manifest.schema.json"
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    validator = Draft202012Validator(schema)

    assert is_manifest_relative_path(path) is expected
    if expected:
        validate_manifest(candidate)
        validator.validate(candidate.to_dict())
    else:
        with pytest.raises(ValueError):
            validate_manifest(candidate)
        with pytest.raises(ValidationError):
            validator.validate(candidate.to_dict())


@pytest.mark.parametrize(
    ("section", "path"),
    [
        ("configurations", "configuration/run.json"),
        ("artifacts", "video/onboard.mp4"),
        ("incomplete_paths", "rosbag"),
        ("evidence_paths", "scoring/events.jsonl#12"),
    ],
)
def test_manifest_schema_accepts_portable_relative_paths(tmp_path, section, path):
    _complete_run_directory(tmp_path)
    document = build_manifest(tmp_path, "run-7", "COMPLETED", "mission_complete").to_dict()
    if section == "configurations":
        document[section] = [{"relative_path": path, "sha256": "a" * 64}]
    elif section == "artifacts":
        document[section][0]["relative_path"] = path
    elif section == "evidence_paths":
        document["scoring"][section] = [path]
    else:
        document[section] = [path]
    schema_path = Path(__file__).parents[1] / "schemas/manifest.schema.json"
    schema = json.loads(schema_path.read_text(encoding="utf-8"))

    Draft202012Validator(schema).validate(document)


@pytest.mark.parametrize(
    "path",
    ["", ".", "/absolute", r"logs/docker/bad\name.log", "../escape", "a/../escape"],
)
@pytest.mark.parametrize(
    "section", ["configurations", "artifacts", "incomplete_paths", "evidence_paths"]
)
def test_manifest_schema_rejects_nonportable_paths(tmp_path, section, path):
    _complete_run_directory(tmp_path)
    document = build_manifest(tmp_path, "run-7", "COMPLETED", "mission_complete").to_dict()
    if section == "configurations":
        document[section] = [{"relative_path": path, "sha256": "a" * 64}]
    elif section == "artifacts":
        document[section][0]["relative_path"] = path
    elif section == "evidence_paths":
        document["scoring"][section] = [path]
    else:
        document[section] = [path]
    schema_path = Path(__file__).parents[1] / "schemas/manifest.schema.json"
    schema = json.loads(schema_path.read_text(encoding="utf-8"))

    with pytest.raises(ValidationError):
        Draft202012Validator(schema).validate(document)


@pytest.mark.parametrize(
    "changes",
    [
        {
            "configurations": (
                ConfigurationRecord(r"configuration\run.json", "a" * 64),
            )
        },
        {
            "artifacts": (
                ArtifactRecord(r"logs/docker/bad\name.log", 1, "a" * 64, "valid", "valid"),
            )
        },
        {"incomplete_paths": (r"video\onboard.mp4",)},
        {"evidence_paths": (r"scoring\events.jsonl#12",)},
    ],
)
def test_manifest_persistence_rejects_nonportable_paths_before_publication(tmp_path, changes):
    manifest = build_manifest(tmp_path, "run-7", "FAILED", "recording_failed")

    with pytest.raises(ValueError):
        write_manifest_atomic(tmp_path, replace(manifest, **changes))

    assert not (tmp_path / "manifest.json").exists()


def test_manifest_schema_rejects_phase_one_present_validation(tmp_path):
    _complete_run_directory(tmp_path)
    manifest = build_manifest(tmp_path, "run-7", "COMPLETED", "mission_complete")
    document = manifest.to_dict()
    document["artifacts"][0]["validation"] = "present"
    schema_path = Path(__file__).parents[1] / "schemas/manifest.schema.json"
    schema = json.loads(schema_path.read_text(encoding="utf-8"))

    with pytest.raises(Exception):
        Draft202012Validator(schema).validate(document)


def test_persistence_rejects_completed_manifest_with_incomplete_required_path(tmp_path):
    manifest = build_manifest(tmp_path, "run-7", "FAILED", "recording_failed")

    with pytest.raises(ValueError, match="COMPLETED"):
        write_manifest_atomic(tmp_path, replace(manifest, terminal_status="COMPLETED"))


def test_persistence_rejects_incomplete_paths_that_disagree_with_artifacts(tmp_path):
    manifest = build_manifest(tmp_path, "run-7", "FAILED", "recording_failed")

    with pytest.raises(ValueError, match="incomplete_paths"):
        write_manifest_atomic(tmp_path, replace(manifest, incomplete_paths=()))


def test_persistence_rejects_duplicate_evidence_paths(tmp_path):
    manifest = build_manifest(tmp_path, "run-7", "FAILED", "recording_failed")

    with pytest.raises(ValueError, match="evidence paths must be unique"):
        write_manifest_atomic(
            tmp_path,
            replace(manifest, evidence_paths=("scoring/events.jsonl#1",) * 2),
        )
