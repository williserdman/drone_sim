from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path

import pytest
from jsonschema import Draft202012Validator

from artifacts.manifest import REQUIRED_ARTIFACT_PATHS, build_manifest, write_manifest_atomic


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


def test_write_manifest_atomic_replaces_manifest_without_a_temporary_sibling(tmp_path):
    """Leaving the temporary file behind would make finalization non-atomic."""
    _complete_run_directory(tmp_path)
    manifest = build_manifest(tmp_path, "run-7", "COMPLETED", "mission_complete")

    manifest_path = write_manifest_atomic(tmp_path, manifest)

    assert manifest_path == tmp_path / "manifest.json"
    assert json.loads(manifest_path.read_text(encoding="utf-8"))["run_id"] == "run-7"
    assert not (tmp_path / "manifest.json.tmp").exists()


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
