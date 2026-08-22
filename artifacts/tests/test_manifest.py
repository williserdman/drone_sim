import hashlib
import json
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
            (run_dir / relative_path).mkdir(parents=True, exist_ok=True)
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
    assert onboard.validation == "present"


def test_build_manifest_classifies_required_paths_as_present_missing_or_invalid(tmp_path):
    """A directory at a required file path must not be reported as present."""
    _complete_run_directory(tmp_path)
    (tmp_path / "video/observer.mp4").unlink()
    (tmp_path / "video/observer.mp4").mkdir()
    (tmp_path / "rosbag").rmdir()

    manifest = build_manifest(tmp_path, "run-7", "FAILED", "recorder_failed")
    records = {record.relative_path: record for record in manifest.artifacts}

    assert records["video/onboard.mp4"].validation == "present"
    assert records["rosbag"].validation == "missing"
    assert records["video/observer.mp4"].validation == "invalid"


def test_completed_manifest_rejects_missing_required_artifact(tmp_path):
    """Accepting incomplete completed runs would violate bundle completeness."""
    with pytest.raises(ValueError, match="COMPLETED"):
        build_manifest(tmp_path, "run-7", "COMPLETED", "mission_complete")


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


def test_manifest_schema_accepts_a_completed_manifest(tmp_path):
    """A schema drift that rejects persisted manifests must be detected."""
    _complete_run_directory(tmp_path)
    manifest = build_manifest(tmp_path, "run-7", "COMPLETED", "mission_complete")
    schema_path = Path(__file__).parents[1] / "schemas/manifest.schema.json"
    schema = json.loads(schema_path.read_text(encoding="utf-8"))

    Draft202012Validator.check_schema(schema)
    Draft202012Validator(schema).validate(manifest.to_dict())
