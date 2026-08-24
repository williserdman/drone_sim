"""Read-only acceptance inspection for a finalized Phase 3 run bundle."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time
from typing import Any

from ._adapters.rosbag import RosbagValidator
from ._adapters.video import VideoValidator
from .manifest import MODULE_LOGS, REQUIRED_ARTIFACT_PATHS, REQUIRED_DIRECTORY_PATHS
from .runtime_configuration import resolve_recording_runtime_config
from .score_validation import ScoreValidationError, validate_descent_score_outputs
from .validation import (
    ValidationStatus,
    read_regular_file_bytes,
    validate_gazebo_state,
    validate_nonempty_regular_file,
    validate_regular_file,
    validate_tree,
)


_LOG_FIELDS = {
    "run_id",
    "module",
    "severity",
    "event",
    "sim_timestamp",
    "wall_timestamp",
    "fields",
}


class BundleAcceptanceError(RuntimeError):
    """A finalized bundle does not meet the Phase 3 acceptance contract."""


@dataclass(frozen=True)
class BundleAcceptanceReport:
    run_id: str
    achieved_score: float
    maximum_available_score: float
    compose_project: str
    manifest_sha256: str

    def to_dict(self) -> dict[str, object]:
        return {
            "accepted": True,
            "run_id": self.run_id,
            "achieved_score": self.achieved_score,
            "maximum_available_score": self.maximum_available_score,
            "compose_project": self.compose_project,
            "manifest_sha256": self.manifest_sha256,
        }


ComposeResources = Callable[[str], Sequence[str]]
SemanticCheck = Callable[[Path, str, int], None]


def _read_json(run_directory: Path, relative_path: str) -> tuple[dict[str, Any], str]:
    validation, payload = read_regular_file_bytes(run_directory, relative_path)
    if validation.status is not ValidationStatus.VALID or payload is None:
        raise BundleAcceptanceError(f"{relative_path} is not safe readable evidence")
    try:
        document = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise BundleAcceptanceError(f"{relative_path} is not valid JSON") from error
    if not isinstance(document, dict):
        raise BundleAcceptanceError(f"{relative_path} must contain a JSON object")
    return document, hashlib.sha256(payload).hexdigest()


def _validate_config(run_directory: Path, run_id: str) -> tuple[dict[str, Any], int]:
    document, _digest = _read_json(run_directory, "configuration/run.json")
    checksum = document.get("config_sha256")
    without_checksum = {
        key: value for key, value in document.items() if key != "config_sha256"
    }
    expected_checksum = hashlib.sha256(
        json.dumps(without_checksum, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    if checksum != expected_checksum:
        raise BundleAcceptanceError("configuration checksum is invalid")
    if document.get("run_id") != run_id or document.get("runtime_profile") != "phase3":
        raise BundleAcceptanceError("configuration is not for this Phase 3 run")
    try:
        contract = resolve_recording_runtime_config(document)
    except ValueError as error:
        raise BundleAcceptanceError(f"recording configuration is invalid: {error}") from error
    if not contract.physical_run:
        raise BundleAcceptanceError("configuration does not select physical validation")
    return document, contract.expected_camera_frames


def _validate_manifest_inventory(run_directory: Path, manifest: dict[str, Any]) -> None:
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, list):
        raise BundleAcceptanceError("manifest artifact inventory is invalid")
    records: dict[str, dict[str, Any]] = {}
    for record in artifacts:
        if not isinstance(record, dict) or not isinstance(record.get("relative_path"), str):
            raise BundleAcceptanceError("manifest artifact record is invalid")
        relative_path = record["relative_path"]
        if relative_path in records:
            raise BundleAcceptanceError("manifest artifact paths are not unique")
        records[relative_path] = record
    if not set(REQUIRED_ARTIFACT_PATHS).issubset(records):
        raise BundleAcceptanceError("manifest omits required artifacts")

    for relative_path in REQUIRED_ARTIFACT_PATHS:
        record = records[relative_path]
        validator = (
            validate_tree
            if relative_path in REQUIRED_DIRECTORY_PATHS
            else validate_regular_file
        )
        actual = validator(run_directory, relative_path)
        if (
            record.get("validation") != ValidationStatus.VALID.value
            or actual.status is not ValidationStatus.VALID
            or record.get("size_bytes") != actual.size_bytes
            or record.get("sha256") != actual.sha256
        ):
            raise BundleAcceptanceError(
                f"artifact checksum does not match manifest: {relative_path}"
            )


def _validate_module_logs(run_directory: Path, run_id: str) -> None:
    logs_directory = run_directory / "logs"
    expected_names = {Path(path).name for path in MODULE_LOGS}
    try:
        actual_names = {path.name for path in logs_directory.glob("*.jsonl")}
    except OSError as error:
        raise BundleAcceptanceError("module logs could not be inventoried") from error
    if actual_names != expected_names:
        raise BundleAcceptanceError("bundle must contain exactly seven JSONL module logs")
    for relative_path in MODULE_LOGS:
        validation, payload = read_regular_file_bytes(run_directory, relative_path)
        if validation.status is not ValidationStatus.VALID or payload is None:
            raise BundleAcceptanceError(f"module log is unsafe: {relative_path}")
        module = Path(relative_path).stem
        try:
            lines = payload.decode("utf-8").splitlines()
            documents = [json.loads(line) for line in lines]
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise BundleAcceptanceError(f"module log is invalid JSONL: {relative_path}") from error
        if not documents or any(
            not isinstance(document, dict)
            or set(document) != _LOG_FIELDS
            or document.get("run_id") != run_id
            or document.get("module") != module
            for document in documents
        ):
            raise BundleAcceptanceError(f"module log contract is invalid: {relative_path}")


def _production_semantic_check(
    run_directory: Path, run_id: str, expected_camera_frames: int
) -> None:
    deadline = time.monotonic() + 120.0
    video_validator = VideoValidator()
    for stream in ("onboard", "observer"):
        result = video_validator.validate(
            run_directory,
            f"video/{stream}.mp4",
            expected_frame_count=expected_camera_frames,
            outcome="COMPLETED",
            deadline=deadline,
        )
        if result.status is not ValidationStatus.VALID:
            raise BundleAcceptanceError(f"{stream} video is invalid: {result.detail}")
    bag = RosbagValidator(
        run_id,
        expected_camera_frames=expected_camera_frames,
        physical_run=True,
    ).validate(run_directory, "rosbag")
    if bag.status is not ValidationStatus.VALID:
        raise BundleAcceptanceError(f"physical MCAP is invalid: {bag.detail}")
    state = validate_gazebo_state(run_directory, "gazebo/state")
    server_log = validate_nonempty_regular_file(run_directory, "gazebo/server.log")
    if state.status is not ValidationStatus.VALID:
        raise BundleAcceptanceError(f"Gazebo state is invalid: {state.detail}")
    if server_log.status is not ValidationStatus.VALID:
        raise BundleAcceptanceError(f"Gazebo log is invalid: {server_log.detail}")


def docker_compose_resources(project_name: str) -> tuple[str, ...]:
    """Return project-labelled Docker resources without modifying them."""
    commands = (
        ("container", ["docker", "ps", "-a"], "{{.ID}}"),
        ("network", ["docker", "network", "ls"], "{{.ID}}"),
        ("volume", ["docker", "volume", "ls"], "{{.Name}}"),
    )
    resources: list[str] = []
    for kind, command, output_format in commands:
        result = subprocess.run(
            [
                *command,
                "--filter",
                f"label=com.docker.compose.project={project_name}",
                "--format",
                output_format,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            shell=False,
            check=False,
            timeout=30,
        )
        if result.returncode != 0:
            raise BundleAcceptanceError(
                f"Docker {kind} inventory failed: {result.stdout.strip()}"
            )
        resources.extend(
            f"{kind}:{identifier}"
            for identifier in result.stdout.splitlines()
            if identifier.strip()
        )
    return tuple(resources)


def inspect_phase3_bundle(
    run_directory: Path | str,
    *,
    rules_path: Path | str,
    require_maximum_score: bool = False,
    compose_resources: ComposeResources = docker_compose_resources,
    semantic_check: SemanticCheck = _production_semantic_check,
) -> BundleAcceptanceReport:
    """Assert final Phase 3 evidence without writing to the bundle or Docker."""
    if not isinstance(require_maximum_score, bool):
        raise TypeError("require_maximum_score must be a boolean")
    directory = Path(run_directory).resolve()
    manifest, manifest_sha256 = _read_json(directory, "manifest.json")
    run_id = manifest.get("run_id")
    if not isinstance(run_id, str) or not run_id:
        raise BundleAcceptanceError("manifest run_id is invalid")
    if manifest.get("schema_version") != 1 or manifest.get("terminal_status") != "COMPLETED":
        raise BundleAcceptanceError("manifest is not a completed schema-v1 run")
    if manifest.get("incomplete_paths") != []:
        raise BundleAcceptanceError("completed manifest contains incomplete artifacts")
    if not isinstance(manifest.get("source_revisions"), list) or not manifest["source_revisions"]:
        raise BundleAcceptanceError("manifest has no source revision evidence")
    if not isinstance(manifest.get("image_digests"), list) or not manifest["image_digests"]:
        raise BundleAcceptanceError("manifest has no image digest evidence")

    configuration, expected_frames = _validate_config(directory, run_id)
    configurations = manifest.get("configurations")
    if configurations != [
        {
            "relative_path": "configuration/run.json",
            "sha256": configuration["config_sha256"],
        }
    ]:
        raise BundleAcceptanceError("manifest configuration provenance is invalid")
    _validate_manifest_inventory(directory, manifest)
    _validate_module_logs(directory, run_id)
    semantic_check(directory, run_id, expected_frames)

    try:
        score = validate_descent_score_outputs(
            directory,
            run_id=run_id,
            rules_path=rules_path,
        )
    except ScoreValidationError as error:
        raise BundleAcceptanceError(f"score evidence is invalid: {error}") from error
    scoring = manifest.get("scoring")
    if scoring != {
        "achieved_score": score.achieved_score,
        "maximum_available_score": score.maximum_available_score,
        "scoring_checksum": score.scoring_checksum,
        "evidence_paths": list(score.evidence_paths),
    }:
        raise BundleAcceptanceError("manifest scoring provenance does not match score evidence")
    if require_maximum_score and not (
        math.isclose(score.achieved_score, 100.0)
        and math.isclose(score.maximum_available_score, 100.0)
    ):
        raise BundleAcceptanceError("bundle did not achieve the required 100/100 score")

    compose_project = "drone-sim-" + run_id.replace("-", "")
    leftovers = tuple(compose_resources(compose_project))
    if leftovers:
        raise BundleAcceptanceError(
            "leftover Compose resources: " + ", ".join(leftovers)
        )
    return BundleAcceptanceReport(
        run_id,
        score.achieved_score,
        score.maximum_available_score,
        compose_project,
        manifest_sha256,
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--rules-path", type=Path, required=True)
    parser.add_argument("--require-maximum-score", action="store_true")
    arguments = parser.parse_args(argv)
    try:
        report = inspect_phase3_bundle(
            arguments.run_directory,
            rules_path=arguments.rules_path,
            require_maximum_score=arguments.require_maximum_score,
        )
    except BundleAcceptanceError as error:
        print(json.dumps({"accepted": False, "detail": str(error)}, sort_keys=True))
        return 1
    print(json.dumps(report.to_dict(), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "BundleAcceptanceError",
    "BundleAcceptanceReport",
    "docker_compose_resources",
    "inspect_phase3_bundle",
    "main",
]
