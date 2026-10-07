"""Freeze calibration source identity before launch and import accepted gains."""

from __future__ import annotations

from collections.abc import Callable, Mapping
import hashlib
import json
from pathlib import Path
import re
import subprocess
from typing import Any

from .config import CalibrationImport


_SHA256 = re.compile(r"[0-9a-f]{64}")
_RUNTIME_IMAGES = (
    "drone-sim-ardupilot-runtime:phase3",
    "drone-sim-gazebo-runtime:phase3",
)
_VEHICLES = ("iris_flight", "iris_moving_pad", "iris_competition")


def _model_path(root: Path, vehicle: str) -> Path:
    if vehicle not in _VEHICLES:
        raise ValueError(f"unsupported calibration consumer vehicle: {vehicle}")
    return root / f"gazebo/resources/models/{vehicle}/model.sdf"


def _parameters(path: Path) -> dict[str, float]:
    values = {}
    for line in path.read_text().splitlines():
        fields = line.split("#", 1)[0].split()
        if fields:
            name, value = fields
            values[name] = float(value)
    return values


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _profile_hash(profile: Mapping[str, Any]) -> str:
    payload = {key: value for key, value in profile.items() if key != "profile_sha256"}
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(canonical).hexdigest()


def _local_image_digests(
    runner: Callable[..., Any] = subprocess.run,
) -> dict[str, str]:
    result = runner(
        ["docker", "image", "inspect", "--format", "{{.Id}}", *_RUNTIME_IMAGES],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
        timeout=30,
    )
    if result.returncode != 0:
        raise ValueError("calibration runtime image identity is unavailable")
    lines = result.stdout.splitlines()
    if len(lines) != len(_RUNTIME_IMAGES):
        raise ValueError("calibration runtime image identity is incomplete")
    values: dict[str, str] = {}
    for name, raw in zip(_RUNTIME_IMAGES, lines, strict=True):
        value = raw.decode("ascii").strip() if isinstance(raw, bytes) else raw.strip()
        if value.startswith("sha256:"):
            value = value[7:]
        if _SHA256.fullmatch(value) is None:
            raise ValueError(f"calibration runtime image {name} has no immutable digest")
        values[name] = value
    return values


def build_source_calibration_profile(
    project_directory: Path | str,
    *,
    image_digests: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    """Build source metadata that is persisted before an AutoTune run launches."""
    root = Path(project_directory).resolve()
    provenance = json.loads(
        (root / "ardupilot_sitl/provenance/ardupilot.json").read_text(encoding="utf-8")
    )
    images = dict(image_digests) if image_digests is not None else _local_image_digests()
    if set(images) != set(_RUNTIME_IMAGES) or any(
        not isinstance(value, str) or _SHA256.fullmatch(value) is None
        for value in images.values()
    ):
        raise ValueError("calibration profile image digests are invalid")
    profile: dict[str, Any] = {
        "schema_version": 2,
        "id": "competition-airframe-v2",
        "vehicle": "iris_flight",
        "physical_model_sha256": _sha256(
            root / "gazebo/resources/models/iris_flight/model.sdf"
        ),
        "base_parameter_sha256": _sha256(root / "ardupilot_sitl/params/descent.parm"),
        "overlay_parameter_sha256s": [],
        "firmware_revision": provenance["revision"],
        "image_digests": dict(sorted(images.items())),
        "baseline_parameters": {},
    }
    from artifacts.calibration import airframe_fingerprint
    fingerprints = {airframe_fingerprint(_model_path(root, vehicle).read_bytes()) for vehicle in _VEHICLES}
    if len(fingerprints) != 1:
        raise ValueError("calibration vehicle variants have incompatible airframe physics")
    profile["airframe_sha256"] = fingerprints.pop()
    profile["vehicle_models_sha256"] = {vehicle: _sha256(_model_path(root, vehicle)) for vehicle in _VEHICLES}
    profile["profile_sha256"] = _profile_hash(profile)
    return profile


def freeze_calibration_import(
    source_run_directory: Path | str,
    *,
    project_directory: Path | str,
    consumer_vehicle: str = "iris_flight",
    consumer_scenario: str = "descent_v1",
    inspector: Callable[..., Any] | None = None,
    image_digests: Mapping[str, str] | None = None,
    artifact_reader: Callable[[Path], tuple[str, dict[str, float]]] | None = None,
    baseline_reader: Callable[[Path], dict[str, float]] | None = None,
) -> CalibrationImport:
    """Accept a source bundle, check compatibility, and freeze exact source bytes."""
    from artifacts.acceptance import inspect_phase3_via_container

    root = Path(project_directory).resolve()
    source = Path(source_run_directory).resolve()
    manifest_path = source / "manifest.json"
    artifact_path = source / "ardupilot_sitl/autotune.parm"
    run_config_path = source / "configuration/run.json"
    manifest_bytes = manifest_path.read_bytes()
    artifact_bytes = artifact_path.read_bytes()
    manifest = json.loads(manifest_bytes)
    source_config = json.loads(run_config_path.read_text(encoding="utf-8"))
    expected_images = {
        row["name"]: row["digest"] for row in manifest.get("image_digests", [])
    }
    expected_revisions = {
        row["name"]: row["revision"] for row in manifest.get("source_revisions", [])
    }
    inspect = inspector or inspect_phase3_via_container
    report = inspect(
        source,
        rules_path=root / "scorekeeper/rules/calibration_v1.json",
        expected_source_revisions=expected_revisions,
        expected_source_dirty={row["name"]: row["dirty"] for row in manifest["source_revisions"]},
        expected_image_digests=expected_images,
        require_maximum_score=True,
    )
    if source_config.get("mission") != "autotune" or source_config.get("scenario") != "calibration_v1":
        raise ValueError("calibration source must be mission autotune/scenario calibration_v1")
    source_run_id = source_config.get("run_id")
    if manifest.get("run_id") != source_run_id or report.run_id != source_run_id:
        raise ValueError("calibration source run identity is inconsistent")
    if artifact_reader is None or baseline_reader is None:
        from artifacts.calibration import (
            read_calibration_baseline,
            read_calibration_parameters,
        )
    read_artifact = artifact_reader or read_calibration_parameters
    artifact_run_id, gains = read_artifact(artifact_path)
    if artifact_run_id != source_run_id:
        raise ValueError("calibration artifact run identity is inconsistent")
    profile = source_config.get("calibration_profile")
    if not isinstance(profile, dict) or profile.get("profile_sha256") != _profile_hash(profile):
        raise ValueError("calibration source profile is missing or invalid")
    expected_profile = build_source_calibration_profile(
        root, image_digests=image_digests or _local_image_digests()
    )
    legacy = profile.get("id") == "competition-unloaded-v1" and "schema_version" not in profile
    if legacy:
        if consumer_vehicle != "iris_flight":
            raise ValueError("legacy calibration supports iris_flight only")
        expected_profile = {key: value for key, value in expected_profile.items()
            if key not in {"schema_version", "airframe_sha256", "vehicle_models_sha256"}}
        expected_profile["id"] = "competition-unloaded-v1"
    for field in (
        "id", "vehicle", "physical_model_sha256", "base_parameter_sha256",
        "overlay_parameter_sha256s", "firmware_revision", "image_digests",
    ):
        if profile.get(field) != expected_profile[field]:
            raise ValueError(f"calibration source profile is incompatible: {field}")
    if not legacy:
        for field in ("schema_version", "airframe_sha256", "vehicle_models_sha256"):
            if profile.get(field) != expected_profile[field]:
                raise ValueError(f"calibration source profile is incompatible: {field}")
    read_baseline = baseline_reader or read_calibration_baseline
    frozen_profile = dict(profile)
    frozen_profile["baseline_parameters"] = dict(sorted(read_baseline(source).items()))
    if not legacy:
        model = _model_path(root, consumer_vehicle)
        overlays = {
            "moving_pad_v1": [root / "ardupilot_sitl/params/moving-pad.parm"],
            "competition_v1": [root / "ardupilot_sitl/params/competition.parm"],
        }.get(consumer_scenario, [])
        settings = _parameters(root / "ardupilot_sitl/params/descent.parm")
        for overlay in overlays:
            settings.update(_parameters(overlay))
        effective = dict(frozen_profile["baseline_parameters"])
        for name in settings:
            if name in effective or name.startswith("PLND_") or name == "LAND_SPD_MS" or (
                consumer_scenario == "moving_pad_v1" and name in {"AHRS_EKF_TYPE", "PSC_NE_POS_P"}
            ) or (
                consumer_scenario == "competition_v1" and name in {
                    "FLTMODE_CH", "FLTMODE1", "FLTMODE4", "FLTMODE6",
                }
            ):
                effective[name] = settings[name]
        frozen_profile.update(
            consumer_vehicle=consumer_vehicle,
            consumer_model_resource=f"gazebo/resources/models/{consumer_vehicle}/model.sdf",
            consumer_model_sha256=_sha256(model),
            overlay_parameter_sha256s=[_sha256(path) for path in overlays],
            effective_baseline_parameters=dict(sorted(effective.items())),
        )
    frozen_profile["source_image_digests"] = dict(sorted(expected_images.items()))
    frozen_profile["profile_sha256"] = _profile_hash(frozen_profile)
    document = {
        "schema_version": 1,
        "source_run_id": source_run_id,
        "source_manifest_sha256": hashlib.sha256(manifest_bytes).hexdigest(),
        "source_artifact_sha256": hashlib.sha256(artifact_bytes).hexdigest(),
        "gains": dict(sorted(gains.items())),
        "profile": frozen_profile,
    }
    return CalibrationImport(
        json.dumps(document, sort_keys=True, separators=(",", ":")),
        artifact_path,
        manifest_path,
    )


__all__ = ["build_source_calibration_profile", "freeze_calibration_import"]
