import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from orchestration.calibration import (
    build_source_calibration_profile,
    freeze_calibration_import,
)


RUN_ID = "00000000-0000-4000-8000-000000000707"
IMAGES = {
    "drone-sim-ardupilot-runtime:phase3": "a" * 64,
    "drone-sim-gazebo-runtime:phase3": "b" * 64,
}


def test_source_profile_binds_all_compatible_vehicle_models():
    profile = build_source_calibration_profile(Path(__file__).parents[2], image_digests=IMAGES)
    assert profile["schema_version"] == 2
    assert set(profile["vehicle_models_sha256"]) == {
        "iris_flight", "iris_moving_pad", "iris_competition"}
    assert len(profile["airframe_sha256"]) == 64


@pytest.mark.parametrize("vehicle", ["iris_flight", "iris_moving_pad", "iris_competition"])
def test_calibration_import_binds_consumer_and_effective_overlay(tmp_path, vehicle):
    project = Path(__file__).parents[2]
    source, _ = _source(tmp_path, project)
    scenario = {
        "iris_moving_pad": "moving_pad_v1",
        "iris_competition": "competition_v1",
    }.get(vehicle, "descent_v1")
    frozen = freeze_calibration_import(
        source, project_directory=project, consumer_vehicle=vehicle,
        consumer_scenario=scenario,
        inspector=lambda *_args, **_kwargs: SimpleNamespace(run_id=RUN_ID),
        image_digests=IMAGES,
        artifact_reader=lambda _path: (RUN_ID, {"ATC_RAT_RLL_P": 0.1}),
        baseline_reader=lambda _path: {"ATC_RATE_FF_ENAB": 1.0, "ATC_RAT_YAW_D": 0.0},
    )
    profile = json.loads(frozen.calibration_json)["profile"]
    assert profile["consumer_vehicle"] == vehicle
    assert profile["consumer_model_sha256"] == hashlib.sha256(
        (project / f"gazebo/resources/models/{vehicle}/model.sdf").read_bytes()).hexdigest()
    assert profile["baseline_parameters"] == {"ATC_RATE_FF_ENAB": 1.0, "ATC_RAT_YAW_D": 0.0}
    if vehicle == "iris_moving_pad":
        assert profile["effective_baseline_parameters"]["AHRS_EKF_TYPE"] == 3.0
        assert profile["effective_baseline_parameters"]["PLND_LAG"] == 0.04
        assert len(profile["overlay_parameter_sha256s"]) == 1
    elif vehicle == "iris_competition":
        assert {
            name: profile["effective_baseline_parameters"][name]
            for name in (
                "FLTMODE_CH", "FLTMODE1", "FLTMODE4", "FLTMODE6",
                "RNGFND1_MIN", "RNGFND1_MAX", "RNGFND1_TYPE",
            )
        } == {
            "FLTMODE_CH": 7.0,
            "FLTMODE1": 0.0,
            "FLTMODE4": 4.0,
            "FLTMODE6": 5.0,
            "RNGFND1_MIN": 0.05,
            "RNGFND1_MAX": 40.0,
            "RNGFND1_TYPE": 100.0,
        }
        assert len(profile["overlay_parameter_sha256s"]) == 1


def _source(tmp_path: Path, project: Path) -> tuple[Path, dict]:
    source = tmp_path / "source"
    artifact = source / "ardupilot_sitl/autotune.parm"
    artifact.parent.mkdir(parents=True)
    artifact.write_bytes(b"ATC_RAT_RLL_P 0.1\n")
    profile = build_source_calibration_profile(project, image_digests=IMAGES)
    run = {
        "run_id": RUN_ID,
        "mission": "autotune",
        "scenario": "calibration_v1",
        "calibration_profile": profile,
    }
    (source / "configuration").mkdir()
    (source / "configuration/run.json").write_text(json.dumps(run), encoding="utf-8")
    manifest = {
        "run_id": RUN_ID,
        "source_revisions": [{"name": "drone_sim", "revision": "d" * 40, "dirty": False}],
        "image_digests": [
            {"name": name, "digest": digest} for name, digest in IMAGES.items()
        ],
    }
    (source / "manifest.json").write_text(
        json.dumps(manifest, sort_keys=True, separators=(",", ":")), encoding="utf-8"
    )
    return source, profile


def test_freeze_calibration_import_accepts_before_copy_and_binds_exact_bytes(tmp_path):
    project = Path(__file__).parents[2]
    source, profile = _source(tmp_path, project)
    calls = []

    def inspector(path, **kwargs):
        calls.append((path, kwargs))
        return SimpleNamespace(run_id=RUN_ID)

    frozen = freeze_calibration_import(
        source,
        project_directory=project,
        inspector=inspector,
        image_digests=IMAGES,
        artifact_reader=lambda _path: (RUN_ID, {"ATC_RAT_RLL_P": 0.1}),
        baseline_reader=lambda _path: {"ATC_RAT_YAW_D": 0.0, "ATC_RAT_RLL_FF": 0.0},
    )

    document = json.loads(frozen.calibration_json)
    assert calls[0][1]["require_maximum_score"] is True
    assert document["source_manifest_sha256"] == hashlib.sha256(
        (source / "manifest.json").read_bytes()
    ).hexdigest()
    assert document["source_artifact_sha256"] == hashlib.sha256(
        (source / "ardupilot_sitl/autotune.parm").read_bytes()
    ).hexdigest()
    assert document["profile"]["baseline_parameters"]["ATC_RAT_YAW_D"] == 0.0
    assert document["profile"]["profile_sha256"] != profile["profile_sha256"]


def test_freeze_calibration_import_rejects_changed_airframe_before_return(tmp_path):
    project = Path(__file__).parents[2]
    source, _profile = _source(tmp_path, project)
    path = source / "configuration/run.json"
    document = json.loads(path.read_text(encoding="utf-8"))
    document["calibration_profile"]["physical_model_sha256"] = "f" * 64
    without_hash = {
        key: value for key, value in document["calibration_profile"].items()
        if key != "profile_sha256"
    }
    document["calibration_profile"]["profile_sha256"] = hashlib.sha256(
        json.dumps(without_hash, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match="physical_model_sha256"):
        freeze_calibration_import(
            source,
            project_directory=project,
            inspector=lambda *_args, **_kwargs: SimpleNamespace(run_id=RUN_ID),
            image_digests=IMAGES,
            artifact_reader=lambda _path: (RUN_ID, {"ATC_RAT_RLL_P": 0.1}),
            baseline_reader=lambda _path: {},
        )


def test_legacy_profile_remains_limited_to_unloaded_consumer(tmp_path):
    project = Path(__file__).parents[2]
    source, profile = _source(tmp_path, project)
    for name in ("schema_version", "vehicle_models_sha256", "airframe_sha256"):
        profile.pop(name)
    profile["id"] = "competition-unloaded-v1"
    from orchestration.calibration import _profile_hash
    profile["profile_sha256"] = _profile_hash(profile)
    document = json.loads((source / "configuration/run.json").read_text())
    document["calibration_profile"] = profile
    (source / "configuration/run.json").write_text(json.dumps(document))
    kwargs = dict(project_directory=project,
        inspector=lambda *_args, **_kwargs: SimpleNamespace(run_id=RUN_ID),
        image_digests=IMAGES, artifact_reader=lambda _path: (RUN_ID, {"ATC_RAT_RLL_P": 0.1}),
        baseline_reader=lambda _path: {})
    assert json.loads(freeze_calibration_import(source, **kwargs).calibration_json)["profile"]["id"] == "competition-unloaded-v1"
    with pytest.raises(ValueError, match="iris_flight only"):
        freeze_calibration_import(source, consumer_vehicle="iris_competition", **kwargs)
