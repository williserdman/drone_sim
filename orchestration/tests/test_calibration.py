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
