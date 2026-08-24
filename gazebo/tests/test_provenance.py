"""Supply-chain and local-model contracts for the Phase 3 Gazebo assets."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "gazebo/provenance/ardupilot_gazebo-assets.json"
UPSTREAM_ROOT = ROOT / "gazebo/provenance/upstream/iris_with_standoffs"
MODEL_ROOT = ROOT / "gazebo/resources/models/iris_phase3"

EXPECTED = {
    "LICENSE.md": "1a45b1d0a8603dfe2cfc644f9dab970b1762f92babe2aac6eb2f5d4572c4a680",
    "models/iris_with_standoffs/model.config": "e419cc7681f730edf14398baa577a4f05cd34e2cebaa764c508119c4465b34b7",
    "models/iris_with_standoffs/model.sdf": "2c4e8ccf4385f329af905cce1cbceff96656da4ace9353c0ec6ca09df3c442d0",
    "models/iris_with_standoffs/meshes/iris.dae": "697956cfc0fe608afc52a0f688de50354b89e5201056451cf6bb767de8f46323",
    "models/iris_with_standoffs/meshes/iris_collision.stl": "66b85caa021497ea73dbaed08c72da381a6157116244ea3860a1ba445f50102c",
    "models/iris_with_standoffs/meshes/iris_prop_ccw.dae": "5f8b01668ee24a5b663ca8c4dd56e294fd6480db7eec1e5a7c11e45ba9004e99",
    "models/iris_with_standoffs/meshes/iris_prop_cw.dae": "6cbc686772dccd46253fb65ece000e7ffa6a74b9c097bda349884bd1e78cd879",
}

EXPECTED_IMPORTS = {
    "LICENSE.md": {"gazebo/provenance/LICENSE.ardupilot_gazebo.md"},
    "models/iris_with_standoffs/model.config": {
        "gazebo/provenance/upstream/iris_with_standoffs/model.config"
    },
    "models/iris_with_standoffs/model.sdf": {
        "gazebo/provenance/upstream/iris_with_standoffs/model.sdf"
    },
    "models/iris_with_standoffs/meshes/iris.dae": {
        "gazebo/provenance/upstream/iris_with_standoffs/meshes/iris.dae",
        "gazebo/resources/models/iris_phase3/meshes/iris.dae",
    },
    "models/iris_with_standoffs/meshes/iris_collision.stl": {
        "gazebo/provenance/upstream/iris_with_standoffs/meshes/iris_collision.stl",
        "gazebo/resources/models/iris_phase3/meshes/iris_collision.stl",
    },
    "models/iris_with_standoffs/meshes/iris_prop_ccw.dae": {
        "gazebo/provenance/upstream/iris_with_standoffs/meshes/iris_prop_ccw.dae",
        "gazebo/resources/models/iris_phase3/meshes/iris_prop_ccw.dae",
    },
    "models/iris_with_standoffs/meshes/iris_prop_cw.dae": {
        "gazebo/provenance/upstream/iris_with_standoffs/meshes/iris_prop_cw.dae",
        "gazebo/resources/models/iris_phase3/meshes/iris_prop_cw.dae",
    },
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _manifest() -> dict:
    assert MANIFEST.is_file(), "the reviewed provenance manifest is missing"
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


def test_imported_assets_match_pinned_upstream_bytes():
    """A floated revision or altered imported byte must fail provenance review."""
    manifest = _manifest()

    assert manifest["origin"] == "https://github.com/ArduPilot/ardupilot_gazebo"
    assert manifest["revision"] == "082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5"
    assert {item["source_path"]: item["sha256"] for item in manifest["files"]} == EXPECTED
    for item in manifest["files"]:
        assert set(item) == {"source_path", "imported_path", "sha256", "license"}
        assert sha256(ROOT / item["imported_path"]) == item["sha256"]
        assert item["license"] == "LGPL-3.0-only"


def test_manifest_records_every_imported_copy_truthfully():
    """An unrecorded duplicate could bypass the byte and license audit."""
    manifest = _manifest()
    actual = {source_path: set() for source_path in EXPECTED}
    for item in manifest["files"]:
        actual[item["source_path"]].add(item["imported_path"])

    assert actual == EXPECTED_IMPORTS
    assert len(manifest["files"]) == sum(map(len, EXPECTED_IMPORTS.values()))


def test_upstream_snapshot_contains_only_the_reviewed_original_model_files():
    """Adding legacy payload, gimbal, world, or plugin assets expands reviewed scope."""
    expected_paths = {
        Path("model.config"),
        Path("model.sdf"),
        Path("meshes/iris.dae"),
        Path("meshes/iris_collision.stl"),
        Path("meshes/iris_prop_ccw.dae"),
        Path("meshes/iris_prop_cw.dae"),
    }
    assert UPSTREAM_ROOT.is_dir(), "the pinned upstream snapshot is missing"
    actual_paths = {
        path.relative_to(UPSTREAM_ROOT)
        for path in UPSTREAM_ROOT.rglob("*")
        if path.is_file()
    }

    assert actual_paths == expected_paths


def test_phase3_meshes_are_hash_identical_to_the_reviewed_upstream_meshes():
    """Local mesh editing would invalidate the reviewed upstream byte identity."""
    for mesh_name in (
        "iris.dae",
        "iris_collision.stl",
        "iris_prop_ccw.dae",
        "iris_prop_cw.dae",
    ):
        assert sha256(MODEL_ROOT / "meshes" / mesh_name) == sha256(
            UPSTREAM_ROOT / "meshes" / mesh_name
        )


def test_local_model_is_new_phase3_sdf_with_one_downward_camera():
    """Reusing the legacy model would retain flight-control coupling and omit the camera."""
    config = ET.parse(MODEL_ROOT / "model.config").getroot()
    sdf = ET.parse(MODEL_ROOT / "model.sdf").getroot()
    model = sdf.find("model")

    assert config.findtext("name") == "iris_phase3"
    assert config.findtext("sdf") == "model.sdf"
    assert model is not None
    assert model.attrib["name"] == "iris_phase3"
    cameras = model.findall(".//sensor[@type='camera']")
    assert [camera.attrib["name"] for camera in cameras] == ["onboard_camera"]
    assert cameras[0].findtext("update_rate") == "20"
    assert cameras[0].findtext("camera/image/width") == "320"
    assert cameras[0].findtext("camera/image/height") == "240"
    assert cameras[0].findtext("camera/image/format") == "R8G8B8"


def test_phase3_model_rejects_plugins_remote_and_unreviewed_payload_features():
    """A forbidden URI or legacy feature would make the runtime nonlocal or coupled."""
    model_sdf = (MODEL_ROOT / "model.sdf").read_text(encoding="utf-8")
    lowered = model_sdf.lower()
    forbidden = (
        "ardupilotplugin",
        ".so",
        ".dylib",
        "http://",
        "https://",
        "fuel.gazebosim.org",
        "gimbal",
        "lidar",
        "payload",
        "gst",
    )

    assert not any(token in lowered for token in forbidden)
    root = ET.fromstring(model_sdf)
    uris = [uri.text.strip() for uri in root.findall(".//uri") if uri.text]
    assert uris
    assert all(uri.startswith("meshes/") for uri in uris)


def test_local_model_files_are_not_misrepresented_as_upstream_imports():
    """Locally authored SDF/config bytes must not be attributed to upstream."""
    imported_paths = {item["imported_path"] for item in _manifest()["files"]}

    assert "gazebo/resources/models/iris_phase3/model.config" not in imported_paths
    assert "gazebo/resources/models/iris_phase3/model.sdf" not in imported_paths
