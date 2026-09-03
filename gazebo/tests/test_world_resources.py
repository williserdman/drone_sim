"""Deterministic local-resource contracts for the Phase 3 Gazebo world."""

from __future__ import annotations

from dataclasses import FrozenInstanceError
import hashlib
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).resolve().parents[2]
RESOURCES = ROOT / "gazebo/resources"
MODEL_RESOURCES = RESOURCES / "models"
MODEL = MODEL_RESOURCES / "iris_phase3/model.sdf"
WORLD = RESOURCES / "worlds/phase3_foundation.sdf"

EXPECTED_RESOURCE_PATHS = (
    "models/iris_competition/model.config",
    "models/iris_competition/model.sdf",
    "models/iris_flight/model.config",
    "models/iris_flight/model.sdf",
    "models/iris_phase3/meshes/iris.dae",
    "models/iris_phase3/meshes/iris_collision.stl",
    "models/iris_phase3/meshes/iris_prop_ccw.dae",
    "models/iris_phase3/meshes/iris_prop_cw.dae",
    "models/iris_phase3/model.config",
    "models/iris_phase3/model.sdf",
    "models/payload_2/materials/textures/marker_2.png",
    "models/payload_2/model.config",
    "models/payload_2/model.sdf",
    "models/payload_3/materials/textures/marker_3.png",
    "models/payload_3/model.config",
    "models/payload_3/model.sdf",
    "models/payload_4/materials/textures/marker_4.png",
    "models/payload_4/model.config",
    "models/payload_4/model.sdf",
    "worlds/competition_mission.sdf",
    "worlds/competition_mission_1x.sdf",
    "worlds/phase3_foundation.sdf",
    "worlds/vertical_descent.sdf",
)


def _minimal_resources(tmp_path: Path) -> Path:
    resources = tmp_path / "resources"
    (resources / "worlds").mkdir(parents=True)
    (resources / "models/iris_phase3").mkdir(parents=True)
    shutil.copyfile(MODEL, resources / "models/iris_phase3/model.sdf")
    shutil.copyfile(WORLD, resources / "worlds/phase3_foundation.sdf")
    return resources


def _resolved(package_root: Path | None = None):
    from drone_sim_gazebo.worlds import WorldConfig, resolve_world

    return resolve_world(
        WorldConfig(world="phase3_foundation", vehicle="iris"),
        package_root=package_root,
    )


def _world() -> ET.Element:
    return ET.parse(WORLD).getroot().find("world")


def test_resolve_phase3_world_is_local_frozen_and_stable():
    """A floated path or mutable result could change server inputs after readiness."""
    resolved = _resolved()
    repeated = _resolved()

    assert resolved == repeated
    assert resolved.world_name == "phase3_foundation"
    assert resolved.vehicle_id == "iris"
    assert resolved.path == WORLD.resolve()
    assert resolved.resource_path == MODEL_RESOURCES.resolve()
    assert resolved.path.is_file()
    assert resolved.resource_path.is_dir()
    assert resolved.world_sha256 == hashlib.sha256(WORLD.read_bytes()).hexdigest()
    assert tuple(path for path, _digest in resolved.resource_sha256s) == (
        EXPECTED_RESOURCE_PATHS
    )
    assert tuple(sorted(resolved.resource_sha256s)) == resolved.resource_sha256s
    assert dict(resolved.resource_sha256s)["worlds/phase3_foundation.sdf"] == (
        resolved.world_sha256
    )
    assert all(len(digest) == 64 for _path, digest in resolved.resource_sha256s)
    with pytest.raises(FrozenInstanceError):
        resolved.world_name = "changed"


def test_resolve_competition_world_uses_the_committed_vehicle_and_world():
    from drone_sim_gazebo.worlds import WorldConfig, resolve_world

    resolved = resolve_world(
        WorldConfig("competition_mission", "iris_competition")
    )

    assert resolved.world_name == "competition_mission"
    assert resolved.vehicle_id == "iris_competition"
    assert resolved.path == (RESOURCES / "worlds/competition_mission.sdf").resolve()


@pytest.mark.parametrize(
    ("world", "vehicle"),
    (("other", "iris"), ("phase3_foundation", "other")),
)
def test_resolve_world_rejects_every_non_phase3_fixture(world: str, vehicle: str):
    """Accepting another pair would silently widen the immutable Phase 3 fixture."""
    from drone_sim_gazebo.worlds import WorldConfig, resolve_world

    with pytest.raises(ValueError, match="supports only phase3_foundation/iris"):
        resolve_world(WorldConfig(world=world, vehicle=vehicle))


def test_model_api_maps_iris_only_to_the_conventional_local_uri():
    """A changed URI mapping could re-enable a remote or incorrectly nested lookup."""
    from drone_sim_gazebo.models import model_uri_for_vehicle

    assert model_uri_for_vehicle("iris") == "model://iris_phase3"
    with pytest.raises(ValueError, match="supports only vehicle iris"):
        model_uri_for_vehicle("other")


def test_resolver_rejects_symlinks_anywhere_in_resource_tree(tmp_path: Path):
    """Following a tree symlink would hash bytes outside the image-owned snapshot."""
    resources = _minimal_resources(tmp_path)
    outside = tmp_path / "outside.bin"
    outside.write_bytes(b"outside")
    (resources / "models/iris_phase3/escape.bin").symlink_to(outside)

    with pytest.raises(ValueError, match="symlinks are not allowed"):
        _resolved(resources)


def test_resolver_rejects_a_symlink_resource_root(tmp_path: Path):
    """Resolving a caller-supplied root symlink would hide its mutable ownership."""
    resources = _minimal_resources(tmp_path)
    alias = tmp_path / "resources-alias"
    alias.symlink_to(resources, target_is_directory=True)

    with pytest.raises(ValueError, match="resource root must not be a symlink"):
        _resolved(alias)


def test_resolver_rejects_multiply_linked_resource_files(tmp_path: Path):
    """A hard-linked file could be mutated through an untracked external name."""
    resources = _minimal_resources(tmp_path)
    source = resources / "models/iris_phase3/model.sdf"
    os.link(source, tmp_path / "external-model.sdf")

    with pytest.raises(ValueError, match="exactly one hard link"):
        _resolved(resources)


def test_resolver_rejects_special_files_without_blocking(tmp_path: Path):
    """A FIFO or device must never be opened as though it were immutable bytes."""
    resources = _minimal_resources(tmp_path)
    os.mkfifo(resources / "models/iris_phase3/pipe")

    with pytest.raises(ValueError, match="regular files and directories"):
        _resolved(resources)


def test_resolver_rejects_identical_byte_inode_replacement_during_resolution(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    """A byte-identical replacement must not escape the retained tree identity."""
    resources = _minimal_resources(tmp_path)
    target = resources / "models/iris_phase3/model.sdf"
    original_inode = target.stat().st_ino
    replacement = tmp_path / "replacement-model.sdf"
    replacement.write_bytes(target.read_bytes())
    from drone_sim_gazebo.worlds import api

    real_read = api.os.read
    replaced = False

    def replace_other_file_during_hash(descriptor: int, count: int) -> bytes:
        nonlocal replaced
        chunk = real_read(descriptor, count)
        if not replaced and b"phase3_foundation" in chunk:
            replaced = True
            os.replace(replacement, target)
        return chunk

    monkeypatch.setattr(api.os, "read", replace_other_file_during_hash)

    with pytest.raises(ValueError, match="changed during hashing"):
        _resolved(resources)
    assert replaced
    assert target.stat().st_ino != original_inode


def test_resolver_rejects_hash_substitution_restored_before_final_verification(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    """A transient substitute must not contribute a hash to a restored tree."""
    resources = _minimal_resources(tmp_path)
    world = resources / "worlds/phase3_foundation.sdf"
    original = world.read_bytes()
    substitute = original.replace(b"phase3_foundation", b"phase3_foundatioX")
    assert substitute != original and len(substitute) == len(original)
    backup = tmp_path / "original-world.sdf"
    from drone_sim_gazebo.worlds import api

    real_hash = api._sha256_regular
    substituted = False

    def substitute_only_while_hashing(path: Path, **kwargs) -> str:
        nonlocal substituted
        if path == world and not substituted:
            substituted = True
            os.replace(world, backup)
            world.write_bytes(substitute)
            try:
                return real_hash(path, **kwargs)
            finally:
                os.replace(backup, world)
        return real_hash(path, **kwargs)

    monkeypatch.setattr(api, "_sha256_regular", substitute_only_while_hashing)

    with pytest.raises(ValueError, match="changed during hashing"):
        _resolved(resources)
    assert substituted
    assert world.read_bytes() == original


def test_resolver_closes_all_descriptors_on_success_and_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    """A resolver call must not retain descriptors beyond either terminal path."""
    resources = _minimal_resources(tmp_path)
    from drone_sim_gazebo.worlds import api

    real_open = api.os.open
    real_close = api.os.close
    active: set[int] = set()

    def tracked_open(*args, **kwargs) -> int:
        descriptor = real_open(*args, **kwargs)
        active.add(descriptor)
        return descriptor

    def tracked_close(descriptor: int) -> None:
        active.discard(descriptor)
        real_close(descriptor)

    monkeypatch.setattr(api.os, "open", tracked_open)
    monkeypatch.setattr(api.os, "close", tracked_close)

    _resolved(resources)
    assert not active

    escape = resources / "models/iris_phase3/escape"
    escape.symlink_to(tmp_path)
    with pytest.raises(ValueError, match="symlinks are not allowed"):
        _resolved(resources)
    assert not active

    escape.unlink()
    world = resources / "worlds/phase3_foundation.sdf"
    real_read = api.os.read
    mutated = False

    def mutate_during_late_hash(descriptor: int, count: int) -> bytes:
        nonlocal mutated
        chunk = real_read(descriptor, count)
        if not mutated and b"phase3_foundation" in chunk:
            mutated = True
            world.write_bytes(world.read_bytes() + b"\n")
        return chunk

    monkeypatch.setattr(api.os, "read", mutate_during_late_hash)
    with pytest.raises(ValueError, match="changed during hashing"):
        _resolved(resources)
    assert mutated
    assert not active


def test_resolver_closes_a_child_descriptor_when_identity_capture_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    """A failure between child open and retention must close that child descriptor."""
    resources = _minimal_resources(tmp_path)
    from drone_sim_gazebo.worlds import api

    real_open = api.os.open
    real_close = api.os.close
    real_fstat = api.os.fstat
    active: set[int] = set()
    failed = False

    def tracked_open(*args, **kwargs) -> int:
        descriptor = real_open(*args, **kwargs)
        active.add(descriptor)
        return descriptor

    def tracked_close(descriptor: int) -> None:
        active.discard(descriptor)
        real_close(descriptor)

    def fail_first_child_identity(descriptor: int):
        nonlocal failed
        if not failed and len(active) > 1:
            failed = True
            raise OSError("simulated identity failure")
        return real_fstat(descriptor)

    monkeypatch.setattr(api.os, "open", tracked_open)
    monkeypatch.setattr(api.os, "close", tracked_close)
    monkeypatch.setattr(api.os, "fstat", fail_first_child_identity)

    with pytest.raises(ValueError, match="changed during hashing"):
        _resolved(resources)
    assert failed
    assert not active


def test_resolver_rejects_file_change_during_descriptor_read(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    """Pre/post descriptor checks must catch content changed during hashing."""
    resources = _minimal_resources(tmp_path)
    world = resources / "worlds/phase3_foundation.sdf"
    from drone_sim_gazebo.worlds import api

    real_read = api.os.read
    mutated = False

    def mutate_during_read(descriptor: int, count: int) -> bytes:
        nonlocal mutated
        chunk = real_read(descriptor, count)
        if not mutated and b"phase3_foundation" in chunk:
            mutated = True
            world.write_bytes(world.read_bytes() + b"\n")
        return chunk

    monkeypatch.setattr(api.os, "read", mutate_during_read)

    with pytest.raises(ValueError, match="changed during hashing"):
        _resolved(resources)
    assert mutated


def test_sdf_has_exact_physics_and_required_harmonic_systems():
    """Default pacing or an omitted stock system would break the headless fixture."""
    world = _world()
    physics = world.find("physics")
    assert physics is not None
    assert physics.attrib == {"name": "phase3_physics", "type": "ode"}
    assert physics.findtext("max_step_size") == "0.001"
    assert physics.findtext("real_time_factor") == "0.1"
    assert physics.findtext("real_time_update_rate") == "100"
    assert float(physics.findtext("max_step_size")) * float(
        physics.findtext("real_time_update_rate")
    ) == pytest.approx(0.1)

    plugins = {
        (plugin.attrib["filename"], plugin.attrib["name"])
        for plugin in world.findall("plugin")
    }
    assert {
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-sensors-system", "gz::sim::systems::Sensors"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        (
            "gz-sim-scene-broadcaster-system",
            "gz::sim::systems::SceneBroadcaster",
        ),
        ("gz-sim-contact-system", "gz::sim::systems::Contact"),
    } <= plugins


def test_sdf_has_one_local_dynamic_iris_and_fixed_static_scene_models():
    """A duplicate, remote, or movable fixture entity would alter physical truth."""
    world = _world()
    includes = world.findall("include")
    assert len(includes) == 1
    iris = includes[0]
    assert iris.findtext("uri") == "model://iris_phase3"
    assert iris.findtext("name") == "iris"
    assert iris.findtext("static") == "false"
    assert iris.findtext("pose") == "0 0 1.2 0 0 0"

    models = {model.attrib["name"]: model for model in world.findall("model")}
    assert set(models) == {"ground_plane", "landing_marker", "observer_station"}
    assert all(model.findtext("static") == "true" for model in models.values())
    assert models["ground_plane"].findtext("pose") == "0 0 0 0 0 0"
    assert models["landing_marker"].findtext("pose") == "0 0 0.002 0 0 0"
    assert models["observer_station"].findtext("pose") == (
        "3 -3 2.5 0 0.45 2.356194490192345"
    )
    assert models["ground_plane"].find(".//geometry/plane") is not None
    assert models["landing_marker"].find(".//visual") is not None

    resolved = _resolved()
    model_name = iris.findtext("uri").removeprefix("model://")
    assert (resolved.resource_path / model_name).resolve(strict=True) == (
        MODEL_RESOURCES / "iris_phase3"
    ).resolve()


def test_sdf_has_exact_native_camera_contract_at_fixed_poses():
    """Geometry or cadence drift would invalidate exact camera/ground-truth pairing."""
    world = _world()
    model = ET.parse(MODEL).getroot().find("model")
    onboard = model.find(".//sensor[@name='onboard_camera']")
    observer_model = world.find("model[@name='observer_station']")
    observer = observer_model.find(".//sensor[@name='observer_camera']")
    cameras = {item.attrib["name"]: item for item in (onboard, observer)}

    assert set(cameras) == {"onboard_camera", "observer_camera"}
    assert onboard.findtext("pose") == "0 0 -0.08 0 1.5707963267948966 0"
    assert observer.findtext("pose") == "0 0 0 0 0 0"
    for name, camera in cameras.items():
        assert camera.attrib["type"] == "camera"
        assert camera.findtext("always_on") == "true"
        assert camera.findtext("update_rate") == "20"
        assert camera.findtext("camera/image/width") == "320"
        assert camera.findtext("camera/image/height") == "240"
        assert camera.findtext("camera/image/format") == "R8G8B8"
        assert camera.findtext("topic") == f"/gazebo/private/camera/{name[:-7]}/image"


def test_sdf_exposes_pose_twist_and_contact_at_camera_cadence():
    """Ground truth cannot pair exactly if any native physical source is off-cadence."""
    world = _world()
    iris = world.find("include")
    pose = iris.find("plugin[@name='gz::sim::systems::PosePublisher']")
    odometry = iris.find("plugin[@name='gz::sim::systems::OdometryPublisher']")
    contact = world.find("model[@name='ground_plane']//sensor[@type='contact']")

    assert pose is not None
    assert pose.attrib["filename"] == "gz-sim-pose-publisher-system"
    assert pose.findtext("publish_model_pose") == "true"
    assert pose.findtext("publish_link_pose") == "true"
    assert pose.findtext("use_pose_vector_msg") == "true"
    assert pose.findtext("update_frequency") == "20"

    assert odometry is not None
    assert odometry.attrib["filename"] == "gz-sim-odometry-publisher-system"
    assert odometry.findtext("dimensions") == "3"
    assert odometry.findtext("odom_publish_frequency") == "20"
    assert odometry.findtext("odom_topic") == "/gazebo/private/iris/odometry"

    assert contact is not None
    assert contact.attrib["name"] == "iris_ground_contact"
    assert contact.findtext("always_on") == "true"
    assert contact.findtext("update_rate") == "20"
    assert contact.findtext("contact/collision") == "ground_collision"


def test_sdf_contains_no_remote_gui_or_out_of_scope_runtime_coupling():
    """Forbidden plugins or URIs would violate the passive, local Phase 3 boundary."""
    documents = (WORLD.read_text(encoding="utf-8"), MODEL.read_text(encoding="utf-8"))
    lowered = "\n".join(documents).lower()

    assert "<gui" not in lowered
    assert "ardupilot" not in lowered
    assert "electromagnet" not in lowered
    assert "http://" not in lowered
    assert "https://" not in lowered
    assert "fuel.gazebosim.org" not in lowered
    assert "multicopter-control" not in lowered
    assert "motor-model" not in lowered
    assert ".so" not in lowered
    uris = [
        uri.text.strip()
        for document in documents
        for uri in ET.fromstring(document).findall(".//uri")
        if uri.text
    ]
    assert uris == [
        "model://iris_phase3",
        "meshes/iris_collision.stl",
        "meshes/iris.dae",
        "meshes/iris_prop_ccw.dae",
        "meshes/iris_prop_ccw.dae",
        "meshes/iris_prop_cw.dae",
        "meshes/iris_prop_cw.dae",
    ]
