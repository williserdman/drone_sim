"""Contracts for the compact one-payload search-and-deliver fixture."""

from __future__ import annotations

import hashlib
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
RESOURCES = ROOT / "gazebo/resources"
COURSE = ROOT / "config/course-search-delivery.yaml"
SCENARIO = ROOT / "config/scenario-search-delivery.yaml"


def _prepare_assets(output_root: Path) -> Path:
    scripts = ROOT / "gazebo/scripts"
    sys.path.insert(0, str(scripts))
    try:
        from prepare_competition_assets import prepare_assets
    finally:
        sys.path.remove(str(scripts))
    return prepare_assets(
        RESOURCES,
        output_root,
        COURSE,
        SCENARIO,
        profile="search_delivery",
        generate_markers=False,
    )


def test_search_delivery_profile_loads_one_grounded_payload_and_search_mission():
    """Competition-only validation must not reject the approved compact fixture."""
    from drone_sim_gazebo.competition_config import load_course, load_scenario

    course = load_course(COURSE, profile="search_delivery")
    scenario = load_scenario(SCENARIO, course, profile="search_delivery")

    assert course.attempt.duration_seconds == 240
    assert [(item.aruco_id, item.color, item.initial) for item in scenario.payloads] == [
        (3, "yellow", "WA")
    ]
    assert scenario.search_mission is not None
    assert (
        scenario.search_mission.aruco_id,
        scenario.search_mission.pickup_zone,
        scenario.search_mission.drop_zone,
        scenario.search_mission.search_start_xy,
    ) == (3, "WA", "F2", (16.0, 8.0))


@pytest.mark.parametrize(
    ("path", "value"),
    (
        (("camera", "horizontal_fov_rad"), 0.7),
        (("payload_interaction", "pickup_max_center_error_m"), 0.1),
        (("payload_geometry", "size_in"), [7, 6, 2]),
    ),
)
def test_search_delivery_profile_rejects_drift_from_calibrated_physics(
    tmp_path: Path,
    path: tuple[str, str],
    value: object,
):
    """An accepted calibration drift would decouple search and pickup from physics."""
    from drone_sim_gazebo.competition_config import (
        CompetitionConfigError,
        load_course,
        load_scenario,
    )

    raw = yaml.safe_load(SCENARIO.read_text(encoding="utf-8"))
    raw[path[0]][path[1]] = value
    changed = tmp_path / "scenario.yaml"
    changed.write_text(yaml.safe_dump(raw), encoding="utf-8")
    course = load_course(COURSE, profile="search_delivery")

    with pytest.raises(CompetitionConfigError, match="approved calibration"):
        load_scenario(changed, course, profile="search_delivery")


def test_search_delivery_generator_creates_separate_empty_vehicle_and_compact_world(
    tmp_path: Path,
):
    """A stale attached payload or competition URI would invalidate the search start."""
    output = _prepare_assets(tmp_path)
    world = ET.parse(output).getroot().find("world")
    vehicle = ET.parse(
        tmp_path / "models/iris_search_delivery/model.sdf"
    ).getroot().find("model")

    assert world is not None
    assert vehicle is not None
    assert output == tmp_path / "worlds/search_delivery.sdf"
    assert world.attrib["name"] == "search_delivery"
    assert world.findtext("physics/real_time_factor") == "1.0"
    assert world.findtext("physics/real_time_update_rate") == "1000"
    ground_width, ground_height = map(float, world.findtext(
        "model[@name='ground_plane']/link/collision/geometry/plane/size"
    ).split())
    for pad in world.findall("model"):
        if not pad.attrib["name"].startswith("pad_"):
            continue
        x, y, *_ = map(float, pad.findtext("pose").split())
        width, height, _ = map(float, pad.findtext("link/collision/geometry/box/size").split())
        assert abs(x) + width / 2 < ground_width / 2
        assert abs(y) + height / 2 < ground_height / 2
    assert world.findtext("include[uri='model://iris_search_delivery']/name") == "iris"
    assert world.find("include[uri='model://payload_2']") is None
    assert world.find("include[uri='model://payload_4']") is None
    payload = world.find("include[uri='model://payload_3']")
    assert payload is not None
    assert payload.findtext("pose").split()[:3] == ["18", "8", "0.0354"]
    assert vehicle.attrib["name"] == "iris_search_delivery"
    joints = vehicle.findall("plugin[@name='drone_sim::gazebo::DetachableJoint']")
    assert len(joints) == 1
    assert joints[0].findtext("child_model") == "payload_3"
    assert joints[0].findtext("initially_attached") == "false"
    assert not (tmp_path / "models/payload_3").exists()
    assert world.find("model[@name='pad_h']//collision[@name='wall_north_collision']") is None

    observer = world.find("model[@name='observer_station']")
    camera = observer.find("link/sensor[@name='observer_camera']")
    assert observer.findtext("pose") == "9 10 30 0 1.570796327 0"
    assert camera.findtext("camera/image/width") == "1280"
    assert camera.findtext("camera/image/height") == "960"
    assert float(camera.findtext("camera/horizontal_fov")) == pytest.approx(
        0.761012754
    )


def test_search_delivery_runtime_selects_only_payload_three_and_its_bridge():
    """Requiring absent payloads 2 and 4 would prevent readiness and completion."""
    from drone_sim_gazebo.ros_adapter.topics import (
        gazebo_topics_for_world,
        payload_ids_for_world,
        private_command_topics_for_world,
        private_publisher_topics_for_world,
        readiness_publisher_topics_for_world,
        recorder_topics_for_world,
    )
    from drone_sim_gazebo.runtime.children import gazebo_child_specs
    from drone_sim_gazebo.runtime.paths import bridge_config_for_world

    assert payload_ids_for_world("search_delivery") == (3,)
    assert bridge_config_for_world("search_delivery") == Path(
        "/etc/drone_sim/gazebo-bridge-search-delivery.yaml"
    )
    assert private_command_topics_for_world("search_delivery") == (
        "/gazebo/private/payload_3/command",
    )
    assert readiness_publisher_topics_for_world("search_delivery") == ()
    assert recorder_topics_for_world(
        "search_delivery", competition_evidence=True
    ) == (
        "/camera/onboard/image_raw",
        "/camera/onboard/frame_metadata",
        "/camera/observer/image_raw",
        "/camera/observer/frame_metadata",
        "/simulation/ground_truth",
        "/simulation/payload_state",
        "/competition/range/downward",
    )
    for topics in (
        gazebo_topics_for_world("search_delivery"),
        private_publisher_topics_for_world("search_delivery"),
    ):
        assert any("payload_3" in topic or "/payload/3/" in topic for topic in topics)
        assert not any("payload_2" in topic or "/payload/2/" in topic for topic in topics)
        assert not any("payload_4" in topic or "/payload/4/" in topic for topic in topics)

    bridge = yaml.safe_load(
        (ROOT / "gazebo/config/bridge-search-delivery.yaml").read_text(
            encoding="utf-8"
        )
    )
    ros_topics = {item["ros_topic_name"] for item in bridge}
    assert ros_topics == {
        "/gazebo/native/clock",
        "/gazebo/private/iris/odometry",
        "/gazebo/private/iris/contact",
        "/gazebo/private/range/downward",
        "/gazebo/private/payload_3/pose",
        "/gazebo/private/payload_3/contact_state",
        "/gazebo/private/payload_3/command",
        "/gazebo/private/payload_3/joint_state",
        "/gazebo/private/payload_3/result",
    }
    children = gazebo_child_specs(
        bridge_config=bridge_config_for_world("search_delivery"),
        environment={"GZ_PARTITION": "search"},
        world_name="search_delivery",
    )
    assert tuple(child.name for child in children) == (
        "clock_decimator",
        "bridge",
        "image_bridge_onboard",
        "image_bridge_observer",
    )


def test_search_delivery_world_resolves_and_server_accepts_only_realtime(
    tmp_path: Path,
):
    """The compact run must use its own immutable fixture at the approved 1x rate."""
    from drone_sim_gazebo.server import server_spec
    from drone_sim_gazebo.worlds import WorldConfig, resolve_world
    from orchestration.config import SimulationConfig

    resolved = resolve_world(
        WorldConfig("search_delivery", "iris_search_delivery"),
        package_root=RESOURCES,
    )
    run_directory = tmp_path / "00000000-0000-4000-8000-000000000505"
    run_directory.mkdir()
    spec = server_spec(
        run_id=run_directory.name,
        run_directory=run_directory,
        resolved_world=resolved,
        config=SimulationConfig(2026, 240_000_000_000, 1.0),
    )

    assert resolved.path == (RESOURCES / "worlds/search_delivery.sdf").resolve()
    assert resolved.world_sha256 == hashlib.sha256(resolved.path.read_bytes()).hexdigest()
    assert spec.argv[-1].endswith("/worlds/search_delivery.sdf")
    assert spec.environment["GZ_SIM_SYSTEM_PLUGIN_PATH"] == (
        "/opt/drone_sim/gazebo/plugins"
    )

    other_run = tmp_path / "00000000-0000-4000-8000-000000000506"
    other_run.mkdir()
    with pytest.raises(ValueError, match="target_real_time_factor"):
        server_spec(
            run_id=other_run.name,
            run_directory=other_run,
            resolved_world=resolved,
            config=SimulationConfig(2026, 240_000_000_000, 0.25),
        )
