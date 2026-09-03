"""Strict contracts for deterministic Comp2026 Gazebo assets."""

from __future__ import annotations

from dataclasses import FrozenInstanceError
from importlib.metadata import requires
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
from packaging.requirements import Requirement


ROOT = Path(__file__).resolve().parents[2]
RESOURCES = ROOT / "gazebo/resources"
COURSE = ROOT / "config/course.yaml"
SCENARIO = ROOT / "config/scenario.yaml"


def _prepare_assets(output_root: Path) -> Path:
    scripts = ROOT / "gazebo/scripts"
    sys.path.insert(0, str(scripts))
    try:
        from prepare_competition_assets import prepare_assets
    finally:
        sys.path.remove(str(scripts))
    return prepare_assets(RESOURCES, output_root, COURSE, SCENARIO)


def test_strict_config_is_immutable_and_home_relative():
    """A mutable or non-H origin could silently move every physical target."""
    from drone_sim_gazebo.competition_config import (
        load_course,
        load_scenario,
        world_xy,
    )

    course = load_course(COURSE)
    scenario = load_scenario(SCENARIO, course)

    assert world_xy(course, "H") == (0.0, 0.0)
    assert world_xy(course, "L") == (-91.44, 0.0)
    assert world_xy(course, "F2") == (-152.4, 0.0)
    assert world_xy(course, "WA") == (-45.72, -9.144)
    assert world_xy(course, "WM") == (-45.72, 9.144)
    assert scenario.range_sensor.update_rate_hz == 20.0
    assert [(item.aruco_id, item.color, item.initial) for item in scenario.payloads] == [
        (2, "red", "attached"),
        (3, "yellow", "WA"),
        (4, "blue", "WM"),
    ]
    with pytest.raises(FrozenInstanceError):
        scenario.payload_capacity = 2
    with pytest.raises(TypeError):
        course.waypoints["H"] = course.waypoints["L"]


def test_generated_competition_world_has_exact_course_and_payload_layout(tmp_path):
    """Wrong origin math or staging would make the mission physically impossible."""
    output = _prepare_assets(tmp_path)
    world = ET.parse(output).getroot().find("world")

    assert world is not None
    assert world.attrib["name"] == "competition_mission"
    assert world.find("model[@name='pad_h']/pose").text.split()[:2] == ["0", "0"]
    assert world.find("model[@name='pad_l']/pose").text.split()[:2] == [
        "-91.44",
        "0",
    ]
    assert world.find("model[@name='pad_f2']/pose").text.split()[:2] == [
        "-152.4",
        "0",
    ]
    assert world.find("model[@name='pad_wa']/pose").text.split()[:2] == [
        "-45.72",
        "-9.144",
    ]
    assert world.find("model[@name='pad_wm']/pose").text.split()[:2] == [
        "-45.72",
        "9.144",
    ]
    poses = {node.findtext("name"): node.findtext("pose") for node in world.findall("include")}
    assert poses["payload_2"].split()[:3] == ["0", "0", "0.065"]
    assert poses["payload_3"].split()[:3] == ["-45.72", "-9.144", "0.0354"]
    assert poses["payload_4"].split()[:3] == ["-45.72", "9.144", "0.0354"]
    assert "set_pose" not in output.read_text(encoding="utf-8").lower()


def test_generated_realtime_world_preserves_identity_at_one_x(tmp_path):
    _prepare_assets(tmp_path)
    world = ET.parse(
        tmp_path / "worlds/competition_mission_1x.sdf"
    ).getroot().find("world")

    assert world is not None
    assert world.attrib["name"] == "competition_mission"
    physics = world.find("physics")
    assert physics.findtext("max_step_size") == "0.001"
    assert physics.findtext("real_time_factor") == "1.0"
    assert physics.findtext("real_time_update_rate") == "1000"


def test_grounded_payload_bottom_meets_pad_top_without_interpenetration(tmp_path):
    """A center at ground height embeds each payload through its physical pad."""
    world = ET.parse(_prepare_assets(tmp_path)).getroot().find("world")

    assert world is not None
    for zone, marker_id, expected_xy in (
        ("wa", 3, (-45.72, -9.144)),
        ("wm", 4, (-45.72, 9.144)),
    ):
        pad = world.find(f"model[@name='pad_{zone}']")
        payload_include = next(
            node
            for node in world.findall("include")
            if node.findtext("name") == f"payload_{marker_id}"
        )
        pad_pose = tuple(float(value) for value in pad.findtext("pose").split())
        pad_size = tuple(
            float(value)
            for value in pad.findtext(
                "link/collision[@name='tarp_collision']/geometry/box/size"
            ).split()
        )
        payload_pose = tuple(
            float(value) for value in payload_include.findtext("pose").split()
        )
        payload_size = tuple(
            float(value)
            for value in ET.parse(
                tmp_path / f"models/payload_{marker_id}/model.sdf"
            ).findtext(".//collision/geometry/box/size").split()
        )

        assert payload_pose[:2] == expected_xy
        assert payload_pose[2] == pytest.approx(0.0354)
        assert payload_pose[2] - payload_size[2] / 2 == pytest.approx(
            pad_pose[2] + pad_size[2] / 2
        )


def test_generated_world_uses_exact_pad_footprints_and_physical_systems(tmp_path):
    """Visual-only or stale pad dimensions would disagree with physical truth."""
    world = ET.parse(_prepare_assets(tmp_path)).getroot().find("world")

    assert world is not None
    expected_sizes = {
        "h": "4.572 4.572 0.01",
        "l": "4.572 4.572 0.01",
        "f2": "0.9144 0.9144 0.01",
        "wa": "6.096 6.096 0.01",
        "wm": "6.096 6.096 0.01",
    }
    for name, expected in expected_sizes.items():
        model = world.find(f"model[@name='pad_{name}']")
        assert model.findtext("link/collision[@name='tarp_collision']/geometry/box/size") == expected
        assert model.findtext("link/visual[@name='tarp_visual']/geometry/box/size") == expected
    plugins = {(node.attrib["filename"], node.attrib["name"]) for node in world.findall("plugin")}
    assert ("gz-sim-physics-system", "gz::sim::systems::Physics") in plugins
    assert ("gz-sim-contact-system", "gz::sim::systems::Contact") in plugins
    assert ("gz-sim-sensors-system", "gz::sim::systems::Sensors") in plugins


def test_generated_vehicle_has_centered_camera_offset_range_and_one_hardpoint(tmp_path):
    """A shifted camera, centered beam, or multiple hardpoints breaks pickup truth."""
    _prepare_assets(tmp_path)
    model = ET.parse(tmp_path / "models/iris_competition/model.sdf").getroot().find("model")

    assert model is not None
    camera = model.find("link[@name='competition_sensor_link']/sensor[@name='downward_camera']")
    assert camera is not None
    assert camera.findtext("pose") == "0 0 -0.1 0 1.570796327 0"
    assert camera.findtext("topic") == (
        "/gazebo/private/camera/competition_onboard/image"
    )
    assert camera.findtext("update_rate") == "20"
    assert camera.findtext("camera/image/width") == "640"
    assert camera.findtext("camera/image/height") == "480"

    inherited = ET.parse(
        RESOURCES / "models/iris_phase3/model.sdf"
    ).getroot().find("model/.//sensor[@name='onboard_camera']")
    assert inherited is not None
    assert inherited.findtext("topic") != camera.findtext("topic")

    sensor = model.find("link[@name='competition_sensor_link']/sensor[@name='downward_range']")
    assert sensor is not None
    assert sensor.attrib["type"] == "gpu_lidar"
    assert sensor.findtext("pose") == "0.3 0 -0.1 0 1.570796327 0"
    assert sensor.findtext("topic") == "/gazebo/private/range/downward"
    assert sensor.findtext("update_rate") == "20"
    assert sensor.findtext("ray/scan/horizontal/samples") == "1"

    hardpoints = model.findall("link[@name='payload_hardpoint']")
    assert len(hardpoints) == 1
    assert hardpoints[0].findtext("pose") == "0 0 -0.13 0 0 0"
    assert model.findtext("joint[@name='payload_hardpoint_joint']/parent") == "airframe::base_link"
    assert model.findtext("joint[@name='payload_hardpoint_joint']/child") == "payload_hardpoint"


def test_generated_competition_vehicle_contact_source_covers_only_its_four_legs(
    tmp_path,
):
    """Pad or payload contact must not masquerade as Iris landing truth."""
    _prepare_assets(tmp_path)
    model = ET.parse(
        tmp_path / "models/iris_competition/model.sdf"
    ).getroot().find("model")

    assert model is not None
    airframe = model.find("model[@name='airframe']")
    assert airframe is not None
    sensor = airframe.find(
        "link[@name='base_link']/sensor[@name='vehicle_leg_contact']"
    )
    assert sensor is not None
    assert sensor.attrib["type"] == "contact"
    assert sensor.find("topic") is None
    assert sensor.findtext("always_on") == "true"
    assert sensor.findtext("update_rate") == "20"
    assert [node.text for node in sensor.findall("contact/collision")] == [
        "front_left_leg_collision",
        "front_right_leg_collision",
        "rear_left_leg_collision",
        "rear_right_leg_collision",
    ]
    assert sensor.findtext("contact/topic") == "/gazebo/private/iris/contact"

    inherited = ET.parse(
        RESOURCES / "models/iris_phase3/model.sdf"
    ).getroot().find("model")
    assert inherited is not None
    assert inherited.find(".//sensor[@name='vehicle_leg_contact']") is None


def test_generated_payload_uses_current_scenario_geometry_and_marker_identity(tmp_path):
    """Stale transfer geometry or the wrong ArUco dictionary corrupts pickup physics."""
    _prepare_assets(tmp_path)
    payload = ET.parse(tmp_path / "models/payload_3/model.sdf")
    marker_path = tmp_path / "models/payload_3/materials/textures/marker_3.png"

    assert payload.findtext(".//inertial/mass") == "1.133981"
    assert payload.findtext(".//collision/geometry/box/size") == "0.1524 0.1524 0.0508"
    assert payload.findtext(".//visual[@name='marker']/geometry/plane/size") == "0.1 0.1"
    assert marker_path.stat().st_size > 1000

    import cv2

    image = cv2.imread(str(marker_path), cv2.IMREAD_GRAYSCALE)
    # Detection needs scene context beyond the marker's black outer cell. The
    # generated texture intentionally fills the exact 100 mm marker plane.
    image = cv2.copyMakeBorder(
        image, 32, 32, 32, 32, cv2.BORDER_CONSTANT, value=255
    )
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary)
    assert len(corners) == 1
    assert ids.flatten().tolist() == [3]


def test_installed_gazebo_package_declares_its_yaml_runtime_dependency():
    """A standalone Gazebo package install must bring the loader's YAML parser."""
    requirement_names = {
        Requirement(value).name.lower()
        for value in requires("drone-sim-gazebo") or ()
    }

    assert "pyyaml" in requirement_names


def test_payload_joint_topics_and_initial_states_are_exact(tmp_path):
    """Wrong child names, states, or topics would make commands non-causal."""
    _prepare_assets(tmp_path)
    model = ET.parse(tmp_path / "models/iris_competition/model.sdf").getroot().find("model")

    assert model is not None
    joints = model.findall("plugin[@name='drone_sim::gazebo::DetachableJoint']")
    coordinators = model.findall(
        "plugin[@name='drone_sim::gazebo::PayloadCommandCoordinator']"
    )
    assert len(joints) == len(coordinators) == 3
    for marker_id, expected_initial in ((2, "true"), (3, "false"), (4, "false")):
        base = f"/gazebo/private/payload/{marker_id}"
        joint = next(node for node in joints if node.findtext("child_model") == f"payload_{marker_id}")
        coordinator = next(node for node in coordinators if node.findtext("command_topic") == f"{base}/command")
        assert joint.findtext("parent_link") == "payload_hardpoint"
        assert joint.findtext("child_link") == "body"
        assert joint.findtext("attach_topic") == f"{base}/physical/attach"
        assert joint.findtext("detach_topic") == f"{base}/physical/detach"
        assert joint.findtext("output_topic") == f"{base}/joint_state"
        assert joint.findtext("contact_sensor") == "ground_contact"
        assert joint.findtext("contact_state_topic") == f"{base}/contact_state"
        assert joint.findtext("initially_attached") == expected_initial
        assert joint.findtext("exclusive_parent") == "true"
        assert joint.findtext("state_publish_period") == "0.05"
        assert coordinator.findtext("physical_state_topic") == f"{base}/joint_state"
        assert coordinator.findtext("result_topic") == f"{base}/result"
        assert coordinator.findtext("stock_attach_topic") == f"{base}/physical/attach"
        assert coordinator.findtext("stock_detach_topic") == f"{base}/physical/detach"
        assert coordinator.find("initially_attached") is None


def test_payloads_publish_poses_and_contacts_without_pose_mutation(tmp_path):
    """Missing Gazebo-owned pose/contact output would tempt consumers to synthesize truth."""
    _prepare_assets(tmp_path)

    for marker_id in (2, 3, 4):
        model = ET.parse(tmp_path / f"models/payload_{marker_id}/model.sdf")
        pose = model.find(".//plugin[@name='gz::sim::systems::PosePublisher']")
        contact = model.find(".//sensor[@name='ground_contact']")
        assert pose is not None
        assert pose.findtext("publish_model_pose") == "true"
        assert pose.findtext("publish_link_pose") == "false"
        assert pose.find("topic") is None
        assert contact is not None
        assert contact.attrib["type"] == "contact"
        assert contact.findtext("contact/collision") == "body_collision"
        assert contact.findtext("update_rate") == "20"
        assert contact.findtext("topic") == f"/gazebo/private/payload/{marker_id}/contacts"
        text = (tmp_path / f"models/payload_{marker_id}/model.sdf").read_text()
        assert "set_pose" not in text.lower()


def test_generated_observer_camera_is_fixed_640x480_at_twenty_hertz(tmp_path):
    """A mismatched observer stream would break synchronized competition evidence."""
    world = ET.parse(_prepare_assets(tmp_path)).getroot().find("world")

    observer = world.find("model[@name='observer_station']")
    camera = observer.find("link/sensor[@name='observer_camera']")
    assert observer.findtext("static") == "true"
    assert camera.findtext("topic") == "/gazebo/private/camera/observer/image"
    assert camera.findtext("update_rate") == "20"
    assert camera.findtext("camera/image/width") == "640"
    assert camera.findtext("camera/image/height") == "480"
