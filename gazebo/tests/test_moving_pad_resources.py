from pathlib import Path
import json
import xml.etree.ElementTree as ET

import pytest
import yaml

from drone_sim_gazebo.ros_adapter.topics import (
    camera_topics_for_world,
    gazebo_topics_for_world,
    private_publisher_topics_for_world,
    recorder_topics_for_world,
)
from drone_sim_gazebo.runtime.paths import bridge_config_for_world
from drone_sim_gazebo.runtime.children import gazebo_child_specs
from drone_sim_gazebo.runtime.entrypoint import GazeboTransport
from drone_sim_gazebo.server import server_spec
from drone_sim_gazebo.worlds import WorldConfig, resolve_world
from orchestration.config import SimulationConfig


ROOT = Path(__file__).parents[2]
RESOURCES = ROOT / "gazebo/resources"
MOVING_WORLD = RESOURCES / "worlds/moving_pad_landing.sdf"
STATIONARY_WORLD = RESOURCES / "worlds/moving_pad_stationary.sdf"
PAD_MODEL = RESOURCES / "models/moving_pad/model.sdf"
IRIS_MODEL = RESOURCES / "models/iris_moving_pad/model.sdf"


@pytest.mark.parametrize(
    ("world_name", "start_x", "velocity"),
    (("moving_pad_landing", "10", "0.5"), ("moving_pad_stationary", "35", "0")),
)
def test_worlds_select_one_physical_rail_deck(world_name, start_x, velocity):
    world = ET.parse(RESOURCES / f"worlds/{world_name}.sdf").getroot().find("world")

    assert world is not None
    assert world.attrib["name"] == world_name
    assert world.findtext("physics/real_time_factor") == "0.1"
    assert world.findtext("physics/real_time_update_rate") == "100"
    pad = next(
        include
        for include in world.findall("include")
        if include.findtext("name") == "moving_pad"
    )
    assert pad.findtext("uri") == "model://moving_pad"
    assert pad.findtext("pose").split()[:3] == [start_x, "0", "0"]
    controller = pad.find("plugin[@name='drone_sim::gazebo::MovingPadController']")
    assert controller is not None
    assert controller.findtext("joint_name") == "rail_joint"
    assert controller.findtext("motion_start_sim_time_s") == "90"
    assert controller.findtext("velocity_mps") == velocity

    iris = next(
        include
        for include in world.findall("include")
        if include.findtext("name") == "iris"
    )
    assert iris.findtext("uri") == "model://iris_moving_pad"
    assert iris.findtext("pose").split()[:3] == ["0", "0", "0.195"]
    assert iris.findtext("plugin/odom_publish_frequency") == "20"


def test_pad_is_a_dynamic_three_meter_friction_surface_with_marker_seven():
    model = ET.parse(PAD_MODEL).getroot().find("model")

    assert model is not None
    assert model.attrib["name"] == "moving_pad"
    assert model.findtext("static") == "false"
    joint = model.find("joint[@name='rail_joint']")
    assert joint is not None
    assert joint.attrib["type"] == "prismatic"
    assert joint.findtext("parent") == "world"
    assert joint.findtext("child") == "deck"
    assert joint.findtext("axis/xyz") == "1 0 0"
    assert joint.findtext("axis/limit/effort") == "1000000"

    deck = model.find("link[@name='deck']")
    assert deck is not None
    collision = deck.find("collision[@name='deck_collision']")
    assert collision.findtext("geometry/box/size") == "3 3 0.2"
    assert float(collision.findtext("surface/friction/ode/mu")) >= 1.0
    assert float(collision.findtext("surface/friction/ode/mu2")) >= 1.0
    marker = deck.find("visual[@name='marker_7']")
    assert marker.findtext("geometry/plane/normal") == "0 0 1"
    assert marker.findtext("geometry/plane/size") == "0.1 0.1"
    assert marker.findtext("pose").split()[2] == "0.2005"
    assert marker.findtext("material/ambient") == "1 1 1 1"
    assert marker.findtext("material/diffuse") == "1 1 1 1"
    assert marker.findtext("material/specular") == "0 0 0 1"
    assert marker.findtext("material/pbr/metal/albedo_map") == (
        "materials/textures/marker_7.png"
    )
    assert (
        PAD_MODEL.parent / "materials/textures/marker_7.png"
    ).is_file()

    sensor = deck.find("sensor[@name='pad_contact']")
    assert sensor is not None
    assert sensor.attrib["type"] == "contact"
    assert sensor.findtext("update_rate") == "20"
    assert sensor.findtext("contact/collision") == "deck_collision"
    assert sensor.findtext("contact/topic") == "/gazebo/private/moving_pad/contact"


def test_moving_iris_has_calibrated_camera_range_and_no_payload_hardware():
    model = ET.parse(IRIS_MODEL).getroot().find("model")

    assert model is not None
    assert model.attrib["name"] == "iris_moving_pad"
    camera = model.find("link[@name='competition_sensor_link']/sensor[@name='downward_camera']")
    assert camera is not None
    assert camera.findtext("update_rate") == "20"
    assert camera.findtext("camera/horizontal_fov") == "0.6"
    assert camera.findtext("camera/image/width") == "640"
    assert camera.findtext("camera/image/height") == "480"
    assert camera.findtext("topic") == "/gazebo/private/camera/competition_onboard/image"
    assert model.find("link[@name='competition_sensor_link']/sensor[@name='downward_range']") is not None
    assert model.find("link[@name='payload_hardpoint']") is None
    assert not any(
        plugin.attrib.get("filename", "").startswith("libcwru_payload")
        or "detachable_joint" in plugin.attrib.get("filename", "")
        for plugin in model.findall("plugin")
    )


def test_worlds_have_twenty_hertz_route_wide_observer_and_native_pad_truth():
    for world_path in (MOVING_WORLD, STATIONARY_WORLD):
        world = ET.parse(world_path).getroot().find("world")
        observer = world.find(
            "model[@name='observer_station']/link/sensor[@name='observer_camera']"
        )
        assert observer is not None
        assert observer.findtext("update_rate") == "20"
        assert observer.findtext("camera/image/width") == "640"
        assert observer.findtext("camera/image/height") == "480"
        assert float(observer.findtext("camera/clip/far")) >= 100
        pad_include = next(
            include
            for include in world.findall("include")
            if include.findtext("name") == "moving_pad"
        )
        odometry = pad_include.find(
            "plugin[@name='gz::sim::systems::OdometryPublisher']"
        )
        assert odometry.findtext("odom_publish_frequency") == "20"
        assert odometry.findtext("odom_topic") == "/gazebo/private/moving_pad/odometry"


def test_moving_pad_runtime_inventories_are_exact():
    expected_gazebo = {
        "/clock",
        "/gazebo/private/camera/competition_onboard/image",
        "/gazebo/private/camera/observer/image",
        "/gazebo/private/iris/odometry",
        "/gazebo/private/iris/contact",
        "/gazebo/private/range/downward",
        "/gazebo/private/moving_pad/odometry",
        "/gazebo/private/moving_pad/contact",
    }
    for world_name in ("moving_pad_landing", "moving_pad_stationary"):
        resolved = resolve_world(WorldConfig(world_name, "iris_moving_pad"))
        assert resolved.world_name == world_name
        assert resolved.vehicle_id == "iris_moving_pad"
        assert set(gazebo_topics_for_world(world_name)) == expected_gazebo
        assert set(private_publisher_topics_for_world(world_name)) == {
            topic if topic != "/clock" else "/gazebo/private/clock"
            for topic in expected_gazebo
        }
        assert camera_topics_for_world(world_name) == (
            "/gazebo/private/camera/competition_onboard/image",
            "/gazebo/private/camera/observer/image",
        )
        assert "/simulation/landing_pad_state" in recorder_topics_for_world(
            world_name, competition_evidence=True
        )
        assert bridge_config_for_world(world_name) == Path(
            "/etc/drone_sim/gazebo-bridge-moving-pad.yaml"
        )

    entries = yaml.safe_load((ROOT / "gazebo/config/bridge-moving-pad.yaml").read_text())
    bridged = {(entry["gz_topic_name"], entry["ros_topic_name"]) for entry in entries}
    assert (
        "/gazebo/private/moving_pad/odometry",
        "/gazebo/private/moving_pad/odometry",
    ) in bridged
    assert (
        "/gazebo/private/moving_pad/contact",
        "/gazebo/private/moving_pad/contact",
    ) in bridged


def test_moving_pad_server_and_children_select_flight_plugins_and_clock_decimator(
    tmp_path,
):
    world = resolve_world(WorldConfig("moving_pad_landing", "iris_moving_pad"))
    run_id = "00000000-0000-4000-8000-000000000507"
    run_directory = tmp_path / run_id
    run_directory.mkdir()

    spec = server_spec(
        run_id=run_id,
        run_directory=run_directory,
        resolved_world=world,
        config=SimulationConfig(1, 90_000_000_000, 0.1),
    )
    assert spec.argv[-1].endswith("/worlds/moving_pad_landing.sdf")
    assert "GZ_SIM_SYSTEM_PLUGIN_PATH" in spec.environment

    transport = GazeboTransport(
        environment=spec.environment,
        world_name="moving_pad_landing",
        run=lambda *_args, **_kwargs: None,
    )
    transport.assert_typed_readiness_supported()

    specs = gazebo_child_specs(
        bridge_config=bridge_config_for_world("moving_pad_landing"),
        environment=spec.environment,
        world_name="moving_pad_landing",
    )
    assert tuple(child.name for child in specs) == (
        "clock_decimator",
        "bridge",
        "image_bridge_onboard",
        "image_bridge_observer",
    )


def test_operator_moving_pad_template_resolves_to_launchable_server_spec(tmp_path):
    document = json.loads((ROOT / "config/configured-moving-pad-run.json").read_text())
    simulation = document["simulation"]
    world = resolve_world(WorldConfig(document["world"], document["vehicle"]))
    run_id = "00000000-0000-4000-8000-000000000508"
    run_directory = tmp_path / run_id
    run_directory.mkdir()

    spec = server_spec(
        run_id=run_id,
        run_directory=run_directory,
        resolved_world=world,
        config=SimulationConfig(
            simulation["seed"],
            int(simulation["duration_sim_seconds"] * 1_000_000_000),
            simulation["target_real_time_factor"],
        ),
    )

    assert spec.argv[-1].endswith("/worlds/moving_pad_landing.sdf")
