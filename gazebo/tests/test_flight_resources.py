"""Observable contracts for the separate ArduPilot-controlled flight fixture."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).resolve().parents[2]
RESOURCES = ROOT / "gazebo/resources"
WORLD = RESOURCES / "worlds/vertical_descent.sdf"
MODEL = RESOURCES / "models/iris_flight/model.sdf"
PLUGIN_PROVENANCE = ROOT / "gazebo/provenance/ardupilot_gazebo-plugin.json"
PLUGIN_PATCH = ROOT / "gazebo/plugin/0001-paused-initial-json.patch"


def test_resolver_selects_the_separate_local_flight_world():
    """Mapping the flight config to the passive world would remove actuator physics."""
    from drone_sim_gazebo.worlds import WorldConfig, resolve_world

    resolved = resolve_world(
        WorldConfig(world="vertical_descent", vehicle="iris_flight"),
        package_root=RESOURCES,
    )

    assert resolved.world_name == "vertical_descent"
    assert resolved.vehicle_id == "iris_flight"
    assert resolved.path == WORLD.resolve()
    assert resolved.world_sha256 == hashlib.sha256(WORLD.read_bytes()).hexdigest()
    assert dict(resolved.resource_sha256s)["worlds/vertical_descent.sdf"] == (
        resolved.world_sha256
    )


def test_flight_model_has_the_official_four_rotor_json_seam():
    """Missing rotor dynamics or a wrong JSON channel would leave the Iris passive."""
    model = ET.parse(MODEL).getroot().find("model")

    assert model.attrib["name"] == "iris_flight"
    assert model.findtext("include/uri") == "model://iris_phase3"
    assert model.findtext("include/name") == "airframe"

    imu = model.find("link[@name='imu_link']/sensor[@name='imu_sensor']")
    assert imu is not None
    assert imu.attrib["type"] == "imu"
    assert imu.findtext("update_rate") == "1000.0"

    rotor_names = {f"rotor_{index}" for index in range(4)}
    assert {link.attrib["name"] for link in model.findall("link") if link.attrib["name"].startswith("rotor_")} == rotor_names
    assert {
        joint.attrib["name"]
        for joint in model.findall("joint")
        if joint.attrib["name"].startswith("rotor_")
    } == {f"rotor_{index}_joint" for index in range(4)}

    lift_drag = model.findall("plugin[@name='gz::sim::systems::LiftDrag']")
    assert len(lift_drag) == 8
    assert {plugin.findtext("link_name") for plugin in lift_drag} == rotor_names
    assert all(plugin.attrib["filename"] == "gz-sim-lift-drag-system" for plugin in lift_drag)

    ardupilot = model.find("plugin[@name='ArduPilotPlugin']")
    assert ardupilot is not None
    assert ardupilot.attrib["filename"] == "ArduPilotPlugin"
    assert ardupilot.findtext("fdm_addr") == "0.0.0.0"
    assert ardupilot.findtext("fdm_port_in") == "9002"
    assert ardupilot.findtext("lock_step") == "1"
    assert ardupilot.findtext("no_time_sync") == "1"
    assert ardupilot.findtext("imuName") == "imu_link::imu_sensor"
    controls = ardupilot.findall("control")
    assert [control.attrib["channel"] for control in controls] == ["0", "1", "2", "3"]
    assert [control.findtext("jointName") for control in controls] == [
        f"rotor_{index}_joint" for index in range(4)
    ]
    assert [float(control.findtext("multiplier")) for control in controls] == [
        838.0,
        838.0,
        -838.0,
        -838.0,
    ]


def test_competition_generator_preserves_the_validated_flight_motor_layout(tmp_path):
    """Accepting a changed motor seam would generate an unflyable competition Iris."""
    import sys

    scripts = ROOT / "gazebo/scripts"
    sys.path.insert(0, str(scripts))
    try:
        from prepare_competition_assets import prepare_assets
    finally:
        sys.path.remove(str(scripts))

    prepare_assets(
        RESOURCES,
        tmp_path,
        ROOT / "config/course.yaml",
        ROOT / "config/scenario.yaml",
    )
    source = ET.parse(MODEL).getroot().find("model")
    generated = ET.parse(
        tmp_path / "models/iris_competition/model.sdf"
    ).getroot().find("model")

    source_controls = source.findall("plugin[@name='ArduPilotPlugin']/control")
    generated_controls = generated.findall(
        "plugin[@name='ArduPilotPlugin']/control"
    )
    assert [node.attrib["channel"] for node in generated_controls] == [
        node.attrib["channel"] for node in source_controls
    ] == ["0", "1", "2", "3"]
    assert [node.findtext("jointName") for node in generated_controls] == [
        node.findtext("jointName") for node in source_controls
    ] == [f"rotor_{index}_joint" for index in range(4)]
    assert [node.findtext("multiplier") for node in generated_controls] == [
        node.findtext("multiplier") for node in source_controls
    ] == ["838", "838", "-838", "-838"]


def test_flight_world_retains_project_cameras_marker_and_exact_sim_cadence():
    """Replacing the project scene or camera cadence would corrupt public evidence."""
    world = ET.parse(WORLD).getroot().find("world")
    physics = world.find("physics")
    assert physics.findtext("max_step_size") == "0.001"

    plugins = {
        (plugin.attrib["filename"], plugin.attrib["name"])
        for plugin in world.findall("plugin")
    }
    assert ("gz-sim-imu-system", "gz::sim::systems::Imu") in plugins
    assert ("gz-sim-sensors-system", "gz::sim::systems::Sensors") in plugins

    iris = world.find("include[uri='model://iris_flight']")
    assert iris is not None
    assert iris.findtext("name") == "iris"
    assert iris.findtext("pose") == "0 0 0.195 0 0 0"

    marker = world.find("model[@name='landing_marker']")
    observer = world.find(
        "model[@name='observer_station']/link/sensor[@name='observer_camera']"
    )
    assert marker is not None
    assert observer is not None
    assert observer.findtext("update_rate") == "20"
    assert observer.findtext("camera/image/width") == "320"
    assert observer.findtext("camera/image/height") == "240"
    assert observer.findtext("camera/image/format") == "R8G8B8"
    assert 1 / float(observer.findtext("update_rate")) == pytest.approx(0.05)


def test_flight_runtime_routes_contact_bridge_to_the_selected_world():
    """Hard-coding the passive contact topic would prevent flight readiness."""
    from drone_sim_gazebo.ros_adapter.topics import contact_topic_for_world
    from drone_sim_gazebo.runtime.paths import bridge_config_for_world

    assert contact_topic_for_world("vertical_descent") == (
        "/world/vertical_descent/model/ground_plane/link/ground_link/sensor/"
        "iris_ground_contact/contact"
    )
    assert bridge_config_for_world("vertical_descent") == Path(
        "/etc/drone_sim/gazebo-bridge-flight.yaml"
    )


def test_competition_runtime_routes_private_topics_to_the_selected_world():
    from drone_sim_gazebo.ros_adapter.topics import (
        camera_topics_for_world,
        contact_topic_for_world,
        gazebo_topics_for_world,
        private_publisher_topics_for_world,
        readiness_publisher_topics_for_world,
        recorder_topics_for_world,
    )
    from drone_sim_gazebo.runtime.paths import bridge_config_for_world

    assert contact_topic_for_world("competition_mission") == (
        "/gazebo/private/iris/contact"
    )
    assert bridge_config_for_world("competition_mission") == Path(
        "/etc/drone_sim/gazebo-bridge-competition.yaml"
    )
    selected = "/gazebo/private/camera/competition_onboard/image"
    inherited = "/gazebo/private/camera/onboard/image"
    assert camera_topics_for_world("competition_mission") == (
        selected,
        "/gazebo/private/camera/observer/image",
    )
    assert selected in gazebo_topics_for_world("competition_mission")
    assert selected in private_publisher_topics_for_world("competition_mission")
    assert inherited not in gazebo_topics_for_world("competition_mission")
    assert inherited not in private_publisher_topics_for_world(
        "competition_mission"
    )
    assert readiness_publisher_topics_for_world("competition_mission") == ()
    assert "/competition/range/downward" not in recorder_topics_for_world(
        "competition_mission", competition_evidence=False
    )
    assert "/competition/range/downward" in recorder_topics_for_world(
        "competition_mission", competition_evidence=True
    )


def test_flight_clock_bridge_has_a_bounded_reliable_delivery_queue():
    bridge = (ROOT / "gazebo/config/bridge-flight.yaml").read_text(encoding="utf-8")
    clock = bridge.split(
        '- ros_topic_name: "/gazebo/private/clock"', 1
    )[1].split('- ros_topic_name: "/gazebo/private/iris/odometry"', 1)[0]

    assert "publisher_queue: 1000" in clock
    assert "subscriber_queue: 1000" in clock


def test_official_plugin_supply_is_pinned_to_the_reviewed_harmonic_revision():
    """A floating plugin source could silently change the JSON and lockstep seam."""
    provenance = json.loads(PLUGIN_PROVENANCE.read_text(encoding="utf-8"))

    assert provenance == {
        "build_target": "ArduPilotPlugin",
        "downstream_patches": [
            {
                "path": "gazebo/plugin/0001-paused-initial-json.patch",
                "purpose": "bootstrap paused JSON, bound duplicate recovery, and preserve one-for-one UDP lockstep",
            }
        ],
        "gazebo_release": "harmonic",
        "installed_library": "/opt/drone_sim/gazebo/plugins/libArduPilotPlugin.so",
        "license": "LGPL-3.0-only",
        "origin": "https://github.com/ArduPilot/ardupilot_gazebo.git",
        "revision": "082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5",
    }


def test_downstream_patch_bounds_paused_bootstrap_to_one_round_trip():
    """Paused readiness must not keep advancing SITL before RUNNING."""
    patch = PLUGIN_PATCH.read_text(encoding="utf-8")

    assert (
        "if (_info.paused && this->dataPtr->motorUpdates.load() < 1)"
        in patch
    )
    assert "this->dataPtr->motorUpdates.load() < 2" not in patch
    assert "this->ReceiveServoPacket();" in patch
    assert (
        "this->CreateStateJSON(initialPausedState ? 0.000001 : t, _ecm);"
        in patch
    )
    assert "!this->dataPtr->imuMsgValid && _simTime != 0.000001" in patch
    assert "this->dataPtr->initialStateSent = true;" in patch
    assert "ApplyMotorForces" not in patch


def test_downstream_patch_bounds_duplicate_recovery_to_one_per_burst():
    """Queued duplicate frames must not amplify JSON into new SITL frames."""
    patch = PLUGIN_PATCH.read_text(encoding="utf-8")

    assert "public: bool duplicateRecoverySent{false};" in patch
    assert "!this->dataPtr->duplicateRecoverySent" in patch
    assert "this->dataPtr->duplicateRecoverySent = true;" in patch
    assert patch.count("this->dataPtr->duplicateRecoverySent = false;") == 2
