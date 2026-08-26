#!/usr/bin/env python3
"""Generate the deterministic Comp2026 Gazebo world and physical models."""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

from drone_sim_gazebo.competition_config import (
    Camera,
    CourseConfig,
    Payload,
    PayloadGeometry,
    RangeSensor,
    ScenarioConfig,
    load_course,
    load_scenario,
    world_xy,
)


ROLE_COLORS = {
    "home": "0.04 0.04 0.04 1",
    "landing": "0.05 0.75 0.15 1",
    "fire": "0.85 0.05 0.05 1",
    "autonomous_pickup": "0.05 0.70 0.85 1",
    "manual_pickup": "0.05 0.70 0.85 1",
}
PAYLOAD_COLORS = {
    "red": "0.85 0.05 0.05 1",
    "yellow": "0.95 0.80 0.05 1",
    "blue": "0.05 0.20 0.90 1",
}
PAYLOAD_MOTOR_TORQUE_NM = 3.4
PAYLOAD_HARDPOINT_Z_M = -0.13
VEHICLE_INITIAL_Z_M = 0.195
POSE_RATE_HZ = 20
PAD_THICKNESS_M = 0.01


def _fmt(value: float) -> str:
    if abs(value) < 0.0000005:
        return "0"
    return f"{value:.6f}".rstrip("0").rstrip(".")


def _text(parent: ET.Element, tag: str, value: object) -> ET.Element:
    child = ET.SubElement(parent, tag)
    child.text = str(value)
    return child


def _write_xml(root: ET.Element, path: Path) -> None:
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(path, encoding="unicode", xml_declaration=True)


def _write_model_config(path: Path, name: str, description: str) -> None:
    path.write_text(
        "<?xml version=\"1.0\"?>\n"
        "<model>\n"
        f"  <name>{name}</name>\n"
        "  <version>1.0</version>\n"
        "  <sdf version=\"1.9\">model.sdf</sdf>\n"
        "  <author><name>Drone Sim</name></author>\n"
        f"  <description>{description}</description>\n"
        "</model>\n",
        encoding="utf-8",
    )


def _validate_motor_layout(model: ET.Element) -> None:
    rotor_names = {f"rotor_{index}" for index in range(4)}
    links = {node.attrib["name"] for node in model.findall("link") if node.attrib["name"].startswith("rotor_")}
    joints = {
        node.attrib["name"]
        for node in model.findall("joint")
        if node.attrib["name"].startswith("rotor_")
    }
    controls = model.findall("plugin[@name='ArduPilotPlugin']/control")
    lift_drag = model.findall("plugin[@name='gz::sim::systems::LiftDrag']")
    if links != rotor_names or joints != {f"rotor_{index}_joint" for index in range(4)}:
        raise RuntimeError("iris_flight must expose the exact four-rotor link and joint layout")
    if [node.attrib.get("channel") for node in controls] != ["0", "1", "2", "3"]:
        raise RuntimeError("iris_flight must expose ordered ArduPilot channels 0 through 3")
    if [node.findtext("jointName") for node in controls] != [
        f"rotor_{index}_joint" for index in range(4)
    ]:
        raise RuntimeError("iris_flight ArduPilot controls target unexpected joints")
    if [float(node.findtext("multiplier", "nan")) for node in controls] != [
        838.0,
        838.0,
        -838.0,
        -838.0,
    ]:
        raise RuntimeError("iris_flight ArduPilot motor multipliers are unexpected")
    if len(lift_drag) != 8 or {node.findtext("link_name") for node in lift_drag} != rotor_names:
        raise RuntimeError("iris_flight must expose two LiftDrag blades per rotor")
    if any(node.find("cmd_max") is None or node.find("cmd_min") is None for node in controls):
        raise RuntimeError("iris_flight motor controls must expose force limits")


def _add_pose_publisher(model: ET.Element) -> None:
    plugin = ET.SubElement(
        model,
        "plugin",
        {
            "filename": "gz-sim-pose-publisher-system",
            "name": "gz::sim::systems::PosePublisher",
        },
    )
    for tag, value in (
        ("publish_link_pose", "false"),
        ("publish_visual_pose", "false"),
        ("publish_collision_pose", "false"),
        ("publish_sensor_pose", "false"),
        ("publish_model_pose", "true"),
        ("publish_nested_model_pose", "false"),
        ("use_pose_vector_msg", "true"),
        ("update_frequency", str(POSE_RATE_HZ)),
    ):
        _text(plugin, tag, value)


def _add_sensor_link(model: ET.Element, camera: Camera, range_sensor: RangeSensor) -> None:
    link = ET.SubElement(model, "link", {"name": "competition_sensor_link"})
    _text(link, "pose", "0 0 0 0 0 0")
    inertial = ET.SubElement(link, "inertial")
    _text(inertial, "mass", "0.05")
    inertia = ET.SubElement(inertial, "inertia")
    for tag, value in (
        ("ixx", "0.0001"),
        ("iyy", "0.0001"),
        ("izz", "0.0001"),
        ("ixy", "0"),
        ("ixz", "0"),
        ("iyz", "0"),
    ):
        _text(inertia, tag, value)

    onboard = ET.SubElement(link, "sensor", {"name": "downward_camera", "type": "camera"})
    _text(
        onboard,
        "pose",
        " ".join(_fmt(value) for value in camera.body_position_m)
        + " 0 1.570796327 0",
    )
    _text(onboard, "topic", "/gazebo/private/camera/onboard/image")
    _text(onboard, "always_on", "true")
    _text(onboard, "update_rate", _fmt(camera.update_rate_hz))
    _text(onboard, "visualize", "false")
    camera_sdf = ET.SubElement(onboard, "camera")
    _text(camera_sdf, "horizontal_fov", _fmt(camera.horizontal_fov_rad))
    image = ET.SubElement(camera_sdf, "image")
    _text(image, "width", camera.width_px)
    _text(image, "height", camera.height_px)
    _text(image, "format", "R8G8B8")
    clip = ET.SubElement(camera_sdf, "clip")
    _text(clip, "near", "0.05")
    _text(clip, "far", "200")

    lidar = ET.SubElement(link, "sensor", {"name": "downward_range", "type": "gpu_lidar"})
    _text(lidar, "pose", "0.3 0 -0.1 0 1.570796327 0")
    _text(lidar, "topic", "/gazebo/private/range/downward")
    _text(lidar, "always_on", "true")
    _text(lidar, "update_rate", _fmt(range_sensor.update_rate_hz))
    _text(lidar, "visualize", "false")
    ray = ET.SubElement(lidar, "ray")
    scan = ET.SubElement(ray, "scan")
    horizontal = ET.SubElement(scan, "horizontal")
    _text(horizontal, "samples", "1")
    _text(horizontal, "resolution", "1")
    _text(horizontal, "min_angle", "0")
    _text(horizontal, "max_angle", "0")
    distance = ET.SubElement(ray, "range")
    _text(distance, "min", "0.05")
    _text(distance, "max", "40")
    _text(distance, "resolution", "0.01")

    joint = ET.SubElement(model, "joint", {"name": "competition_sensor_joint", "type": "fixed"})
    _text(joint, "parent", "airframe::base_link")
    _text(joint, "child", "competition_sensor_link")


def _add_hardpoint(model: ET.Element) -> None:
    link = ET.SubElement(model, "link", {"name": "payload_hardpoint"})
    _text(link, "pose", f"0 0 {_fmt(PAYLOAD_HARDPOINT_Z_M)} 0 0 0")
    inertial = ET.SubElement(link, "inertial")
    _text(inertial, "mass", "0.01")
    inertia = ET.SubElement(inertial, "inertia")
    for tag, value in (
        ("ixx", "0.000001"),
        ("iyy", "0.000001"),
        ("izz", "0.000001"),
        ("ixy", "0"),
        ("ixz", "0"),
        ("iyz", "0"),
    ):
        _text(inertia, tag, value)
    joint = ET.SubElement(model, "joint", {"name": "payload_hardpoint_joint", "type": "fixed"})
    _text(joint, "parent", "airframe::base_link")
    _text(joint, "child", "payload_hardpoint")


def _add_payload_joints(model: ET.Element, payloads: tuple[Payload, ...]) -> None:
    for payload in sorted(payloads, key=lambda item: item.aruco_id):
        marker_id = payload.aruco_id
        base = f"/gazebo/private/payload/{marker_id}"
        coordinator = ET.SubElement(
            model,
            "plugin",
            {
                "filename": "libcwru_payload_command_coordinator.so",
                "name": "drone_sim::gazebo::PayloadCommandCoordinator",
            },
        )
        _text(coordinator, "command_topic", f"{base}/command")
        _text(coordinator, "physical_state_topic", f"{base}/joint_state")
        _text(coordinator, "result_topic", f"{base}/result")
        _text(coordinator, "stock_attach_topic", f"{base}/physical/attach")
        _text(coordinator, "stock_detach_topic", f"{base}/physical/detach")

        detachable = ET.SubElement(
            model,
            "plugin",
            {
                "filename": "libdrone_sim_detachable_joint_system.so",
                "name": "drone_sim::gazebo::DetachableJoint",
            },
        )
        _text(detachable, "parent_link", "payload_hardpoint")
        _text(detachable, "child_model", f"payload_{marker_id}")
        _text(detachable, "child_link", "body")
        _text(detachable, "attach_topic", f"{base}/physical/attach")
        _text(detachable, "detach_topic", f"{base}/physical/detach")
        _text(detachable, "output_topic", f"{base}/joint_state")
        _text(detachable, "initially_attached", "true" if payload.initial == "attached" else "false")
        _text(detachable, "exclusive_parent", "true")


def _write_vehicle(source_root: Path, output_root: Path, scenario: ScenarioConfig) -> None:
    source = source_root / "models/iris_flight/model.sdf"
    try:
        root = ET.parse(source).getroot()
    except (OSError, ET.ParseError) as error:
        raise RuntimeError(f"unable to parse source Iris model {source}: {error}") from error
    model = root.find("model")
    if model is None or model.attrib.get("name") != "iris_flight":
        raise RuntimeError("source Iris model must be named iris_flight")
    if model.findtext("include/uri") != "model://iris_phase3" or model.findtext("include/name") != "airframe":
        raise RuntimeError("iris_flight must include the validated iris_phase3 airframe")
    _validate_motor_layout(model)
    for control in model.findall("plugin[@name='ArduPilotPlugin']/control"):
        control.find("cmd_max").text = _fmt(PAYLOAD_MOTOR_TORQUE_NM)
        control.find("cmd_min").text = _fmt(-PAYLOAD_MOTOR_TORQUE_NM)
    model.attrib["name"] = "iris_competition"
    _add_sensor_link(model, scenario.camera, scenario.range_sensor)
    _add_hardpoint(model)
    _add_payload_joints(model, scenario.payloads)
    _add_pose_publisher(model)

    target = output_root / "models/iris_competition"
    target.mkdir(parents=True, exist_ok=True)
    _write_xml(root, target / "model.sdf")
    _write_model_config(
        target / "model.config",
        "Drone Sim Iris Competition",
        "Validated Iris flight model with Comp2026 sensors and one payload hardpoint.",
    )


def _write_marker(path: Path, marker_id: int) -> None:
    try:
        import cv2
    except ImportError as error:
        raise RuntimeError(
            "opencv-contrib-python-headless is required to generate ArUco markers"
        ) from error
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    image = cv2.aruco.generateImageMarker(dictionary, marker_id, 512)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image):
        raise RuntimeError(f"failed to write marker texture {path}")


def _write_payload(
    output_root: Path,
    payload: Payload,
    geometry: PayloadGeometry,
    generate_marker: bool,
) -> None:
    marker_id = payload.aruco_id
    model_dir = output_root / f"models/payload_{marker_id}"
    texture = model_dir / f"materials/textures/marker_{marker_id}.png"
    if generate_marker:
        _write_marker(texture, marker_id)
    else:
        texture.parent.mkdir(parents=True, exist_ok=True)
        texture.write_bytes(b"test-marker")

    size_x, size_y, size_z = geometry.size_m
    mass = geometry.mass_kg
    inertia = {
        "ixx": mass * (size_y**2 + size_z**2) / 12,
        "iyy": mass * (size_x**2 + size_z**2) / 12,
        "izz": mass * (size_x**2 + size_y**2) / 12,
    }
    root = ET.Element("sdf", {"version": "1.9"})
    model = ET.SubElement(root, "model", {"name": f"payload_{marker_id}"})
    link = ET.SubElement(model, "link", {"name": "body"})
    inertial = ET.SubElement(link, "inertial")
    _text(inertial, "mass", _fmt(mass))
    inertia_sdf = ET.SubElement(inertial, "inertia")
    for tag in ("ixx", "iyy", "izz"):
        _text(inertia_sdf, tag, _fmt(inertia[tag]))
    for tag in ("ixy", "ixz", "iyz"):
        _text(inertia_sdf, tag, "0")

    size = " ".join(_fmt(value) for value in geometry.size_m)
    collision = ET.SubElement(link, "collision", {"name": "body_collision"})
    collision_geometry = ET.SubElement(collision, "geometry")
    collision_box = ET.SubElement(collision_geometry, "box")
    _text(collision_box, "size", size)
    visual = ET.SubElement(link, "visual", {"name": "body_visual"})
    visual_geometry = ET.SubElement(visual, "geometry")
    visual_box = ET.SubElement(visual_geometry, "box")
    _text(visual_box, "size", size)
    material = ET.SubElement(visual, "material")
    _text(material, "ambient", PAYLOAD_COLORS[payload.color])
    _text(material, "diffuse", PAYLOAD_COLORS[payload.color])

    marker = ET.SubElement(link, "visual", {"name": "marker"})
    _text(marker, "pose", f"0 0 {_fmt(size_z / 2 + 0.001)} 0 0 0")
    marker_geometry = ET.SubElement(marker, "geometry")
    plane = ET.SubElement(marker_geometry, "plane")
    _text(plane, "normal", "0 0 1")
    _text(
        plane,
        "size",
        f"{_fmt(geometry.marker_size_m)} {_fmt(geometry.marker_size_m)}",
    )
    marker_material = ET.SubElement(marker, "material")
    _text(marker_material, "ambient", "1 1 1 1")
    _text(marker_material, "diffuse", "1 1 1 1")
    _text(marker_material, "specular", "0 0 0 1")
    pbr = ET.SubElement(marker_material, "pbr")
    metal = ET.SubElement(pbr, "metal")
    _text(metal, "albedo_map", f"materials/textures/marker_{marker_id}.png")
    _text(metal, "roughness", "1")
    _text(metal, "metalness", "0")

    contact = ET.SubElement(link, "sensor", {"name": "ground_contact", "type": "contact"})
    _text(contact, "topic", f"/gazebo/private/payload/{marker_id}/contacts")
    _text(contact, "always_on", "true")
    _text(contact, "update_rate", "20")
    contact_sdf = ET.SubElement(contact, "contact")
    _text(contact_sdf, "collision", "body_collision")
    _add_pose_publisher(model)

    model_dir.mkdir(parents=True, exist_ok=True)
    _write_xml(root, model_dir / "model.sdf")
    _write_model_config(
        model_dir / "model.config",
        f"Comp2026 Payload {marker_id}",
        f"Physical 6x6x2 inch payload with ArUco 4x4-250 marker {marker_id}.",
    )


def _add_box(
    parent: ET.Element,
    tag: str,
    name: str,
    size: tuple[float, float, float],
    pose: tuple[float, float, float],
    color: str | None = None,
) -> ET.Element:
    node = ET.SubElement(parent, tag, {"name": name})
    _text(node, "pose", " ".join(_fmt(value) for value in (*pose, 0, 0, 0)))
    geometry = ET.SubElement(node, "geometry")
    box = ET.SubElement(geometry, "box")
    _text(box, "size", " ".join(_fmt(value) for value in size))
    if color is not None:
        material = ET.SubElement(node, "material")
        _text(material, "ambient", color)
        _text(material, "diffuse", color)
    return node


def _add_pad(world: ET.Element, course: CourseConfig, name: str) -> None:
    point = course.waypoints[name]
    x_m, y_m = world_xy(course, name)
    model = ET.SubElement(world, "model", {"name": f"pad_{name.lower()}"})
    _text(model, "static", "true")
    _text(
        model,
        "pose",
        f"{_fmt(x_m)} {_fmt(y_m)} {_fmt(PAD_THICKNESS_M / 2)} 0 0 0",
    )
    link = ET.SubElement(model, "link", {"name": "pad_link"})
    pad_size = (point.width_m, point.height_m, PAD_THICKNESS_M)
    _add_box(link, "collision", "tarp_collision", pad_size, (0, 0, 0))
    _add_box(
        link,
        "visual",
        "tarp_visual",
        pad_size,
        (0, 0, 0),
        ROLE_COLORS[point.role],
    )
    if name not in {"H", "L"}:
        return
    wall_height = 0.3048
    wall_thickness = 0.04
    wall_specs = (
        ("north", (point.width_m, wall_thickness, wall_height), (0, point.height_m / 2, wall_height / 2)),
        ("south", (point.width_m, wall_thickness, wall_height), (0, -point.height_m / 2, wall_height / 2)),
        ("east", (wall_thickness, point.height_m, wall_height), (point.width_m / 2, 0, wall_height / 2)),
        ("west", (wall_thickness, point.height_m, wall_height), (-point.width_m / 2, 0, wall_height / 2)),
    )
    for wall_name, size, pose in wall_specs:
        _add_box(link, "collision", f"wall_{wall_name}_collision", size, pose)
        _add_box(
            link,
            "visual",
            f"wall_{wall_name}_visual",
            size,
            pose,
            "0.9 0.9 0.9 0.25",
        )


def _add_observer(world: ET.Element) -> None:
    model = ET.SubElement(world, "model", {"name": "observer_station"})
    _text(model, "pose", "-76.2 -100 90 0 0.733 1.570796327")
    _text(model, "static", "true")
    link = ET.SubElement(model, "link", {"name": "observer_link"})
    sensor = ET.SubElement(link, "sensor", {"name": "observer_camera", "type": "camera"})
    _text(sensor, "pose", "0 0 0 0 0 0")
    _text(sensor, "topic", "/gazebo/private/camera/observer/image")
    _text(sensor, "always_on", "true")
    _text(sensor, "update_rate", "20")
    _text(sensor, "visualize", "false")
    camera = ET.SubElement(sensor, "camera")
    _text(camera, "horizontal_fov", "1.5")
    image = ET.SubElement(camera, "image")
    _text(image, "width", "640")
    _text(image, "height", "480")
    _text(image, "format", "R8G8B8")
    clip = ET.SubElement(camera, "clip")
    _text(clip, "near", "0.1")
    _text(clip, "far", "300")


def _write_world(output_root: Path, course: CourseConfig, scenario: ScenarioConfig) -> Path:
    root = ET.Element("sdf", {"version": "1.9"})
    world = ET.SubElement(root, "world", {"name": "competition_mission"})
    _text(world, "gravity", "0 0 -9.8")
    physics = ET.SubElement(world, "physics", {"name": "competition_physics", "type": "ode"})
    _text(physics, "max_step_size", "0.001")
    _text(physics, "real_time_factor", "0.25")
    _text(physics, "real_time_update_rate", "250")
    scene = ET.SubElement(world, "scene")
    _text(scene, "ambient", "0.4 0.4 0.4 1")
    _text(scene, "background", "0.7 0.8 0.9 1")
    _text(scene, "shadows", "false")
    sun = ET.SubElement(world, "light", {"name": "sun", "type": "directional"})
    _text(sun, "pose", "0 0 10 0 0 0")
    _text(sun, "cast_shadows", "false")
    _text(sun, "diffuse", "0.8 0.8 0.8 1")
    _text(sun, "specular", "0.2 0.2 0.2 1")
    _text(sun, "direction", "-0.5 0.1 -0.9")

    for filename, name in (
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-sensors-system", "gz::sim::systems::Sensors"),
        ("gz-sim-imu-system", "gz::sim::systems::Imu"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
        ("gz-sim-contact-system", "gz::sim::systems::Contact"),
    ):
        plugin = ET.SubElement(world, "plugin", {"filename": filename, "name": name})
        if name == "gz::sim::systems::Sensors":
            _text(plugin, "render_engine", "ogre2")

    ground = ET.SubElement(world, "model", {"name": "ground_plane"})
    _text(ground, "static", "true")
    ground_link = ET.SubElement(ground, "link", {"name": "ground_link"})
    collision = ET.SubElement(ground_link, "collision", {"name": "ground_collision"})
    geometry = ET.SubElement(collision, "geometry")
    plane = ET.SubElement(geometry, "plane")
    _text(plane, "normal", "0 0 1")
    _text(plane, "size", "400 100")
    visual = ET.SubElement(ground_link, "visual", {"name": "ground_visual"})
    visual_geometry = ET.SubElement(visual, "geometry")
    visual_plane = ET.SubElement(visual_geometry, "plane")
    _text(visual_plane, "normal", "0 0 1")
    _text(visual_plane, "size", "400 100")
    material = ET.SubElement(visual, "material")
    _text(material, "ambient", "0.25 0.25 0.25 1")
    _text(material, "diffuse", "0.35 0.35 0.35 1")
    ground_contact = ET.SubElement(
        ground_link,
        "sensor",
        {"name": "iris_ground_contact", "type": "contact"},
    )
    _text(ground_contact, "always_on", "true")
    _text(ground_contact, "update_rate", "20")
    ground_contact_sdf = ET.SubElement(ground_contact, "contact")
    _text(ground_contact_sdf, "collision", "ground_collision")

    for name in ("H", "L", "F2", "WA", "WM"):
        _add_pad(world, course, name)

    iris = ET.SubElement(world, "include")
    _text(iris, "uri", "model://iris_competition")
    _text(iris, "name", "iris")
    _text(iris, "pose", f"0 0 {_fmt(VEHICLE_INITIAL_Z_M)} 0 0 0")
    _text(iris, "static", "false")
    odometry = ET.SubElement(
        iris,
        "plugin",
        {
            "filename": "gz-sim-odometry-publisher-system",
            "name": "gz::sim::systems::OdometryPublisher",
        },
    )
    _text(odometry, "dimensions", "3")
    _text(odometry, "odom_publish_frequency", "20")
    _text(odometry, "odom_topic", "/gazebo/private/iris/odometry")
    _text(odometry, "odom_frame", "competition_mission")
    _text(odometry, "robot_base_frame", "iris")

    for payload in sorted(scenario.payloads, key=lambda item: item.aruco_id):
        include = ET.SubElement(world, "include")
        _text(include, "uri", f"model://payload_{payload.aruco_id}")
        _text(include, "name", f"payload_{payload.aruco_id}")
        if payload.initial == "attached":
            x_m, y_m = 0.0, 0.0
            z_m = VEHICLE_INITIAL_Z_M + PAYLOAD_HARDPOINT_Z_M
        else:
            x_m, y_m = world_xy(course, payload.initial)
            z_m = PAD_THICKNESS_M + scenario.payload_geometry.size_m[2] / 2
        _text(include, "pose", f"{_fmt(x_m)} {_fmt(y_m)} {_fmt(z_m)} 0 0 0")

    _add_observer(world)
    world_path = output_root / "worlds/competition_mission.sdf"
    _write_xml(root, world_path)
    return world_path


def prepare_assets(
    source_root: Path,
    output_root: Path,
    course_path: Path,
    scenario_path: Path,
    *,
    generate_markers: bool = True,
) -> Path:
    """Generate only Comp2026-owned outputs from validated source resources."""
    course = load_course(course_path)
    scenario = load_scenario(scenario_path, course)
    _write_vehicle(source_root, output_root, scenario)
    for payload in sorted(scenario.payloads, key=lambda item: item.aruco_id):
        _write_payload(output_root, payload, scenario.payload_geometry, generate_markers)
    return _write_world(output_root, course, scenario)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--course", required=True, type=Path)
    parser.add_argument("--scenario", required=True, type=Path)
    args = parser.parse_args()
    print(
        prepare_assets(
            args.source_root,
            args.output_root,
            args.course,
            args.scenario,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
