"""ROS 2 process adapter for inactive descent and competition payload authority."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
from types import MappingProxyType
from typing import Any, Mapping
from uuid import UUID

import yaml

from artifacts.runtime_protocol import RuntimeProtocol, canonical_run_id

from .controller import PayloadGateway, ScenarioController, parse_physical_result
from .payload import PayloadAuthority, PayloadRequest, PickupZone
from .scenario import InactiveScenarioEvent, ScenarioPolicy


PAYLOAD_IDS = (2, 3, 4)


@dataclass(frozen=True)
class RuntimeConfig:
    run_id: str
    run_directory: Path
    config_path: Path
    scenario: str
    pickup_zones: Mapping[str, PickupZone]
    payload_zones: Mapping[int, str | None]
    payload_capacity: int
    max_center_error_m: float

    @classmethod
    def competition_defaults(cls, run_id: str) -> "RuntimeConfig":
        return cls(
            run_id=run_id,
            run_directory=Path("/unused"),
            config_path=Path("/unused/configuration/run.json"),
            scenario="competition_v1",
            pickup_zones=MappingProxyType(
                {
                    "WA": PickupZone(-45.72, -9.144, 6.096, 6.096),
                    "WM": PickupZone(-45.72, 9.144, 6.096, 6.096),
                }
            ),
            payload_zones=MappingProxyType({2: None, 3: "WA", 4: "WM"}),
            payload_capacity=1,
            max_center_error_m=0.075,
        )

    @classmethod
    def from_environment(cls, environment: Mapping[str, str]) -> "RuntimeConfig":
        run_id = canonical_run_id(environment["SIM_RUN_ID"])
        run_directory = Path(environment["SIM_RUN_DIRECTORY"])
        config_path = Path(
            environment.get(
                "SIM_CONFIG_PATH", str(run_directory / "configuration/run.json")
            )
        )
        if not run_directory.is_absolute() or not config_path.is_absolute():
            raise ValueError("run and configuration paths must be absolute")
        if config_path != run_directory / "configuration/run.json":
            raise ValueError("SIM_CONFIG_PATH must be the current resolved configuration")
        try:
            document = json.loads(config_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise ValueError("resolved run configuration must be readable JSON") from error
        if not isinstance(document, dict) or document.get("run_id") != run_id:
            raise ValueError("resolved run configuration must match SIM_RUN_ID")
        scenario = document.get("scenario")
        if scenario == "descent_v1":
            return cls(
                run_id,
                run_directory,
                config_path,
                scenario,
                MappingProxyType({}),
                MappingProxyType({}),
                0,
                0.0,
            )
        if scenario != "competition_v1":
            raise ValueError("electromagnet scenario must be descent_v1 or competition_v1")
        competition = document.get("competition")
        if not isinstance(competition, dict) or not {"course", "scenario"}.issubset(
            competition
        ):
            raise ValueError("competition sources are missing from resolved configuration")
        if competition["course"] != "course.yaml" or competition["scenario"] != "scenario.yaml":
            raise ValueError("competition sources must use resolved artifact names")
        course_path = config_path.parent / "course.yaml"
        scenario_path = config_path.parent / "scenario.yaml"
        course_bytes = course_path.read_bytes()
        scenario_bytes = scenario_path.read_bytes()
        for name, payload in (("course", course_bytes), ("scenario", scenario_bytes)):
            expected = competition.get(f"{name}_sha256")
            if expected is not None and expected != hashlib.sha256(payload).hexdigest():
                raise ValueError(f"resolved {name} hash does not match copied bytes")
        try:
            course = yaml.safe_load(course_bytes)
            payload_scenario = yaml.safe_load(scenario_bytes)
        except yaml.YAMLError as error:
            raise ValueError("resolved competition YAML is invalid") from error
        if not isinstance(course, dict) or not isinstance(payload_scenario, dict):
            raise ValueError("resolved competition sources must be mappings")
        waypoint_documents = course.get("waypoints")
        payload_documents = payload_scenario.get("payloads")
        interaction = payload_scenario.get("payload_interaction")
        vehicle = payload_scenario.get("vehicle")
        if (
            not isinstance(waypoint_documents, dict)
            or not isinstance(payload_documents, list)
            or not isinstance(interaction, dict)
            or not isinstance(vehicle, dict)
        ):
            raise ValueError("resolved competition payload configuration is incomplete")
        pickup_zones: dict[str, PickupZone] = {}
        for name in ("WA", "WM"):
            waypoint = waypoint_documents.get(name)
            if not isinstance(waypoint, dict):
                raise ValueError(f"resolved pickup zone {name} is missing")
            try:
                pickup_zones[name] = PickupZone(
                    float(waypoint["x"]),
                    float(waypoint["y"]),
                    float(waypoint["width"]),
                    float(waypoint["height"]),
                )
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(f"resolved pickup zone {name} is invalid") from error
        payload_zones: dict[int, str | None] = {}
        for payload in payload_documents:
            if not isinstance(payload, dict):
                raise ValueError("resolved payload inventory is invalid")
            marker = payload.get("aruco_id")
            initial = payload.get("initial")
            if type(marker) is not int or marker not in PAYLOAD_IDS:
                raise ValueError("resolved payload inventory contains an unknown marker")
            payload_zones[marker] = None if initial == "attached" else str(initial)
        if payload_zones != {2: None, 3: "WA", 4: "WM"}:
            raise ValueError("resolved payload inventory must contain exact IDs 2, 3, and 4")
        capacity = vehicle.get("payload_capacity")
        tolerance = interaction.get("pickup_max_center_error_m")
        if capacity != 1 or isinstance(capacity, bool):
            raise ValueError("resolved payload capacity must be one")
        if isinstance(tolerance, bool) or not isinstance(tolerance, (int, float)):
            raise ValueError("resolved pickup center tolerance is invalid")
        tolerance_value = float(tolerance)
        if tolerance_value != 0.075:
            raise ValueError("resolved pickup center tolerance must be 0.075 m")
        return cls(
            run_id,
            run_directory,
            config_path,
            scenario,
            MappingProxyType(pickup_zones),
            MappingProxyType(payload_zones),
            capacity,
            tolerance_value,
        )

    def authority(self) -> PayloadAuthority:
        if self.scenario != "competition_v1":
            raise ValueError("payload authority is available only for competition_v1")
        return PayloadAuthority(
            run_id=self.run_id,
            pickup_zones=self.pickup_zones,
            payload_zones=self.payload_zones,
            payload_capacity=self.payload_capacity,
            max_center_error_m=self.max_center_error_m,
        )


def stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def assign_stamp(stamp: Any, timestamp_ns: int) -> None:
    stamp.sec = timestamp_ns // 1_000_000_000
    stamp.nanosec = timestamp_ns % 1_000_000_000


def _descent_main(config: RuntimeConfig) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from simulation_interfaces.msg import RunState, ScenarioEvent

    run_id = config.run_id
    run_directory = config.run_directory
    protocol = RuntimeProtocol(run_directory, run_id)
    rclpy.init()
    node = Node("drone_sim_electromagnet")
    publisher = node.create_publisher(
        ScenarioEvent,
        "/simulation/scenario_events",
        QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )

    def publish(event: InactiveScenarioEvent) -> None:
        message = ScenarioEvent()
        message.run_id = event.run_id
        assign_stamp(message.sim_timestamp, event.timestamp_ns)
        message.event_id = event.event_id
        message.magnet_id = event.magnet_id
        message.state = event.state
        publisher.publish(message)

    controller = ScenarioController(
        run_id=run_id,
        policy=ScenarioPolicy(run_id=run_id),
        publish=publish,
        protocol=protocol,
        stream=sys.stdout,
    )
    finalizing = False
    requested_stop = False
    last_timestamp_ns: int | None = None
    failure: str | None = None

    def stop(_signum: int, _frame: Any) -> None:
        nonlocal requested_stop
        requested_stop = True

    def clock_callback(message: Any) -> None:
        nonlocal last_timestamp_ns, failure
        value = stamp_ns(message.clock)
        try:
            controller.observe_clock(value)
            last_timestamp_ns = value
        except Exception as error:
            failure = str(error)

    def state_callback(message: Any) -> None:
        nonlocal finalizing
        if message.run_id == run_id and message.state == RunState.FINALIZING:
            finalizing = True

    node.create_subscription(
        Clock,
        "/clock",
        clock_callback,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    node.create_subscription(
        RunState,
        "/simulation/run_state",
        state_callback,
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    controller.mark_ready()
    exit_code = 0
    try:
        while rclpy.ok() and not finalizing and not requested_stop:
            rclpy.spin_once(node, timeout_sec=0.05)
            if failure is not None:
                controller.fail(last_timestamp_ns, failure)
                exit_code = 1
                break
            if protocol.read_finalize_request() is not None:
                finalizing = True
    finally:
        controller.finalize(last_timestamp_ns)
        node.destroy_node()
        protocol.close()
        rclpy.shutdown()
    return exit_code


def _competition_main(config: RuntimeConfig) -> int:
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from simulation_interfaces.msg import GroundTruth, PayloadEvent, PayloadState, RunState
    from simulation_interfaces.srv import PayloadCommand
    from std_msgs.msg import String

    protocol = RuntimeProtocol(config.run_directory, config.run_id)
    rclpy.init()
    node = Node("drone_sim_electromagnet")
    callback_group = ReentrantCallbackGroup()

    def qos(depth: int, *, transient: bool = False) -> QoSProfile:
        return QoSProfile(
            depth=depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=(
                DurabilityPolicy.TRANSIENT_LOCAL
                if transient
                else DurabilityPolicy.VOLATILE
            ),
        )

    event_publisher = node.create_publisher(
        PayloadEvent, "/simulation/payload_events", qos(100, transient=True)
    )
    command_publishers = {
        marker: node.create_publisher(
            String, f"/gazebo/private/payload_{marker}/command", qos(10)
        )
        for marker in PAYLOAD_IDS
    }

    def publish_command(marker: int, wire: str) -> None:
        message = String()
        message.data = wire
        command_publishers[marker].publish(message)

    def publish_event(record: Any) -> None:
        message = PayloadEvent()
        message.run_id = record.run_id
        assign_stamp(message.sim_timestamp, record.timestamp_ns)
        message.event_id = record.event_id
        message.aruco_id = record.aruco_id
        message.command_id = record.command_id
        message.action = record.action
        message.state = record.state
        message.code = record.code
        event_publisher.publish(message)

    gateway = PayloadGateway(
        config.authority(),
        publish_command=publish_command,
        publish_event=publish_event,
    )
    controller = ScenarioController(
        run_id=config.run_id,
        policy=None,
        publish=lambda _event: None,
        protocol=protocol,
        stream=sys.stdout,
        scenario="competition_v1",
    )
    finalizing = False
    requested_stop = False
    last_timestamp_ns: int | None = None

    def stop(_signum: int, _frame: Any) -> None:
        nonlocal requested_stop
        requested_stop = True

    def ground_truth_callback(message: Any) -> None:
        nonlocal last_timestamp_ns
        timestamp_ns = stamp_ns(message.sim_timestamp)
        if message.run_id == config.run_id:
            last_timestamp_ns = timestamp_ns
        gateway.accept_vehicle(
            message.run_id,
            timestamp_ns,
            (float(message.pose.position.x), float(message.pose.position.y)),
            bool(message.in_contact),
        )

    def payload_state_callback(message: Any) -> None:
        nonlocal last_timestamp_ns
        timestamp_ns = stamp_ns(message.sim_timestamp)
        if message.run_id == config.run_id:
            last_timestamp_ns = timestamp_ns
        gateway.accept_payload(
            message.run_id,
            timestamp_ns,
            int(message.aruco_id),
            (float(message.pose.position.x), float(message.pose.position.y)),
            bool(message.grounded),
            bool(message.attached),
        )

    def result_callback(marker: int, message: Any) -> None:
        try:
            gateway.accept_result(marker, message.data)
        except ValueError:
            return

    def state_callback(message: Any) -> None:
        nonlocal finalizing
        if message.run_id == config.run_id and message.state == RunState.FINALIZING:
            finalizing = True

    def service_callback(request: Any, response: Any) -> Any:
        if request.action == PayloadCommand.Request.ATTACH:
            action = "attach"
        elif request.action == PayloadCommand.Request.RELEASE:
            action = "release"
        else:
            action = "invalid"
        outcome = gateway.execute(
            PayloadRequest(
                run_id=request.run_id,
                aruco_id=int(request.aruco_id),
                action=action,
                command_id=request.command_id,
            )
        )
        response.accepted = outcome.accepted
        response.code = outcome.code
        response.detail = outcome.detail
        response.command_id = outcome.command_id
        response.response_sequence = outcome.response_sequence
        return response

    node.create_subscription(
        GroundTruth,
        "/simulation/ground_truth",
        ground_truth_callback,
        qos(10),
        callback_group=callback_group,
    )
    node.create_subscription(
        PayloadState,
        "/simulation/payload_state",
        payload_state_callback,
        qos(100),
        callback_group=callback_group,
    )
    for marker in PAYLOAD_IDS:
        node.create_subscription(
            String,
            f"/gazebo/private/payload_{marker}/result",
            lambda message, marker=marker: result_callback(marker, message),
            qos(10),
            callback_group=callback_group,
        )
    node.create_subscription(
        RunState,
        "/simulation/run_state",
        state_callback,
        qos(1, transient=True),
        callback_group=callback_group,
    )
    service = node.create_service(
        PayloadCommand,
        "/simulation/payload_command",
        service_callback,
        callback_group=callback_group,
    )
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    ready = False
    try:
        while rclpy.ok() and not finalizing and not requested_stop:
            executor.spin_once(timeout_sec=0.05)
            if not ready:
                result_publishers = frozenset(
                    marker
                    for marker in PAYLOAD_IDS
                    if node.count_publishers(
                        f"/gazebo/private/payload_{marker}/result"
                    )
                    >= 1
                )
                if gateway.ready(
                    result_publishers=result_publishers,
                    service_ready=service is not None,
                ):
                    controller.mark_ready()
                    ready = True
            if protocol.read_finalize_request() is not None:
                finalizing = True
    finally:
        executor.shutdown(timeout_sec=6.0)
        controller.finalize(last_timestamp_ns)
        node.destroy_node()
        protocol.close()
        rclpy.shutdown()
    return 0


def main() -> int:
    config = RuntimeConfig.from_environment(os.environ)
    if config.scenario == "descent_v1":
        return _descent_main(config)
    return _competition_main(config)


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "RuntimeConfig",
    "assign_stamp",
    "main",
    "parse_physical_result",
    "stamp_ns",
]
