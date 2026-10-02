from __future__ import annotations

from collections import deque
import json
from pathlib import Path
from types import ModuleType, SimpleNamespace
import signal
import sys

from pymavlink import mavutil

from artifacts.runtime_status import RuntimeStatus, status_document, status_name
import drone_sim_companion.configured_runtime as configured_runtime
import drone_sim_companion.runtime_node as runtime_node
from drone_sim_companion.configured_runtime import run_configured
from drone_sim_companion.mission_plan import parse_mission_plan


RUN_ID = "00000000-0000-4000-8000-000000000001"


def heartbeat(mode: int, *, armed: bool):
    base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    if armed:
        base_mode |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    return mavutil.mavlink.MAVLink_heartbeat_message(
        mavutil.mavlink.MAV_TYPE_QUADROTOR, mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode, mode, mavutil.mavlink.MAV_STATE_ACTIVE, 3)


class FakeMav:
    def __init__(self, connection):
        self.connection = connection
        self.command_ids = []

    def command_long_send(self, *arguments):
        command = arguments[2]
        self.command_ids.append(command)
        accepted = mavutil.mavlink.MAV_RESULT_ACCEPTED
        if command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            self.connection.messages.extend([
                mavutil.mavlink.MAVLink_command_ack_message(command, accepted),
                heartbeat(4, armed=False),
            ])
        elif command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            self.connection.messages.extend([
                mavutil.mavlink.MAVLink_command_ack_message(command, accepted),
                heartbeat(4, armed=True),
            ])
        elif command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            self.connection.messages.extend([
                mavutil.mavlink.MAVLink_command_ack_message(command, accepted),
                mavutil.mavlink.MAVLink_extended_sys_state_message(
                    0, mavutil.mavlink.MAV_LANDED_STATE_IN_AIR),
                mavutil.mavlink.MAVLink_global_position_int_message(
                    0, 374003371, -1220800351, 2_000, 2_000, 0, 0, 0, 0),
                heartbeat(4, armed=True),
            ])
        elif command == mavutil.mavlink.MAV_CMD_NAV_LAND:
            self.connection.messages.extend([
                mavutil.mavlink.MAVLink_command_ack_message(command, accepted),
                mavutil.mavlink.MAVLink_extended_sys_state_message(
                    0, mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND),
                heartbeat(9, armed=False),
            ])

    def request_data_stream_send(self, *arguments):
        pass

    def param_request_read_send(self, *arguments):
        name = arguments[2].decode("ascii")
        self.connection.parameter_requests.append(name)
        if name in self.connection.parameter_values:
            self.connection.messages.append(mavutil.mavlink.MAVLink_param_value_message(
                name.encode("ascii"), self.connection.parameter_values[name],
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32, 1, 0,
            ))

    def param_request_list_send(self, *arguments):
        self.connection.parameter_list_requests.append(arguments)
        for name, value in self.connection.parameter_values.items():
            self.connection.messages.append(mavutil.mavlink.MAVLink_param_value_message(
                name.encode("ascii"), value,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                len(self.connection.parameter_values), 0,
            ))


class FakeConnection:
    def __init__(self):
        prearm = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
        self.target_system = 1
        self.target_component = 1
        self.messages = deque([
            heartbeat(0, armed=False),
            mavutil.mavlink.MAVLink_sys_status_message(
                0, prearm, prearm, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
            mavutil.mavlink.MAVLink_extended_sys_state_message(
                0, mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND),
        ])
        self.mav = FakeMav(self)
        self.parameter_requests = []
        self.parameter_list_requests = []
        self.parameter_values = {}
        self.closed = False

    def recv_match(self, *, blocking):
        assert blocking is False
        return self.messages.popleft() if self.messages else None

    def close(self):
        self.closed = True


class Protocol:
    def __init__(self):
        self.statuses = {}
        self.quiescent = []
        self.closed = False

    def write_status(self, status: RuntimeStatus):
        self.statuses[status_name(type(status))] = status_document(status)

    def write_quiescence(self, module):
        self.quiescent.append(module)

    def read_finalize_request(self):
        return None

    def close(self):
        self.closed = True


class FakeRos:
    clock_values = (
        0, 50_000_000, 100_000_000, 150_000_000, 200_000_000, 250_000_000, 300_000_000)

    def __init__(self, run_state):
        self.run_state = run_state
        self.node = None
        self.loop = 0
        self.initialized = False
        self.publisher_qos = {}

    def init(self):
        self.initialized = True

    def ok(self):
        return True

    def spin_once(self, node, *, timeout_sec):
        assert timeout_sec == 0.02
        if self.loop == 0:
            node.callbacks["/simulation/run_state"](SimpleNamespace(
                run_id=RUN_ID, state=self.run_state.RUNNING))
        if self.loop == len(self.clock_values) - 1:
            node.callbacks["/simulation/run_state"](SimpleNamespace(
                run_id=RUN_ID, state=self.run_state.FINALIZING))
        stamp = self.clock_values[self.loop]
        node.callbacks["/clock"](SimpleNamespace(
            clock=SimpleNamespace(sec=0, nanosec=stamp)))
        self.loop += 1

    def shutdown(self):
        self.initialized = False


class FinalizingRos(FakeRos):
    def __init__(self, run_state, *, finalize_on_loop):
        super().__init__(run_state)
        self.finalize_on_loop = finalize_on_loop

    def spin_once(self, node, *, timeout_sec):
        assert timeout_sec == 0.02
        if self.loop == 0:
            node.callbacks["/simulation/run_state"](SimpleNamespace(
                run_id=RUN_ID, state=self.run_state.RUNNING))
        if self.loop == self.finalize_on_loop:
            node.callbacks["/simulation/run_state"](SimpleNamespace(
                run_id=RUN_ID, state=self.run_state.FINALIZING))
        stamp = min(self.loop, 1) * 100_000_000
        node.callbacks["/clock"](SimpleNamespace(
            clock=SimpleNamespace(sec=0, nanosec=stamp)))
        self.loop += 1


def install_ros(monkeypatch, fake_ros):
    class RunState:
        RUNNING = 2
        FINALIZING = 3

    class Node:
        def __init__(self, *_args, **_kwargs):
            self.callbacks = {}
            self.destroyed = False
            fake_ros.node = self

        def create_subscription(self, _message_type, topic, callback, _qos):
            self.callbacks[topic] = callback
            return callback

        def create_publisher(self, _message_type, _topic, _qos):
            fake_ros.publisher_qos[_topic] = _qos.settings
            return SimpleNamespace(publish=lambda _message: None)

        def destroy_node(self):
            self.callbacks.clear()
            self.destroyed = True

    class QoSProfile:
        def __init__(self, **kwargs):
            self.settings = kwargs

    rclpy = ModuleType("rclpy")
    rclpy.init = fake_ros.init
    rclpy.ok = fake_ros.ok
    rclpy.spin_once = fake_ros.spin_once
    rclpy.shutdown = fake_ros.shutdown
    modules = {
        "rclpy": rclpy,
        "rclpy.node": SimpleNamespace(Node=Node),
        "rclpy.qos": SimpleNamespace(
            DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=1),
            ReliabilityPolicy=SimpleNamespace(RELIABLE=1),
            QoSProfile=QoSProfile,
        ),
        "rosgraph_msgs.msg": SimpleNamespace(Clock=object),
        "sensor_msgs.msg": SimpleNamespace(Image=object, LaserScan=object),
        "simulation_interfaces.msg": SimpleNamespace(
            RunState=RunState,
            MissionEvent=lambda: SimpleNamespace(sim_timestamp=SimpleNamespace()),
        ),
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    fake_ros.run_state = RunState


def test_live_loop_executes_configured_plan_and_releases_resources(monkeypatch):
    protocol = Protocol()
    connection = FakeConnection()
    fake_ros = FakeRos(None)
    install_ros(monkeypatch, fake_ros)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(runtime_node, "connect_mavlink", lambda *_args, **_kwargs: connection)
    previous_handlers = {item: signal.getsignal(item) for item in (signal.SIGTERM, signal.SIGINT)}
    config = SimpleNamespace(
        run_id=RUN_ID,
        mission_plan=parse_mission_plan({
            "schema_version": 1,
            "steps": [
                {"tool": "set_mode", "args": {"mode": "GUIDED"}},
                {"tool": "arm", "args": {}},
                {"tool": "takeoff", "args": {"altitude_m": 2}},
                {"tool": "hold", "args": {"duration_sim_s": 0.05}},
                {"tool": "land", "args": {}},
            ],
        }),
        mavlink_endpoint="tcp:ardupilot-sitl:5760",
        startup_timeout_seconds=1.0,
        max_wall_seconds=10.0,
        finalization_wall_seconds=1.0,
    )

    assert run_configured(config) == 0

    commands = [command for command in connection.mav.command_ids
                if command != mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL]
    assert commands == [
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
    ]
    assert protocol.statuses["mission-finished"]["outcome"] == "LANDED"
    assert protocol.statuses["mission-finished"]["sim_timestamp_ns"] == 250_000_000
    assert protocol.quiescent == ["companion"]
    assert protocol.closed and connection.closed
    assert fake_ros.node.destroyed and fake_ros.node.callbacks == {}
    assert fake_ros.initialized is False and not connection.messages
    assert {item: signal.getsignal(item) for item in previous_handlers} == previous_handlers


def test_live_loop_requests_frozen_calibration_and_baseline_before_first_command(monkeypatch):
    protocol = Protocol()
    connection = FakeConnection()
    expected = {"ATC_RAT_RLL_P": 0.041, "INS_GYRO_FILTER": 20.0}
    connection.parameter_values = expected
    fake_ros = FakeRos(None)
    install_ros(monkeypatch, fake_ros)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(runtime_node, "connect_mavlink", lambda *_args, **_kwargs: connection)
    config = SimpleNamespace(
        run_id=RUN_ID,
        mission_plan=parse_mission_plan({"schema_version": 1, "steps": [
            {"tool": "set_mode", "args": {"mode": "GUIDED"}},
            {"tool": "arm", "args": {}},
            {"tool": "takeoff", "args": {"altitude_m": 2}},
            {"tool": "land", "args": {}},
        ]}),
        calibration_json=json.dumps({
            "gains": {"ATC_RAT_RLL_P": expected["ATC_RAT_RLL_P"]},
            "profile": {"baseline_parameters": {"INS_GYRO_FILTER": expected["INS_GYRO_FILTER"]}},
        }),
        mavlink_endpoint="tcp:ardupilot-sitl:5760",
        startup_timeout_seconds=1.0,
        max_wall_seconds=10.0,
        finalization_wall_seconds=1.0,
    )

    assert run_configured(config) == 0
    assert connection.parameter_list_requests == [(1, 1)]
    assert connection.parameter_requests == []
    assert mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM in connection.mav.command_ids


def test_missing_calibration_readback_reaches_deadline_with_zero_flight_commands(monkeypatch):
    protocol = Protocol()
    connection = FakeConnection()
    fake_ros = FakeRos(None)
    install_ros(monkeypatch, fake_ros)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(runtime_node, "connect_mavlink", lambda *_args, **_kwargs: connection)
    monotonic_values = iter((0.0, 2.0))
    monkeypatch.setattr(configured_runtime.time, "monotonic", lambda: next(monotonic_values))
    config = SimpleNamespace(
        run_id=RUN_ID,
        mission_plan=parse_mission_plan({
            "schema_version": 1,
            "steps": [{"tool": "arm", "args": {}}],
        }),
        calibration_json=json.dumps({
            "gains": {"ATC_RAT_RLL_P": 0.041},
            "profile": {"baseline_parameters": {}},
        }),
        mavlink_endpoint="tcp:ardupilot-sitl:5760",
        startup_timeout_seconds=1.0,
        max_wall_seconds=1.0,
        finalization_wall_seconds=1.0,
    )

    assert run_configured(config) == 1
    assert connection.parameter_list_requests == [(1, 1)]
    assert connection.parameter_requests == []
    flight_commands = {
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
    }
    assert not flight_commands.intersection(connection.mav.command_ids)
    assert protocol.statuses["runtime-failure"]["reason"] == (
        "configured mission wall deadline expired"
    )


def test_global_finalization_does_not_start_recovery_land(monkeypatch):
    protocol = Protocol()
    connection = FakeConnection()
    connection.messages = deque([
        heartbeat(4, armed=True),
        mavutil.mavlink.MAVLink_extended_sys_state_message(
            0, mavutil.mavlink.MAV_LANDED_STATE_IN_AIR),
    ])
    fake_ros = FinalizingRos(None, finalize_on_loop=0)
    install_ros(monkeypatch, fake_ros)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(runtime_node, "connect_mavlink", lambda *_args, **_kwargs: connection)
    config = SimpleNamespace(
        run_id=RUN_ID,
        mission_plan=parse_mission_plan({
            "schema_version": 1,
            "steps": [{"tool": "hold", "args": {"duration_sim_s": 1}}],
        }),
        mavlink_endpoint="tcp:ardupilot-sitl:5760",
        startup_timeout_seconds=1.0,
        max_wall_seconds=10.0,
        finalization_wall_seconds=1.0,
    )

    assert run_configured(config) == 1
    assert mavutil.mavlink.MAV_CMD_NAV_LAND not in connection.mav.command_ids
    assert fake_ros.loop == 1
    assert protocol.statuses["runtime-failure"]["reason"] == (
        "configured mission interrupted before completion"
    )


def test_global_finalization_cancels_pending_recovery_without_waiting(monkeypatch):
    protocol = Protocol()
    connection = FakeConnection()
    prearm = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
    connection.messages = deque([
        heartbeat(4, armed=True),
        mavutil.mavlink.MAVLink_sys_status_message(
            0, prearm, prearm, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        mavutil.mavlink.MAVLink_extended_sys_state_message(
            0, mavutil.mavlink.MAV_LANDED_STATE_IN_AIR),
    ])

    def command_long_send(*arguments):
        command = arguments[2]
        connection.mav.command_ids.append(command)
        if command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            connection.messages.append(mavutil.mavlink.MAVLink_command_ack_message(
                command, mavutil.mavlink.MAV_RESULT_DENIED,
            ))

    connection.mav.command_long_send = command_long_send
    fake_ros = FinalizingRos(None, finalize_on_loop=2)
    install_ros(monkeypatch, fake_ros)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(runtime_node, "connect_mavlink", lambda *_args, **_kwargs: connection)
    monotonic_values = iter(range(1_000))
    monkeypatch.setattr(configured_runtime.time, "monotonic", lambda: next(monotonic_values))
    config = SimpleNamespace(
        run_id=RUN_ID,
        mission_plan=parse_mission_plan({
            "schema_version": 1,
            "steps": [{"tool": "takeoff", "args": {"altitude_m": 2}}],
        }),
        mavlink_endpoint="tcp:ardupilot-sitl:5760",
        startup_timeout_seconds=1.0,
        max_wall_seconds=500.0,
        finalization_wall_seconds=120.0,
    )

    assert run_configured(config) == 1
    assert connection.mav.command_ids.count(mavutil.mavlink.MAV_CMD_NAV_LAND) == 1
    assert fake_ros.loop == 3
    assert protocol.statuses["runtime-failure"]["reason"] == (
        "command rejected: TAKEOFF, result 2"
    )


def test_moving_camera_starts_before_flight_and_closes_during_teardown(monkeypatch):
    protocol = Protocol()
    connection = FakeConnection()
    fake_ros = FakeRos(None)
    install_ros(monkeypatch, fake_ros)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(runtime_node, "connect_mavlink", lambda *_args, **_kwargs: connection)
    trace = []

    class CameraManager:
        def __init__(self, **_kwargs): pass

    class Camera:
        def __init__(self, *_args, **_kwargs): pass

    class Vision:
        def __init__(self, *_args, **_kwargs): pass
        def start(self): trace.append("vision_started")
        def latest(self): return None
        def close(self, _timeout):
            trace.append("vision_closed")
            return True

    monkeypatch.syspath_prepend(str(Path(__file__).parents[1] / "comp2026/src"))
    import drone.sensors.camera._camera_manager as manager_module
    import drone.sensors.camera.camera as camera_module
    import drone_sim_companion.moving_vision as vision_module
    monkeypatch.setattr(manager_module, "CameraManager", CameraManager)
    monkeypatch.setattr(camera_module, "Camera", Camera)
    monkeypatch.setattr(vision_module, "MovingVision", Vision)

    config = SimpleNamespace(
        run_id=RUN_ID,
        mission_plan=parse_mission_plan({
            "schema_version": 1,
            "steps": [{
                "tool": "precision_land",
                "args": {"marker_id": 7, "settle_by_sim_s": 45, "acquire_by_sim_s": 60},
                "timeout_sim_s": 45,
            }],
        }),
        mavlink_endpoint="tcp:ardupilot-sitl:5760",
        startup_timeout_seconds=1.0,
        max_wall_seconds=10.0,
        finalization_wall_seconds=1.0,
    )

    assert run_configured(config) == 1
    assert trace == ["vision_started", "vision_closed"]
    flight_commands = {
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
    }
    assert not flight_commands.intersection(connection.mav.command_ids)
    assert fake_ros.publisher_qos["/simulation/mission_events"]["durability"] == 1
