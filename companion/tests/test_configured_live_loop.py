from __future__ import annotations

from collections import deque
from types import ModuleType, SimpleNamespace
import signal
import sys

from pymavlink import mavutil

from artifacts.runtime_status import RuntimeStatus, status_document, status_name
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
        "simulation_interfaces.msg": SimpleNamespace(RunState=RunState),
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
