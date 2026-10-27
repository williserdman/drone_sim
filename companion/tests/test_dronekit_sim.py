import collections
import collections.abc
import importlib
import threading
from types import SimpleNamespace

import pytest
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink


def _simulation_vehicle_type():
    if not hasattr(collections, "MutableMapping"):
        collections.MutableMapping = collections.abc.MutableMapping
    try:
        module = importlib.import_module("drone_sim_companion.dronekit_sim")
    except ModuleNotFoundError:
        pytest.fail("simulation DroneKit vehicle is missing")
    return module.SimulationVehicle


class Mav:
    def __init__(self):
        self.reads = []
        self.lists = []
        self.commands = []

    def param_request_read_send(self, *arguments):
        self.reads.append(arguments)

    def param_request_list_send(self, *arguments):
        self.lists.append(arguments)

    def command_long_send(self, *arguments):
        self.commands.append(arguments)


class Handler:
    def __init__(self, master=None):
        self.master = master or SimpleNamespace(
            mav=Mav(), target_system=1, target_component=1
        )
        self.target_system = 0
        self.loop_listeners = []
        self.message_listeners = []

    def forward_loop(self, callback):
        self.loop_listeners.append(callback)
        return callback

    def forward_message(self, callback):
        self.message_listeners.append(callback)
        return callback


def parameter(*, count, index, name, value):
    return SimpleNamespace(
        param_count=count,
        param_index=index,
        param_id=name,
        param_value=value,
        get_srcSystem=lambda: 1,
        get_srcComponent=lambda: 1,
    )


def sourced_message(message, *, system_id, component_id):
    encoder = mavlink.MAVLink(
        None, srcSystem=system_id, srcComponent=component_id
    )
    message.pack(encoder)
    return message


def test_simulation_vehicle_keeps_real_parameter_cache_without_automatic_reads():
    vehicle = _simulation_vehicle_type()(Handler())
    handler = vehicle._handler
    watchdog = handler.loop_listeners[0]
    sentinels = (
        (1357, "UNSOLICITED_A", 1.0),
        (1370, "UNSOLICITED_B", 2.0),
        (1357, "UNSOLICITED_C", 3.0),
        (1370, "UNSOLICITED_D", 4.0),
    )

    for count, name, value in sentinels:
        vehicle.notify_message_listeners(
            "PARAM_VALUE",
            parameter(count=count, index=65_535, name=name, value=value),
        )
        vehicle._params_last = -1e9
        watchdog(handler)

    assert handler.master.mav.reads == []
    assert vehicle._params_loaded is False
    assert vehicle._params_map == {
        name: value for _count, name, value in sentinels
    }

    vehicle.notify_message_listeners(
        "PARAM_VALUE", parameter(count=2, index=0, name="REAL_A", value=10.0)
    )
    watchdog(handler)
    assert vehicle._params_loaded is False
    vehicle.notify_message_listeners(
        "PARAM_VALUE", parameter(count=2, index=1, name="REAL_B", value=20.0)
    )
    assert vehicle._params_loaded is False
    watchdog(handler)

    assert vehicle._params_loaded is True
    assert vehicle.parameters["REAL_A"] == 10.0
    assert vehicle.parameters["REAL_B"] == 20.0
    assert handler.master.mav.reads == []

    handler.master.mav.param_request_list_send(1, 1)
    handler.master.mav.param_request_read_send(1, 1, b"REAL_A", -1)
    assert handler.master.mav.lists == [(1, 1)]
    assert handler.master.mav.reads == [(1, 1, b"REAL_A", -1)]


def test_simulation_vehicle_rejects_all_foreign_telemetry_before_dronekit_callbacks():
    vehicle = _simulation_vehicle_type()(Handler())
    observed = []
    vehicle.add_message_listener("*", lambda _vehicle, name, _message: observed.append(name))

    gcs_heartbeat = sourced_message(
        mavlink.MAVLink_heartbeat_message(
            mavlink.MAV_TYPE_GCS,
            mavlink.MAV_AUTOPILOT_INVALID,
            mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
            4,
            mavlink.MAV_STATE_ACTIVE,
            3,
        ),
        system_id=255,
        component_id=190,
    )
    foreign_ack = sourced_message(
        mavlink.MAVLink_command_ack_message(
            mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            mavlink.MAV_RESULT_ACCEPTED,
        ),
        system_id=42,
        component_id=1,
    )
    vehicle_heartbeat = sourced_message(
        mavlink.MAVLink_heartbeat_message(
            mavlink.MAV_TYPE_QUADROTOR,
            mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            0,
            0,
            mavlink.MAV_STATE_ACTIVE,
            3,
        ),
        system_id=1,
        component_id=1,
    )

    vehicle.notify_message_listeners("HEARTBEAT", gcs_heartbeat)
    vehicle.notify_message_listeners("COMMAND_ACK", foreign_ack)
    vehicle.notify_message_listeners("HEARTBEAT", vehicle_heartbeat)

    assert observed == ["HEARTBEAT"]
    assert vehicle.armed is False


def test_qgc_dispatch_precedes_fc_filter_and_fresh_facts_are_not_refreshed():
    vehicle = _simulation_vehicle_type()(Handler())
    qgc_messages = []
    vehicle.install_qgc_message_callback(qgc_messages.append)
    qgc_command = sourced_message(
        mavlink.MAVLink_command_long_message(
            1, 191, 31020, 0, 123, 0, 0, 0, 0, 0, 0
        ),
        system_id=255,
        component_id=190,
    )
    heartbeat = sourced_message(
        mavlink.MAVLink_heartbeat_message(2, 3, 0, 4, 3, 3),
        system_id=1,
        component_id=1,
    )
    landed = sourced_message(
        mavlink.MAVLink_extended_sys_state_message(0, 1),
        system_id=1,
        component_id=1,
    )

    vehicle.notify_message_listeners("COMMAND_LONG", qgc_command)
    vehicle.notify_message_listeners("HEARTBEAT", heartbeat)
    vehicle.notify_message_listeners("EXTENDED_SYS_STATE", landed)

    assert qgc_messages == [qgc_command, heartbeat, landed]
    observations = vehicle.drain_operator_observations()
    assert len(observations) == 2
    assert observations[0].heartbeat_live is True
    assert observations[0].armed is False
    assert observations[1].landed is True
    assert vehicle.drain_operator_observations() == ()


def test_operator_fact_drain_is_atomic_with_reader_callback() -> None:
    vehicle = _simulation_vehicle_type()(Handler())
    first = sourced_message(
        mavlink.MAVLink_heartbeat_message(2, 3, 0, 0, 3, 3),
        system_id=1,
        component_id=1,
    )
    second = sourced_message(
        mavlink.MAVLink_extended_sys_state_message(0, 1),
        system_id=1,
        component_id=1,
    )
    vehicle.notify_message_listeners("HEARTBEAT", first)

    iteration_started = threading.Event()
    allow_iteration = threading.Event()

    class PausingDeque(collections.deque):
        def __iter__(self):
            iteration_started.set()
            assert allow_iteration.wait(1.0)
            return super().__iter__()

    vehicle._operator_observations = PausingDeque(
        vehicle._operator_observations,
        maxlen=64,
    )
    drained = []
    drain_finished = threading.Event()
    append_finished = threading.Event()
    drain = threading.Thread(
        target=lambda: (drained.extend(vehicle.drain_operator_observations()), drain_finished.set())
    )
    append = threading.Thread(
        target=lambda: (
            vehicle.notify_message_listeners("EXTENDED_SYS_STATE", second),
            append_finished.set(),
        )
    )
    drain.start()
    assert iteration_started.wait(1.0)
    append.start()
    assert not append_finished.wait(0.02)
    allow_iteration.set()
    drain.join(timeout=1.0)
    append.join(timeout=1.0)

    assert len(drained) == 1
    remaining = vehicle.drain_operator_observations()
    assert len(remaining) == 1
    assert remaining[0].landed is True


def test_simulation_vehicle_requests_frequent_vehicle_heartbeat_after_connect(monkeypatch):
    vehicle_type = _simulation_vehicle_type()
    vehicle = vehicle_type(Handler())
    initialized = []
    monkeypatch.setattr(
        vehicle_type.__mro__[1],
        "initialize",
        lambda _self, *, rate, heartbeat_timeout: initialized.append(
            (rate, heartbeat_timeout)
        ),
    )

    vehicle.initialize(rate=4, heartbeat_timeout=30)

    assert initialized == [(4, 30)]
    assert vehicle._master.mav.commands == [
        (
            1,
            1,
            mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavlink.MAVLINK_MSG_ID_HEARTBEAT,
            100_000,
            0,
            0,
            0,
            0,
            0,
        )
    ]


def test_simulation_vehicle_pins_real_pymavlink_discovery_before_dronekit_initialize(
    monkeypatch,
):
    vehicle_type = _simulation_vehicle_type()
    master = mavutil.mavfile(
        None,
        "test",
        source_system=191,
        source_component=191,
    )
    handler = Handler(master)
    vehicle = vehicle_type(handler)
    foreign_heartbeat = sourced_message(
        mavlink.MAVLink_heartbeat_message(
            mavlink.MAV_TYPE_QUADROTOR,
            mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
            4,
            mavlink.MAV_STATE_ACTIVE,
            3,
        ),
        system_id=42,
        component_id=7,
    )
    encoded_targets = []

    def initialize_with_foreign_discovery(_vehicle, *, rate, heartbeat_timeout):
        assert (rate, heartbeat_timeout) == (4, 30)
        assert (master.target_system, master.target_component) == (1, 1)
        master.post_message(foreign_heartbeat)
        vehicle.notify_message_listeners("HEARTBEAT", foreign_heartbeat)
        command = master.mav.command_long_encode(
            master.target_system,
            master.target_component,
            mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        encoded_targets.append((command.target_system, command.target_component))

    monkeypatch.setattr(vehicle_type.__mro__[1], "initialize", initialize_with_foreign_discovery)
    module = importlib.import_module("drone_sim_companion.dronekit_sim")
    monkeypatch.setattr(module, "request_flight_controller_heartbeat", lambda *_args, **_kwargs: None)

    vehicle.initialize(rate=4, heartbeat_timeout=30)

    assert master.target_system == 1
    assert master.target_component == 1
    assert handler.target_system == 1
    assert encoded_targets == [(1, 1)]
    assert vehicle.armed is False


def test_early_fc_observer_precedes_cache_driven_upstream_transition():
    from drone_sim_companion.flight_authority import ExpectedTransition, FlightAuthorityGate
    from drone_sim_companion.mission import Telemetry

    vehicle = _simulation_vehicle_type()(Handler())
    gate = FlightAuthorityGate()
    gate.observe(Telemetry(1, heartbeat=True, mode='GUIDED', armed=True))
    gate.admit('a' * 64)
    with gate.output_transaction(ExpectedTransition(mode='STABILIZE')):
        pass
    observed = []
    def early(_vehicle, name, message):
        if name == 'HEARTBEAT':
            observed.append(message.custom_mode)
            gate.observe(Telemetry(len(observed) + 1, heartbeat=True,
                                   mode={0: 'STABILIZE', 4: 'GUIDED'}[message.custom_mode], armed=True))
    vehicle.install_fc_message_observer(early)
    def upstream_sees_mode(_vehicle, _name, value):
        if value.name == 'STABILIZE':
            # The real upstream force_arm_takeoff does this immediately when
            # its cached armed flag is already true.
            with gate.output_transaction(ExpectedTransition(mode='GUIDED')):
                pass
    vehicle.add_attribute_listener('mode', upstream_sees_mode)
    for custom_mode in (0, 4):
        message = sourced_message(mavlink.MAVLink_heartbeat_message(2, 3, 128, custom_mode, 4, 3),
                                  system_id=1, component_id=1)
        vehicle.notify_message_listeners('HEARTBEAT', message)
    assert observed == [0, 4]
    assert not gate.revoked
    foreign = sourced_message(mavlink.MAVLink_heartbeat_message(6, 8, 0, 0, 4, 3),
                              system_id=255, component_id=190)
    vehicle.notify_message_listeners('HEARTBEAT', foreign)
    assert observed == [0, 4]
    vehicle.remove_fc_message_observer(early)
