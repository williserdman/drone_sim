import collections
import collections.abc
import importlib
from types import SimpleNamespace

import pytest


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

    def param_request_read_send(self, *arguments):
        self.reads.append(arguments)

    def param_request_list_send(self, *arguments):
        self.lists.append(arguments)


class Handler:
    def __init__(self):
        self.master = SimpleNamespace(mav=Mav())
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
    )


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
