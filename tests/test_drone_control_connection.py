import math

import pytest
from pymavlink import mavutil

from drone.common_types import RelPosComplete
from drone.control import drone_control


class _Vehicle:
    def __init__(self):
        self.encoded = None
        self.sent = None
        self.flushed = False
        self.message_factory = self

    def on_message(self, _message_names):
        return lambda callback: callback

    def landing_target_encode(self, *fields):
        self.encoded = fields
        return fields

    def send_mavlink(self, message):
        self.sent = message

    def flush(self):
        self.flushed = True


def test_drone_control_preserves_ready_wait_by_default(monkeypatch):
    captured = {}

    def connect(endpoint, **options):
        captured["endpoint"] = endpoint
        captured["options"] = options
        return _Vehicle()

    monkeypatch.setattr(drone_control, "connect", connect)

    drone_control.DroneControl("tcp:ardupilot-sitl:5760")

    assert captured == {
        "endpoint": "tcp:ardupilot-sitl:5760",
        "options": {
            "wait_ready": True,
            "heartbeat_timeout": 60,
            "timeout": 120,
            "source_system": 1,
            "source_component": 191,
        },
    }


def test_drone_control_can_defer_readiness_to_its_host_startup_budget(monkeypatch):
    captured = {}

    def connect(endpoint, **options):
        captured["endpoint"] = endpoint
        captured["options"] = options
        return _Vehicle()

    monkeypatch.setattr(drone_control, "connect", connect)

    drone_control.DroneControl(
        "tcp:ardupilot-sitl:5760",
        wait_ready=False,
        heartbeat_timeout=120,
    )

    assert captured["options"]["wait_ready"] is False
    assert captured["options"]["heartbeat_timeout"] == 120


def test_precision_landing_packet_uses_frd_and_reconstructs_literal_direction(
    monkeypatch,
):
    vehicle = _Vehicle()
    monkeypatch.setattr(drone_control, "connect", lambda *_args, **_options: vehicle)
    controller = drone_control.DroneControl("tcp:ardupilot-sitl:5760")
    direction = RelPosComplete(0.6, -0.4, 4.0)

    assert controller.land_send_landing_target(direction) == 0

    assert vehicle.encoded is not None
    _, _, frame, angle_x, angle_y, distance, size_x, size_y = vehicle.encoded
    assert frame == mavutil.mavlink.MAV_FRAME_BODY_FRD
    reconstructed = (
        -math.tan(angle_y) * direction.z,
        math.tan(angle_x) * direction.z,
        direction.z,
    )
    assert reconstructed == pytest.approx(
        (direction.x, direction.y, direction.z), abs=1e-12
    )
    assert distance == pytest.approx(math.sqrt(0.6**2 + (-0.4) ** 2 + 4.0**2))
    assert (size_x, size_y) == (0.0, 0.0)
    assert vehicle.sent == vehicle.encoded
    assert vehicle.flushed is True
