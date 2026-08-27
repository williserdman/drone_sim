from drone.control import drone_control


class _Vehicle:
    def on_message(self, _message_names):
        return lambda callback: callback


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


def test_drone_control_can_defer_readiness_to_its_host_gate(monkeypatch):
    captured = {}

    def connect(endpoint, **options):
        captured["endpoint"] = endpoint
        captured["options"] = options
        return _Vehicle()

    monkeypatch.setattr(drone_control, "connect", connect)

    drone_control.DroneControl("tcp:ardupilot-sitl:5760", wait_ready=False)

    assert captured["options"]["wait_ready"] is False
