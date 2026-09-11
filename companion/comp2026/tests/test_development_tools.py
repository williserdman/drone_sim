import builtins
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PROJECT_ROOT / "src"
GC_MAIN_PATH = SOURCE_ROOT / "gc" / "main.py"


def load_gc_main():
    spec = importlib.util.spec_from_file_location("development_gc_main", GC_MAIN_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    "module_name",
    [
        "drone.entry",
        "drone.aruco_land_only",
        "drone.waypoint_only",
        str(GC_MAIN_PATH),
    ],
)
def test_development_tool_import_does_not_load_hardware_or_connection_modules(
    module_name, tmp_path
):
    """Regression: importing a demo must not load drivers or create flight state."""
    script = """
import importlib
import importlib.abc
import importlib.util
import sys

FORBIDDEN = (
    "drone.control",
    "drone.missions",
    "drone.sensors",
    "drone.utils",
    "dronekit",
    "gpiozero",
    "pymavlink",
)

class RejectForbiddenImports(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname in FORBIDDEN or fullname.startswith(tuple(name + "." for name in FORBIDDEN)):
            raise AssertionError(f"forbidden dependency imported: {fullname}")
        return None

sys.meta_path.insert(0, RejectForbiddenImports())
if sys.argv[1].endswith("/gc/main.py"):
    spec = importlib.util.spec_from_file_location("development_gc_main", sys.argv[1])
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
else:
    importlib.import_module(sys.argv[1])
print("IMPORT_OK")
"""
    env = os.environ.copy()
    env["PYTHONPATH"] = str(SOURCE_ROOT)
    env["PYTHONDONTWRITEBYTECODE"] = "1"

    result = subprocess.run(
        [sys.executable, "-c", script, module_name],
        cwd=tmp_path,
        env=env,
        capture_output=True,
        text=True,
        timeout=10,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout == "IMPORT_OK\n"
    assert list(tmp_path.iterdir()) == []


class RecordingMav:
    def __init__(self):
        self.calls = []

    def named_value_int_send(self, *args):
        self.calls.append(("named_value_int_send", args))

    def mission_item_int_send(self, *args):
        self.calls.append(("mission_item_int_send", args))


class FakeTransport:
    def __init__(self):
        self.mav = RecordingMav()


class FakeHeartbeat:
    type = 2
    autopilot = 3
    base_mode = 4
    system_status = 5

    def __init__(self, system_id, component_id):
        self.system_id = system_id
        self.component_id = component_id

    def get_srcSystem(self):
        return self.system_id

    def get_srcComponent(self):
        return self.component_id


class FakeLiveTransport(FakeTransport):
    def __init__(self, initial_heartbeat, later_heartbeat):
        super().__init__()
        self.initial_heartbeat = initial_heartbeat
        self.later_heartbeat = later_heartbeat
        self.target_system = 42
        self.target_component = 0
        self.closed = False

    def wait_heartbeat(self, timeout):
        assert timeout == 5
        return self.initial_heartbeat

    def recv_match(self, *, type, blocking, timeout):
        assert (type, blocking, timeout) == ("HEARTBEAT", True, 5)
        return self.later_heartbeat

    def close(self):
        self.closed = True


def run_live_cli(monkeypatch, transport, *, system_id=42, component_id=191):
    def connect(port, *, baud):
        assert (port, baud) == ("test-endpoint", 57600)
        return transport

    fake_pymavlink = SimpleNamespace(
        mavutil=SimpleNamespace(mavlink_connection=connect)
    )
    monkeypatch.setitem(sys.modules, "pymavlink", fake_pymavlink)
    load_gc_main().main(
        [
            "--port",
            "test-endpoint",
            "--target-system",
            str(system_id),
            "--target-component",
            str(component_id),
            "--synthetic",
            "--servo",
            "4",
        ]
    )


def test_synthetic_servo_selection_emits_only_named_value_message():
    """Regression: the synthetic demo must not encode servo data as navigation."""
    run_servo = load_gc_main().run_servo

    transport = FakeTransport()

    run_servo(transport, servo_num=4)

    assert transport.mav.calls == [
        ("named_value_int_send", (2000, b"servo_num", 4))
    ]


def test_navigation_servo_operation_is_disabled_before_transport_access():
    """Regression: a servo test must never send MAV_CMD_NAV_WAYPOINT."""
    mavcmd_run_servo = load_gc_main().mavcmd_run_servo

    transport = FakeTransport()

    with pytest.raises(RuntimeError, match="actuator receiver contract"):
        mavcmd_run_servo(transport, servo_num=4, duty=1500)

    assert transport.mav.calls == []


def test_live_servo_cli_requires_endpoint():
    """Regression: invoking a live demo cannot fall back to a guessed device."""
    main = load_gc_main().main

    with pytest.raises(SystemExit) as exc_info:
        main([])

    assert exc_info.value.code == 2


def test_live_servo_cli_requires_expected_identity_before_loading_mavlink(
    monkeypatch,
):
    """Regression: an endpoint alone is not authority to contact a receiver."""
    main = load_gc_main().main
    real_import = builtins.__import__

    def reject_mavlink(name, *args, **kwargs):
        if name == "pymavlink":
            raise AssertionError("pymavlink loaded before identity validation")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", reject_mavlink)

    with pytest.raises(SystemExit) as exc_info:
        main(["--port", "test-endpoint", "--synthetic", "--servo", "4"])

    assert exc_info.value.code == 2


def test_live_servo_cli_accepts_matching_heartbeat_source_and_closes(monkeypatch):
    """Regression: identity comes from heartbeat headers, not cached targets."""
    heartbeat = FakeHeartbeat(42, 191)
    transport = FakeLiveTransport(heartbeat, heartbeat)

    run_live_cli(monkeypatch, transport)

    assert transport.mav.calls == [
        ("named_value_int_send", (2000, b"servo_num", 4))
    ]
    assert transport.closed is True


def test_live_servo_cli_refuses_mismatching_initial_heartbeat_and_closes(
    monkeypatch,
):
    """Regression: cached target identity cannot mask the packet sender."""
    transport = FakeLiveTransport(
        FakeHeartbeat(43, 191),
        FakeHeartbeat(42, 191),
    )
    transport.target_component = 191

    with pytest.raises(RuntimeError, match="identity mismatch"):
        run_live_cli(monkeypatch, transport)

    assert transport.mav.calls == []
    assert transport.closed is True


def test_live_servo_cli_refuses_initial_heartbeat_timeout_and_closes(monkeypatch):
    """Regression: no initial heartbeat never authorizes an output."""
    transport = FakeLiveTransport(None, FakeHeartbeat(42, 191))
    transport.target_component = 191

    with pytest.raises(RuntimeError, match="heartbeat timeout"):
        run_live_cli(monkeypatch, transport)

    assert transport.mav.calls == []
    assert transport.closed is True


def test_live_servo_cli_refuses_later_heartbeat_from_other_source_and_closes(
    monkeypatch,
):
    """Regression: the identity gate applies to every checked heartbeat."""
    transport = FakeLiveTransport(
        FakeHeartbeat(42, 191),
        FakeHeartbeat(42, 192),
    )
    transport.target_component = 191

    with pytest.raises(RuntimeError, match="identity mismatch"):
        run_live_cli(monkeypatch, transport)

    assert transport.mav.calls == []
    assert transport.closed is True
