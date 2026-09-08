from __future__ import annotations

import builtins
import importlib
import sys
from types import SimpleNamespace

import pytest


OPTIONAL_ROOTS = {"cv2", "dronekit", "dronekit_sitl", "flask", "pymavlink"}


def test_development_tool_imports_do_not_load_optional_packages(monkeypatch):
    real_import = builtins.__import__

    def reject_optional(name, *args, **kwargs):
        if name.split(".", 1)[0] in OPTIONAL_ROOTS:
            raise AssertionError(f"optional package imported: {name}")
        return real_import(name, *args, **kwargs)

    for name in (
        "drone.control.sender",
        "drone.sitl.simulation",
        "drone.missions",
        "drone.missions.fm1",
        "drone.missions.fm2",
        "drone.missions.fm3",
        "lidar_test",
        "backup.video_streamer",
    ):
        sys.modules.pop(name, None)
    monkeypatch.setattr(builtins, "__import__", reject_optional)

    for name in (
        "drone.control.sender",
        "drone.sitl.simulation",
        "drone.missions.fm3",
        "lidar_test",
        "backup.video_streamer",
    ):
        importlib.import_module(name)


def test_inactive_fm3_route_and_payload_stubs_fail_without_output_or_controller_access(
    capsys,
):
    fm3 = importlib.import_module("drone.missions.fm3")

    class AccessTrap:
        def __getattr__(self, name):
            raise AssertionError(f"inactive route accessed {name}")

    with pytest.raises(fm3.InactiveMissionError, match="inactive"):
        fm3.fm3(AccessTrap(), AccessTrap(), None, None, None, None, 30)
    with pytest.raises(fm3.InactiveMissionError, match="inactive"):
        fm3.pickup()
    with pytest.raises(fm3.InactiveMissionError, match="inactive"):
        fm3.drop()

    captured = capsys.readouterr()
    assert captured.out == ""
    assert captured.err == ""


@pytest.mark.parametrize("mission_name", ["fm1", "fm2", "fm3"])
def test_missions_package_loads_public_callable_only_when_requested(mission_name):
    for name in (
        "drone.missions",
        "drone.missions.fm1",
        "drone.missions.fm2",
        "drone.missions.fm3",
    ):
        sys.modules.pop(name, None)

    missions = importlib.import_module("drone.missions")

    assert "drone.missions.fm1" not in sys.modules
    assert "drone.missions.fm2" not in sys.modules
    assert "drone.missions.fm3" not in sys.modules
    assert callable(getattr(missions, mission_name))
    assert f"drone.missions.{mission_name}" in sys.modules


@pytest.mark.parametrize("mission_name", ["fm1", "fm2", "fm3"])
def test_missions_package_preserves_callable_after_direct_submodule_import(
    mission_name,
):
    for name in (
        "drone.missions",
        "drone.missions.fm1",
        "drone.missions.fm2",
        "drone.missions.fm3",
    ):
        sys.modules.pop(name, None)

    missions = importlib.import_module("drone.missions")
    submodule = importlib.import_module(f"drone.missions.{mission_name}")

    exported = getattr(missions, mission_name)
    assert callable(exported)
    assert exported is getattr(submodule, mission_name)


class FakeHeartbeat:
    def __init__(self, system: int, component: int):
        self._system = system
        self._component = component

    def get_srcSystem(self):
        return self._system

    def get_srcComponent(self):
        return self._component


class FakeTransport:
    def __init__(self, heartbeat):
        self.heartbeat = heartbeat
        self.closed = False
        self.sent = []
        self.mav = SimpleNamespace(statustext_send=self._send)

    def wait_heartbeat(self, timeout):
        self.timeout = timeout
        return self.heartbeat

    def _send(self, severity, text):
        self.sent.append((severity, text))

    def close(self):
        self.closed = True


def test_sender_validates_packet_identity_labels_traffic_and_closes():
    from drone.control import sender

    transport = FakeTransport(FakeHeartbeat(42, 7))
    connection_calls = []
    fake_mavutil = SimpleNamespace(
        mavlink=SimpleNamespace(MAV_SEVERITY_INFO=6),
        mavlink_connection=lambda endpoint, **kwargs: (
            connection_calls.append((endpoint, kwargs)) or transport
        ),
    )

    sender.start_sender(
        endpoint="udpout:127.0.0.1:14560",
        source_system=201,
        source_component=190,
        expected_receiver_system=42,
        expected_receiver_component=7,
        heartbeat_timeout_seconds=2.5,
        messages=("bench pulse",),
        interval_seconds=0,
        iterations=1,
        mavutil_module=fake_mavutil,
        sleep=lambda _seconds: None,
    )

    assert connection_calls == [
        (
            "udpout:127.0.0.1:14560",
            {"source_system": 201, "source_component": 190},
        )
    ]
    assert transport.timeout == 2.5
    assert transport.sent == [(6, b"[SYNTHETIC TEST] bench pulse")]
    assert transport.closed


def test_sender_rejects_unexpected_packet_identity_and_closes():
    from drone.control import sender

    transport = FakeTransport(FakeHeartbeat(9, 8))
    fake_mavutil = SimpleNamespace(
        mavlink=SimpleNamespace(MAV_SEVERITY_INFO=6),
        mavlink_connection=lambda *_args, **_kwargs: transport,
    )

    with pytest.raises(sender.ReceiverIdentityError, match="9:8"):
        sender.start_sender(
            endpoint="udpout:127.0.0.1:14560",
            source_system=201,
            source_component=190,
            expected_receiver_system=42,
            expected_receiver_component=7,
            heartbeat_timeout_seconds=1,
            messages=("bench pulse",),
            interval_seconds=0,
            iterations=1,
            mavutil_module=fake_mavutil,
        )

    assert transport.sent == []
    assert transport.closed


def test_sitl_requires_explicit_executable_and_never_downloads():
    from drone.sitl import simulation

    class FakeSitl:
        def __init__(self, executable):
            self.executable = executable
            self.launched = None

        def launch(self, args, **kwargs):
            self.launched = (args, kwargs)

        def connection_string(self):
            return "tcp:127.0.0.1:5760"

        def download(self, *_args):
            raise AssertionError("download must not be used")

        def stop(self):
            pass

    fake_module = SimpleNamespace(SITL=FakeSitl)
    session = simulation.start_sitl(
        executable_path="/opt/verified/arducopter", sitl_module=fake_module
    )

    assert session.connection_string == "tcp:127.0.0.1:5760"
    assert session.process.executable == "/opt/verified/arducopter"
    assert session.process.launched[1] == {"await_ready": True, "restart": True}


def test_prearm_relaxation_is_disabled_before_any_vehicle_access():
    from drone.sitl import simulation

    class VehicleAccessTrap:
        @property
        def parameters(self):
            raise AssertionError("disabled helper accessed vehicle parameters")

    class FakeSitl:
        def __init__(self, _executable):
            self.stopped = False

        def launch(self, _args, **_kwargs):
            pass

        def connection_string(self):
            return "tcp:127.0.0.1:5760"

        def stop(self):
            self.stopped = True

    minted = simulation.start_sitl(
        executable_path="/opt/verified/arducopter",
        sitl_module=SimpleNamespace(SITL=FakeSitl),
    )
    stopped = simulation.start_sitl(
        executable_path="/opt/verified/arducopter",
        sitl_module=SimpleNamespace(SITL=FakeSitl),
    )
    stopped.stop()

    for session in (SimpleNamespace(), minted, stopped):
        with pytest.raises(
            simulation.PrearmRelaxationDisabledError, match="disabled"
        ):
            simulation.relax_prearm_checks(
                VehicleAccessTrap(),
                session=session,
                sleep=lambda _seconds: pytest.fail("disabled helper slept"),
            )


def test_prearm_relaxation_produces_no_output(capsys):
    from drone.sitl import simulation

    with pytest.raises(simulation.PrearmRelaxationDisabledError):
        simulation.relax_prearm_checks(
            SimpleNamespace(), session=SimpleNamespace(), sleep=lambda _seconds: None
        )

    captured = capsys.readouterr()
    assert captured.out == ""
    assert captured.err == ""


@pytest.mark.parametrize("failed_operation", ["goto_waypoint", "simple_land"])
def test_scenario_never_reports_success_when_checked_operation_fails(
    failed_operation, capsys
):
    from drone.sitl import simulation

    class Controller:
        vehicle = object()

        def goto_waypoint(self, _waypoint):
            return -1 if failed_operation == "goto_waypoint" else 0

        def simple_land(self):
            return -1 if failed_operation == "simple_land" else 0

    with pytest.raises(simulation.ScenarioFailure, match=failed_operation):
        simulation.run_scenario(
            Controller(),
            waypoint=object(),
            target_altitude_m=30,
            takeoff=lambda _vehicle, _altitude: None,
            hold_seconds=0,
            sleep=lambda _seconds: None,
        )

    assert "SUCCESS" not in capsys.readouterr().out


def test_scenario_propagates_takeoff_failure_without_claiming_success(capsys):
    from drone.sitl import simulation

    controller = SimpleNamespace(vehicle=object())

    def failed_takeoff(_vehicle, _altitude):
        raise RuntimeError("climb rejected")

    with pytest.raises(RuntimeError, match="climb rejected"):
        simulation.run_scenario(
            controller,
            waypoint=object(),
            target_altitude_m=30,
            takeoff=failed_takeoff,
            hold_seconds=0,
            sleep=lambda _seconds: None,
        )

    assert "SUCCESS" not in capsys.readouterr().out


def test_lidar_diagnostic_uses_explicit_profile_metres_and_stops(capsys):
    import lidar_test
    from drone.sensors.lidar.lidar import LidarStopStatus

    class Adapter:
        def __init__(self, **kwargs):
            self.configuration = kwargs
            self.values = iter((1.25, 1.75))
            self.stopped = False

        def get_distance(self):
            return next(self.values)

        def stop(self):
            self.stopped = True
            return LidarStopStatus(True, True)

    acquired = []

    def factory(**kwargs):
        adapter = Adapter(**kwargs)
        acquired.append(adapter)
        return adapter

    result = lidar_test.run_diagnostic(
        raw_min_cm=5,
        raw_max_cm=400,
        mounting_offset_cm=12,
        stale_after_seconds=0.5,
        startup_timeout_seconds=2,
        poll_interval_seconds=0.02,
        sample_count=2,
        sample_interval_seconds=0,
        adapter_factory=factory,
        sleep=lambda _seconds: None,
    )

    assert result == 0
    assert acquired[0].configuration == {
        "raw_min_cm": 5,
        "raw_max_cm": 400,
        "mounting_offset_cm": 12,
        "stale_after_seconds": 0.5,
        "startup_timeout_seconds": 2,
        "poll_interval_seconds": 0.02,
    }
    assert acquired[0].stopped
    assert capsys.readouterr().out.splitlines() == [
        "Distance: 1.250 m",
        "Distance: 1.750 m",
        "Average: 1.500 m",
    ]


def test_lidar_diagnostic_reports_read_failure_and_stops(capsys):
    import lidar_test
    from drone.sensors.lidar.lidar import LidarStopStatus

    adapter = SimpleNamespace(
        get_distance=lambda: (_ for _ in ()).throw(RuntimeError("I2C read failed")),
        stop=lambda: (
            setattr(adapter, "stopped", True) or LidarStopStatus(True, True)
        ),
        stopped=False,
    )

    result = lidar_test.run_diagnostic(
        raw_min_cm=5,
        raw_max_cm=400,
        mounting_offset_cm=12,
        stale_after_seconds=0.5,
        startup_timeout_seconds=2,
        poll_interval_seconds=0.02,
        sample_count=1,
        sample_interval_seconds=0,
        adapter_factory=lambda **_kwargs: adapter,
        sleep=lambda _seconds: None,
    )

    assert result == 1
    assert adapter.stopped
    assert "I2C read failed" in capsys.readouterr().err


def test_lidar_diagnostic_reports_unconfirmed_startup_shutdown(capsys):
    import lidar_test
    from drone.sensors.lidar.lidar import LidarStartupError, LidarStopStatus

    startup_error = LidarStartupError(
        "no initial sample",
        LidarStopStatus(worker_stopped=False, cleanup_completed=False),
    )

    result = lidar_test.run_diagnostic(
        raw_min_cm=5,
        raw_max_cm=400,
        mounting_offset_cm=12,
        stale_after_seconds=0.5,
        startup_timeout_seconds=2,
        poll_interval_seconds=0.02,
        sample_count=1,
        sample_interval_seconds=0,
        adapter_factory=lambda **_kwargs: (_ for _ in ()).throw(startup_error),
        sleep=lambda _seconds: None,
    )

    assert result == 1
    error_output = capsys.readouterr().err
    assert "no initial sample" in error_output
    assert "worker" in error_output


def test_lidar_diagnostic_reports_startup_cleanup_error_and_incomplete_status(capsys):
    import lidar_test
    from drone.sensors.lidar.lidar import LidarStartupError, LidarStopStatus

    startup_error = LidarStartupError(
        "no initial sample",
        LidarStopStatus(
            worker_stopped=True,
            cleanup_completed=False,
            cleanup_error="GPIO close failed",
        ),
    )

    result = lidar_test.run_diagnostic(
        raw_min_cm=5,
        raw_max_cm=400,
        mounting_offset_cm=12,
        stale_after_seconds=0.5,
        startup_timeout_seconds=2,
        poll_interval_seconds=0.02,
        sample_count=1,
        sample_interval_seconds=0,
        adapter_factory=lambda **_kwargs: (_ for _ in ()).throw(startup_error),
        sleep=lambda _seconds: None,
    )

    assert result == 1
    error_output = capsys.readouterr().err
    assert "cleanup did not complete" in error_output
    assert "GPIO close failed" in error_output


@pytest.mark.parametrize(
    ("shutdown_status", "expected_errors"),
    [
        pytest.param(
            (False, False, None),
            ("worker",),
            id="worker-still-running",
        ),
        pytest.param(
            (True, False, None),
            ("cleanup",),
            id="cleanup-incomplete",
        ),
        pytest.param(
            (True, False, "GPIO close failed"),
            ("cleanup did not complete", "GPIO close failed"),
            id="cleanup-error",
        ),
    ],
)
def test_lidar_diagnostic_rejects_incomplete_shutdown(
    shutdown_status, expected_errors, capsys
):
    import lidar_test
    from drone.sensors.lidar.lidar import LidarStopStatus

    adapter = SimpleNamespace(
        get_distance=lambda: 1.5,
        stop=lambda: LidarStopStatus(*shutdown_status),
    )

    result = lidar_test.run_diagnostic(
        raw_min_cm=5,
        raw_max_cm=400,
        mounting_offset_cm=12,
        stale_after_seconds=0.5,
        startup_timeout_seconds=2,
        poll_interval_seconds=0.02,
        sample_count=1,
        sample_interval_seconds=0,
        adapter_factory=lambda **_kwargs: adapter,
        sleep=lambda _seconds: None,
    )

    assert result == 1
    error_output = capsys.readouterr().err
    assert all(expected_error in error_output for expected_error in expected_errors)


def test_lidar_diagnostic_reports_read_and_stop_failures(capsys):
    import lidar_test

    adapter = SimpleNamespace(
        get_distance=lambda: (_ for _ in ()).throw(RuntimeError("I2C read failed")),
        stop=lambda: (_ for _ in ()).throw(RuntimeError("stop failed")),
    )

    result = lidar_test.run_diagnostic(
        raw_min_cm=5,
        raw_max_cm=400,
        mounting_offset_cm=12,
        stale_after_seconds=0.5,
        startup_timeout_seconds=2,
        poll_interval_seconds=0.02,
        sample_count=1,
        sample_interval_seconds=0,
        adapter_factory=lambda **_kwargs: adapter,
        sleep=lambda _seconds: None,
    )

    assert result == 1
    error_output = capsys.readouterr().err
    assert "I2C read failed" in error_output
    assert "stop failed" in error_output


def test_lidar_diagnostic_attempts_stop_when_failure_reporting_raises(monkeypatch):
    import lidar_test
    from drone.sensors.lidar.lidar import LidarStopStatus

    class FailingStderr:
        def write(self, _text):
            raise OSError("stderr unavailable")

    adapter = SimpleNamespace(
        get_distance=lambda: (_ for _ in ()).throw(RuntimeError("I2C read failed")),
        stop=lambda: (
            setattr(adapter, "stopped", True) or LidarStopStatus(True, True)
        ),
        stopped=False,
    )

    with monkeypatch.context() as patch:
        patch.setattr(lidar_test.sys, "stderr", FailingStderr())
        with pytest.raises(OSError, match="stderr unavailable"):
            lidar_test.run_diagnostic(
                raw_min_cm=5,
                raw_max_cm=400,
                mounting_offset_cm=12,
                stale_after_seconds=0.5,
                startup_timeout_seconds=2,
                poll_interval_seconds=0.02,
                sample_count=1,
                sample_interval_seconds=0,
                adapter_factory=lambda **_kwargs: adapter,
                sleep=lambda _seconds: None,
            )

    assert adapter.stopped


def test_lidar_diagnostic_preserves_ctrl_c_when_stop_reporting_raises(monkeypatch):
    import lidar_test

    class FailingStderr:
        def write(self, _text):
            raise OSError("stderr unavailable")

    adapter = SimpleNamespace(
        get_distance=lambda: (_ for _ in ()).throw(KeyboardInterrupt()),
        stop=lambda: (
            setattr(adapter, "stopped", True)
            or (_ for _ in ()).throw(RuntimeError("stop failed"))
        ),
        stopped=False,
    )

    with monkeypatch.context() as patch:
        patch.setattr(lidar_test.sys, "stderr", FailingStderr())
        with pytest.raises(KeyboardInterrupt):
            lidar_test.run_diagnostic(
                raw_min_cm=5,
                raw_max_cm=400,
                mounting_offset_cm=12,
                stale_after_seconds=0.5,
                startup_timeout_seconds=2,
                poll_interval_seconds=0.02,
                sample_count=1,
                sample_interval_seconds=0,
                adapter_factory=lambda **_kwargs: adapter,
                sleep=lambda _seconds: None,
            )

    assert adapter.stopped


@pytest.mark.parametrize("malformed_status", [None, object(), (True, True, None)])
def test_lidar_diagnostic_rejects_malformed_shutdown_status(
    malformed_status, capsys
):
    import lidar_test

    adapter = SimpleNamespace(
        get_distance=lambda: 1.5,
        stop=lambda: malformed_status,
    )

    result = lidar_test.run_diagnostic(
        raw_min_cm=5,
        raw_max_cm=400,
        mounting_offset_cm=12,
        stale_after_seconds=0.5,
        startup_timeout_seconds=2,
        poll_interval_seconds=0.02,
        sample_count=1,
        sample_interval_seconds=0,
        adapter_factory=lambda **_kwargs: adapter,
        sleep=lambda _seconds: None,
    )

    assert result == 1
    assert "shutdown status" in capsys.readouterr().err.lower()


def test_lidar_diagnostic_propagates_ctrl_c_after_reporting_stop_failure(capsys):
    import lidar_test

    stopped = False

    def stop():
        nonlocal stopped
        stopped = True
        raise RuntimeError("stop failed")

    adapter = SimpleNamespace(
        get_distance=lambda: (_ for _ in ()).throw(KeyboardInterrupt()),
        stop=stop,
    )

    with pytest.raises(KeyboardInterrupt):
        lidar_test.run_diagnostic(
            raw_min_cm=5,
            raw_max_cm=400,
            mounting_offset_cm=12,
            stale_after_seconds=0.5,
            startup_timeout_seconds=2,
            poll_interval_seconds=0.02,
            sample_count=1,
            sample_interval_seconds=0,
            adapter_factory=lambda **_kwargs: adapter,
            sleep=lambda _seconds: None,
        )

    assert stopped
    assert "stop failed" in capsys.readouterr().err


@pytest.mark.parametrize(
    ("sample_count", "sample_interval_seconds"),
    [
        pytest.param(0, 0, id="zero-count"),
        pytest.param(1.5, 0, id="fractional-count"),
        pytest.param(1, -0.1, id="negative-interval"),
        pytest.param(1, float("inf"), id="infinite-interval"),
    ],
)
def test_lidar_diagnostic_validates_sampling_options_before_factory(
    sample_count, sample_interval_seconds
):
    import lidar_test

    factory_called = False

    def factory(**_kwargs):
        nonlocal factory_called
        factory_called = True
        raise AssertionError("factory must not be called")

    with pytest.raises(ValueError):
        lidar_test.run_diagnostic(
            raw_min_cm=5,
            raw_max_cm=400,
            mounting_offset_cm=12,
            stale_after_seconds=0.5,
            startup_timeout_seconds=2,
            poll_interval_seconds=0.02,
            sample_count=sample_count,
            sample_interval_seconds=sample_interval_seconds,
            adapter_factory=factory,
            sleep=lambda _seconds: None,
        )

    assert not factory_called


def test_camera_settings_use_only_the_requested_device():
    from backup import video_streamer

    commands = []
    video_streamer.setup_camera(
        device="/dev/video9",
        command_runner=lambda command, **_kwargs: commands.append(command),
        settle=lambda _seconds: None,
    )

    assert commands
    assert all(command[2] == "/dev/video9" for command in commands)


def test_camera_runner_opens_requested_device_and_releases_on_server_exit():
    from backup import video_streamer

    camera = SimpleNamespace(release=lambda: setattr(camera, "released", True))
    camera.released = False
    opened = []
    app = SimpleNamespace(run=lambda **kwargs: setattr(app, "run_kwargs", kwargs))

    video_streamer.run_server(
        device="/dev/video9",
        host="127.0.0.1",
        port=4321,
        camera_factory=lambda device: opened.append(device) or camera,
        app_factory=lambda acquired_camera: (
            app if acquired_camera is camera else pytest.fail("wrong camera")
        ),
    )

    assert opened == ["/dev/video9"]
    assert app.run_kwargs == {"host": "127.0.0.1", "port": 4321}
    assert camera.released
