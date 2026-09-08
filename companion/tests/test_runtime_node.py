from __future__ import annotations

import ast
import json
from io import StringIO
from pathlib import Path
import sys
import threading
from types import ModuleType

import pytest

from artifacts.runtime_status import (
    MissionCommandDeliveredStatus,
    MissionReadyStatus,
    RuntimeFailureStatus,
)
import drone_sim_companion.runtime_node as runtime_node
from drone_sim_companion.runtime_node import (
    RuntimeConfig,
    autotune_control_timestamp_ns,
    comp2026_initial_command_timestamp_ns,
    connect_autotune_vehicle,
    connect_mavlink,
    quiesce_comp2026_runtime,
)
from drone_sim_companion.controller import MissionController
from drone_sim_companion.lifecycle import CompanionLifecycle
from drone_sim_companion.mission import CommandKind, Telemetry


RUN_ID = "00000000-0000-4000-8000-000000000001"


def test_autotune_can_deliver_first_command_at_public_zero_before_clock_ticks() -> None:
    assert autotune_control_timestamp_ns(
        mission_running=True, latest_clock_ns=None, first_command_pending=True
    ) == 0
    assert autotune_control_timestamp_ns(
        mission_running=True, latest_clock_ns=None, first_command_pending=False
    ) is None
    assert autotune_control_timestamp_ns(
        mission_running=True, latest_clock_ns=12, first_command_pending=False
    ) == 12
    assert autotune_control_timestamp_ns(
        mission_running=False, latest_clock_ns=12, first_command_pending=True
    ) is None


def test_comp2026_delivers_initial_command_after_public_zero_was_skipped() -> None:
    assert comp2026_initial_command_timestamp_ns(
        mission_running=True,
        latest_clock_ns=2_000_000,
        mission_ready=True,
        command_delivered=False,
        failed=False,
    ) == 2_000_000
    assert comp2026_initial_command_timestamp_ns(
        mission_running=True,
        latest_clock_ns=50_000_000,
        mission_ready=True,
        command_delivered=False,
        failed=False,
    ) == 50_000_000

    for overrides in (
        {"mission_running": False},
        {"latest_clock_ns": None},
        {"mission_ready": False},
        {"command_delivered": True},
        {"failed": True},
    ):
        inputs = {
            "mission_running": True,
            "latest_clock_ns": 2_000_000,
            "mission_ready": True,
            "command_delivered": False,
            "failed": False,
            **overrides,
        }
        assert comp2026_initial_command_timestamp_ns(**inputs) is None


def test_comp2026_polls_start_inputs_until_complete_gate_is_ready() -> None:
    assert runtime_node.comp2026_start_gate_poll_required(
        mission_running=True,
        mission_start_ready=False,
        failed=False,
    )
    assert not runtime_node.comp2026_start_gate_poll_required(
        mission_running=True,
        mission_start_ready=True,
        failed=False,
    )
    assert not runtime_node.comp2026_start_gate_poll_required(
        mission_running=True,
        mission_start_ready=False,
        failed=True,
    )
    assert not runtime_node.comp2026_start_gate_poll_required(
        mission_running=False,
        mission_start_ready=False,
        failed=False,
    )


def test_comp2026_stops_sensor_inputs_after_attempt_finishes() -> None:
    assert runtime_node.comp2026_sensor_inputs_required(
        mission_running=True,
        mission_worker_alive=True,
    )
    assert not runtime_node.comp2026_sensor_inputs_required(
        mission_running=True,
        mission_worker_alive=False,
    )
    assert runtime_node.comp2026_sensor_inputs_required(
        mission_running=False,
        mission_worker_alive=True,
    )


def write_resolved_config(
    run_directory: Path,
    *,
    run_id: str = RUN_ID,
    startup_wall_seconds: object = 120,
    max_wall_seconds: object = 3600,
    mission: str = "controlled_descent",
) -> Path:
    config_path = run_directory / "configuration/run.json"
    config_path.parent.mkdir(parents=True)
    config_path.write_text(
        json.dumps(
            {
                "run_id": run_id,
                "startup_wall_seconds": startup_wall_seconds,
                "max_wall_seconds": max_wall_seconds,
                "finalization_wall_seconds": 120,
                "mission": mission,
            }
        ),
        encoding="utf-8",
    )
    return config_path


def test_runtime_config_uses_resolved_run_startup_deadline(tmp_path: Path) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(run_directory)

    config = RuntimeConfig.from_environment(
        {
            "SIM_RUN_ID": RUN_ID,
            "SIM_RUN_DIRECTORY": str(run_directory),
            "SIM_CONFIG_PATH": str(config_path),
        }
    )

    assert config.mavlink_endpoint == "tcp:ardupilot-sitl:5760"
    assert config.startup_timeout_seconds == 120.0
    assert config.max_wall_seconds == 3600.0
    assert config.finalization_wall_seconds == 120.0
    assert config.mission == "controlled_descent"


def test_runtime_config_accepts_roll_autotune_without_competition_sources(
    tmp_path: Path,
) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(run_directory, mission="autotune_roll")

    config = RuntimeConfig.from_environment(
        {
            "SIM_RUN_ID": RUN_ID,
            "SIM_RUN_DIRECTORY": str(run_directory),
            "SIM_CONFIG_PATH": str(config_path),
        }
    )

    assert config.mission == "autotune_roll"
    assert config.course_path is None
    assert config.scenario_path is None


def test_runtime_config_accepts_roll_hover_without_competition_sources(
    tmp_path: Path,
) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(run_directory, mission="hover_roll")

    config = RuntimeConfig.from_environment(
        {
            "SIM_RUN_ID": RUN_ID,
            "SIM_RUN_DIRECTORY": str(run_directory),
            "SIM_CONFIG_PATH": str(config_path),
        }
    )

    assert config.mission == "hover_roll"


def test_runtime_config_preserves_explicit_startup_timeout_override(tmp_path: Path) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(run_directory)
    config = RuntimeConfig.from_environment(
        {
            "SIM_RUN_ID": RUN_ID,
            "SIM_RUN_DIRECTORY": str(run_directory),
            "SIM_CONFIG_PATH": str(config_path),
            "SIM_COMPANION_STARTUP_TIMEOUT_SECONDS": "17.5",
        }
    )

    assert config.startup_timeout_seconds == 17.5
    assert config.max_wall_seconds == 3600.0


def test_runtime_selects_original_competition_host_from_resolved_mission(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    run_directory = tmp_path / RUN_ID
    configuration = run_directory / "configuration"
    config_path = write_resolved_config(run_directory, mission="comp2026_auto")
    (configuration / "course.yaml").write_text("waypoints: {}\n", encoding="utf-8")
    (configuration / "scenario.yaml").write_text("payloads: []\n", encoding="utf-8")
    document = json.loads(config_path.read_text(encoding="utf-8"))
    document["competition"] = {"course": "course.yaml", "scenario": "scenario.yaml"}
    config_path.write_text(json.dumps(document), encoding="utf-8")
    selected: list[str] = []
    monkeypatch.setattr(runtime_node, "_run_controlled_descent", lambda _config: selected.append("controlled") or 0)
    monkeypatch.setattr(runtime_node, "_run_comp2026", lambda _config: selected.append("comp2026") or 0)
    monkeypatch.setattr(
        runtime_node.os,
        "environ",
        {
            "SIM_RUN_ID": RUN_ID,
            "SIM_RUN_DIRECTORY": str(run_directory),
            "SIM_CONFIG_PATH": str(config_path),
        },
    )

    assert runtime_node.main() == 0
    assert selected == ["comp2026"]


def test_runtime_selects_roll_autotune_host(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(run_directory, mission="autotune_roll")
    selected: list[str] = []
    monkeypatch.setattr(
        runtime_node,
        "_run_controlled_descent",
        lambda _config: selected.append("controlled") or 0,
    )
    monkeypatch.setattr(
        runtime_node,
        "_run_autotune_roll",
        lambda _config: selected.append("autotune") or 0,
    )
    monkeypatch.setattr(
        runtime_node,
        "_run_comp2026",
        lambda _config: selected.append("comp2026") or 0,
    )
    monkeypatch.setattr(
        runtime_node.os,
        "environ",
        {
            "SIM_RUN_ID": RUN_ID,
            "SIM_RUN_DIRECTORY": str(run_directory),
            "SIM_CONFIG_PATH": str(config_path),
        },
    )

    assert runtime_node.main() == 0
    assert selected == ["autotune"]


def test_first_heartbeat_ignores_startup_wall_deadline_after_transport_connects() -> None:
    assert (
        runtime_node.first_heartbeat_wall_failure(
            heartbeat_observed=False,
            wall_now=120.0,
            overall_wall_deadline=3600.0,
        )
        is None
    )
    assert runtime_node.first_heartbeat_wall_failure(
        heartbeat_observed=False,
        wall_now=3600.0,
        overall_wall_deadline=3600.0,
    ) == "MAVLink heartbeat was unavailable before the overall run wall failsafe"


def test_autotune_vehicle_connection_requires_run_state_subscription() -> None:
    calls: list[tuple[str, bool, float]] = []

    def connect(endpoint: str, *, wait_ready: bool, heartbeat_timeout: float) -> object:
        calls.append((endpoint, wait_ready, heartbeat_timeout))
        return object()

    with pytest.raises(RuntimeError, match="run-state subscription"):
        connect_autotune_vehicle(
            connect,
            "tcp:ardupilot-sitl:5760",
            heartbeat_timeout=120.0,
            run_state_subscription=None,
        )

    subscription = object()
    vehicle = connect_autotune_vehicle(
        connect,
        "tcp:ardupilot-sitl:5760",
        heartbeat_timeout=120.0,
        run_state_subscription=subscription,
    )

    assert vehicle is not None
    assert calls == [("tcp:ardupilot-sitl:5760", False, 120.0)]


def test_runtime_config_rejects_config_outside_current_run(tmp_path: Path) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(tmp_path / "another-run")

    with pytest.raises(ValueError, match="run's resolved configuration"):
        RuntimeConfig.from_environment(
            {
                "SIM_RUN_ID": RUN_ID,
                "SIM_RUN_DIRECTORY": str(run_directory),
                "SIM_CONFIG_PATH": str(config_path),
            }
        )


def test_runtime_config_rejects_stale_run_configuration(tmp_path: Path) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(
        run_directory,
        run_id="00000000-0000-4000-8000-000000000002",
    )

    with pytest.raises(ValueError, match="run_id must match SIM_RUN_ID"):
        RuntimeConfig.from_environment(
            {
                "SIM_RUN_ID": RUN_ID,
                "SIM_RUN_DIRECTORY": str(run_directory),
                "SIM_CONFIG_PATH": str(config_path),
            }
        )


@pytest.mark.parametrize("startup_wall_seconds", [True, 0, -1, 1.5, "120"])
def test_runtime_config_rejects_invalid_resolved_startup_deadline(
    tmp_path: Path,
    startup_wall_seconds: object,
) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(
        run_directory,
        startup_wall_seconds=startup_wall_seconds,
    )

    with pytest.raises(ValueError, match="startup_wall_seconds must be a positive integer"):
        RuntimeConfig.from_environment(
            {
                "SIM_RUN_ID": RUN_ID,
                "SIM_RUN_DIRECTORY": str(run_directory),
                "SIM_CONFIG_PATH": str(config_path),
            }
        )


@pytest.mark.parametrize("max_wall_seconds", [True, 0, -1, 1.5, "3600"])
def test_runtime_config_rejects_invalid_overall_wall_failsafe(
    tmp_path: Path,
    max_wall_seconds: object,
) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(
        run_directory,
        max_wall_seconds=max_wall_seconds,
    )

    with pytest.raises(ValueError, match="max_wall_seconds must be a positive integer"):
        RuntimeConfig.from_environment(
            {
                "SIM_RUN_ID": RUN_ID,
                "SIM_RUN_DIRECTORY": str(run_directory),
                "SIM_CONFIG_PATH": str(config_path),
            }
        )


@pytest.mark.parametrize(
    "environment",
    [
        {"SIM_RUN_ID": "bad", "SIM_RUN_DIRECTORY": "/runs/bad"},
        {
            "SIM_RUN_ID": RUN_ID,
            "SIM_RUN_DIRECTORY": "/runs/x",
            "SIM_COMPANION_STARTUP_TIMEOUT_SECONDS": "0",
        },
    ],
)
def test_runtime_config_rejects_invalid_infrastructure_configuration(
    environment: dict[str, str],
) -> None:
    with pytest.raises(ValueError):
        RuntimeConfig.from_environment(environment)


def test_runtime_has_no_gazebo_ground_truth_dependency() -> None:
    source = (
        Path(__file__).parents[1]
        / "src/drone_sim_companion/runtime_node.py"
    ).read_text(encoding="utf-8")

    assert "GroundTruth" not in source
    assert '"/simulation/ground_truth"' not in source
    assert "vertical_truth" not in source


def test_competition_runtime_defers_dronekit_readiness_to_its_live_gate() -> None:
    source = (
        Path(__file__).parents[1]
        / "src/drone_sim_companion/runtime_node.py"
    ).read_text(encoding="utf-8")
    tree = ast.parse(source)
    run_comp2026 = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "_run_comp2026"
    )
    constructors = [
        node
        for node in ast.walk(run_comp2026)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "DroneControl"
    ]

    assert len(constructors) == 1
    keywords = {
        keyword.arg: keyword.value
        for keyword in constructors[0].keywords
    }
    wait_ready = keywords["wait_ready"]
    assert isinstance(wait_ready, ast.Constant)
    assert wait_ready.value is False
    heartbeat_timeout = keywords["heartbeat_timeout"]
    assert isinstance(heartbeat_timeout, ast.Attribute)
    assert isinstance(heartbeat_timeout.value, ast.Name)
    assert heartbeat_timeout.value.id == "config"
    assert heartbeat_timeout.attr == "startup_timeout_seconds"


def test_competition_ros_callbacks_separate_ordered_control_from_camera_work() -> None:
    source = (
        Path(__file__).parents[1]
        / "src/drone_sim_companion/runtime_node.py"
    ).read_text(encoding="utf-8")
    tree = ast.parse(source)
    run_comp2026 = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "_run_comp2026"
    )
    subscriptions = [
        node
        for node in ast.walk(run_comp2026)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "create_subscription"
    ]

    assert len(subscriptions) == 4
    expected_groups = {
        "state_callback": "clock_callback_group",
        "clock_callback": "clock_callback_group",
        "range_callback": "range_callback_group",
        "image_callback": "image_callback_group",
    }
    for subscription in subscriptions:
        callback = subscription.args[2]
        assert isinstance(callback, ast.Name)
        callback_group = next(
            (keyword.value for keyword in subscription.keywords if keyword.arg == "callback_group"),
            None,
        )
        assert isinstance(callback_group, ast.Name)
        assert callback_group.id == expected_groups[callback.id]

        if callback.id in {"clock_callback", "range_callback"}:
            qos_call = subscription.args[3]
            assert isinstance(qos_call, ast.Call)
            assert isinstance(qos_call.func, ast.Name)
            assert qos_call.func.id == "qos"
            assert isinstance(qos_call.args[0], ast.Constant)
            assert qos_call.args[0].value == 1

    executors = [
        node
        for node in ast.walk(run_comp2026)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "MultiThreadedExecutor"
    ]
    assert len(executors) == 1
    thread_count = next(
        keyword.value
        for keyword in executors[0].keywords
        if keyword.arg == "num_threads"
    )
    assert isinstance(thread_count, ast.Constant)
    assert thread_count.value == 4


def test_competition_runtime_emits_start_readiness_changes() -> None:
    source = (
        Path(__file__).parents[1]
        / "src/drone_sim_companion/runtime_node.py"
    ).read_text(encoding="utf-8")

    assert '"mission_start_readiness"' in source
    assert "gate.readiness" in source


def test_mavlink_connect_retries_only_within_wall_infrastructure_deadline() -> None:
    attempts = 0
    clock = iter((0.0, 0.2, 0.4))

    def factory(endpoint: str, **options: object) -> object:
        nonlocal attempts
        attempts += 1
        assert endpoint == "tcp:ardupilot-sitl:5760"
        assert options == {"autoreconnect": False, "source_system": 255}
        if attempts < 3:
            raise ConnectionRefusedError("not listening")
        return "connected"

    pauses: list[float] = []
    assert (
        connect_mavlink(
            factory,
            "tcp:ardupilot-sitl:5760",
            deadline=1.0,
            now=lambda: next(clock),
            pause=pauses.append,
        )
        == "connected"
    )
    assert pauses == [0.1, 0.1]


def test_runtime_wires_passive_facts_to_durable_mission_readiness() -> None:
    class Protocol:
        def __init__(self) -> None:
            self.statuses = []

        def write_status(self, status) -> None:
            self.statuses.append(status)

        def write_quiescence(self, _module: str) -> None:
            pass

    class Vehicle:
        def __init__(self) -> None:
            self.sent: list[tuple[CommandKind, float | None]] = []

        def send(self, command: CommandKind, altitude_m: float | None) -> None:
            self.sent.append((command, altitude_m))

    protocol = Protocol()
    lifecycle = CompanionLifecycle(run_id=RUN_ID, protocol=protocol, stream=StringIO())
    vehicle = Vehicle()
    controller = MissionController(vehicle, lifecycle.emit)

    runtime_node.process_runtime_telemetry(
        controller,
        lifecycle,
        Telemetry(10, prearm_checks_healthy=True),
        mission_running=False,
        public_clock_observed=False,
    )
    runtime_node.process_runtime_telemetry(
        controller,
        lifecycle,
        Telemetry(20, heartbeat=True, mode="STABILIZE", armed=False),
        mission_running=False,
        public_clock_observed=False,
    )

    assert protocol.statuses == [MissionReadyStatus(RUN_ID)]
    assert vehicle.sent == []


def test_controlled_descent_startup_failure_reaches_lifecycle(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    statuses = []
    quiescence: list[str] = []
    mavutil = ModuleType("mavutil")
    mavutil.mavlink_connection = object()  # type: ignore[attr-defined]

    class Protocol:
        def __init__(self, _config: RuntimeConfig) -> None:
            pass

        def write_status(self, status) -> None:
            statuses.append(status)

        def write_quiescence(self, module: str) -> None:
            quiescence.append(module)

        def close(self) -> None:
            pass

    dependency_members = {
        "pymavlink": {"mavutil": mavutil},
        "rclpy.node": {"Node": object},
        "rclpy.qos": {
            "DurabilityPolicy": object,
            "QoSProfile": object,
            "ReliabilityPolicy": object,
        },
        "rosgraph_msgs.msg": {"Clock": object},
        "simulation_interfaces.msg": {"RunState": object},
    }
    for name in ("rclpy", "rosgraph_msgs", "simulation_interfaces"):
        monkeypatch.setitem(sys.modules, name, ModuleType(name))
    for name, members in dependency_members.items():
        module = ModuleType(name)
        for member_name, member in members.items():
            setattr(module, member_name, member)
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", Protocol)
    monkeypatch.setattr(
        runtime_node,
        "connect_mavlink",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            TimeoutError("MAVLink connection deadline expired")
        ),
    )
    config = RuntimeConfig(run_id=RUN_ID, run_directory=tmp_path / RUN_ID)

    assert runtime_node._run_controlled_descent(config) == 1
    assert statuses == [
        RuntimeFailureStatus(
            RUN_ID,
            "companion",
            "MAVLink connection deadline expired",
            ("logs/docker/companion.log.partial",),
        )
    ]
    assert quiescence == ["companion"]


def test_initial_command_delivery_is_durable_and_exactly_at_public_zero() -> None:
    class Protocol:
        def __init__(self) -> None:
            self.statuses = []

        def write_status(self, status) -> None:
            self.statuses.append(status)

        def write_quiescence(self, _module: str) -> None:
            pass

    protocol = Protocol()
    lifecycle = CompanionLifecycle(run_id=RUN_ID, protocol=protocol, stream=StringIO())

    lifecycle.observe_command_delivery(CommandKind.SET_GUIDED, 0)

    assert protocol.statuses == [MissionCommandDeliveredStatus(RUN_ID, 0)]


def test_comp2026_quiescence_follows_terminated_worker_and_executor() -> None:
    events: list[str] = []
    stop_worker = threading.Event()
    stop_executor = threading.Event()

    def worker_target() -> None:
        stop_worker.wait()
        events.append("worker_stopped")

    def executor_target() -> None:
        stop_executor.wait()
        events.append("executor_stopped")

    worker = threading.Thread(target=worker_target)
    executor_thread = threading.Thread(target=executor_target)
    worker.start()
    executor_thread.start()

    class Executor:
        def shutdown(self, *, timeout_sec: float) -> bool:
            assert timeout_sec == 1.0
            events.append("stop_executor")
            stop_executor.set()
            return True

    def stop_attempt(reason: str) -> None:
        assert reason == "finalization"
        events.append("stop_attempt")
        stop_worker.set()

    def finalize() -> None:
        assert not worker.is_alive()
        assert not executor_thread.is_alive()
        events.append("quiescence")

    failures: list[str] = []
    assert quiesce_comp2026_runtime(
        stop_attempt=stop_attempt,
        mission_worker=worker,
        executor=Executor(),
        executor_thread=executor_thread,
        close_output_producers=lambda: events.append("producers_closed"),
        finalize=finalize,
        write_failure=failures.append,
        timeout_seconds=1.0,
    ) is True

    assert failures == []
    assert events.index("worker_stopped") < events.index("stop_executor")
    assert events.index("executor_stopped") < events.index("producers_closed")
    assert events[-1] == "quiescence"


def test_comp2026_never_writes_quiescence_with_a_surviving_worker() -> None:
    events: list[str] = []

    class StuckWorker:
        def join(self, timeout: float) -> None:
            assert timeout == 1.0
            events.append("worker_join_timed_out")

        def is_alive(self) -> bool:
            return True

    class StoppedExecutorThread:
        def join(self, timeout: float) -> None:
            assert timeout == 1.0
            events.append("executor_joined")

        def is_alive(self) -> bool:
            return False

    class Executor:
        def shutdown(self, *, timeout_sec: float) -> bool:
            assert timeout_sec == 1.0
            events.append("executor_shutdown")
            return True

    failures: list[str] = []
    assert quiesce_comp2026_runtime(
        stop_attempt=lambda _reason: events.append("stop_attempt"),
        mission_worker=StuckWorker(),
        executor=Executor(),
        executor_thread=StoppedExecutorThread(),
        close_output_producers=lambda: events.append("producers_closed"),
        finalize=lambda: events.append("quiescence"),
        write_failure=failures.append,
        timeout_seconds=1.0,
    ) is False

    assert failures == ["original mission worker did not stop for finalization"]
    assert "quiescence" not in events
