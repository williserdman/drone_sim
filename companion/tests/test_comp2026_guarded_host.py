from __future__ import annotations

from dataclasses import dataclass, replace
from contextlib import contextmanager
import ast
import math
from pathlib import Path
import subprocess
import sys
import threading
import textwrap
from types import SimpleNamespace

import pytest
from artifacts.runtime_status import status_document, status_name

import drone_sim_companion.runtime_node as runtime_node
from drone_sim_companion.comp2026_host import StaleSensorError


RUN_ID = "00000000-0000-4000-8000-000000000001"
ROOT = Path(__file__).parents[2]


def rgb8_image(timestamp_ns: int) -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=timestamp_ns // 1_000_000_000,
                nanosec=timestamp_ns % 1_000_000_000,
            ),
            frame_id="camera/onboard",
        ),
        width=640,
        height=480,
        encoding="rgb8",
        is_bigendian=False,
        step=640 * 3,
        data=bytes((10, 20, 30)) * (640 * 480),
    )


@dataclass
class Cleanup:
    lidar_stopped: bool = True
    lidar_cleanup_completed: bool = True
    camera_stopped: bool = True
    recordings_completed: bool = True
    payload_closed: bool = True
    vehicle_closed: bool = True
    diagnostics: tuple[str, ...] = ()


class FakeProtocol:
    def __init__(self, events: list[object]) -> None:
        self.events = events
        self.finalize_requested = threading.Event()

    def write_status(self, status: object) -> None:
        self.events.append(
            ("status", status_name(type(status)), status_document(status))  # type: ignore[arg-type]
        )

    def write_quiescence(self, module: str) -> None:
        self.events.append(("quiescence", module))

    def read_finalize_request(self) -> object | None:
        return {} if self.finalize_requested.is_set() else None

    def close(self) -> None:
        self.events.append("protocol-close")


class FakeExecutor:
    def __init__(self, events: list[object]) -> None:
        self.events = events
        self.release = threading.Event()

    def add_node(self, _node: object) -> None:
        self.events.append("executor-add-node")

    def spin(self) -> None:
        self.events.append("executor-spin")
        self.release.wait(2.0)

    def shutdown(self, *, timeout_sec: float) -> bool:
        self.events.append(("executor-shutdown", timeout_sec))
        self.release.set()
        return True


class FakeNode:
    def __init__(self, events: list[object]) -> None:
        self.events = events
        self.callbacks: dict[str, object] = {}
        self.publishers: dict[str, object] = {}
        self.publish_error: BaseException | None = None
        self.publish_error_state: str | None = None
        self.publisher_acked = True
        self.publisher_ack_error: BaseException | None = None
        self.publisher_ack_callback = lambda: None
        self.publisher_subscribers = [
            self.subscription_endpoint("drone_sim_scorekeeper"),
            self.subscription_endpoint("rosbag2_recorder_deadbeef"),
        ]

    @staticmethod
    def subscription_endpoint(
        node_name: str,
        *,
        node_namespace: str = "/",
        topic_type: str = "simulation_interfaces/msg/MissionEvent",
        reliability: str = "reliable",
        durability: str = "transient",
    ) -> object:
        return SimpleNamespace(
            node_name=node_name,
            node_namespace=node_namespace,
            topic_type=topic_type,
            qos_profile=SimpleNamespace(
                reliability=reliability,
                durability=durability,
            ),
        )

    def get_subscriptions_info_by_topic(self, topic: str) -> list[object]:
        assert topic == "/simulation/mission_events"
        return list(self.publisher_subscribers)

    def create_subscription(
        self,
        _message_type: object,
        topic: str,
        callback: object,
        _qos: object,
        *,
        callback_group: object,
    ) -> object:
        assert callback_group is not None
        self.events.append(("subscribe", topic))
        self.callbacks[topic] = callback
        return SimpleNamespace(topic=topic)

    def create_client(
        self, _service_type: object, name: str, *, callback_group: object
    ) -> object:
        assert callback_group is not None
        self.events.append(("client", name))
        return SimpleNamespace(service_is_ready=lambda: True)

    def create_publisher(
        self, message_type: object, topic: str, qos: object
    ) -> object:
        node = self

        class Publisher:
            def publish(self, message: object) -> None:
                node.events.append(("publish", topic, message))
                if node.publish_error is not None and (
                    node.publish_error_state is None
                    or getattr(message, "state", None) == node.publish_error_state
                ):
                    raise node.publish_error

            def wait_for_all_acked(self, *, timeout: object) -> bool:
                node.events.append(("publisher-acked", topic, timeout.seconds))
                if node.publisher_ack_error is not None:
                    raise node.publisher_ack_error
                node.publisher_ack_callback()
                return node.publisher_acked

            def get_subscription_count(self) -> int:
                return len(node.publisher_subscribers)

        publisher = Publisher()
        self.events.append(("publisher", topic, message_type, qos))
        self.publishers[topic] = publisher
        return publisher

    def destroy_subscription(self, subscription: object) -> None:
        self.events.append(("destroy-subscription", subscription.topic))

    def destroy_publisher(self, publisher: object) -> None:
        topic = next(
            name for name, candidate in self.publishers.items() if candidate is publisher
        )
        self.events.append(("destroy-publisher", topic))

    def destroy_node(self) -> None:
        self.events.append("destroy-node")


class FakeFactories:
    def __init__(self, **values: object) -> None:
        vars(self).update(values)


class FakeVehicle:
    def __init__(self, events: list[object], *, heartbeat: object = 0.25, armable: object = True):
        self._events = events
        self.last_heartbeat = heartbeat
        self.is_armable = armable
        self.mode_writes: list[object] = []

    def close(self) -> None:
        self._events.append("vehicle-close")


class FakeController:
    def __init__(self, vehicle: FakeVehicle, events: list[object]) -> None:
        self.vehicle = vehicle
        self.events = events
        self.telemetry_verifier: object | None = None

    def install_startup_telemetry_verifier(
        self, verifier: object, *, verified: bool
    ) -> None:
        assert verified is False
        assert callable(verifier)
        self.telemetry_verifier = verifier

    def arm_after_guided(self) -> None:
        assert callable(self.telemetry_verifier)
        assert self.telemetry_verifier() is None
        self.events.append("arm-output")


class FakeRclpy:
    def __init__(self, events: list[object]) -> None:
        self.events = events
        self.init_options: dict[str, object] | None = None

    def init(self, **options: object) -> None:
        self.init_options = options
        self.events.append("rclpy-init")

    def shutdown(self) -> None:
        self.events.append("rclpy-shutdown")


class FakeTimebase:
    def __init__(self) -> None:
        self.active: object | None = None

    @contextmanager
    def configured(self, clock: object):
        previous, self.active = self.active, clock
        try:
            yield
        finally:
            self.active = previous

    @staticmethod
    def epoch() -> float:
        return 1_800_000_000.0


class EmptyDetailError(BaseException):
    pass


class RaisingDetailError(BaseException):
    def __str__(self) -> str:
        raise RuntimeError("exception detail unavailable")


class NonStringDetailError(BaseException):
    def __str__(self) -> str:
        return object()  # type: ignore[return-value]


class FatalStopError(BaseException):
    pass


class HostHarness:
    def __init__(self, *, heartbeat: object = 0.25, armable: object = True) -> None:
        self.events: list[object] = []
        self.node = FakeNode(self.events)
        self.executor = FakeExecutor(self.events)
        self.vehicle = FakeVehicle(
            self.events, heartbeat=heartbeat, armable=armable
        )
        self.factories: FakeFactories | None = None
        self.admission = None
        self.ready_hook = None
        self.runtime = SimpleNamespace(
            controller=FakeController(self.vehicle, self.events),
            request_abort=lambda reason: self.events.append(("abort", reason)),
            stop_monitoring=lambda: self.events.append("stop-monitoring"),
        )
        self.clock_seen: list[object] = []
        self.payload_ids: list[int] = []
        self.timebase = FakeTimebase()
        self.validated_artifacts = object()
        self.received_artifacts: object | None = None
        self.guided_delivery_callback: object | None = None
        self.deliver_guided = True
        self.stop_lidar_then_send_late_range = False
        self.send_bad_range_before_lidar_stop = False
        self.publish_phase_events = False
        self.phase_event_timestamp: tuple[int, int] | None = None
        self.drop_subscriber_before_phase_events = False
        self.drop_subscriber_after_phase_events = False
        self.remaining_subscribers: list[object] = []
        self.phase_observer: object | None = None
        self.camera_subscription_expected = False
        self.rclpy = FakeRclpy(self.events)

    def dependencies(self) -> object:
        harness = self

        class Node:
            def __new__(cls, *_args: object, **_kwargs: object) -> FakeNode:
                return harness.node

        class Executor:
            def __new__(cls, *_args: object, **_kwargs: object) -> FakeExecutor:
                return harness.executor

        class CallbackGroup:
            pass

        class QoSProfile:
            def __init__(self, **kwargs: object) -> None:
                vars(self).update(kwargs)

        class MissionEvent:
            __slots__ = ("run_id", "sim_timestamp", "event_id", "phase", "state", "detail")

            def __init__(self) -> None:
                self.run_id = ""
                self.sim_timestamp = SimpleNamespace(sec=0, nanosec=0)
                self.event_id = -1
                self.phase = ""
                self.state = ""
                self.detail = ""

        class Duration:
            def __init__(self, *, seconds: float) -> None:
                self.seconds = seconds

        class RunState:
            RUNNING = 2
            FINALIZING = 3

        class PayloadCommand:
            class Request:
                pass

        def drone_control(*args: object, **kwargs: object) -> FakeController:
            harness.events.append(("controller", args, kwargs))
            harness.guided_delivery_callback = kwargs.get(
                "guided_output_delivery_callback"
            )
            subscribed = {
                event[1]
                for event in harness.events
                if isinstance(event, tuple) and event[0] == "subscribe"
            }
            expected_subscriptions = {
                "/simulation/run_state",
                "/clock",
                "/competition/range/downward",
            }
            if harness.camera_subscription_expected:
                expected_subscriptions.add("/competition/camera/onboard")
            assert subscribed == expected_subscriptions
            assert "executor-spin" in harness.events
            harness.clock_seen.append(harness.timebase.active)
            return harness.runtime.controller

        class AckTransport:
            def __init__(self, vehicle: object, **kwargs: object) -> None:
                harness.events.append(("ack", vehicle, kwargs))

        class Collector:
            def __init__(self, **kwargs: object) -> None:
                harness.events.append(("collector", kwargs))

            def prepare(self) -> None:
                harness.clock_seen.append(harness.timebase.active)
                harness.events.append("telemetry-prepared")

            def verify_after_guided(self) -> object:
                harness.node.callbacks["/clock"](
                    SimpleNamespace(clock=SimpleNamespace(sec=0, nanosec=500_000_000))
                )
                harness.events.append("telemetry-verified")
                return object()

            def close(self) -> None:
                if "telemetry-close" not in harness.events:
                    harness.events.append("telemetry-close")

        class LidarAdapter:
            def __init__(self, lidar: object, *, range_ingress: object) -> None:
                self.lidar = lidar
                self.range_ingress = range_ingress
                harness.events.append(("lidar-adapter", lidar))

            def stop(self) -> object:
                self.range_ingress.close_and_stop(
                    lambda: harness.events.append("lidar-stop")
                )
                return SimpleNamespace(worker_stopped=True, cleanup_completed=True)

        class Fm2PayloadAdapter:
            supports_attachment = False

            def __init__(self, dropper: object, *, aruco_id: int) -> None:
                harness.payload_ids.append(aruco_id)
                self.dropper = dropper

        class CompetitionPayloadAdapter:
            supports_attachment = True

            def __init__(
                self,
                run_id: str,
                client: object,
                clock: object,
                *,
                permission: object,
                delay_wall_timeout_seconds: float,
            ) -> None:
                harness.payload_ids.extend((2, 3, 4))
                self.arguments = (
                    run_id,
                    client,
                    clock,
                    permission,
                    delay_wall_timeout_seconds,
                )

        def start_repl(
            artifacts: object,
            config: object,
            *,
            factories: FakeFactories,
            diagnostics: object,
            monitoring_stop: threading.Event,
            startup_admission_check: object,
            on_listener_ready: object,
            manage_signals: bool,
            phase_observer: object | None = None,
        ) -> object:
            del diagnostics, monitoring_stop
            assert manage_signals is False
            harness.received_artifacts = artifacts
            assert artifacts is harness.validated_artifacts
            harness.factories = factories
            harness.camera_subscription_expected = 31002 in config.enabled_phases
            harness.admission = startup_admission_check
            harness.ready_hook = on_listener_ready
            harness.phase_observer = phase_observer
            with pytest.raises(Exception, match="mission-ready"):
                startup_admission_check()
            harness.events.append("closed-admission")
            controller = factories.controller_factory(
                config=config,
                flight_state="flight-state",
                permission_guard="permission",
                decoders=SimpleNamespace(
                    heartbeat_mode_decoder="heartbeat-decoder",
                    rc_health_decoder="rc-decoder",
                    observe_sys_status="sys-status-decoder",
                    heartbeat_failsafe_decoder="failsafe-decoder",
                ),
            )
            harness.node.callbacks["/simulation/run_state"](
                SimpleNamespace(run_id=RUN_ID, state=RunState.RUNNING)
            )
            harness.node.callbacks["/clock"](
                SimpleNamespace(
                    clock=SimpleNamespace(sec=0, nanosec=0)
                )
            )
            factories.ack_transport_factory(controller=controller, config=config)
            collector = factories.telemetry_collector_factory(
                controller=controller, profile="flight-profile", config=config
            )
            assert factories.telemetry_startup_mode == "staged_simulation"
            collector.prepare()
            controller.install_startup_telemetry_verifier(
                lambda: (collector.verify_after_guided(), None)[1],
                verified=False,
            )
            lidar = factories.lidar_factory(config=config)
            assert isinstance(lidar, LidarAdapter)
            class Permission:
                def __call__(self) -> bool:
                    return True

                @staticmethod
                def actuate(output: object) -> None:
                    output()

            permission = Permission()
            payload = factories.dropper_factory(config=config, permission=permission)
            if 31002 in config.enabled_phases:
                assert isinstance(payload, CompetitionPayloadAdapter)
                assert payload.supports_attachment is True
                assert factories.supports_attachment is True
            else:
                assert isinstance(payload, Fm2PayloadAdapter)
                assert payload.supports_attachment is False
                assert factories.supports_attachment is False
            on_listener_ready(harness.runtime)
            assert startup_admission_check() is None
            if harness.send_bad_range_before_lidar_stop:
                harness.node.callbacks["/competition/range/downward"](
                    SimpleNamespace(header=SimpleNamespace(stamp=object()))
                )
                lidar.stop()
            if harness.stop_lidar_then_send_late_range:
                lidar.stop()
                harness.node.callbacks["/competition/range/downward"](
                    SimpleNamespace(header=SimpleNamespace(stamp=object()))
                )
            if harness.deliver_guided and callable(harness.guided_delivery_callback):
                harness.guided_delivery_callback()
                controller.arm_after_guided()
            if harness.publish_phase_events:
                assert callable(phase_observer)
                if harness.drop_subscriber_before_phase_events:
                    harness.node.publisher_subscribers = list(
                        harness.remaining_subscribers
                    )
                if harness.phase_event_timestamp is not None:
                    seconds, nanoseconds = harness.phase_event_timestamp
                    harness.node.callbacks["/clock"](
                        SimpleNamespace(
                            clock=SimpleNamespace(sec=seconds, nanosec=nanoseconds)
                        )
                    )
                phase_observer("FM1", "STARTED")
                phase_observer("FM1", "COMPLETE")
                phase_observer("FM2", "STARTED")
                phase_observer("FM2", "COMPLETE")
                if harness.drop_subscriber_after_phase_events:
                    harness.node.publisher_subscribers = list(
                        harness.remaining_subscribers
                    )
            collector.close()
            harness.events.append("listener-run")
            harness.clock_seen.append(harness.timebase.active)
            harness.events.append("nested-cleanup")
            return SimpleNamespace(
                mission_result="SUCCEEDED",
                recovery_outcome="HOME_LANDED",
                monitoring_exit_reason="NOT_REQUIRED",
                cleanup_report=Cleanup(),
            )

        return SimpleNamespace(
            rclpy=self.rclpy,
            SignalHandlerOptions=SimpleNamespace(NO="no-signal-handlers"),
            Node=Node,
            MultiThreadedExecutor=Executor,
            MutuallyExclusiveCallbackGroup=CallbackGroup,
            QoSProfile=QoSProfile,
            ReliabilityPolicy=SimpleNamespace(RELIABLE="reliable"),
            DurabilityPolicy=SimpleNamespace(
                TRANSIENT_LOCAL="transient", VOLATILE="volatile"
            ),
            Clock=object,
            Image=object,
            LaserScan=object,
            MissionEvent=MissionEvent,
            Duration=Duration,
            RunState=RunState,
            PayloadCommand=PayloadCommand,
            DroneControl=drone_control,
            DroneKitQGCAckTransport=AckTransport,
            TelemetryStartupCollector=Collector,
            LiveComponentFactories=FakeFactories,
            CommandRejected=RuntimeError,
            QgcRosLidarAdapter=LidarAdapter,
            QgcFm2PayloadAdapter=Fm2PayloadAdapter,
            QgcCompetitionPayloadAdapter=CompetitionPayloadAdapter,
            start_repl=start_repl,
            CameraManager=object,
            Camera=object,
            timebase=self.timebase,
            thread_factory=threading.Thread,
            wall_now=runtime_node.time.monotonic,
        )


def config(tmp_path: Path, *, qgc: object = object()) -> runtime_node.RuntimeConfig:
    run_directory = tmp_path / RUN_ID
    run_directory.mkdir()
    return runtime_node.RuntimeConfig(
        run_id=RUN_ID,
        run_directory=run_directory,
        mission="comp2026_auto",
        course_path=run_directory / "configuration/course.yaml",
        scenario_path=run_directory / "configuration/scenario.yaml",
        qgc=qgc,
        output_root=tmp_path / "outputs",
    )


def projection(*, harness: HostHarness | None = None, full: bool = False) -> object:
    runtime_config = SimpleNamespace(
        connection=SimpleNamespace(
            endpoint="tcp:ardupilot-sitl:5760",
            source_identity="companion-source",
            target_identity="fc-target",
            wire_protocol="2.0",
            wait_ready=False,
            heartbeat_timeout_s=17.0,
        ),
        operating_site=SimpleNamespace(mission_home_check="home-check"),
        fc_home_position_tolerance_m=0.5,
        fc_home_altitude_tolerance_m=0.5,
        clearance_calibration="clearance",
        release_stability="stability",
        telemetry=SimpleNamespace(
            home_request_timeout_s=4.0,
            poll_interval_s=0.05,
        ),
        autopilot_version="arducopter-4.5.7-contract",
        components=SimpleNamespace(backend="drone-sim-ros-confirmed-v1"),
        enabled_phases=frozenset(
            {31000, 31001, 31002} if full else {31000, 31001}
        ),
        cleanup_timeout_s=1.0,
    )
    return SimpleNamespace(
        runtime_configuration=runtime_config,
        validated_listener_artifacts=(
            harness.validated_artifacts if harness is not None else object()
        ),
        flight_profile=SimpleNamespace(
            freshness_bounds={"heartbeat": 0.5}
        ),
        payload_delay_wall_timeout_s=3.0,
    )


def install_harness(
    monkeypatch: pytest.MonkeyPatch,
    harness: HostHarness,
    projected: object,
) -> FakeProtocol:
    protocol = FakeProtocol(harness.events)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(
        runtime_node,
        "project_qgc_runtime",
        lambda _config, *, epoch_seconds: projected,
        raising=False,
    )
    monkeypatch.setattr(
        runtime_node,
        "_load_qgc_live_dependencies",
        harness.dependencies,
        raising=False,
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_timebase", lambda: harness.timebase, raising=False
    )
    monkeypatch.setattr(runtime_node.signal, "signal", lambda *_args: None)
    monkeypatch.setattr(runtime_node.signal, "getsignal", lambda *_args: None)
    return protocol


def guarded_host(
    tmp_path: Path,
    *,
    clock: object | None = None,
    stop: object | None = None,
) -> tuple[runtime_node._Comp2026QgcRosHost, object]:
    harness = HostHarness()
    selected_clock = runtime_node.SimulationClock() if clock is None else clock
    selected_stop = (
        runtime_node._QgcStopCoordinator(selected_clock, threading.Event())
        if stop is None
        else stop
    )
    return (
        runtime_node._Comp2026QgcRosHost(
            config(tmp_path),
            projection(harness=harness),
            selected_clock,
            SimpleNamespace(),
            harness.dependencies(),
            selected_stop,
            threading.Event(),
            FakeProtocol(harness.events),
        ),
        selected_stop,
    )


def test_missing_qgc_rejects_before_any_live_dependency(tmp_path, monkeypatch) -> None:
    calls: list[str] = []

    def reject_projection(_config: object, *, epoch_seconds: float) -> object:
        assert math.isfinite(epoch_seconds)
        calls.append("offline-validation")
        raise ValueError("comp2026_auto requires resolved QGC inputs")

    monkeypatch.setattr(
        runtime_node, "project_qgc_runtime", reject_projection, raising=False
    )
    monkeypatch.setattr(
        runtime_node,
        "_load_qgc_live_dependencies",
        lambda: pytest.fail("live dependencies constructed before offline validation"),
        raising=False,
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_timebase", FakeTimebase, raising=False
    )
    events: list[object] = []
    protocol = FakeProtocol(events)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)

    assert runtime_node._run_comp2026_qgc(config(tmp_path, qgc=None)) == 1
    assert calls == ["offline-validation"]
    assert any(event[:2] == ("status", "runtime-failure") for event in events if isinstance(event, tuple))


def test_guarded_host_composes_only_qgc_fm1_fm2_after_ros_is_listening(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness(heartbeat=0.5)
    projected = projection(harness=harness)
    install_harness(monkeypatch, harness, projected)
    monkeypatch.setattr(
        runtime_node,
        "create_comp2026_lidar",
        lambda clock: harness.clock_seen.append(clock) or "ros-lidar",
    )

    assert runtime_node._run_comp2026(config(tmp_path)) == 0

    assert harness.factories is not None
    assert harness.factories.backend == "drone-sim-ros-confirmed-v1"
    assert harness.factories.telemetry_startup_mode == "staged_simulation"
    assert harness.received_artifacts is harness.validated_artifacts
    assert harness.payload_ids == [2]
    assert all(clock is harness.clock_seen[0] for clock in harness.clock_seen)
    controller_event = next(event for event in harness.events if isinstance(event, tuple) and event[0] == "controller")
    assert controller_event[1] == ("tcp:ardupilot-sitl:5760",)
    kwargs = controller_event[2]
    assert kwargs == {
        "source_identity": "companion-source",
        "flight_controller_target": "fc-target",
        "wait_ready": False,
        "heartbeat_timeout": 17.0,
        "flight_state": "flight-state",
        "permission_guard": "permission",
        "heartbeat_mode_decoder": "heartbeat-decoder",
        "rc_health_decoder": "rc-decoder",
        "sys_status_observer": "sys-status-decoder",
        "failsafe_decoders": {"HEARTBEAT": "failsafe-decoder"},
        "mission_home_check": "home-check",
        "fc_home_position_tolerance_m": 0.5,
        "fc_home_altitude_tolerance_m": 0.5,
        "clearance_calibration": "clearance",
        "release_stability_config": "stability",
        "home_request_timeout_s": 4.0,
        "telemetry_poll_interval_s": 0.05,
        "guided_output_delivery_callback": harness.guided_delivery_callback,
    }
    assert harness.rclpy.init_options == {
        "signal_handler_options": "no-signal-handlers"
    }
    assert any(
        isinstance(event, tuple)
        and event[:2] == ("status", "mission-command-delivered")
        and event[2]["command"] == "SET_GUIDED"
        and event[2]["sim_timestamp_ns"] == 0
        for event in harness.events
    )
    ready_index = next(
        index
        for index, event in enumerate(harness.events)
        if isinstance(event, tuple) and event[:2] == ("status", "mission-ready")
    )
    guided_index = next(
        index
        for index, event in enumerate(harness.events)
        if isinstance(event, tuple)
        and event[:2] == ("status", "mission-command-delivered")
    )
    assert harness.events.index("telemetry-prepared") < ready_index
    assert ready_index < guided_index
    assert guided_index < harness.events.index("telemetry-verified")
    assert harness.events.index("telemetry-verified") < harness.events.index("arm-output")
    assert harness.vehicle.mode_writes == []
    assert "nested-cleanup" in harness.events
    assert harness.events.index("nested-cleanup") < harness.events.index("destroy-node")
    finished = next(
        index
        for index, event in enumerate(harness.events)
        if isinstance(event, tuple) and event[:2] == ("status", "mission-finished")
    )
    assert harness.events.index("destroy-node") < finished
    assert not any(
        thread.name == "companion-qgc-host-control"
        for thread in threading.enumerate()
    )
    clock = harness.clock_seen[0]
    assert clock.timestamp_ns == 500_000_000
    harness.node.callbacks["/clock"](
        SimpleNamespace(clock=SimpleNamespace(sec=1, nanosec=0))
    )
    assert clock.timestamp_ns == 500_000_000


def test_guarded_host_wires_one_full_payload_adapter_with_attachment_support(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness(heartbeat=0.5)
    projected = projection(harness=harness, full=True)
    install_harness(monkeypatch, harness, projected)
    monkeypatch.setattr(
        runtime_node,
        "create_comp2026_lidar",
        lambda clock: harness.clock_seen.append(clock) or "ros-lidar",
    )

    assert runtime_node._run_comp2026(config(tmp_path)) == 0

    assert harness.factories is not None
    assert harness.factories.supports_attachment is True
    assert harness.payload_ids == [2, 3, 4]


def test_qgc_host_subscribes_to_onboard_images_before_listener_construction(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    harness = HostHarness(heartbeat=0.5)
    install_harness(monkeypatch, harness, projection(harness=harness, full=True))
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: "lidar")

    assert runtime_node._run_comp2026(config(tmp_path)) == 0

    subscription = ("subscribe", "/competition/camera/onboard")
    controller_index = next(
        index
        for index, event in enumerate(harness.events)
        if isinstance(event, tuple) and event[0] == "controller"
    )
    assert harness.events.index(subscription) < controller_index


def test_qgc_host_full_phase_factory_constructs_simulator_camera(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    nested_source = ROOT / "companion/comp2026/src"
    monkeypatch.syspath_prepend(str(nested_source))
    from drone.sensors.camera._camera_manager import CameraManager
    from drone.sensors.camera.camera import Camera

    harness = HostHarness()
    dependencies = harness.dependencies()
    dependencies.CameraManager = CameraManager
    dependencies.Camera = Camera
    selected_config = replace(
        config(tmp_path), scenario_path=ROOT / "config/scenario.yaml"
    )
    clock = runtime_node.SimulationClock()
    host = runtime_node._Comp2026QgcRosHost(
        selected_config,
        projection(harness=harness, full=True),
        clock,
        SimpleNamespace(),
        dependencies,
        runtime_node._QgcStopCoordinator(clock, threading.Event()),
        threading.Event(),
        FakeProtocol(harness.events),
    )
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    host._start_ros()
    harness.node.callbacks["/simulation/run_state"](
        SimpleNamespace(run_id=RUN_ID, state=dependencies.RunState.RUNNING)
    )
    clock.accept(1_000_000_000)
    harness.node.callbacks["/competition/camera/onboard"](
        rgb8_image(1_000_000_000)
    )
    camera = host.camera_factory(
        config=host.runtime_config, lidar=host.lidar, controller=object()
    )
    assert host.frame_source is not None
    assert host.frame_source.ready is True
    observation = camera.cm.capture_observation()
    cleanup = host.close()

    assert type(camera) is Camera
    assert camera.cm.frame_source is host.frame_source
    assert tuple(observation.frame[0, 0]) == (30, 20, 10)
    assert observation.metadata.exposure_timestamp_ns == 1_000_000_000
    assert cleanup.confirmed is True


def test_qgc_host_limited_factory_does_not_require_camera_frames(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    harness = HostHarness(heartbeat=0.5)
    dependencies = harness.dependencies()
    dependencies.CameraManager = lambda **_kwargs: pytest.fail(
        "limited mode constructed a camera manager"
    )
    dependencies.Camera = lambda *_args, **_kwargs: pytest.fail(
        "limited mode constructed a camera"
    )
    install_harness(monkeypatch, harness, projection(harness=harness))
    monkeypatch.setattr(
        runtime_node, "_load_qgc_live_dependencies", lambda: dependencies
    )
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: "lidar")

    assert runtime_node._run_comp2026(config(tmp_path)) == 0
    assert ("subscribe", "/competition/camera/onboard") not in harness.events


def test_qgc_host_cleanup_stops_frame_source_before_destroying_ros_inputs(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    harness = HostHarness()
    clock = runtime_node.SimulationClock()
    host = runtime_node._Comp2026QgcRosHost(
        config(tmp_path),
        projection(harness=harness, full=True),
        clock,
        SimpleNamespace(),
        harness.dependencies(),
        runtime_node._QgcStopCoordinator(clock, threading.Event()),
        threading.Event(),
        FakeProtocol(harness.events),
    )
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())
    host._start_ros()
    source = host.frame_source
    assert source is not None
    original_stop = source.stop

    def record_stop(reason: str) -> None:
        harness.events.append("frame-source-stop")
        original_stop(reason)

    source.stop = record_stop  # type: ignore[method-assign]
    capture_result: list[BaseException] = []
    capture_started = threading.Event()
    capture_finished = threading.Event()

    def capture() -> None:
        capture_started.set()
        try:
            source.capture_frame()
        except BaseException as error:
            capture_result.append(error)
        finally:
            capture_finished.set()

    consumer = threading.Thread(target=capture)
    consumer.start()
    assert capture_started.wait(1.0)
    assert capture_finished.wait(0.05) is False

    first_cleanup = host.close()
    consumer.join(timeout=1.0)
    events_after_first_close = list(harness.events)
    second_cleanup = host.close()

    assert consumer.is_alive() is False
    assert len(capture_result) == 1
    assert isinstance(capture_result[0], StaleSensorError)
    assert harness.events.index("frame-source-stop") < harness.events.index(
        ("executor-shutdown", 1.0)
    )
    assert harness.events.index("frame-source-stop") < harness.events.index(
        ("destroy-subscription", "/competition/camera/onboard")
    )
    assert first_cleanup.confirmed is True
    assert second_cleanup is first_cleanup
    assert harness.events == events_after_first_close


def test_qgc_nested_cleanup_wakes_real_camera_worker_before_parent_ros_cleanup(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    nested_source = ROOT / "companion/comp2026/src"
    monkeypatch.syspath_prepend(str(nested_source))
    from drone.control.listener import LiveListenerRuntime
    from drone.sensors.camera._camera_manager import CameraManager
    from drone.sensors.camera.camera import Camera

    harness = HostHarness()
    dependencies = harness.dependencies()
    dependencies.CameraManager = CameraManager
    dependencies.Camera = Camera
    selected_config = replace(
        config(tmp_path), scenario_path=ROOT / "config/scenario.yaml"
    )
    clock = runtime_node.SimulationClock()
    host = runtime_node._Comp2026QgcRosHost(
        selected_config,
        projection(harness=harness, full=True),
        clock,
        SimpleNamespace(),
        dependencies,
        runtime_node._QgcStopCoordinator(clock, threading.Event()),
        threading.Event(),
        FakeProtocol(harness.events),
    )
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())
    host._start_ros()
    harness.node.callbacks["/simulation/run_state"](
        SimpleNamespace(run_id=RUN_ID, state=dependencies.RunState.RUNNING)
    )
    clock.accept(1_000_000_000)
    source = host.frame_source
    assert source is not None
    original_capture = source.capture_frame
    capture_calls = 0
    second_capture_entered = threading.Event()

    def observed_capture(quality: int = 4, deadline_sim_ns: int | None = None):
        nonlocal capture_calls
        capture_calls += 1
        if capture_calls == 2:
            second_capture_entered.set()
        return original_capture(quality=quality, deadline_sim_ns=deadline_sim_ns)

    source.capture_frame = observed_capture  # type: ignore[method-assign]
    harness.node.callbacks["/competition/camera/onboard"](
        rgb8_image(1_000_000_000)
    )
    camera = host.camera_factory()
    first = camera.cm.capture_observation_bounded(timeout_s=1.0)
    assert first.metadata.exposure_timestamp_ns == 1_000_000_000
    assert second_capture_entered.wait(1.0)

    nested_runtime = LiveListenerRuntime(
        listener=None,
        owner=None,
        supervisor=None,
        controller=SimpleNamespace(vehicle=SimpleNamespace(close=lambda: None)),
        flight_state=None,
        tracker=None,
        lidar=SimpleNamespace(
            stop=lambda *, timeout_seconds: SimpleNamespace(
                worker_stopped=True, cleanup_completed=True
            )
        ),
        camera=camera,
        dropper=SimpleNamespace(cleanup_passive=lambda _timeout: True),
        cleanup_timeout_s=0.05,
        diagnostics=lambda _message: None,
        monitoring_stop=threading.Event(),
        manage_signals=False,
    )

    nested_cleanup = nested_runtime.close()
    assert ("executor-shutdown", 1.0) not in harness.events
    parent_cleanup = host.close()
    camera.cm._acquisition_thread.join(1.0)

    assert nested_cleanup.camera_stopped is True
    assert (
        runtime_node._qgc_cleanup_failure(
            SimpleNamespace(cleanup_report=nested_cleanup)
        )
        is None
    )
    assert camera.cm._acquisition_thread.is_alive() is False
    assert parent_cleanup.confirmed is True
    assert host.close() is parent_cleanup


def test_guarded_host_publishes_exact_qgc_phase_prefix_and_stops_emitter_before_node(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    harness.publish_phase_events = True
    harness.phase_event_timestamp = (2, 345_678_901)
    install_harness(monkeypatch, harness, projection(harness=harness))
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: "lidar")

    assert runtime_node._run_comp2026(config(tmp_path)) == 0

    publisher_event = next(
        event
        for event in harness.events
        if isinstance(event, tuple) and event[:2] == ("publisher", "/simulation/mission_events")
    )
    assert publisher_event[2].__name__ == "MissionEvent"
    assert vars(publisher_event[3]) == {
        "depth": 100,
        "reliability": "reliable",
        "durability": "transient",
    }
    messages = [
        event[2]
        for event in harness.events
        if isinstance(event, tuple) and event[:2] == ("publish", "/simulation/mission_events")
    ]
    assert [
        (
            message.run_id,
            message.sim_timestamp.sec * 1_000_000_000 + message.sim_timestamp.nanosec,
            message.event_id,
            message.phase,
            message.state,
            message.detail,
        )
        for message in messages
    ] == [
        (RUN_ID, 2_345_678_901, 0, "FM1", "STARTED", "automatic attempt"),
        (RUN_ID, 2_345_678_901, 1, "FM1", "COMPLETE", "automatic attempt"),
        (RUN_ID, 2_345_678_901, 2, "FM2", "STARTED", "automatic attempt"),
        (RUN_ID, 2_345_678_901, 3, "FM2", "COMPLETE", "automatic attempt"),
    ]
    assert callable(harness.phase_observer)
    with pytest.raises(RuntimeError, match="stopped"):
        harness.phase_observer("HOME", "COMPLETE")
    destroy_publisher = harness.events.index(
        ("destroy-publisher", "/simulation/mission_events")
    )
    acknowledged = next(
        index
        for index, event in enumerate(harness.events)
        if isinstance(event, tuple) and event[:2] == ("publisher-acked", "/simulation/mission_events")
    )
    assert acknowledged < destroy_publisher
    assert destroy_publisher < harness.events.index("destroy-node")
    finished = next(
        index
        for index, event in enumerate(harness.events)
        if isinstance(event, tuple) and event[:2] == ("status", "mission-finished")
    )
    assert acknowledged < finished


def test_mission_event_publication_failure_fails_host_without_retry(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    harness.publish_phase_events = True
    harness.node.publish_error = RuntimeError("DDS writer failed")
    install_harness(monkeypatch, harness, projection(harness=harness))
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: "lidar")

    assert runtime_node._run_comp2026(config(tmp_path)) == 1

    attempts = [
        event
        for event in harness.events
        if isinstance(event, tuple) and event[:2] == ("publish", "/simulation/mission_events")
    ]
    assert len(attempts) == 1
    assert not any(
        isinstance(event, tuple) and event[:2] == ("status", "mission-finished")
        for event in harness.events
    )


def test_complete_publication_failure_keeps_phase_truth_separate_from_host_failure(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    harness.publish_phase_events = True
    harness.node.publish_error = RuntimeError("DDS COMPLETE failed")
    harness.node.publish_error_state = "COMPLETE"
    install_harness(monkeypatch, harness, projection(harness=harness))
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: "lidar")

    assert runtime_node._run_comp2026(config(tmp_path)) == 1

    attempts = [
        event[2]
        for event in harness.events
        if isinstance(event, tuple) and event[:2] == ("publish", "/simulation/mission_events")
    ]
    assert [(message.event_id, message.phase, message.state) for message in attempts] == [
        (0, "FM1", "STARTED"),
        (1, "FM1", "COMPLETE"),
    ]
    failure = next(
        event[2]
        for event in harness.events
        if isinstance(event, tuple) and event[:2] == ("status", "runtime-failure")
    )
    assert "mission event publication failed" in failure["reason"]


@pytest.mark.parametrize("failure_type", [KeyboardInterrupt, SystemExit])
def test_empty_control_flow_publication_error_retains_type_in_host_failure(
    tmp_path, monkeypatch, failure_type
) -> None:
    harness = HostHarness()
    harness.publish_phase_events = True
    harness.node.publish_error = failure_type()
    install_harness(monkeypatch, harness, projection(harness=harness))
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: "lidar")

    assert runtime_node._run_comp2026(config(tmp_path)) == 1

    failure = next(
        event[2]
        for event in harness.events
        if isinstance(event, tuple) and event[:2] == ("status", "runtime-failure")
    )
    assert failure["reason"] == f"mission event publication failed: {failure_type.__name__}"


def test_unconfirmed_mission_event_delivery_prevents_mission_finished(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    harness.publish_phase_events = True
    harness.node.publisher_acked = False
    install_harness(monkeypatch, harness, projection(harness=harness))
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: "lidar")

    assert runtime_node._run_comp2026(config(tmp_path)) == 1

    assert not any(
        isinstance(event, tuple) and event[:2] == ("status", "mission-finished")
        for event in harness.events
    )
    failure = next(
        event[2]
        for event in harness.events
        if isinstance(event, tuple) and event[:2] == ("status", "runtime-failure")
    )
    assert "delivery was not acknowledged" in failure["reason"]


@pytest.mark.parametrize(
    "subscribers",
    [
        [],
        [FakeNode.subscription_endpoint("drone_sim_scorekeeper")],
        [FakeNode.subscription_endpoint("rosbag2_recorder_deadbeef")],
        [
            FakeNode.subscription_endpoint("arbitrary_one"),
            FakeNode.subscription_endpoint("arbitrary_two"),
        ],
        [
            FakeNode.subscription_endpoint(
                "drone_sim_scorekeeper", reliability="best-effort"
            ),
            FakeNode.subscription_endpoint("rosbag2_recorder_deadbeef"),
        ],
        [
            FakeNode.subscription_endpoint("drone_sim_scorekeeper"),
            FakeNode.subscription_endpoint(
                "rosbag2_recorder_deadbeef", durability="volatile"
            ),
        ],
        [
            FakeNode.subscription_endpoint("drone_sim_scorekeeper"),
            FakeNode.subscription_endpoint(
                "rosbag2_recorder_deadbeef", topic_type="std_msgs/msg/String"
            ),
        ],
        [
            FakeNode.subscription_endpoint(
                "drone_sim_scorekeeper", node_namespace="/unrelated"
            ),
            FakeNode.subscription_endpoint("rosbag2_recorder_deadbeef"),
        ],
    ],
)
def test_missing_required_mission_event_identity_keeps_admission_closed(
    tmp_path, monkeypatch, subscribers
) -> None:
    harness = HostHarness()
    harness.node.publisher_subscribers = subscribers
    install_harness(monkeypatch, harness, projection(harness=harness))
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: "lidar")

    assert runtime_node._run_comp2026(config(tmp_path)) == 1

    with pytest.raises(RuntimeError, match="admission is closed"):
        harness.admission()
    assert not any(
        isinstance(event, tuple) and event[0] == "publish"
        for event in harness.events
    )


@pytest.mark.parametrize("missing", ["scorekeeper", "rosbag"])
def test_required_identity_disconnect_blocks_publication_after_admission(
    tmp_path, monkeypatch, missing
) -> None:
    harness = HostHarness()
    harness.publish_phase_events = True
    harness.drop_subscriber_before_phase_events = True
    retained = (
        "rosbag2_recorder_deadbeef"
        if missing == "scorekeeper"
        else "drone_sim_scorekeeper"
    )
    harness.remaining_subscribers = [
        FakeNode.subscription_endpoint(retained),
        FakeNode.subscription_endpoint("arbitrary_replacement"),
    ]
    install_harness(monkeypatch, harness, projection(harness=harness))
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: "lidar")

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert not any(
        isinstance(event, tuple) and event[0] == "publish"
        for event in harness.events
    )


@pytest.mark.parametrize("missing", ["scorekeeper", "rosbag"])
def test_required_identity_disconnect_makes_ack_flush_fail_closed(
    tmp_path, monkeypatch, missing
) -> None:
    harness = HostHarness()
    harness.publish_phase_events = True
    harness.drop_subscriber_after_phase_events = True
    retained = (
        "rosbag2_recorder_deadbeef"
        if missing == "scorekeeper"
        else "drone_sim_scorekeeper"
    )
    harness.remaining_subscribers = [
        FakeNode.subscription_endpoint(retained),
        FakeNode.subscription_endpoint("arbitrary_replacement"),
    ]
    install_harness(monkeypatch, harness, projection(harness=harness))
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: "lidar")

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert not any(
        isinstance(event, tuple) and event[0] == "publisher-acked"
        for event in harness.events
    )
    assert not any(
        isinstance(event, tuple) and event[:2] == ("status", "mission-finished")
        for event in harness.events
    )


@pytest.mark.parametrize("missing", ["scorekeeper", "rosbag"])
@pytest.mark.parametrize("substitute", [False, True])
def test_required_identity_loss_during_ack_wait_fails_final_confirmation(
    tmp_path, monkeypatch, missing, substitute
) -> None:
    harness = HostHarness()
    harness.publish_phase_events = True
    retained = (
        "rosbag2_recorder_deadbeef"
        if missing == "scorekeeper"
        else "drone_sim_scorekeeper"
    )

    def lose_required_consumer() -> None:
        harness.node.publisher_subscribers = [
            FakeNode.subscription_endpoint(retained)
        ]
        if substitute:
            harness.node.publisher_subscribers.append(
                FakeNode.subscription_endpoint("arbitrary_replacement")
            )

    harness.node.publisher_ack_callback = lose_required_consumer
    install_harness(monkeypatch, harness, projection(harness=harness))
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: "lidar")

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert any(
        isinstance(event, tuple)
        and event[:2] == ("publisher-acked", "/simulation/mission_events")
        for event in harness.events
    )
    assert not any(
        isinstance(event, tuple) and event[:2] == ("status", "mission-finished")
        for event in harness.events
    )
    failure = next(
        event[2]
        for event in harness.events
        if isinstance(event, tuple) and event[:2] == ("status", "runtime-failure")
    )
    assert "after acknowledgement" in failure["reason"]


def test_late_range_after_nested_lidar_stop_is_silent_and_keeps_success(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    harness.stop_lidar_then_send_late_range = True
    protocol = install_harness(monkeypatch, harness, projection(harness=harness))
    monkeypatch.setattr(
        runtime_node, "create_comp2026_lidar", lambda _clock: object()
    )

    assert runtime_node._run_comp2026(config(tmp_path)) == 0
    assert "lidar-stop" in harness.events
    assert any(
        isinstance(event, tuple) and event[:2] == ("status", "mission-finished")
        for event in protocol.events
    )


def test_inflight_bad_range_before_nested_lidar_stop_remains_fatal(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    harness.send_bad_range_before_lidar_stop = True
    protocol = install_harness(monkeypatch, harness, projection(harness=harness))
    monkeypatch.setattr(
        runtime_node, "create_comp2026_lidar", lambda _clock: object()
    )

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    failure = next(
        event[2]
        for event in protocol.events
        if isinstance(event, tuple) and event[:2] == ("status", "runtime-failure")
    )
    assert failure["reason"].startswith("range input failed:")


def test_guarded_host_contains_no_legacy_automatic_command_writer() -> None:
    source = Path(runtime_node.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    run_host = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "_run_comp2026"
    )
    called_names = {
        node.func.id
        for node in ast.walk(run_host)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }
    mode_writes = [
        node
        for node in ast.walk(run_host)
        if isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign))
        and any(
            isinstance(target, ast.Attribute) and target.attr == "mode"
            for target in (
                node.targets if isinstance(node, ast.Assign) else [node.target]
            )
        )
    ]

    assert "run_auto_attempt" not in called_names
    assert "VehicleMode" not in called_names
    assert mode_writes == []


def test_admission_without_guided_transport_output_publishes_no_delivery_status(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    harness.deliver_guided = False
    protocol = install_harness(monkeypatch, harness, projection(harness=harness))
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    assert runtime_node._run_comp2026(config(tmp_path)) == 0
    assert not any(
        isinstance(event, tuple)
        and event[:2] == ("status", "mission-command-delivered")
        for event in protocol.events
    )
    assert "telemetry-prepared" in harness.events
    assert "telemetry-verified" not in harness.events
    assert "arm-output" not in harness.events
    assert harness.events.count("telemetry-close") == 1


@pytest.mark.parametrize(
    ("heartbeat", "armable"),
    [(None, True), (float("nan"), True), (0.500_001, True), (0.1, False)],
)
def test_mission_ready_requires_actual_fresh_heartbeat_and_literal_armable(
    tmp_path, monkeypatch, heartbeat, armable
) -> None:
    harness = HostHarness(heartbeat=heartbeat, armable=armable)
    install_harness(monkeypatch, harness, projection(harness=harness))
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert "rclpy-init" in harness.events
    assert any(
        isinstance(event, tuple) and event[:2] == ("status", "runtime-failure")
        for event in harness.events
    )
    assert "listener-run" not in harness.events
    assert not any(
        isinstance(event, tuple) and event[:2] == ("status", "mission-ready")
        for event in harness.events
    )
    assert "closed-admission" in harness.events


def test_listener_ready_requires_matching_running_state_and_public_clock(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    dependencies = harness.dependencies()

    def start_without_public_time(
        _artifacts: object,
        config_value: object,
        *,
        factories: FakeFactories,
        on_listener_ready: object,
        **_kwargs: object,
    ) -> object:
        controller = factories.controller_factory(
            config=config_value,
            flight_state="flight-state",
            permission_guard="permission",
            decoders=SimpleNamespace(
                heartbeat_mode_decoder="heartbeat-decoder",
                rc_health_decoder="rc-decoder",
                observe_sys_status="sys-status-decoder",
                heartbeat_failsafe_decoder="failsafe-decoder",
            ),
        )
        on_listener_ready(SimpleNamespace(controller=controller))
        return SimpleNamespace(
            mission_result="SUCCEEDED",
            recovery_outcome="HOME_LANDED",
            monitoring_exit_reason="NOT_REQUIRED",
            cleanup_report=Cleanup(),
        )

    dependencies.start_repl = start_without_public_time
    protocol = FakeProtocol(harness.events)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(
        runtime_node,
        "project_qgc_runtime",
        lambda *_args, **_kwargs: projection(harness=harness),
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_live_dependencies", lambda: dependencies
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_timebase", lambda: harness.timebase
    )
    monkeypatch.setattr(runtime_node.signal, "signal", lambda *_args: None)
    monkeypatch.setattr(runtime_node.signal, "getsignal", lambda *_args: None)
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert not any(
        isinstance(event, tuple) and event[:2] == ("status", "mission-ready")
        for event in harness.events
    )


def test_finalization_before_ready_stops_startup_without_connecting(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    dependencies = harness.dependencies()
    original_node = dependencies.Node

    class FinalizingNode:
        def __new__(cls, *args: object, **kwargs: object) -> FakeNode:
            node = original_node(*args, **kwargs)
            original_create = node.create_subscription

            def create_subscription(*call_args: object, **call_kwargs: object) -> object:
                subscription = original_create(*call_args, **call_kwargs)
                if call_args[1] == "/simulation/run_state":
                    call_args[2](SimpleNamespace(run_id=RUN_ID, state=3))
                return subscription

            node.create_subscription = create_subscription  # type: ignore[method-assign]
            return node

    dependencies.Node = FinalizingNode
    protocol = FakeProtocol(harness.events)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(runtime_node, "project_qgc_runtime", lambda *_args, **_kwargs: projection(harness=harness), raising=False)
    monkeypatch.setattr(runtime_node, "_load_qgc_live_dependencies", lambda: dependencies, raising=False)
    monkeypatch.setattr(runtime_node, "_load_qgc_timebase", lambda: harness.timebase, raising=False)
    monkeypatch.setattr(runtime_node.signal, "signal", lambda *_args: None)
    monkeypatch.setattr(runtime_node.signal, "getsignal", lambda *_args: None)

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert not any(
        isinstance(event, tuple) and event[0] == "controller"
        for event in harness.events
    )


def test_durable_finalize_request_before_ready_prevents_vehicle_connection(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    protocol = install_harness(
        monkeypatch, harness, projection(harness=harness)
    )
    protocol.finalize_requested.set()

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert not any(
        isinstance(event, tuple) and event[0] == "controller"
        for event in harness.events
    )


def test_outer_wall_deadline_stops_startup_before_vehicle_connection(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    dependencies = harness.dependencies()
    wall_reads = iter((0.0, 2.0))
    dependencies.wall_now = lambda: next(wall_reads, 2.0)
    protocol = FakeProtocol(harness.events)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(
        runtime_node,
        "project_qgc_runtime",
        lambda *_args, **_kwargs: projection(harness=harness),
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_live_dependencies", lambda: dependencies
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_timebase", lambda: harness.timebase
    )
    monkeypatch.setattr(runtime_node.signal, "signal", lambda *_args: None)
    monkeypatch.setattr(runtime_node.signal, "getsignal", lambda *_args: None)
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    bounded = replace(config(tmp_path), max_wall_seconds=1.0)
    assert runtime_node._run_comp2026(bounded) == 1
    assert not any(
        isinstance(event, tuple) and event[0] == "controller"
        for event in harness.events
    )


def test_ros_executor_return_before_ready_prevents_vehicle_connection(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    harness.executor.spin = lambda: None
    dependencies = harness.dependencies()

    class ImmediateThread:
        def __init__(self, *, target: object, **_kwargs: object) -> None:
            self.target = target

        def start(self) -> None:
            self.target()

        def join(self, _timeout: float) -> None:
            pass

        def is_alive(self) -> bool:
            return False

    dependencies.thread_factory = ImmediateThread
    protocol = FakeProtocol(harness.events)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(
        runtime_node,
        "project_qgc_runtime",
        lambda *_args, **_kwargs: projection(harness=harness),
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_live_dependencies", lambda: dependencies
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_timebase", lambda: harness.timebase
    )
    monkeypatch.setattr(runtime_node.signal, "signal", lambda *_args: None)
    monkeypatch.setattr(runtime_node.signal, "getsignal", lambda *_args: None)
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert not any(
        isinstance(event, tuple) and event[0] == "controller"
        for event in harness.events
    )


def test_ros_executor_failure_after_ready_requests_nested_abort(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    dependencies = harness.dependencies()
    original_start = dependencies.start_repl

    def start_and_stop_executor(*args: object, **kwargs: object) -> object:
        original_ready = kwargs["on_listener_ready"]

        def ready_then_stop(runtime: object) -> None:
            original_ready(runtime)
            harness.executor.release.set()
            deadline = threading.Event()
            for _ in range(100):
                if any(
                    isinstance(event, tuple) and event[0] == "abort"
                    for event in harness.events
                ):
                    return
                deadline.wait(0.001)

        kwargs["on_listener_ready"] = ready_then_stop
        return original_start(*args, **kwargs)

    dependencies.start_repl = start_and_stop_executor
    protocol = FakeProtocol(harness.events)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(
        runtime_node,
        "project_qgc_runtime",
        lambda *_args, **_kwargs: projection(harness=harness),
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_live_dependencies", lambda: dependencies
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_timebase", lambda: harness.timebase
    )
    monkeypatch.setattr(runtime_node.signal, "signal", lambda *_args: None)
    monkeypatch.setattr(runtime_node.signal, "getsignal", lambda *_args: None)
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert any(
        isinstance(event, tuple)
        and event[0] == "abort"
        and "ROS executor stopped" in event[1]
        for event in harness.events
    )


def test_sigterm_before_ready_prevents_vehicle_connection(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    dependencies = harness.dependencies()
    handlers: dict[int, object] = {}
    protocol = FakeProtocol(harness.events)

    def project_after_sigterm(*_args: object, **_kwargs: object) -> object:
        handlers[runtime_node.signal.SIGTERM](runtime_node.signal.SIGTERM, None)
        return projection(harness=harness)

    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(runtime_node, "project_qgc_runtime", project_after_sigterm)
    monkeypatch.setattr(
        runtime_node, "_load_qgc_live_dependencies", lambda: dependencies
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_timebase", lambda: harness.timebase
    )
    monkeypatch.setattr(
        runtime_node.signal,
        "signal",
        lambda signum, handler: handlers.__setitem__(signum, handler),
    )
    monkeypatch.setattr(runtime_node.signal, "getsignal", lambda *_args: None)

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert not any(
        isinstance(event, tuple) and event[0] == "controller"
        for event in harness.events
    )


def test_sigterm_after_ready_aborts_and_cannot_publish_success(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    dependencies = harness.dependencies()
    original_start = dependencies.start_repl
    handlers: dict[int, object] = {}

    def start_and_terminate(*args: object, **kwargs: object) -> object:
        original_ready = kwargs["on_listener_ready"]

        def ready_then_terminate(runtime: object) -> None:
            original_ready(runtime)
            handlers[runtime_node.signal.SIGTERM](runtime_node.signal.SIGTERM, None)

        kwargs["on_listener_ready"] = ready_then_terminate
        return original_start(*args, **kwargs)

    dependencies.start_repl = start_and_terminate
    protocol = FakeProtocol(harness.events)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(
        runtime_node,
        "project_qgc_runtime",
        lambda *_args, **_kwargs: projection(harness=harness),
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_live_dependencies", lambda: dependencies
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_timebase", lambda: harness.timebase
    )
    monkeypatch.setattr(
        runtime_node.signal,
        "signal",
        lambda signum, handler: handlers.__setitem__(signum, handler),
    )
    monkeypatch.setattr(runtime_node.signal, "getsignal", lambda *_args: None)
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert ("abort", "host termination signal") in harness.events
    assert not any(
        isinstance(event, tuple) and event[:2] == ("status", "mission-finished")
        for event in harness.events
    )


def test_finalization_after_ready_requests_nested_abort(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    dependencies = harness.dependencies()
    original_start = dependencies.start_repl

    def start_and_finalize(*args: object, **kwargs: object) -> object:
        result = original_start(*args, **kwargs)
        callback = harness.node.callbacks["/simulation/run_state"]
        callback(SimpleNamespace(run_id=RUN_ID, state=3))
        return SimpleNamespace(
            mission_result="ABORTED",
            recovery_outcome="UNCONFIRMED",
            monitoring_exit_reason="EXPLICIT_STOP_UNCONFIRMED",
            cleanup_report=result.cleanup_report,
        )

    dependencies.start_repl = start_and_finalize
    protocol = FakeProtocol(harness.events)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(runtime_node, "project_qgc_runtime", lambda *_args, **_kwargs: projection(harness=harness), raising=False)
    monkeypatch.setattr(runtime_node, "_load_qgc_live_dependencies", lambda: dependencies, raising=False)
    monkeypatch.setattr(runtime_node, "_load_qgc_timebase", lambda: harness.timebase, raising=False)
    monkeypatch.setattr(runtime_node.signal, "signal", lambda *_args: None)
    monkeypatch.setattr(runtime_node.signal, "getsignal", lambda *_args: None)
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert ("abort", "orchestration finalization") in harness.events
    assert not any(
        isinstance(event, tuple) and event[:2] == ("status", "mission-finished")
        for event in harness.events
    )


def test_durable_finalize_request_during_listener_run_requests_abort(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    dependencies = harness.dependencies()
    original_start = dependencies.start_repl
    protocol = FakeProtocol(harness.events)

    def start_and_request_finalize(*args: object, **kwargs: object) -> object:
        result = original_start(*args, **kwargs)
        protocol.finalize_requested.set()
        deadline = threading.Event()
        for _ in range(100):
            if any(
                isinstance(event, tuple) and event[0] == "abort"
                for event in harness.events
            ):
                break
            deadline.wait(0.001)
        return SimpleNamespace(
            mission_result="ABORTED",
            recovery_outcome="UNCONFIRMED",
            monitoring_exit_reason="EXPLICIT_STOP_UNCONFIRMED",
            cleanup_report=result.cleanup_report,
        )

    dependencies.start_repl = start_and_request_finalize
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(
        runtime_node,
        "project_qgc_runtime",
        lambda *_args, **_kwargs: projection(harness=harness),
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_live_dependencies", lambda: dependencies
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_timebase", lambda: harness.timebase
    )
    monkeypatch.setattr(runtime_node.signal, "signal", lambda *_args: None)
    monkeypatch.setattr(runtime_node.signal, "getsignal", lambda *_args: None)
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert ("abort", "orchestration finalize request") in harness.events


def test_primary_failure_keeps_secondary_host_cleanup_diagnostic(
    tmp_path, monkeypatch, capsys
) -> None:
    harness = HostHarness()
    dependencies = harness.dependencies()

    def fail_after_controller(
        _artifacts: object,
        config_value: object,
        *,
        factories: FakeFactories,
        **_kwargs: object,
    ) -> object:
        factories.controller_factory(
            config=config_value,
            flight_state="flight-state",
            permission_guard="permission",
            decoders=SimpleNamespace(
                heartbeat_mode_decoder="heartbeat-decoder",
                rc_health_decoder="rc-decoder",
                observe_sys_status="sys-status-decoder",
                heartbeat_failsafe_decoder="failsafe-decoder",
            ),
        )
        raise RuntimeError("primary listener failure")

    dependencies.start_repl = fail_after_controller
    harness.executor.shutdown = lambda **_kwargs: (_ for _ in ()).throw(
        RuntimeError("secondary executor cleanup failure")
    )
    protocol = FakeProtocol(harness.events)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(
        runtime_node,
        "project_qgc_runtime",
        lambda *_args, **_kwargs: projection(harness=harness),
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_live_dependencies", lambda: dependencies
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_timebase", lambda: harness.timebase
    )
    monkeypatch.setattr(runtime_node.signal, "signal", lambda *_args: None)
    monkeypatch.setattr(runtime_node.signal, "getsignal", lambda *_args: None)
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    failure = next(
        event[2]
        for event in harness.events
        if isinstance(event, tuple) and event[:2] == ("status", "runtime-failure")
    )
    assert failure["reason"] == "primary listener failure"
    assert "secondary executor cleanup failure" in capsys.readouterr().out


@pytest.mark.parametrize("secondary_error", [KeyboardInterrupt(), SystemExit()])
def test_host_cleanup_preserves_nonempty_control_flow_diagnostic(
    tmp_path, monkeypatch, capsys, secondary_error
) -> None:
    harness = HostHarness()
    harness.node.publisher_ack_error = RuntimeError("primary publisher cleanup failure")
    harness.executor.shutdown = lambda **_kwargs: (_ for _ in ()).throw(
        secondary_error
    )
    install_harness(monkeypatch, harness, projection(harness=harness))
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    assert runtime_node._run_comp2026(config(tmp_path)) == 1

    failure = next(
        event[2]
        for event in harness.events
        if isinstance(event, tuple) and event[:2] == ("status", "runtime-failure")
    )
    assert failure["reason"] == "primary publisher cleanup failure"
    assert type(secondary_error).__name__ in capsys.readouterr().out


def test_unconfirmed_terminal_monitoring_is_not_published_as_mission_success(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    dependencies = harness.dependencies()
    original_start = dependencies.start_repl

    def start_with_unconfirmed_monitoring(*args: object, **kwargs: object) -> object:
        result = original_start(*args, **kwargs)
        return SimpleNamespace(
            mission_result="SUCCEEDED",
            recovery_outcome="UNCONFIRMED",
            monitoring_exit_reason="EXPLICIT_STOP_UNCONFIRMED",
            cleanup_report=result.cleanup_report,
        )

    dependencies.start_repl = start_with_unconfirmed_monitoring
    protocol = FakeProtocol(harness.events)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(
        runtime_node,
        "project_qgc_runtime",
        lambda *_args, **_kwargs: projection(harness=harness),
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_live_dependencies", lambda: dependencies
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_timebase", lambda: harness.timebase
    )
    monkeypatch.setattr(runtime_node.signal, "signal", lambda *_args: None)
    monkeypatch.setattr(runtime_node.signal, "getsignal", lambda *_args: None)
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert not any(
        isinstance(event, tuple) and event[:2] == ("status", "mission-finished")
        for event in harness.events
    )


def test_listener_return_without_result_is_not_published_as_mission_success(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    dependencies = harness.dependencies()
    dependencies.start_repl = lambda *_args, **_kwargs: None
    protocol = FakeProtocol(harness.events)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(
        runtime_node,
        "project_qgc_runtime",
        lambda *_args, **_kwargs: projection(harness=harness),
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_live_dependencies", lambda: dependencies
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_timebase", lambda: harness.timebase
    )
    monkeypatch.setattr(runtime_node.signal, "signal", lambda *_args: None)
    monkeypatch.setattr(runtime_node.signal, "getsignal", lambda *_args: None)

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert not any(
        isinstance(event, tuple) and event[:2] == ("status", "mission-finished")
        for event in harness.events
    )


def test_signal_handler_does_not_touch_live_runtime_event_wait() -> None:
    script = textwrap.dedent(
        """
        import os
        import signal
        import sys
        import threading

        sys.path.insert(0, "companion/comp2026/src")

        from drone.control.listener import LiveListenerRuntime
        from drone_sim_companion.runtime_node import _QgcSignalLatch

        runtime = object.__new__(LiveListenerRuntime)
        object.__setattr__(runtime, "_monitoring_stop", threading.Event())
        latch = _QgcSignalLatch()
        signal.signal(signal.SIGUSR1, lambda *_args: latch.latch("signal"))
        with runtime._monitoring_stop._cond:
            os.kill(os.getpid(), signal.SIGUSR1)
        assert latch.reason == "signal"
        assert not runtime._monitoring_stop.is_set()
        """
    )
    try:
        completed = subprocess.run(
            [sys.executable, "-c", script],
            check=False,
            capture_output=True,
            text=True,
            timeout=3.0,
        )
    except subprocess.TimeoutExpired:
        pytest.fail("signal handler touched the runtime Event wait path")
    assert completed.returncode == 0, completed.stderr


def test_repeated_prebind_signals_are_latched_and_dispatched_once(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    handlers: dict[int, object] = {}
    projection_entered = threading.Event()

    def project_after_repeated_signals(*_args: object, **_kwargs: object) -> object:
        projection_entered.set()
        handlers[runtime_node.signal.SIGTERM](runtime_node.signal.SIGTERM, None)
        handlers[runtime_node.signal.SIGINT](runtime_node.signal.SIGINT, None)
        return projection(harness=harness)

    monkeypatch.setattr(
        runtime_node, "_ProductionProtocol", lambda _config: FakeProtocol(harness.events)
    )
    monkeypatch.setattr(runtime_node, "project_qgc_runtime", project_after_repeated_signals)
    monkeypatch.setattr(runtime_node, "_load_qgc_live_dependencies", harness.dependencies)
    monkeypatch.setattr(runtime_node, "_load_qgc_timebase", lambda: harness.timebase)
    monkeypatch.setattr(
        runtime_node.signal,
        "signal",
        lambda signum, handler: handlers.__setitem__(signum, handler),
    )
    monkeypatch.setattr(runtime_node.signal, "getsignal", lambda *_args: None)

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert projection_entered.is_set()
    assert not any(
        isinstance(event, tuple) and event[0] == "controller"
        for event in harness.events
    )
    assert [event for event in harness.events if isinstance(event, tuple) and event[0] == "abort"] == []


@pytest.mark.parametrize("trigger", ["finalize", "wall"])
def test_host_control_stops_terminal_monitoring_without_repeating_abort(
    tmp_path, monkeypatch, trigger
) -> None:
    harness = HostHarness()
    dependencies = harness.dependencies()
    protocol = FakeProtocol(harness.events)
    original_start = dependencies.start_repl
    wall_expired = threading.Event()
    dependencies.wall_now = lambda: 2.0 if wall_expired.is_set() else 0.0

    def start_then_trigger(*args: object, **kwargs: object) -> object:
        result = original_start(*args, **kwargs)
        if trigger == "finalize":
            protocol.finalize_requested.set()
            reason = "orchestration finalize request"
        else:
            wall_expired.set()
            reason = "companion exceeded the overall run wall failsafe"
        deadline = threading.Event()
        for _ in range(200):
            if "stop-monitoring" in harness.events:
                break
            deadline.wait(0.001)
        assert [event for event in harness.events if event == ("abort", reason)] == [
            ("abort", reason)
        ]
        assert harness.events.count("stop-monitoring") == 1
        return SimpleNamespace(
            mission_result="ABORTED",
            recovery_outcome="UNCONFIRMED",
            monitoring_exit_reason="EXPLICIT_STOP_UNCONFIRMED",
            cleanup_report=result.cleanup_report,
        )

    dependencies.start_repl = start_then_trigger
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(
        runtime_node,
        "project_qgc_runtime",
        lambda *_args, **_kwargs: projection(harness=harness),
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_live_dependencies", lambda: dependencies
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_timebase", lambda: harness.timebase
    )
    monkeypatch.setattr(runtime_node.signal, "signal", lambda *_args: None)
    monkeypatch.setattr(runtime_node.signal, "getsignal", lambda *_args: None)
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    bounded = replace(config(tmp_path), max_wall_seconds=1.0)
    assert runtime_node._run_comp2026(bounded) == 1
    assert not any(
        thread.name == "companion-qgc-host-control"
        for thread in threading.enumerate()
    )


def test_nested_unconfirmed_cleanup_withholds_quiescence(tmp_path, monkeypatch) -> None:
    harness = HostHarness()
    dependencies = harness.dependencies()
    original_start = dependencies.start_repl

    def incomplete_cleanup(*args: object, **kwargs: object) -> object:
        result = original_start(*args, **kwargs)
        return SimpleNamespace(
            mission_result=result.mission_result,
            recovery_outcome=result.recovery_outcome,
            monitoring_exit_reason=result.monitoring_exit_reason,
            cleanup_report=Cleanup(vehicle_closed=False),
        )

    dependencies.start_repl = incomplete_cleanup
    protocol = install_harness(monkeypatch, harness, projection(harness=harness))
    monkeypatch.setattr(
        runtime_node, "_load_qgc_live_dependencies", lambda: dependencies
    )
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert ("quiescence", "companion") not in protocol.events


def test_surviving_executor_thread_withholds_quiescence(tmp_path, monkeypatch) -> None:
    harness = HostHarness()
    dependencies = harness.dependencies()
    harness.executor.shutdown = lambda **_kwargs: True
    projected = projection(harness=harness)
    projected.runtime_configuration.cleanup_timeout_s = 0.01
    protocol = install_harness(monkeypatch, harness, projected)
    monkeypatch.setattr(
        runtime_node, "_load_qgc_live_dependencies", lambda: dependencies
    )
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert ("quiescence", "companion") not in protocol.events
    harness.executor.release.set()


def test_surviving_control_thread_withholds_quiescence(tmp_path, monkeypatch) -> None:
    harness = HostHarness()
    dependencies = harness.dependencies()
    second_read_entered = threading.Event()
    release_control = threading.Event()

    class BlockingProtocol(FakeProtocol):
        def __init__(self, events: list[object]) -> None:
            super().__init__(events)
            self.reads = 0

        def read_finalize_request(self) -> object | None:
            self.reads += 1
            if self.reads > 1:
                second_read_entered.set()
                release_control.wait(2.0)
            return None

    original_start = dependencies.start_repl

    def run_while_control_blocks(*args: object, **kwargs: object) -> object:
        result = original_start(*args, **kwargs)
        assert second_read_entered.wait(0.5)
        return result

    dependencies.start_repl = run_while_control_blocks
    projected = projection(harness=harness)
    projected.runtime_configuration.cleanup_timeout_s = 0.01
    protocol = BlockingProtocol(harness.events)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(
        runtime_node,
        "project_qgc_runtime",
        lambda *_args, **_kwargs: projected,
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_live_dependencies", lambda: dependencies
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_timebase", lambda: harness.timebase
    )
    monkeypatch.setattr(runtime_node.signal, "signal", lambda *_args: None)
    monkeypatch.setattr(runtime_node.signal, "getsignal", lambda *_args: None)
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert ("quiescence", "companion") not in protocol.events
    assert "protocol-close" not in protocol.events
    release_control.set()


def test_fatal_stop_wakes_simulation_sleep_without_fresh_output() -> None:
    clock = runtime_node.SimulationClock()
    clock.accept(0)
    monitoring_stop = threading.Event()
    stop = runtime_node._QgcStopCoordinator(clock, monitoring_stop)
    outputs = []
    failures = []

    def blocked_operation() -> None:
        try:
            clock.sleep(10.0)
            outputs.append("output")
        except RuntimeError as error:
            failures.append(str(error))

    worker = threading.Thread(target=blocked_operation)
    worker.start()
    stop.request("ROS executor failed", stop_clock=True)
    worker.join(0.5)

    assert not worker.is_alive()
    assert outputs == []
    assert failures == ["simulation clock stopped: ROS executor failed"]
    with pytest.raises(RuntimeError, match="simulation clock stopped"):
        clock.accept(1_000_000_000)
    with pytest.raises(RuntimeError, match="simulation clock stopped"):
        clock.now()


@pytest.mark.parametrize(
    ("abort_fails", "monitor_fails"),
    [(True, False), (False, True), (True, True)],
)
def test_bound_fatal_stop_attempts_each_safety_action_once_across_failures(
    abort_fails: bool, monitor_fails: bool
) -> None:
    events: list[tuple[str, str]] = []
    clock_error = FatalStopError("clock stop failed")
    abort_error = FatalStopError("abort failed")
    monitor_error = RuntimeError("monitor stop failed")

    class HostileClock:
        def stop(self, reason: str) -> None:
            events.append(("clock", reason))
            raise clock_error

    class HostileRuntime:
        def request_abort(self, reason: str) -> None:
            events.append(("abort", reason))
            if abort_fails:
                raise abort_error

        def stop_monitoring(self) -> None:
            events.append(("monitor", ""))
            if monitor_fails:
                raise monitor_error

    stop = runtime_node._QgcStopCoordinator(HostileClock(), threading.Event())
    stop.bind_runtime(HostileRuntime())

    with pytest.raises(BaseExceptionGroup) as raised:
        stop.request("first fatal reason", stop_clock=True)

    expected_errors = [clock_error]
    if abort_fails:
        expected_errors.append(abort_error)
    if monitor_fails:
        expected_errors.append(monitor_error)
    assert list(raised.value.exceptions) == expected_errors
    assert events == [
        ("clock", "first fatal reason"),
        ("abort", "first fatal reason"),
        ("monitor", ""),
    ]
    assert stop.reason == "first fatal reason"

    stop.request("later reason", stop_clock=True)
    assert events == [
        ("clock", "first fatal reason"),
        ("abort", "first fatal reason"),
        ("monitor", ""),
    ]
    assert stop.reason == "first fatal reason"


def test_unbound_fatal_stop_wakes_waiter_and_later_bind_attempts_each_callback_once(
) -> None:
    events: list[tuple[str, str]] = []
    monitoring_stop = threading.Event()
    clock_error = RuntimeError("clock stop failed")
    abort_error = FatalStopError("abort failed")
    monitor_error = FatalStopError("monitor stop failed")

    class HostileClock:
        def stop(self, reason: str) -> None:
            events.append(("clock", reason))
            raise clock_error

    class HostileRuntime:
        def request_abort(self, reason: str) -> None:
            events.append(("abort", reason))
            raise abort_error

        def stop_monitoring(self) -> None:
            events.append(("monitor", ""))
            raise monitor_error

    stop = runtime_node._QgcStopCoordinator(HostileClock(), monitoring_stop)

    with pytest.raises(RuntimeError) as raised_clock:
        stop.request("first fatal reason", stop_clock=True)
    assert raised_clock.value is clock_error
    assert monitoring_stop.is_set()

    runtime = HostileRuntime()
    with pytest.raises(BaseExceptionGroup) as raised_callbacks:
        stop.bind_runtime(runtime)
    assert list(raised_callbacks.value.exceptions) == [abort_error, monitor_error]
    assert events == [
        ("clock", "first fatal reason"),
        ("abort", "first fatal reason"),
        ("monitor", ""),
    ]

    stop.bind_runtime(runtime)
    stop.request("later reason", stop_clock=True)
    assert len(events) == 3
    assert stop.reason == "first fatal reason"


@pytest.mark.parametrize(
    "error",
    [EmptyDetailError(), RaisingDetailError(), NonStringDetailError()],
    ids=["empty-detail", "raising-detail", "non-string-detail"],
)
def test_guard_input_preserves_hostile_exception_and_requests_contextual_fatal_stop(
    tmp_path: Path, error: BaseException
) -> None:
    clock = runtime_node.SimulationClock()
    host, stop = guarded_host(tmp_path, clock=clock)

    assert host._guard_input("clock", lambda: (_ for _ in ()).throw(error)) is False

    assert host.first_error is error
    assert stop.reason == f"clock input failed: {type(error).__name__}"
    assert clock.stopped is True


def test_guard_input_retains_source_error_and_reports_stop_failures_during_cleanup(
    tmp_path: Path,
) -> None:
    source_error = ValueError("malformed clock")
    stop_failure = RuntimeError("clock stop callback failed")
    abort_failure = FatalStopError("abort callback failed")
    monitor_failure = FatalStopError("monitor callback failed")

    class HostileClock:
        def __init__(self) -> None:
            self.calls: list[str] = []

        def stop(self, reason: str) -> None:
            self.calls.append(reason)
            if len(self.calls) == 1:
                raise stop_failure

    clock = HostileClock()
    stop = runtime_node._QgcStopCoordinator(clock, threading.Event())
    host, _ = guarded_host(tmp_path, clock=clock, stop=stop)
    stop.bind_runtime(
        SimpleNamespace(
            request_abort=lambda _reason: (_ for _ in ()).throw(abort_failure),
            stop_monitoring=lambda: (_ for _ in ()).throw(monitor_failure),
        )
    )

    assert (
        host._guard_input(
            "clock", lambda: (_ for _ in ()).throw(source_error)
        )
        is False
    )
    assert host.first_error is source_error
    assert stop.reason == "clock input failed: malformed clock"
    assert clock.calls == ["clock input failed: malformed clock"]

    cleanup = host.close()
    assert cleanup.confirmed is True
    assert cleanup.diagnostics == (
        "clock stop callback failed",
        "abort callback failed",
        "monitor callback failed",
    )
    assert clock.calls == [
        "clock input failed: malformed clock",
        "QGC host cleanup",
    ]


def test_concurrent_fatal_errors_dispatch_the_first_errors_context(
    tmp_path: Path,
) -> None:
    detail_entered = threading.Event()
    release_detail = threading.Event()
    second_started = threading.Event()
    stop_calls: list[str] = []

    class BlockingDetailError(BaseException):
        def __str__(self) -> str:
            detail_entered.set()
            release_detail.wait(1.0)
            return "first failure"

    class RecordingClock:
        def stop(self, reason: str) -> None:
            stop_calls.append(reason)

    first_error = BlockingDetailError()
    second_error = RuntimeError("second failure")
    stop = runtime_node._QgcStopCoordinator(RecordingClock(), threading.Event())
    host, _ = guarded_host(tmp_path, clock=RecordingClock(), stop=stop)
    first = threading.Thread(
        target=lambda: host._record_fatal_error(first_error, "first input failed")
    )

    def record_second() -> None:
        second_started.set()
        host._record_fatal_error(second_error, "second input failed")

    second = threading.Thread(target=record_second)
    first.start()
    assert detail_entered.wait(0.5)
    second.start()
    assert second_started.wait(0.5)
    threading.Event().wait(0.02)
    release_detail.set()
    first.join(0.5)
    second.join(0.5)

    assert not first.is_alive()
    assert not second.is_alive()
    assert host.first_error is first_error
    assert stop.reason == "first input failed: first failure"
    assert stop_calls == ["first input failed: first failure"]


@pytest.mark.parametrize(
    ("first_hard_stop", "second_hard_stop"),
    [(False, True), (True, False)],
)
def test_concurrent_fatal_errors_apply_monotonic_hard_stop_before_callbacks(
    tmp_path: Path, first_hard_stop: bool, second_hard_stop: bool
) -> None:
    detail_entered = threading.Event()
    release_detail = threading.Event()
    second_started = threading.Event()
    actions: list[tuple[str, str]] = []

    class BlockingDetailError(BaseException):
        def __str__(self) -> str:
            detail_entered.set()
            release_detail.wait(1.0)
            return "first failure"

    class RecordingClock:
        def stop(self, reason: str) -> None:
            actions.append(("clock", reason))

    class RecordingRuntime:
        def request_abort(self, reason: str) -> None:
            actions.append(("abort", reason))

        def stop_monitoring(self) -> None:
            actions.append(("monitor", ""))

    first_error = BlockingDetailError()
    clock = RecordingClock()
    stop = runtime_node._QgcStopCoordinator(clock, threading.Event())
    stop.bind_runtime(RecordingRuntime())
    host, _ = guarded_host(tmp_path, clock=clock, stop=stop)
    first = threading.Thread(
        target=lambda: host._record_fatal_error(
            first_error,
            "first input failed",
            stop_clock=first_hard_stop,
        )
    )

    def record_second() -> None:
        second_started.set()
        host._record_fatal_error(
            RuntimeError("second failure"),
            "second input failed",
            stop_clock=second_hard_stop,
        )

    second = threading.Thread(target=record_second)
    first.start()
    assert detail_entered.wait(0.5)
    second.start()
    assert second_started.wait(0.5)
    threading.Event().wait(0.02)
    release_detail.set()
    first.join(0.5)
    second.join(0.5)

    assert not first.is_alive()
    assert not second.is_alive()
    assert host.first_error is first_error
    reason = "first input failed: first failure"
    assert stop.reason == reason
    assert actions == [
        ("clock", reason),
        ("abort", reason),
        ("monitor", ""),
    ]


def test_reentrant_fatal_stop_callback_uses_selected_reason_without_duplicate_actions(
    tmp_path: Path,
) -> None:
    actions: list[tuple[str, str]] = []

    class RecordingClock:
        def stop(self, reason: str) -> None:
            actions.append(("clock", reason))

    clock = RecordingClock()
    stop = runtime_node._QgcStopCoordinator(clock, threading.Event())
    host, _ = guarded_host(tmp_path, clock=clock, stop=stop)
    first_error = ValueError("first failure")
    reentrant_error = RuntimeError("reentrant failure")

    class ReentrantRuntime:
        def request_abort(self, reason: str) -> None:
            actions.append(("abort", reason))
            host._record_fatal_error(reentrant_error, "reentrant input failed")

        def stop_monitoring(self) -> None:
            actions.append(("monitor", ""))

    stop.bind_runtime(ReentrantRuntime())
    worker = threading.Thread(
        target=lambda: host._record_fatal_error(first_error, "first input failed")
    )
    worker.start()
    worker.join(0.5)

    assert not worker.is_alive()
    assert host.first_error is first_error
    assert stop.reason == "first input failed: first failure"
    assert actions == [
        ("clock", "first input failed: first failure"),
        ("abort", "first input failed: first failure"),
        ("monitor", ""),
    ]


def test_phase_detail_reentrant_hard_failure_stops_clock_with_phase_reason(
    tmp_path: Path,
) -> None:
    actions: list[tuple[str, str]] = []
    infrastructure_error = RuntimeError("clock input failed")
    host: runtime_node._Comp2026QgcRosHost

    class ReentrantPhaseError(BaseException):
        def __str__(self) -> str:
            host._record_fatal_error(
                infrastructure_error,
                "clock input failed",
                stop_clock=True,
            )
            return "phase publication failed"

    class RecordingClock:
        def stop(self, reason: str) -> None:
            actions.append(("clock", reason))

    class RecordingRuntime:
        def request_abort(self, reason: str) -> None:
            actions.append(("abort", reason))

        def stop_monitoring(self) -> None:
            actions.append(("monitor", ""))

    phase_error = ReentrantPhaseError()
    clock = RecordingClock()
    stop = runtime_node._QgcStopCoordinator(clock, threading.Event())
    stop.bind_runtime(RecordingRuntime())
    host, _ = guarded_host(tmp_path, clock=clock, stop=stop)

    host._record_fatal_error(
        phase_error,
        "mission event publication failed",
        stop_clock=False,
    )

    reason = "mission event publication failed: phase publication failed"
    assert host.first_error is phase_error
    assert stop.reason == reason
    assert actions == [
        ("clock", reason),
        ("abort", reason),
        ("monitor", ""),
    ]


def test_real_phase_emitter_selects_publication_error_before_hostile_detail_reentry(
    tmp_path: Path,
) -> None:
    actions: list[tuple[str, str]] = []
    infrastructure_error = RuntimeError("clock input failed")
    host: runtime_node._Comp2026QgcRosHost

    class ReentrantPublicationError(BaseException):
        def __str__(self) -> str:
            host._record_fatal_error(
                infrastructure_error,
                "clock input failed",
                stop_clock=True,
            )
            return "publisher failed"

    class RecordingClock:
        def read_timestamp_ns(self) -> int:
            return 123

        def stop(self, reason: str) -> None:
            actions.append(("clock", reason))

    class RecordingRuntime:
        def request_abort(self, reason: str) -> None:
            actions.append(("abort", reason))

        def stop_monitoring(self) -> None:
            actions.append(("monitor", ""))

    publication_error = ReentrantPublicationError()
    clock = RecordingClock()
    stop = runtime_node._QgcStopCoordinator(clock, threading.Event())
    stop.bind_runtime(RecordingRuntime())
    host, _ = guarded_host(tmp_path, clock=clock, stop=stop)

    def fail_publish(_record: object) -> None:
        raise publication_error

    host.mission_event_emitter = runtime_node.MissionEventEmitter(
        RUN_ID,
        clock,
        fail_publish,
    )

    with pytest.raises(ReentrantPublicationError) as raised:
        host.observe_phase("FM1", "STARTED")

    assert raised.value is publication_error
    assert host.first_error is publication_error
    reason = "mission event publication failed: publisher failed"
    assert stop.reason == reason
    assert actions == [
        ("clock", reason),
        ("abort", reason),
        ("monitor", ""),
    ]


@pytest.mark.parametrize(
    ("trigger", "expected_reason"),
    [
        ("signal", "host termination signal"),
        ("finalize", "orchestration finalize request"),
        ("wall", "companion exceeded the overall run wall failsafe"),
    ],
)
def test_control_monitor_contains_stop_callback_failure_and_escalates_hard_stop(
    tmp_path: Path, trigger: str, expected_reason: str
) -> None:
    callback_error = RaisingDetailError()
    actions: list[tuple[str, str]] = []
    clock = runtime_node.SimulationClock()
    stop = runtime_node._QgcStopCoordinator(clock, threading.Event())
    host, _ = guarded_host(tmp_path, clock=clock, stop=stop)

    class HostileRuntime:
        def request_abort(self, reason: str) -> None:
            actions.append(("abort", reason))
            raise callback_error

        def stop_monitoring(self) -> None:
            actions.append(("monitor", ""))

    stop.bind_runtime(HostileRuntime())
    if trigger == "finalize":
        host.protocol.finalize_requested.set()
        host._overall_wall_deadline = -1.0
    elif trigger == "wall":
        host._overall_wall_deadline = -1.0

    host.start_control_monitor(
        lambda: "host termination signal" if trigger == "signal" else None
    )
    assert host.control_thread is not None
    host.control_thread.join(0.5)

    assert not host.control_thread.is_alive()
    assert host.first_error is callback_error
    assert stop.reason == expected_reason
    assert clock.stopped is True
    assert actions == [("abort", expected_reason), ("monitor", "")]


def test_finalizing_callback_contains_abort_failure_and_escalates_once(
    tmp_path: Path,
) -> None:
    callback_error = KeyboardInterrupt("abort failed")
    clock_error = RuntimeError("clock stop callback failed")
    actions: list[tuple[str, str]] = []

    class ReportingClock(runtime_node.SimulationClock):
        def __init__(self) -> None:
            super().__init__()
            self.stop_calls = 0

        def stop(self, reason: str) -> None:
            self.stop_calls += 1
            super().stop(reason)
            if self.stop_calls == 1:
                raise clock_error

    class HostileRuntime:
        def request_abort(self, reason: str) -> None:
            actions.append(("abort", reason))
            raise callback_error

        def stop_monitoring(self) -> None:
            actions.append(("monitor", ""))

    clock = ReportingClock()
    stop = runtime_node._QgcStopCoordinator(clock, threading.Event())
    host, _ = guarded_host(tmp_path, clock=clock, stop=stop)
    stop.bind_runtime(HostileRuntime())
    message = SimpleNamespace(
        run_id=RUN_ID,
        state=host.dependencies.RunState.FINALIZING,
    )
    escaped: list[BaseException] = []

    for _ in range(2):
        try:
            host._state_callback(message)
        except BaseException as error:
            escaped.append(error)

    assert escaped == []
    assert host.first_error is callback_error
    assert stop.reason == "orchestration finalization"
    assert clock.stopped is True
    assert actions == [
        ("abort", "orchestration finalization"),
        ("monitor", ""),
    ]
    cleanup = host.close()
    assert cleanup.confirmed is True
    assert cleanup.diagnostics == ("clock stop callback failed",)


def test_phase_publication_reraises_exact_error_and_contains_stop_failures(
    tmp_path: Path,
) -> None:
    actions: list[tuple[str, str]] = []
    publication_error = RaisingDetailError()
    abort_error = FatalStopError("abort failed")
    monitor_error = FatalStopError("monitor stop failed")

    class RecordingClock:
        def stop(self, reason: str) -> None:
            actions.append(("clock", reason))

    class HostileRuntime:
        def request_abort(self, reason: str) -> None:
            actions.append(("abort", reason))
            raise abort_error

        def stop_monitoring(self) -> None:
            actions.append(("monitor", ""))
            raise monitor_error

    class FailingEmitter:
        def __call__(self, _phase: str, _state: str) -> None:
            raise publication_error

        def stop(self, _reason: str) -> None:
            pass

    clock = RecordingClock()
    stop = runtime_node._QgcStopCoordinator(clock, threading.Event())
    host, _ = guarded_host(tmp_path, clock=clock, stop=stop)
    stop.bind_runtime(HostileRuntime())
    host.mission_event_emitter = FailingEmitter()

    with pytest.raises(RaisingDetailError) as raised:
        host.observe_phase("FM1", "STARTED")

    assert raised.value is publication_error
    assert host.first_error is publication_error
    assert stop.reason == "mission event publication failed: RaisingDetailError"
    assert actions == [
        ("abort", "mission event publication failed: RaisingDetailError"),
        ("monitor", ""),
    ]
    cleanup = host.close()
    assert cleanup.confirmed is True
    assert cleanup.diagnostics == (
        "abort failed",
        "monitor stop failed",
    )
    assert actions[-1] == ("clock", "QGC host cleanup")


def test_executor_failure_retains_exact_error_and_contains_stop_failures(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    harness = HostHarness()
    executor_error = RaisingDetailError()
    clock_error = RuntimeError("clock stop failed")
    abort_error = FatalStopError("abort failed")
    monitor_error = FatalStopError("monitor stop failed")
    actions: list[tuple[str, str]] = []

    class FailOnceClock:
        def stop(self, reason: str) -> None:
            actions.append(("clock", reason))
            if len([action for action in actions if action[0] == "clock"]) == 1:
                raise clock_error

    class HostileRuntime:
        def request_abort(self, reason: str) -> None:
            actions.append(("abort", reason))
            raise abort_error

        def stop_monitoring(self) -> None:
            actions.append(("monitor", ""))
            raise monitor_error

    def fail_spin() -> None:
        raise executor_error

    harness.executor.spin = fail_spin
    clock = FailOnceClock()
    stop = runtime_node._QgcStopCoordinator(clock, threading.Event())
    stop.bind_runtime(HostileRuntime())
    host = runtime_node._Comp2026QgcRosHost(
        config(tmp_path),
        projection(harness=harness),
        clock,
        SimpleNamespace(),
        harness.dependencies(),
        stop,
        threading.Event(),
        FakeProtocol(harness.events),
    )
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    host._start_ros()
    assert host.executor_thread is not None
    host.executor_thread.join(0.5)

    assert not host.executor_thread.is_alive()
    assert host.first_error is executor_error
    assert stop.reason == "ROS executor failed: RaisingDetailError"
    assert actions == [
        ("clock", "ROS executor failed: RaisingDetailError"),
        ("abort", "ROS executor failed: RaisingDetailError"),
        ("monitor", ""),
    ]
    cleanup = host.close()
    assert cleanup.confirmed is True
    assert cleanup.diagnostics == (
        "clock stop failed",
        "abort failed",
        "monitor stop failed",
    )


def test_early_executor_stop_uses_the_same_contextual_fatal_path(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    harness = HostHarness()
    actions: list[tuple[str, str]] = []

    class RecordingClock:
        def stop(self, reason: str) -> None:
            actions.append(("clock", reason))

    class RecordingRuntime:
        def request_abort(self, reason: str) -> None:
            actions.append(("abort", reason))

        def stop_monitoring(self) -> None:
            actions.append(("monitor", ""))

    harness.executor.spin = lambda: None
    clock = RecordingClock()
    stop = runtime_node._QgcStopCoordinator(clock, threading.Event())
    stop.bind_runtime(RecordingRuntime())
    host = runtime_node._Comp2026QgcRosHost(
        config(tmp_path),
        projection(harness=harness),
        clock,
        SimpleNamespace(),
        harness.dependencies(),
        stop,
        threading.Event(),
        FakeProtocol(harness.events),
    )
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    host._start_ros()
    assert host.executor_thread is not None
    host.executor_thread.join(0.5)

    error = host.first_error
    assert type(error) is RuntimeError
    assert str(error) == "ROS executor stopped before host cleanup"
    reason = "ROS executor failed: ROS executor stopped before host cleanup"
    assert stop.reason == reason
    assert actions == [
        ("clock", reason),
        ("abort", reason),
        ("monitor", ""),
    ]
    assert host.close().confirmed is True


@pytest.mark.parametrize(
    "error",
    [EmptyDetailError(), RaisingDetailError(), NonStringDetailError()],
    ids=["empty-detail", "raising-detail", "non-string-detail"],
)
def test_control_monitor_preserves_hostile_exception_and_requests_contextual_fatal_stop(
    tmp_path: Path, error: BaseException
) -> None:
    clock = runtime_node.SimulationClock()
    host, stop = guarded_host(tmp_path, clock=clock)

    host.start_control_monitor(lambda: (_ for _ in ()).throw(error))

    assert host.first_error is error
    assert stop.reason == f"host control monitor failed: {type(error).__name__}"
    assert clock.stopped is True
    assert host.close().confirmed is True


def test_operator_stop_retains_clock_for_recovery_progress() -> None:
    clock = runtime_node.SimulationClock()
    clock.accept(0)
    stop = runtime_node._QgcStopCoordinator(clock, threading.Event())
    recovery = []
    worker = threading.Thread(target=lambda: (clock.sleep(1.0), recovery.append("rtl")))
    worker.start()

    stop.request("host termination signal")
    assert worker.is_alive()
    assert clock.stopped is False
    clock.accept(1_000_000_000)
    worker.join(0.5)

    assert recovery == ["rtl"]


def test_durable_finalize_monitor_keeps_wall_failsafe_alive_during_stalled_recovery(
    tmp_path,
) -> None:
    harness = HostHarness()
    protocol = FakeProtocol(harness.events)
    clock = runtime_node.SimulationClock()
    clock.accept(0)
    wall_now = [0.0]
    dependencies = harness.dependencies()
    dependencies.wall_now = lambda: wall_now[0]
    monitoring_stop = threading.Event()
    stop = runtime_node._QgcStopCoordinator(clock, monitoring_stop)
    host = runtime_node._Comp2026QgcRosHost(
        replace(config(tmp_path), max_wall_seconds=1.0),
        projection(harness=harness),
        clock,
        SimpleNamespace(),
        dependencies,
        stop,
        monitoring_stop,
        protocol,
    )
    stop.bind_runtime(harness.runtime)
    host.start_control_monitor(lambda: None)
    recovery_failures = []
    worker = threading.Thread(
        target=lambda: _capture_clock_sleep_failure(
            clock, recovery_failures
        ),
        daemon=True,
    )

    try:
        protocol.finalize_requested.set()
        for _ in range(100):
            if ("abort", "orchestration finalize request") in harness.events:
                break
            threading.Event().wait(0.001)
        worker.start()
        assert worker.is_alive()
        assert clock.stopped is False
        assert host.control_thread is not None
        assert host.control_thread.is_alive()

        wall_now[0] = 2.0
        worker.join(0.5)

        assert not worker.is_alive()
        assert recovery_failures == [
            "simulation clock stopped: orchestration finalize request"
        ]
        assert harness.events.count(
            ("abort", "orchestration finalize request")
        ) == 1
        assert harness.events.count("stop-monitoring") == 1
    finally:
        if not clock.stopped:
            clock.stop("test cleanup")
        worker.join(0.5)
        host._control_stop.set()
        if host.control_thread is not None:
            host.control_thread.join(0.5)


def _capture_clock_sleep_failure(
    clock: runtime_node.SimulationClock, failures: list[str]
) -> None:
    try:
        clock.sleep(10.0)
    except RuntimeError as error:
        failures.append(str(error))


def test_matching_finalizing_requests_one_abort_and_retains_clock_through_recovery(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    harness.deliver_guided = False
    dependencies = harness.dependencies()
    original_start = dependencies.start_repl
    recovery = []

    def start_then_finalize(*args: object, **kwargs: object) -> object:
        original_ready = kwargs["on_listener_ready"]

        def ready_then_finalize(runtime: object) -> None:
            original_ready(runtime)

            def nested_recovery() -> None:
                harness.timebase.active.sleep(1.0)
                recovery.append("primary-original-H-cruise-LAND-returned")

            worker = threading.Thread(target=nested_recovery, daemon=True)
            worker.start()
            finalizing = SimpleNamespace(run_id=RUN_ID, state=3)
            harness.node.callbacks["/simulation/run_state"](finalizing)
            harness.node.callbacks["/simulation/run_state"](finalizing)
            assert worker.is_alive()
            assert harness.timebase.active.stopped is False
            harness.node.callbacks["/clock"](
                SimpleNamespace(clock=SimpleNamespace(sec=1, nanosec=0))
            )
            worker.join(0.5)
            assert not worker.is_alive()

        kwargs["on_listener_ready"] = ready_then_finalize
        result = original_start(*args, **kwargs)
        return SimpleNamespace(
            mission_result="ABORTED",
            recovery_outcome="HOME_LANDED",
            monitoring_exit_reason="EXPLICIT_STOP_UNCONFIRMED",
            cleanup_report=result.cleanup_report,
        )

    dependencies.start_repl = start_then_finalize
    protocol = install_harness(monkeypatch, harness, projection(harness=harness))
    monkeypatch.setattr(
        runtime_node, "_load_qgc_live_dependencies", lambda: dependencies
    )
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert recovery == ["primary-original-H-cruise-LAND-returned"]
    assert [
        event
        for event in harness.events
        if event == ("abort", "orchestration finalization")
    ] == [
        ("abort", "orchestration finalization")
    ]
    assert harness.events.count("stop-monitoring") == 1
    assert harness.vehicle.mode_writes == []
    failure = next(
        event[2]
        for event in protocol.events
        if isinstance(event, tuple) and event[:2] == ("status", "runtime-failure")
    )
    assert failure["reason"] == "orchestration finalization"


def test_inflight_callback_failure_during_shutdown_wins_terminal_race(
    tmp_path, monkeypatch, capsys
) -> None:
    harness = HostHarness()
    dependencies = harness.dependencies()
    callback_entered = threading.Event()
    release_callback = threading.Event()
    begin_callback = threading.Event()

    class BadClockMessage:
        @property
        def clock(self) -> object:
            callback_entered.set()
            release_callback.wait(1.0)
            raise ValueError("late malformed clock")

    def spin() -> None:
        harness.events.append("executor-spin")
        begin_callback.wait(1.0)
        harness.node.callbacks["/clock"](BadClockMessage())

    def shutdown(*, timeout_sec: float) -> bool:
        harness.events.append(("executor-shutdown", timeout_sec))
        release_callback.set()
        raise RuntimeError("executor cleanup failed")

    harness.executor.spin = spin
    harness.executor.shutdown = shutdown
    original_start = dependencies.start_repl

    def start_with_inflight_callback(*args: object, **kwargs: object) -> object:
        result = original_start(*args, **kwargs)
        begin_callback.set()
        assert callback_entered.wait(0.5)
        return result

    dependencies.start_repl = start_with_inflight_callback
    protocol = install_harness(monkeypatch, harness, projection(harness=harness))
    monkeypatch.setattr(
        runtime_node, "_load_qgc_live_dependencies", lambda: dependencies
    )
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    failure = next(
        event[2]
        for event in protocol.events
        if isinstance(event, tuple) and event[:2] == ("status", "runtime-failure")
    )
    assert failure["reason"] == "late malformed clock"
    assert '"message":"clock input failed: late malformed clock"' in capsys.readouterr().out
    assert not any(
        isinstance(event, tuple) and event[:2] == ("status", "mission-finished")
        for event in protocol.events
    )


def test_executor_thread_start_failure_still_cleans_ros_without_quiescence(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    dependencies = harness.dependencies()

    class StartFailureThread:
        def __init__(self, **_kwargs: object) -> None:
            pass

        def start(self) -> None:
            raise RuntimeError("executor start failed")

        def join(self, _timeout: float) -> None:
            raise AssertionError("unstarted executor thread must not be joined")

        def is_alive(self) -> bool:
            return False

    dependencies.thread_factory = StartFailureThread
    protocol = install_harness(monkeypatch, harness, projection(harness=harness))
    monkeypatch.setattr(
        runtime_node, "_load_qgc_live_dependencies", lambda: dependencies
    )
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert "destroy-node" in harness.events
    assert "rclpy-shutdown" in harness.events
    assert ("quiescence", "companion") not in protocol.events


def test_ready_publication_failure_closes_unhanded_controller(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    dependencies = harness.dependencies()

    class ReadyFailureProtocol(FakeProtocol):
        def write_status(self, name: str, document: dict[str, object]) -> None:
            if name == "companion-ready":
                raise RuntimeError("ready publication failed")
            super().write_status(name, document)

    protocol = ReadyFailureProtocol(harness.events)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(
        runtime_node,
        "project_qgc_runtime",
        lambda *_args, **_kwargs: projection(harness=harness),
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_live_dependencies", lambda: dependencies
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_timebase", lambda: harness.timebase
    )
    monkeypatch.setattr(runtime_node.signal, "signal", lambda *_args: None)
    monkeypatch.setattr(runtime_node.signal, "getsignal", lambda *_args: None)
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert harness.events.count("vehicle-close") == 1
    assert "destroy-node" in harness.events
    assert "rclpy-shutdown" in harness.events
    assert ("quiescence", "companion") not in protocol.events


def test_payload_cleanup_failure_does_not_skip_remaining_cleanup(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    dependencies = harness.dependencies()

    class FailingPayloadClient:
        def __init__(self, *_args: object, **_kwargs: object) -> None:
            pass

        def stop(self) -> None:
            harness.events.append("payload-stop-failed")
            raise RuntimeError("payload cleanup failed")

    protocol = install_harness(monkeypatch, harness, projection(harness=harness))
    monkeypatch.setattr(
        runtime_node, "_load_qgc_live_dependencies", lambda: dependencies
    )
    monkeypatch.setattr(runtime_node, "_RosPayloadClient", FailingPayloadClient)
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert "payload-stop-failed" in harness.events
    assert "rclpy-shutdown" in harness.events
    assert ("quiescence", "companion") not in protocol.events


def test_terminal_and_protocol_close_failures_do_not_escape_or_skip_cleanup(
    tmp_path, monkeypatch
) -> None:
    harness = HostHarness()
    dependencies = harness.dependencies()

    class FailingProtocol(FakeProtocol):
        def write_status(self, name: str, document: dict[str, object]) -> None:
            if name in {"mission-finished", "runtime-failure"}:
                raise RuntimeError(f"{name} publication failed")
            super().write_status(name, document)

        def close(self) -> None:
            self.events.append("protocol-close-attempt")
            raise RuntimeError("protocol close failed")

    protocol = FailingProtocol(harness.events)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", lambda _config: protocol)
    monkeypatch.setattr(
        runtime_node,
        "project_qgc_runtime",
        lambda *_args, **_kwargs: projection(harness=harness),
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_live_dependencies", lambda: dependencies
    )
    monkeypatch.setattr(
        runtime_node, "_load_qgc_timebase", lambda: harness.timebase
    )
    monkeypatch.setattr(runtime_node.signal, "signal", lambda *_args: None)
    monkeypatch.setattr(runtime_node.signal, "getsignal", lambda *_args: None)
    monkeypatch.setattr(runtime_node, "create_comp2026_lidar", lambda _clock: object())

    assert runtime_node._run_comp2026(config(tmp_path)) == 1
    assert "destroy-node" in harness.events
    assert "rclpy-shutdown" in harness.events
    assert "protocol-close-attempt" in harness.events
