from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from drone_sim_companion.comp2026_host import (
    AttemptFailureCoordinator,
    Comp2026StartGate,
    GPSCoord,
    MissionEventEmitter,
    PayloadDropper,
    PayloadResponse,
    RosFrameSource,
    RosLidar,
    SimulationClock,
    StaleSensorError,
    refresh_comp2026_start_gate,
    horizontal_distance_m,
    world_xy_to_gps,
)


RUN_ID = "00000000-0000-4000-8000-000000000001"


def stamp(timestamp_ns: int) -> SimpleNamespace:
    return SimpleNamespace(
        sec=timestamp_ns // 1_000_000_000,
        nanosec=timestamp_ns % 1_000_000_000,
    )


def metadata(frame_id: int, timestamp_ns: int, *, run_id: str = RUN_ID) -> object:
    return SimpleNamespace(
        run_id=run_id,
        sim_timestamp=stamp(timestamp_ns),
        frame_id=frame_id,
        stream="onboard",
    )


def rgb_image(timestamp_ns: int, *, red: int = 10, green: int = 20, blue: int = 30):
    pixel = bytes((red, green, blue))
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp(timestamp_ns), frame_id="camera/onboard"),
        width=640,
        height=480,
        encoding="rgb8",
        is_bigendian=False,
        step=640 * 3,
        data=pixel * (640 * 480),
    )


def test_world_xy_to_gps_maps_x_east_and_y_north() -> None:
    home = GPSCoord(41.501900, -81.675000, 0.0)

    west = world_xy_to_gps(home, x_m=-152.40, y_m=0.0)
    north = world_xy_to_gps(home, x_m=0.0, y_m=91.44)

    assert west.lat == pytest.approx(home.lat)
    assert west.long < home.long
    assert horizontal_distance_m(home, west) == pytest.approx(152.40, abs=0.02)
    assert north.lat > home.lat
    assert north.long == pytest.approx(home.long)
    assert horizontal_distance_m(home, north) == pytest.approx(91.44, abs=0.02)


def test_simulation_clock_sleep_uses_elapsed_accepted_simulation_time() -> None:
    clock = SimulationClock()
    clock.accept(10_000_000_000)
    finished = threading.Event()
    failure: list[BaseException] = []

    def wait_one_second() -> None:
        try:
            clock.sleep(1.0)
            finished.set()
        except BaseException as error:  # pragma: no cover - diagnostic capture
            failure.append(error)

    worker = threading.Thread(target=wait_one_second)
    worker.start()
    clock.accept(10_999_999_999)
    assert not finished.wait(0.05)
    clock.accept(11_000_000_000)
    worker.join(timeout=1.0)

    assert failure == []
    assert finished.is_set()
    assert clock.now() == pytest.approx(11.0)


def test_frame_source_pairs_current_run_rgb_and_never_delivers_one_frame_twice() -> None:
    source = RosFrameSource(width_px=640, height_px=480, run_id=RUN_ID)
    source.accept(metadata(0, 50_000_000, run_id="stale-run"), rgb_image(50_000_000))
    source.accept(metadata(0, 50_000_000), rgb_image(50_000_000))

    frame = source.capture_frame()

    assert frame.shape == (480, 640, 3)
    assert frame.dtype == np.uint8
    assert frame[0, 0].tolist() == [30, 20, 10]
    assert source.last_timestamp_ns == 50_000_000
    with pytest.raises(StaleSensorError, match="newer frame"):
        source.capture_frame(deadline_sim_ns=100_000_000)


def test_frame_source_waits_for_an_exact_strictly_newer_pair() -> None:
    source = RosFrameSource(width_px=640, height_px=480, run_id=RUN_ID)
    source.accept_image(rgb_image(50_000_000))
    source.accept_metadata(metadata(0, 50_000_000))
    source.capture_frame()
    delivered = threading.Event()
    timestamps: list[int | None] = []

    def capture() -> None:
        source.capture_frame()
        timestamps.append(source.last_timestamp_ns)
        delivered.set()

    worker = threading.Thread(target=capture)
    worker.start()
    source.accept_image(rgb_image(100_000_000))
    assert not delivered.wait(0.05)
    source.accept_metadata(metadata(1, 100_000_000))
    worker.join(timeout=1.0)

    assert delivered.is_set()
    assert timestamps == [100_000_000]


def test_lidar_rejects_range_older_than_half_a_simulated_second() -> None:
    clock = SimulationClock()
    lidar = RosLidar(clock)
    scan = SimpleNamespace(ranges=[4.572], range_min=0.1, range_max=30.0)
    clock.accept(1_000_000_000)
    lidar.accept(scan, 1_000_000_000)
    clock.accept(1_500_000_000)
    assert lidar.get_distance() == pytest.approx(4.572)

    clock.accept(1_500_000_001)
    with pytest.raises(StaleSensorError, match="older than 0.5"):
        lidar.get_distance()


class FakePayloadClient:
    def __init__(self) -> None:
        self.requests: list[object] = []
        self._responses: list[PayloadResponse] = []
        self._condition = threading.Condition()

    def call(self, request: object) -> PayloadResponse:
        with self._condition:
            self.requests.append(request)
            self._condition.notify_all()
            self._condition.wait_for(lambda: bool(self._responses), timeout=1.0)
            return self._responses.pop(0)

    def wait_for_request(self) -> None:
        with self._condition:
            assert self._condition.wait_for(lambda: bool(self.requests), timeout=1.0)

    def complete(self, response: PayloadResponse) -> None:
        with self._condition:
            self._responses.append(response)
            self._condition.notify_all()


def test_payload_dropper_waits_for_matching_confirmation() -> None:
    clock = SimulationClock()
    client = FakePayloadClient()
    dropper = PayloadDropper(RUN_ID, 3, client, clock)
    returned: list[bool] = []
    worker = threading.Thread(target=lambda: returned.append(dropper.drop()))
    worker.start()
    client.wait_for_request()
    assert worker.is_alive()
    client.complete(PayloadResponse(True, "OK", "", "run:3:release:1", 1))
    worker.join(timeout=1.0)

    request = client.requests[0]
    assert returned == [True]
    assert request.run_id == RUN_ID
    assert request.aruco_id == 3
    assert request.action == request.RELEASE
    assert request.command_id == "run:3:release:1"


def test_payload_dropper_attach_returns_literal_true_and_rejects_bad_correlation() -> None:
    client = FakePayloadClient()
    dropper = PayloadDropper(RUN_ID, 3, client, SimulationClock())
    client.complete(PayloadResponse(True, "OK", "", "run:3:attach:1", 1))
    assert dropper.attach(3) is True

    client.complete(PayloadResponse(True, "OK", "", "wrong-command", 2))
    with pytest.raises(RuntimeError, match="command_id"):
        dropper.drop()


def test_phase_emitter_preserves_order_and_genuine_clock_timestamp() -> None:
    clock = SimulationClock()
    published: list[object] = []
    emit = MissionEventEmitter(RUN_ID, clock, published.append)
    clock.accept(50_000_000)
    emit("FM1", "STARTED")
    clock.accept(100_000_000)
    emit("FM1", "COMPLETE")

    assert [
        (event.event_id, event.phase, event.state, event.sim_timestamp_ns)
        for event in published
    ] == [
        (0, "FM1", "STARTED", 50_000_000),
        (1, "FM1", "COMPLETE", 100_000_000),
    ]
    assert all(event.run_id == RUN_ID for event in published)
    assert all(event.detail == "automatic attempt" for event in published)


def test_fatal_input_prevents_further_phase_or_success_and_recovers_once() -> None:
    clock = SimulationClock()
    clock.accept(50_000_000)
    published: list[object] = []
    emit = MissionEventEmitter(RUN_ID, clock, published.append)
    failures: list[str] = []
    recoveries: list[str] = []

    def stop_attempt(reason: str) -> None:
        emit.stop(reason)

    coordinator = AttemptFailureCoordinator(
        stop_attempt=stop_attempt,
        write_failure=failures.append,
        recover=lambda: recoveries.append("recovered"),
    )
    emit("FM1", "STARTED")

    coordinator.guard_input(
        "image",
        lambda: (_ for _ in ()).throw(ValueError("bad image")),
    )

    with pytest.raises(RuntimeError, match="bad image"):
        emit("FM1", "COMPLETE")
    terminal: list[str] = []
    assert coordinator.finish_success(lambda: terminal.append("mission-finished")) is False
    assert coordinator.fail("later failure") is False
    assert coordinator.recover_once() is True
    assert coordinator.recover_once() is False
    assert [(event.phase, event.state) for event in published] == [("FM1", "STARTED")]
    assert terminal == []
    assert failures == ["competition image input failed: bad image"]
    assert recoveries == ["recovered"]


def test_callback_failure_claim_precedes_terminal_success() -> None:
    rendering_exception = threading.Event()
    allow_failure_claim = threading.Event()
    callback_results: list[bool] = []
    finish_results: list[bool] = []
    finish_returned = threading.Event()
    failures: list[str] = []
    terminal: list[str] = []

    class PausingError(ValueError):
        def __str__(self) -> str:
            rendering_exception.set()
            assert allow_failure_claim.wait(1.0)
            return "bad image"

    coordinator = AttemptFailureCoordinator(
        stop_attempt=lambda _reason: None,
        write_failure=failures.append,
        recover=lambda: None,
    )

    def fail_callback() -> None:
        callback_results.append(
            coordinator.guard_input(
                "image",
                lambda: (_ for _ in ()).throw(PausingError()),
            )
        )

    def finish() -> None:
        finish_results.append(
            coordinator.finish_success(lambda: terminal.append("mission-finished"))
        )
        finish_returned.set()

    callback_thread = threading.Thread(target=fail_callback)
    finish_thread = threading.Thread(target=finish)
    callback_thread.start()
    assert rendering_exception.wait(1.0)
    finish_thread.start()
    try:
        assert not finish_returned.wait(0.05)
    finally:
        allow_failure_claim.set()
        callback_thread.join(timeout=1.0)
        finish_thread.join(timeout=1.0)

    assert callback_results == [False]
    assert finish_results == [False]
    assert terminal == []
    assert failures == ["competition image input failed: bad image"]


def test_successful_inflight_callback_completes_before_single_terminal_success() -> None:
    callback_started = threading.Event()
    allow_callback = threading.Event()
    callback_results: list[bool] = []
    finish_results: list[bool] = []
    finish_returned = threading.Event()
    terminal: list[str] = []
    coordinator = AttemptFailureCoordinator(
        stop_attempt=lambda _reason: None,
        write_failure=lambda _reason: None,
        recover=lambda: None,
    )

    def accept_callback() -> None:
        callback_started.set()
        assert allow_callback.wait(1.0)

    callback_thread = threading.Thread(
        target=lambda: callback_results.append(
            coordinator.guard_input("range", accept_callback)
        )
    )

    def finish() -> None:
        finish_results.append(
            coordinator.finish_success(lambda: terminal.append("mission-finished"))
        )
        finish_returned.set()

    finish_thread = threading.Thread(target=finish)
    callback_thread.start()
    assert callback_started.wait(1.0)
    finish_thread.start()
    try:
        assert not finish_returned.wait(0.05)
    finally:
        allow_callback.set()
        callback_thread.join(timeout=1.0)
        finish_thread.join(timeout=1.0)

    assert callback_results == [True]
    assert finish_results == [True]
    assert terminal == ["mission-finished"]


def test_process_readiness_can_precede_samples_but_mission_start_cannot() -> None:
    gate = Comp2026StartGate()
    gate.mark_process_ready()

    assert gate.mission_ready is True
    assert gate.mission_start_ready is False

    gate.accept_running()
    gate.accept_clock()
    gate.refresh_live_readiness(
        frame_ready=True,
        payload_service_ready=True,
        heartbeat_live=True,
        armable=False,
        range_is_current=lambda: True,
    )
    assert gate.mission_start_ready is False

    gate.refresh_live_readiness(
        frame_ready=True,
        payload_service_ready=True,
        heartbeat_live=True,
        armable=True,
        range_is_current=lambda: True,
    )
    assert gate.mission_start_ready is True


class MutablePayloadService:
    def __init__(self) -> None:
        self.ready = True

    def service_is_ready(self) -> bool:
        return self.ready


def live_start_inputs():
    gate = Comp2026StartGate()
    gate.mark_process_ready()
    gate.accept_running()
    gate.accept_clock()
    clock = SimulationClock()
    clock.accept(1_000_000_000)
    lidar = RosLidar(clock)
    lidar.accept(
        SimpleNamespace(ranges=[4.572], range_min=0.1, range_max=30.0),
        1_000_000_000,
    )
    frame_source = RosFrameSource(width_px=640, height_px=480, run_id=RUN_ID)
    frame_source.accept(metadata(0, 1_000_000_000), rgb_image(1_000_000_000))
    payload_service = MutablePayloadService()
    vehicle = SimpleNamespace(last_heartbeat=0.25, is_armable=True)
    return gate, clock, lidar, frame_source, payload_service, vehicle


def refresh_live_start(inputs) -> Comp2026StartGate:
    gate, _clock, lidar, frame_source, payload_service, vehicle = inputs
    refresh_comp2026_start_gate(
        gate,
        frame_source=frame_source,
        lidar=lidar,
        payload_client=payload_service,
        vehicle=vehicle,
    )
    return gate


def test_start_readiness_invalidates_when_range_becomes_stale() -> None:
    inputs = live_start_inputs()
    gate = refresh_live_start(inputs)
    assert gate.mission_start_ready is True

    inputs[1].accept(1_500_000_001)
    refresh_live_start(inputs)

    assert gate.mission_start_ready is False


def test_start_gate_cannot_release_when_clock_advances_during_live_predicate_read() -> None:
    gate, clock, lidar, frame_source, _payload_service, vehicle = live_start_inputs()
    clock.accept(1_400_000_000)
    released = threading.Event()

    def wait_for_start() -> None:
        try:
            gate.wait_until_ready()
        except RuntimeError:
            return
        released.set()

    worker = threading.Thread(target=wait_for_start)
    worker.start()

    class ClockAdvancingPayloadService:
        def service_is_ready(self) -> bool:
            clock.accept(1_500_000_001)
            return True

    try:
        refresh_comp2026_start_gate(
            gate,
            frame_source=frame_source,
            lidar=lidar,
            payload_client=ClockAdvancingPayloadService(),
            vehicle=vehicle,
        )

        assert gate.mission_start_ready is False
        assert not released.wait(0.05)

        lidar.accept(
            SimpleNamespace(ranges=[4.572], range_min=0.1, range_max=30.0),
            1_500_000_001,
        )
        refresh_comp2026_start_gate(
            gate,
            frame_source=frame_source,
            lidar=lidar,
            payload_client=MutablePayloadService(),
            vehicle=vehicle,
        )
        worker.join(timeout=1.0)
        assert released.is_set()
    finally:
        gate.stop("test cleanup")
        worker.join(timeout=1.0)


def test_start_readiness_invalidates_when_payload_service_disappears() -> None:
    inputs = live_start_inputs()
    gate = refresh_live_start(inputs)
    assert gate.mission_start_ready is True

    inputs[4].ready = False
    refresh_live_start(inputs)

    assert gate.mission_start_ready is False


def test_start_readiness_invalidates_when_armability_reverts() -> None:
    inputs = live_start_inputs()
    gate = refresh_live_start(inputs)
    assert gate.mission_start_ready is True

    inputs[5].is_armable = False
    refresh_live_start(inputs)

    assert gate.mission_start_ready is False


def test_start_readiness_invalidates_when_current_dronekit_heartbeat_exceeds_existing_timeout() -> None:
    inputs = live_start_inputs()
    gate = refresh_live_start(inputs)
    assert gate.mission_start_ready is True

    inputs[5].last_heartbeat = 2.000_001
    refresh_live_start(inputs)
    assert gate.mission_start_ready is True

    inputs[5].last_heartbeat = 60.000_001
    refresh_live_start(inputs)

    assert gate.mission_start_ready is False


def test_start_readiness_requires_an_undelivered_genuine_frame_pair() -> None:
    inputs = live_start_inputs()
    gate = refresh_live_start(inputs)
    assert gate.mission_start_ready is True

    inputs[3].capture_frame()
    refresh_live_start(inputs)

    assert gate.mission_start_ready is False


def test_production_python_can_import_pinned_dronekit_on_python312() -> None:
    source_path = str(Path(__file__).parents[1] / "src")
    result = subprocess.run(
        [sys.executable, "-c", "import dronekit; print(dronekit.connect.__module__)"],
        check=False,
        capture_output=True,
        text=True,
        env={
            **os.environ,
            "PYTHONPATH": os.pathsep.join(
                filter(None, (source_path, os.environ.get("PYTHONPATH", "")))
            ),
        },
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "dronekit"
