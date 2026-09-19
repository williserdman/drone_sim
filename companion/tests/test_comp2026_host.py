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


class UnprintablePublicationFailure(Exception):
    def __str__(self) -> str:
        raise RuntimeError("publication diagnostic formatting failed")


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


def lidar_sample(
    distance_m: float,
    sampled_at: float,
    sequence: int,
    invalidation_generation: int,
) -> SimpleNamespace:
    return SimpleNamespace(
        distance_m=distance_m,
        sampled_at=sampled_at,
        sequence=sequence,
        invalidation_generation=invalidation_generation,
    )


def make_lidar(clock: SimulationClock) -> RosLidar:
    return RosLidar(clock, sample_factory=lidar_sample)


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


def test_frame_source_delivers_rgb_image_once_without_redundant_metadata() -> None:
    source = RosFrameSource(width_px=640, height_px=480)
    source.accept_image(rgb_image(50_000_000))
    assert source.ready is True

    frame = source.capture_frame()

    assert frame.shape == (480, 640, 3)
    assert frame.dtype == np.uint8
    assert frame[0, 0].tolist() == [30, 20, 10]
    assert source.last_timestamp_ns == 50_000_000
    with pytest.raises(StaleSensorError, match="newer frame"):
        source.capture_frame(deadline_sim_ns=100_000_000)


def test_frame_source_waits_for_an_exact_strictly_newer_image() -> None:
    source = RosFrameSource(width_px=640, height_px=480)
    source.accept_image(rgb_image(50_000_000))
    assert source.ready is True
    source.capture_frame()
    source.accept_image(rgb_image(50_000_000))
    assert source.ready is False
    source.accept_image(rgb_image(100_000_000))

    assert source.ready is True
    source.capture_frame()
    assert source.last_timestamp_ns == 100_000_000


def test_frame_source_hands_over_only_the_newest_available_image() -> None:
    source = RosFrameSource(width_px=640, height_px=480)
    source.accept_image(rgb_image(50_000_000, red=1))
    source.accept_image(rgb_image(100_000_000, red=2))

    frame = source.capture_frame()

    assert source.last_timestamp_ns == 100_000_000
    assert frame[0, 0].tolist() == [30, 20, 2]
    assert source.ready is False


def test_frame_source_discards_only_frames_older_than_threshold() -> None:
    source = RosFrameSource(width_px=640, height_px=480)
    source.accept_image(rgb_image(500_000_000))

    assert source.retain_latest_at_or_after(500_000_000) is True
    source.capture_frame()
    source.accept_image(rgb_image(1_000_000_000))

    assert source.retain_latest_at_or_after(1_000_000_001) is False
    assert source.ready is False
    assert source.last_timestamp_ns == 1_000_000_000

    source.accept_image(rgb_image(1_000_000_000))
    assert source.ready is False
    source.accept_image(rgb_image(1_000_000_001))
    assert source.ready is True


def test_stopped_frame_source_rejects_queued_and_new_images() -> None:
    source = RosFrameSource(width_px=640, height_px=480)
    source.accept_image(rgb_image(50_000_000))

    source.stop("attempt finished")

    assert source.ready is False
    with pytest.raises(StaleSensorError, match="attempt finished"):
        source.capture_frame(deadline_sim_ns=100_000_000)
    with pytest.raises(StaleSensorError, match="attempt finished"):
        source.accept_image(rgb_image(100_000_000))


def test_frame_source_copies_pixels_only_when_autonomy_captures() -> None:
    copies = 0

    class DeferredPixels:
        def __bytes__(self) -> bytes:
            nonlocal copies
            copies += 1
            return bytes((10, 20, 30)) * (640 * 480)

    image = rgb_image(50_000_000)
    image.data = DeferredPixels()
    source = RosFrameSource(width_px=640, height_px=480)

    source.accept_image(image)

    assert copies == 0
    assert source.capture_frame()[0, 0].tolist() == [30, 20, 10]
    assert copies == 1


def test_lidar_rejects_range_older_than_half_a_simulated_second() -> None:
    clock = SimulationClock()
    lidar = make_lidar(clock)
    scan = SimpleNamespace(ranges=[4.572], range_min=0.1, range_max=30.0)
    clock.accept(1_000_000_000)
    lidar.accept(scan, 1_000_000_000)
    clock.accept(1_500_000_000)
    assert lidar.get_distance() == pytest.approx(4.572)

    clock.accept(1_500_000_001)
    with pytest.raises(StaleSensorError, match="older than 0.5"):
        lidar.get_distance()


def test_lidar_retains_recent_committed_range_when_delivery_leads_clock() -> None:
    clock = SimulationClock()
    lidar = make_lidar(clock)
    prior_scan = SimpleNamespace(ranges=[5.35], range_min=0.1, range_max=30.0)
    leading_scan = SimpleNamespace(ranges=[5.40], range_min=0.1, range_max=30.0)
    clock.accept(175_373_000_000)
    lidar.accept(prior_scan, 175_350_000_000)

    lidar.accept(leading_scan, 175_400_000_000)

    assert lidar.get_distance() == pytest.approx(5.35)
    clock.accept(175_400_000_000)
    assert lidar.get_distance() == pytest.approx(5.40)


def test_lidar_promotes_each_pending_same_tick_range_as_clock_catches_up() -> None:
    clock = SimulationClock()
    lidar = make_lidar(clock)
    scan = SimpleNamespace(ranges=[4.572], range_min=0.1, range_max=30.0)
    clock.accept(1_000_000_000)

    lidar.accept(scan, 1_050_000_000)
    clock.accept(1_050_000_000)
    lidar.accept(scan, 1_100_000_000)

    assert lidar.get_distance() == pytest.approx(4.572)


def test_lidar_sample_uses_source_simulation_time_and_advancing_sequence() -> None:
    clock = SimulationClock()
    lidar = make_lidar(clock)
    clock.accept(2_000_000_000)

    lidar.accept(
        SimpleNamespace(ranges=[4.25], range_min=0.1, range_max=30.0),
        1_750_000_000,
    )
    first = lidar.get_sample()
    lidar.accept(
        SimpleNamespace(ranges=[4.20], range_min=0.1, range_max=30.0),
        1_800_000_000,
    )
    second = lidar.get_sample()

    assert vars(first) == {
        "distance_m": 4.25,
        "sampled_at": 1.75,
        "sequence": 1,
        "invalidation_generation": 0,
    }
    assert vars(second) == {
        "distance_m": 4.20,
        "sampled_at": 1.8,
        "sequence": 2,
        "invalidation_generation": 0,
    }
    assert lidar.get_distance() == second.distance_m


class BrokenRanges:
    def __iter__(self):
        raise RuntimeError("range parse failed")


@pytest.mark.parametrize(
    ("invalid_scan", "message"),
    [
        (SimpleNamespace(range_min=0.1, range_max=30.0), "malformed"),
        (
            SimpleNamespace(ranges=[float("nan")], range_min=0.1, range_max=30.0),
            "invalid",
        ),
        (
            SimpleNamespace(ranges=BrokenRanges(), range_min=0.1, range_max=30.0),
            "malformed",
        ),
    ],
)
def test_lidar_invalid_scan_revokes_prior_sample_and_advances_generation(
    invalid_scan: object,
    message: str,
) -> None:
    clock = SimulationClock()
    lidar = make_lidar(clock)
    clock.accept(3_000_000_000)
    valid_scan = SimpleNamespace(ranges=[4.25], range_min=0.1, range_max=30.0)
    lidar.accept(valid_scan, 1_000_000_000)

    with pytest.raises(ValueError, match=message):
        lidar.accept(invalid_scan, 2_000_000_000)
    with pytest.raises(StaleSensorError, match="not ready"):
        lidar.get_sample()

    lidar.accept(valid_scan, 3_000_000_000)
    sample = lidar.get_sample()
    assert sample.sequence == 2
    assert sample.invalidation_generation == 1


@pytest.mark.parametrize("invalid_timestamp_ns", [2_000_000_000, 1_999_999_999])
def test_lidar_duplicate_or_backward_timestamp_revokes_prior_sample(
    invalid_timestamp_ns: int,
) -> None:
    clock = SimulationClock()
    lidar = make_lidar(clock)
    clock.accept(3_000_000_000)
    scan = SimpleNamespace(ranges=[4.25], range_min=0.1, range_max=30.0)
    lidar.accept(scan, 2_000_000_000)

    with pytest.raises(ValueError, match="must advance"):
        lidar.accept(scan, invalid_timestamp_ns)
    with pytest.raises(StaleSensorError, match="not ready"):
        lidar.get_sample()

    lidar.accept(scan, 3_000_000_000)
    sample = lidar.get_sample()
    assert sample.sequence == 2
    assert sample.invalidation_generation == 1


def test_lidar_invalidation_retains_latest_source_ordering() -> None:
    clock = SimulationClock()
    lidar = make_lidar(clock)
    clock.accept(3_000_000_000)
    scan = SimpleNamespace(ranges=[4.25], range_min=0.1, range_max=30.0)
    lidar.accept(scan, 1_000_000_000)
    with pytest.raises(ValueError, match="malformed"):
        lidar.accept(SimpleNamespace(range_min=0.1, range_max=30.0), 2_000_000_000)

    with pytest.raises(ValueError, match="must advance"):
        lidar.accept(scan, 1_500_000_000)

    lidar.accept(scan, 3_000_000_000)
    sample = lidar.get_sample()
    assert sample.sequence == 2
    assert sample.invalidation_generation == 2


def test_lidar_invalidation_clears_pending_future_sample() -> None:
    clock = SimulationClock()
    lidar = make_lidar(clock)
    clock.accept(1_000_000_000)
    scan = SimpleNamespace(ranges=[4.25], range_min=0.1, range_max=30.0)
    lidar.accept(scan, 900_000_000)
    lidar.accept(scan, 2_000_000_000)

    lidar.invalidate()
    clock.accept(2_000_000_000)

    with pytest.raises(StaleSensorError, match="not ready"):
        lidar.get_sample()
    lidar.accept(scan, 2_100_000_000)
    clock.accept(2_100_000_000)
    sample = lidar.get_sample()
    assert sample.sequence == 3
    assert sample.invalidation_generation == 1


def test_lidar_stop_revokes_sample_and_prevents_resurrection() -> None:
    clock = SimulationClock()
    lidar = make_lidar(clock)
    clock.accept(1_000_000_000)
    scan = SimpleNamespace(ranges=[4.25], range_min=0.1, range_max=30.0)
    lidar.accept(scan, 1_000_000_000)

    lidar.stop("test cancellation")

    with pytest.raises(StaleSensorError, match="stopped"):
        lidar.get_sample()
    with pytest.raises(StaleSensorError, match="stopped"):
        lidar.accept(scan, 1_100_000_000)


def test_lidar_rejects_sample_after_simulation_clock_stops() -> None:
    clock = SimulationClock()
    lidar = make_lidar(clock)
    clock.accept(1_000_000_000)
    lidar.accept(
        SimpleNamespace(ranges=[4.25], range_min=0.1, range_max=30.0),
        1_000_000_000,
    )

    clock.stop("test cancellation")

    with pytest.raises(StaleSensorError, match="clock stopped"):
        lidar.get_sample()


def test_lidar_uses_exact_nested_sample_type_when_checkout_is_supplied(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    nested_source = Path(__file__).parents[1] / "comp2026/src"
    if not nested_source.is_dir():
        pytest.skip("separate Comp2026 checkout is not supplied")
    monkeypatch.syspath_prepend(str(nested_source))
    from drone.sensors.lidar.lidar import LidarSample
    from drone_sim_companion.runtime_node import create_comp2026_lidar

    clock = SimulationClock()
    lidar = create_comp2026_lidar(clock)
    clock.accept(2_000_000_000)
    lidar.accept(
        SimpleNamespace(ranges=[4.25], range_min=0.1, range_max=30.0),
        1_750_000_000,
    )

    sample = lidar.get_sample()
    assert type(sample) is LidarSample
    assert sample == LidarSample(4.25, 1.75, 1, 0)


class FakePayloadClient:
    def __init__(self) -> None:
        self.requests: list[object] = []
        self._responses: list[PayloadResponse] = []
        self._responses_returned = 0
        self._condition = threading.Condition()

    def prepare(self, request: object) -> object:
        return request

    def dispatch(self, request: object) -> object:
        with self._condition:
            self.requests.append(request)
            self._condition.notify_all()
        return request

    def await_response(
        self, _pending: object, *, cancelled: threading.Event
    ) -> PayloadResponse:
        with self._condition:
            self._condition.wait_for(lambda: bool(self._responses), timeout=1.0)
            response = self._responses.pop(0)
            self._responses_returned += 1
            self._condition.notify_all()
            return response

    def wait_for_request(self, count: int = 1) -> None:
        with self._condition:
            assert self._condition.wait_for(
                lambda: len(self.requests) >= count, timeout=1.0
            )

    def complete(self, response: PayloadResponse) -> None:
        with self._condition:
            self._responses.append(response)
            self._condition.notify_all()

    def wait_for_response_return(self, count: int = 1) -> None:
        with self._condition:
            assert self._condition.wait_for(
                lambda: self._responses_returned >= count, timeout=1.0
            )


def payload_permission() -> bool:
    return True


payload_permission.actuate = lambda operation: operation()


def test_payload_dropper_waits_for_matching_confirmation() -> None:
    clock = SimulationClock()
    client = FakePayloadClient()
    dropper = PayloadDropper(
        RUN_ID,
        3,
        client,
        clock,
        permission=payload_permission,
        delay_wall_timeout_seconds=1.0,
    )
    returned: list[object] = []
    worker = threading.Thread(target=lambda: returned.append(dropper.drop()))
    worker.start()
    client.wait_for_request()
    assert worker.is_alive()
    client.complete(PayloadResponse(True, "OK", "", "run:3:release:1", 1))
    worker.join(timeout=1.0)

    request = client.requests[0]
    assert returned == [None]
    assert request.run_id == RUN_ID
    assert request.aruco_id == 3
    assert request.action == request.RELEASE
    assert request.command_id == "run:3:release:1"


def test_payload_dropper_does_not_retry_a_stale_release() -> None:
    clock = SimulationClock()
    clock.accept(10_000_000_000)
    client = FakePayloadClient()
    dropper = PayloadDropper(
        RUN_ID,
        4,
        client,
        clock,
        permission=payload_permission,
        delay_wall_timeout_seconds=1.0,
    )
    errors: list[BaseException] = []

    def release() -> None:
        try:
            dropper.drop()
        except BaseException as error:
            errors.append(error)

    worker = threading.Thread(target=release)
    worker.start()
    client.wait_for_request()
    client.complete(
        PayloadResponse(
            False,
            "STALE_PHYSICAL_STATE",
            "truth callbacks are between ticks",
            "run:4:release:1",
            1,
        )
    )

    worker.join(timeout=1.0)

    assert len(errors) == 1
    assert "STALE_PHYSICAL_STATE" in str(errors[0])
    assert [request.command_id for request in client.requests] == [
        "run:4:release:1",
    ]


def test_payload_dropper_attach_returns_literal_true_and_rejects_bad_correlation() -> None:
    client = FakePayloadClient()
    dropper = PayloadDropper(
        RUN_ID,
        3,
        client,
        SimulationClock(),
        permission=payload_permission,
        delay_wall_timeout_seconds=1.0,
    )
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


def test_phase_emitter_rejects_non_prefix_and_events_after_fm2_complete() -> None:
    clock = SimulationClock()
    clock.accept(1)
    published: list[object] = []
    emit = MissionEventEmitter(RUN_ID, clock, published.append)

    with pytest.raises(ValueError, match="next mission event"):
        emit("FM1", "COMPLETE")

    for phase, state in (
        ("FM1", "STARTED"),
        ("FM1", "COMPLETE"),
        ("FM2", "STARTED"),
        ("FM2", "COMPLETE"),
    ):
        emit(phase, state)

    with pytest.raises(RuntimeError, match="mission event sequence is complete"):
        emit("HOME", "COMPLETE")
    assert [event.event_id for event in published] == [0, 1, 2, 3]


def test_phase_emitter_reads_clock_inside_serialized_publication() -> None:
    entered = threading.Event()
    release = threading.Event()
    calls: list[str] = []

    class Clock:
        def read_timestamp_ns(self) -> int:
            calls.append("clock")
            return 9

    def publish(_record: object) -> None:
        calls.append("publish")
        entered.set()
        assert release.wait(1.0)

    emit = MissionEventEmitter(RUN_ID, Clock(), publish)
    worker = threading.Thread(target=lambda: emit("FM1", "STARTED"))
    worker.start()
    assert entered.wait(1.0)
    stopper = threading.Thread(target=lambda: (calls.append("stop-enter"), emit.stop("done"), calls.append("stop-exit")))
    stopper.start()
    assert "stop-exit" not in calls
    release.set()
    worker.join(1.0)
    stopper.join(1.0)

    assert calls == ["clock", "publish", "stop-enter", "stop-exit"]


def test_phase_emitter_latches_publication_failure_without_retry() -> None:
    clock = SimulationClock()
    clock.accept(10)
    attempts: list[object] = []

    def fail(record: object) -> None:
        attempts.append(record)
        raise RuntimeError("DDS writer failed")

    emit = MissionEventEmitter(RUN_ID, clock, fail)
    with pytest.raises(RuntimeError, match="DDS writer failed"):
        emit("FM1", "STARTED")
    with pytest.raises(RuntimeError, match="publication failed"):
        emit("FM1", "STARTED")

    assert len(attempts) == 1


@pytest.mark.parametrize(
    "failure",
    [KeyboardInterrupt(), SystemExit(), UnprintablePublicationFailure()],
)
def test_phase_emitter_preserves_hostile_publication_failure_and_safe_detail(
    failure,
) -> None:
    clock = SimulationClock()
    clock.accept(10)
    attempts = []

    def fail(record: object) -> None:
        attempts.append(record)
        raise failure

    emit = MissionEventEmitter(RUN_ID, clock, fail)

    with pytest.raises(type(failure)) as raised:
        emit("FM1", "STARTED")

    assert raised.value is failure
    with pytest.raises(RuntimeError) as stopped:
        emit("FM1", "STARTED")
    assert type(failure).__name__ in str(stopped.value)
    assert len(attempts) == 1


def test_phase_emitter_latches_failure_without_formatting_the_exception() -> None:
    clock = SimulationClock()
    clock.accept(10)
    string_calls = 0

    class CountingFailure(BaseException):
        def __str__(self) -> str:
            nonlocal string_calls
            string_calls += 1
            return "publisher failed"

    failure = CountingFailure()

    def fail(_record: object) -> None:
        raise failure

    emit = MissionEventEmitter(RUN_ID, clock, fail)

    with pytest.raises(CountingFailure) as raised:
        emit("FM1", "STARTED")

    assert raised.value is failure
    assert string_calls == 0
    with pytest.raises(
        RuntimeError,
        match="mission event emitter stopped: publication failed: CountingFailure",
    ):
        emit("FM1", "STARTED")


def test_phase_emitter_publish_callback_can_stop_without_deadlock_or_duplicate() -> None:
    clock = SimulationClock()
    clock.accept(10)
    attempts: list[object] = []
    errors: list[BaseException] = []
    emit: MissionEventEmitter

    def publish(record: object) -> None:
        attempts.append(record)
        emit.stop("publisher requested stop")

    emit = MissionEventEmitter(RUN_ID, clock, publish)

    def invoke() -> None:
        try:
            emit("FM1", "STARTED")
        except BaseException as error:
            errors.append(error)

    worker = threading.Thread(target=invoke, daemon=True)
    worker.start()
    worker.join(0.5)

    assert not worker.is_alive()
    assert errors == []
    assert len(attempts) == 1
    with pytest.raises(RuntimeError, match="publisher requested stop"):
        emit("FM1", "COMPLETE")
    assert len(attempts) == 1


def test_phase_emitter_reentrant_emit_fails_closed_without_duplicate_or_retry() -> None:
    clock = SimulationClock()
    clock.accept(10)
    attempts: list[object] = []
    emit: MissionEventEmitter

    def publish(record: object) -> None:
        attempts.append(record)
        emit("FM1", "STARTED")

    emit = MissionEventEmitter(RUN_ID, clock, publish)

    with pytest.raises(RuntimeError, match="publication is already in progress"):
        emit("FM1", "STARTED")
    with pytest.raises(RuntimeError, match="publication failed"):
        emit("FM1", "STARTED")
    assert len(attempts) == 1


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


def test_start_gate_requires_command_delivery_after_all_other_predicates() -> None:
    gate = Comp2026StartGate()
    gate.mark_process_ready()

    assert gate.mission_ready is True
    assert gate.mission_start_ready is False
    assert gate.readiness == {
        "process_ready": True,
        "running": False,
        "clock": False,
        "command_delivered": False,
        "frame_ready": False,
        "range_ready": False,
        "payload_service_ready": False,
        "heartbeat_live": False,
        "armable": False,
    }

    gate.accept_running()
    gate.accept_clock()
    gate.refresh_live_readiness(
        frame_ready=True,
        payload_service_ready=True,
        heartbeat_live=False,
        armable=True,
        range_is_current=lambda: True,
    )
    assert gate.mission_start_ready is False

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
    assert gate.mission_start_ready is False

    gate.mark_command_delivered()
    gate.mark_command_delivered()
    assert gate.mission_start_ready is True
    assert gate.readiness["command_delivered"] is True


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
    gate.mark_command_delivered()
    clock = SimulationClock()
    clock.accept(1_000_000_000)
    lidar = make_lidar(clock)
    lidar.accept(
        SimpleNamespace(ranges=[4.572], range_min=0.1, range_max=30.0),
        1_000_000_000,
    )
    frame_source = RosFrameSource(width_px=640, height_px=480)
    frame_source.accept_image(rgb_image(1_000_000_000))
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
