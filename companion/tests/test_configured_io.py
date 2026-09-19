from pathlib import Path
from types import SimpleNamespace

import pytest

from drone_sim_companion import configured_io


RUN_ID = "00000000-0000-0000-0000-000000000001"
MISSION_SEQUENCE = (
    ("FM1", "STARTED"),
    ("FM1", "COMPLETE"),
    ("FM2", "STARTED"),
    ("FM2", "COMPLETE"),
    ("FM3_3", "STARTED"),
    ("FM3_3", "COMPLETE"),
    ("FM3_4", "STARTED"),
    ("FM3_4", "COMPLETE"),
    ("HOME", "STARTED"),
    ("HOME", "DISARMED"),
    ("HOME", "COMPLETE"),
)


class FakeClock:
    def __init__(self):
        self.timestamp_ns = None

    def accept(self, timestamp_ns):
        if self.timestamp_ns is not None and timestamp_ns < self.timestamp_ns:
            raise ValueError("clock regressed")
        self.timestamp_ns = timestamp_ns

    def read_timestamp_ns(self):
        if self.timestamp_ns is None:
            raise RuntimeError("clock unavailable")
        return self.timestamp_ns


class FakeFrameSource:
    def __init__(self, *, width_px, height_px):
        assert (width_px, height_px) == (640, 480)
        self.accepted = []
        self.latest = None
        self.last_timestamp_ns = None
        self.stopped = False

    def accept_image(self, message):
        self.accepted.append(message)
        timestamp_ns = (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec
        )
        if self.last_timestamp_ns is None or timestamp_ns > self.last_timestamp_ns:
            self.latest = message

    @property
    def ready(self):
        return not self.stopped and self.latest is not None

    def retain_latest_at_or_after(self, earliest_timestamp_ns):
        if self.latest is None:
            return False
        timestamp_ns = (
            self.latest.header.stamp.sec * 1_000_000_000
            + self.latest.header.stamp.nanosec
        )
        if timestamp_ns < earliest_timestamp_ns:
            self.last_timestamp_ns = timestamp_ns
            self.latest = None
            return False
        return True

    def consume_timestamp(self):
        assert self.latest is not None
        timestamp_ns = (
            self.latest.header.stamp.sec * 1_000_000_000
            + self.latest.header.stamp.nanosec
        )
        self.last_timestamp_ns = timestamp_ns
        self.latest = None
        return timestamp_ns

    def stop(self, _reason):
        self.stopped = True


class FakeLidar:
    def __init__(self, _clock, *, sample_factory):
        self.sample_factory = sample_factory
        self.sample = SimpleNamespace(sampled_at=0.0)

    def accept(self, _message, _timestamp_ns):
        return None

    def get_sample(self):
        return self.sample


class FakeAdapterStaleSensorError(RuntimeError):
    pass


class FakeCameraManager:
    pass


class FakeCamera:
    def __init__(self, frame_source):
        self.start_calls = 0
        self.capture_calls = 0
        self.capture_error = None
        self.frame_source = frame_source
        self.cm = SimpleNamespace(
            start_acquisition=self.start_acquisition,
            latest_observation=lambda **_kwargs: (_ for _ in ()).throw(TimeoutError()),
            capture_observation=self.capture_observation,
            stop_acquisition=lambda **_kwargs: True,
        )

    def start_acquisition(self, **_kwargs):
        self.start_calls += 1

    def capture_observation(self, **_kwargs):
        self.capture_calls += 1
        if self.capture_error is not None:
            raise self.capture_error
        timestamp_ns = self.frame_source.consume_timestamp()
        return SimpleNamespace(
            metadata=SimpleNamespace(
                sequence=self.capture_calls,
                exposure_timestamp_ns=timestamp_ns,
            )
        )

    def vector_from_observation_3d(self, _observation, _aruco_id):
        return SimpleNamespace(x=0.1, y=-0.2, z=3.0)


class FakeRequest:
    ATTACH = 1
    RELEASE = 2


class FakePayloadCommand:
    Request = FakeRequest


class FakeClient:
    def service_is_ready(self):
        return True

    def call_async(self, _request):
        raise AssertionError("payload service should not be called")


class FakePublisher:
    def __init__(self):
        self.messages = []
        self.ack_timeouts = []

    def publish(self, message):
        self.messages.append(message)

    def wait_for_all_acked(self, *, timeout):
        self.ack_timeouts.append(timeout)
        return True


class FakeMissionEvent:
    def __init__(self):
        self.sim_timestamp = SimpleNamespace(sec=0, nanosec=0)


class FakeQoSProfile:
    def __init__(self, *, depth, reliability, durability="volatile"):
        self.depth = depth
        self.reliability = reliability
        self.durability = durability


class FakeNode:
    def __init__(self):
        self.callbacks = {}
        self.publisher = FakePublisher()
        qos = SimpleNamespace(reliability="reliable", durability="transient")
        self.endpoints = [
            SimpleNamespace(
                node_name="drone_sim_scorekeeper",
                node_namespace="/",
                topic_type="simulation_interfaces/msg/MissionEvent",
                qos_profile=qos,
            ),
            SimpleNamespace(
                node_name="rosbag2_recorder_run",
                node_namespace="/",
                topic_type="simulation_interfaces/msg/MissionEvent",
                qos_profile=qos,
            ),
        ]

    def create_client(self, _service_type, topic):
        assert topic == "/simulation/payload_command"
        return FakeClient()

    def create_subscription(self, _message_type, topic, callback, _qos):
        self.callbacks[topic] = callback
        return object()

    def create_publisher(self, _message_type, topic, qos):
        assert topic == "/simulation/mission_events"
        assert (qos.depth, qos.reliability, qos.durability) == (
            100,
            "reliable",
            "transient",
        )
        return self.publisher

    def get_subscriptions_info_by_topic(self, topic):
        assert topic == "/simulation/mission_events"
        return self.endpoints


def dependencies():
    return SimpleNamespace(
        SimulationClock=FakeClock,
        RosFrameSource=FakeFrameSource,
        RosLidar=FakeLidar,
        CameraManager=FakeCameraManager,
        Camera=FakeCamera,
        LidarSample=lambda *values: values,
        AttitudeSample=lambda **values: SimpleNamespace(**values),
        ClearanceCalibration=lambda **values: SimpleNamespace(**values),
        ClearanceUnavailableError=RuntimeError,
        StaleSensorError=FakeAdapterStaleSensorError,
        project_vertical_clearance=lambda *_args, **_kwargs: None,
        create_simulator_camera=lambda _manager, _camera, source, **_kwargs: FakeCamera(
            source
        ),
        Image=object,
        LaserScan=object,
        PayloadState=object,
        PayloadCommand=FakePayloadCommand,
        MissionEvent=FakeMissionEvent,
        QoSProfile=FakeQoSProfile,
        ReliabilityPolicy=SimpleNamespace(RELIABLE="reliable"),
        DurabilityPolicy=SimpleNamespace(
            VOLATILE="volatile", TRANSIENT_LOCAL="transient"
        ),
        Duration=lambda *, seconds: seconds,
    )


def make_io(monkeypatch, *, scenario="competition_v1"):
    monkeypatch.setattr(configured_io, "_load_live_dependencies", dependencies)
    node = FakeNode()
    config = SimpleNamespace(
        run_id=RUN_ID,
        mission="configured",
        scenario=scenario,
        scenario_path=Path("/unused/scenario.yaml"),
        finalization_wall_seconds=3.0,
    )
    return configured_io.CompetitionIO(config, node, lambda *_args: None), node


def image(timestamp_ns):
    seconds, nanoseconds = divmod(timestamp_ns, 1_000_000_000)
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=seconds, nanosec=nanoseconds)
        )
    )


def test_publish_event_enforces_full_competition_grammar(monkeypatch):
    bridge, node = make_io(monkeypatch)

    assert bridge.ready is True
    with pytest.raises(ValueError, match="next mission event"):
        bridge.publish_event("FM2", "STARTED", 1)

    for timestamp_ns, (phase, state) in enumerate(MISSION_SEQUENCE, start=1):
        bridge.publish_event(phase, state, timestamp_ns)

    assert [
        (message.event_id, message.phase, message.state)
        for message in node.publisher.messages
    ] == [
        (event_id, phase, state)
        for event_id, (phase, state) in enumerate(MISSION_SEQUENCE)
    ]
    assert all(message.run_id == RUN_ID for message in node.publisher.messages)
    assert [message.sim_timestamp.nanosec for message in node.publisher.messages] == list(
        range(1, 12)
    )
    with pytest.raises(RuntimeError, match="sequence is complete"):
        bridge.publish_event("HOME", "COMPLETE", 12)


def test_publish_event_requires_strictly_new_clock_time(monkeypatch):
    bridge, _node = make_io(monkeypatch)
    bridge.publish_event("FM1", "STARTED", 10)

    with pytest.raises(ValueError, match="timestamp must increase"):
        bridge.publish_event("FM1", "COMPLETE", 10)


def test_future_images_are_staged_until_clock_reaches_their_source_time(monkeypatch):
    bridge, node = make_io(monkeypatch)
    callback = node.callbacks["/camera/onboard/image_raw"]
    at_15 = image(15)
    at_20 = image(20)

    bridge.accept_clock(10)
    callback(at_20)
    callback(at_15)
    assert bridge._frame_source.accepted == []

    bridge.accept_clock(15)
    assert bridge._frame_source.accepted == [at_15]
    bridge.accept_clock(19)
    assert bridge._frame_source.accepted == [at_15]
    bridge.accept_clock(20)
    assert bridge._frame_source.accepted == [at_15, at_20]


def test_marker_discards_stale_ready_frame_without_starting_worker(monkeypatch):
    bridge, node = make_io(monkeypatch)
    callback = node.callbacks["/camera/onboard/image_raw"]
    stale = image(99_999_999)
    bridge.accept_clock(600_000_000)
    callback(stale)

    assert bridge.marker(3) is None
    assert bridge._frame_source.ready is False
    assert bridge._frame_source.last_timestamp_ns == 99_999_999
    assert bridge._camera.capture_calls == 0


def test_marker_synchronously_captures_frame_at_inclusive_freshness_bound(monkeypatch):
    bridge, node = make_io(monkeypatch)
    callback = node.callbacks["/camera/onboard/image_raw"]
    bridge.accept_clock(600_000_000)
    callback(image(100_000_000))

    marker = bridge.marker(3)

    assert marker is not None
    assert (marker.timestamp_ns, marker.sequence, marker.aruco_id) == (
        100_000_000,
        1,
        3,
    )
    assert (marker.forward_m, marker.right_m, marker.down_m) == (0.1, -0.2, 3.0)
    assert bridge._camera.capture_calls == 1


def test_marker_without_ready_frame_does_not_start_or_block_on_camera(monkeypatch):
    bridge, _node = make_io(monkeypatch)
    bridge.accept_clock(600_000_000)

    assert bridge.marker(3) is None
    assert bridge._camera.start_calls == 0
    assert bridge._camera.capture_calls == 0


def test_marker_propagates_synchronous_camera_failure(monkeypatch):
    bridge, node = make_io(monkeypatch)
    callback = node.callbacks["/camera/onboard/image_raw"]
    bridge.accept_clock(600_000_000)
    callback(image(600_000_000))
    bridge._camera.capture_error = ValueError("malformed camera frame")

    with pytest.raises(ValueError, match="malformed camera frame"):
        bridge.marker(3)


def test_clearance_uses_oldest_actual_vehicle_attitude_timestamp(monkeypatch):
    from drone_sim_companion.mission import Telemetry
    from drone_sim_companion.operations import DroneOperations

    bridge, _node = make_io(monkeypatch)
    operations = DroneOperations(SimpleNamespace())
    operations.observe(Telemetry(100, heartbeat=True, roll_rad=0.01))
    operations.observe(Telemetry(120, heartbeat=True, pitch_rad=0.02))
    operations.observe(Telemetry(140, heartbeat=True, yaw_rad=0.03))
    state = operations.read_vehicle_state()
    bridge._lidar.sample = SimpleNamespace(sampled_at=190 / 1_000_000_000)
    observed_attitudes = []

    def project(sample, attitude, _calibration, *, now):
        observed_attitudes.append((attitude, now))
        return SimpleNamespace(projected_clearance_m=10.1, range_sample=sample)

    bridge._deps.project_vertical_clearance = project

    assert bridge.clearance(state, 200) == (10.1, 100)
    assert observed_attitudes[0][0].sampled_at == 100 / 1_000_000_000


@pytest.mark.parametrize("detail", ["downward range is not ready", "range is stale"])
def test_clearance_returns_none_for_adapter_range_unavailability(monkeypatch, detail):
    bridge, _node = make_io(monkeypatch)
    bridge._lidar.get_sample = lambda: (_ for _ in ()).throw(
        FakeAdapterStaleSensorError(detail)
    )
    state = {
        "roll_rad": 0.0,
        "pitch_rad": 0.0,
        "yaw_rad": 0.0,
        "observed_at_ns": {"roll_rad": 1, "pitch_rad": 1, "yaw_rad": 1},
    }

    assert bridge.clearance(state, 1) is None


def test_flush_rejects_incomplete_event_grammar(monkeypatch):
    bridge, node = make_io(monkeypatch)

    with pytest.raises(RuntimeError, match="mission event sequence is incomplete"):
        bridge.flush()
    assert node.publisher.ack_timeouts == []

    for timestamp_ns, (phase, state) in enumerate(MISSION_SEQUENCE, start=1):
        bridge.publish_event(phase, state, timestamp_ns)
    bridge.flush()
    assert node.publisher.ack_timeouts == [3.0]


def test_search_delivery_requires_its_seven_events_before_flush(monkeypatch):
    bridge, node = make_io(monkeypatch, scenario="search_delivery_v1")
    with pytest.raises(ValueError, match="next mission event"):
        bridge.publish_event("FM1", "STARTED", 1)
    sequence = (
        ("SEARCH", "STARTED"), ("SEARCH", "COMPLETE"),
        ("DELIVERY", "STARTED"), ("DELIVERY", "COMPLETE"),
        ("HOME", "STARTED"), ("HOME", "DISARMED"), ("HOME", "COMPLETE"),
    )
    for timestamp, event in enumerate(sequence[:-1], start=1):
        bridge.publish_event(*event, timestamp)
    with pytest.raises(RuntimeError, match="sequence is incomplete"):
        bridge.flush()
    bridge.publish_event(*sequence[-1], 7)
    bridge.flush()
    assert [(msg.phase, msg.state) for msg in node.publisher.messages] == list(sequence)
    assert node.publisher.ack_timeouts == [3.0]
