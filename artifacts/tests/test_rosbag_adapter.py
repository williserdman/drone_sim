from dataclasses import FrozenInstanceError, replace
import importlib
import os
from pathlib import Path
from types import SimpleNamespace
import signal
import subprocess
from uuid import UUID

import pytest

from artifacts._adapters.rosbag import (
    FIXED_TOPIC_TYPES,
    FIXED_TOPICS,
    BagMessage,
    BagMetadata,
    BagTopicMetadata,
    RosbagRecorder,
    RosbagValidator,
)
from artifacts.validation import ValidationStatus


RECORDER_UUID = UUID("01234567-89ab-cdef-0123-456789abcdef")
RUN_ID = "run-7"


class FakeProcess:
    def __init__(self, wait_outcomes=(), *, advance=None):
        self.returncode = None
        self.wait_outcomes = list(wait_outcomes)
        self.wait_timeouts = []
        self.advance = advance or (lambda seconds: None)

    def poll(self):
        return self.returncode

    def wait(self, timeout):
        self.wait_timeouts.append(timeout)
        outcome = self.wait_outcomes.pop(0) if self.wait_outcomes else 0
        if outcome == "timeout":
            self.advance(timeout)
            raise subprocess.TimeoutExpired("ros2 bag record", timeout)
        self.returncode = outcome
        return outcome


class FakeProcessFactory:
    def __init__(self, process=None):
        self.process = process or FakeProcess()
        self.calls = []

    def __call__(self, command, **kwargs):
        self.calls.append((tuple(command), kwargs))
        return self.process


class FakeClock:
    def __init__(self, now=0.0):
        self.now = now

    def monotonic(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class Endpoint:
    def __init__(self, node_name, qos_profile="compatible"):
        self.node_name = node_name
        self.qos_profile = qos_profile


class FakeGraph:
    def __init__(self, recorder_node, *, missing_topic=None, incompatible_topic=None):
        self.recorder_node = recorder_node
        self.nodes = [(recorder_node, "/")]
        self.missing_topic = missing_topic
        self.incompatible_topic = incompatible_topic

    def get_node_names_and_namespaces(self):
        return list(self.nodes)

    def get_subscriptions_info_by_topic(self, topic):
        if topic == self.missing_topic:
            return []
        return [Endpoint(self.recorder_node)]

    def get_publishers_info_by_topic(self, topic):
        qos = "incompatible" if topic == self.incompatible_topic else "compatible"
        return [Endpoint("publisher", qos)]


def _recorder(tmp_path, **changes):
    values = {
        "run_directory": tmp_path,
        "process_factory": FakeProcessFactory(),
        "uuid_factory": lambda: RECORDER_UUID,
        "qos_compatible": lambda offered, requested: offered == requested,
    }
    values.update(changes)
    return RosbagRecorder(**values)


def test_command_has_absolute_output_qos_node_and_frozen_topic_order(tmp_path):
    recorder = _recorder(tmp_path)

    assert recorder.command() == (
        "ros2",
        "bag",
        "record",
        "--storage",
        "mcap",
        "--output",
        str((tmp_path / "rosbag").resolve()),
        "--disable-keyboard-controls",
        "--include-unpublished-topics",
        "--qos-profile-overrides-path",
        "/etc/drone_sim/recording-qos.yaml",
        "--node-name",
        "rosbag2_recorder_0123456789abcdef0123456789abcdef",
        "--topics",
        *FIXED_TOPICS,
    )
    assert "--use-sim-time" not in recorder.command()


def test_start_uses_shell_free_process_and_appends_combined_recorder_log(tmp_path):
    log_path = tmp_path / "logs/docker/rosbag2.log.partial"
    log_path.parent.mkdir(parents=True)
    log_path.write_bytes(b"existing diagnostics\n")
    factory = FakeProcessFactory()
    recorder = _recorder(tmp_path, process_factory=factory)

    recorder.start()

    command, kwargs = factory.calls[0]
    assert command == recorder.command()
    assert kwargs["shell"] is False
    assert kwargs["stderr"] is subprocess.STDOUT
    assert kwargs["stdin"] is subprocess.DEVNULL
    assert kwargs["stdout"].name == str(log_path)
    assert log_path.read_bytes() == b"existing diagnostics\n"


def test_start_refuses_existing_nonempty_bag_directory_without_spawning(tmp_path):
    (tmp_path / "rosbag").mkdir()
    (tmp_path / "rosbag/existing.mcap").write_bytes(b"do not overwrite")
    factory = FakeProcessFactory()
    recorder = _recorder(tmp_path, process_factory=factory)

    with pytest.raises(FileExistsError, match="nonempty"):
        recorder.start()

    assert factory.calls == []
    assert (tmp_path / "rosbag/existing.mcap").read_bytes() == b"do not overwrite"


def test_readiness_requires_live_process_recorder_node_and_all_subscriptions(tmp_path):
    factory = FakeProcessFactory()
    recorder = _recorder(tmp_path, process_factory=factory)
    recorder.start()
    graph = FakeGraph(recorder.node_name)

    assert recorder.is_ready(graph) is True

    factory.process.returncode = 2
    assert recorder.is_ready(graph) is False


def test_readiness_is_false_when_recorder_node_is_absent(tmp_path):
    recorder = _recorder(tmp_path)
    recorder.start()
    graph = FakeGraph(recorder.node_name)
    graph.nodes = [("some_other_node", "/")]

    assert recorder.is_ready(graph) is False


def test_readiness_is_false_when_one_frozen_topic_lacks_recorder_subscription(tmp_path):
    recorder = _recorder(tmp_path)
    recorder.start()
    graph = FakeGraph(recorder.node_name, missing_topic=FIXED_TOPICS[-1])

    assert recorder.is_ready(graph) is False


def test_readiness_is_false_when_offered_and_requested_qos_are_incompatible(tmp_path):
    recorder = _recorder(tmp_path)
    recorder.start()
    graph = FakeGraph(recorder.node_name, incompatible_topic=FIXED_TOPICS[0])

    assert recorder.is_ready(graph) is False


def test_finalize_sends_only_sigint_when_recorder_exits_gracefully(tmp_path):
    clock = FakeClock(10.0)
    process = FakeProcess(wait_outcomes=(0,), advance=clock.advance)
    signals = []
    recorder = _recorder(
        tmp_path,
        process_factory=FakeProcessFactory(process),
        monotonic=clock.monotonic,
        signal_sender=lambda proc, signum: signals.append(signum),
    )
    recorder.start()

    result = recorder.finalize(deadline=19.0)

    assert signals == [signal.SIGINT]
    assert result.exited is True
    assert result.returncode == 0
    assert result.signals == ("SIGINT",)
    assert result.escalated is False
    assert sum(process.wait_timeouts) <= 9.0


def test_finalize_escalates_within_one_shared_absolute_deadline(tmp_path):
    clock = FakeClock(10.0)
    process = FakeProcess(
        wait_outcomes=("timeout", "timeout", -signal.SIGKILL),
        advance=clock.advance,
    )
    signals = []
    recorder = _recorder(
        tmp_path,
        process_factory=FakeProcessFactory(process),
        monotonic=clock.monotonic,
        signal_sender=lambda proc, signum: signals.append(signum),
    )
    recorder.start()

    result = recorder.finalize(deadline=19.0)

    assert signals == [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]
    assert result.exited is True
    assert result.returncode == -signal.SIGKILL
    assert result.signals == ("SIGINT", "SIGTERM", "SIGKILL")
    assert result.escalated is True
    assert all(timeout >= 0 for timeout in process.wait_timeouts)
    assert sum(process.wait_timeouts) <= 9.0
    assert clock.monotonic() <= 19.0


def test_finalize_reports_exit_when_process_ends_during_signal_race(tmp_path):
    process = FakeProcess()

    def exit_before_signal_delivery(proc, signum):
        proc.returncode = 0
        raise ProcessLookupError("process exited")

    recorder = _recorder(
        tmp_path,
        process_factory=FakeProcessFactory(process),
        signal_sender=exit_before_signal_delivery,
    )
    recorder.start()

    result = recorder.finalize(deadline=10.0)

    assert result.exited is True
    assert result.returncode == 0
    assert result.signals == ()


def _stamp(ns):
    return SimpleNamespace(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def _custom_message(timestamp_ns, *, run_id=RUN_ID, **fields):
    return SimpleNamespace(run_id=run_id, sim_timestamp=_stamp(timestamp_ns), **fields)


def _image(timestamp_ns, *, data=b"rgb"):
    return SimpleNamespace(header=SimpleNamespace(stamp=_stamp(timestamp_ns)), data=data)


def _valid_messages():
    return [
        BagMessage("/clock", SimpleNamespace(clock=_stamp(0)), 100),
        BagMessage("/simulation/run_state", _custom_message(0), 101),
        BagMessage("/simulation/artifact_status", _custom_message(0), 102),
        BagMessage("/simulation/ground_truth", _custom_message(0), 103),
        BagMessage("/simulation/scenario_events", _custom_message(0), 104),
        BagMessage("/simulation/score_events", _custom_message(0), 105),
        BagMessage("/camera/onboard/image_raw", _image(0), 106),
        BagMessage(
            "/camera/onboard/frame_metadata",
            _custom_message(0, frame_id=0, stream="onboard"),
            107,
        ),
        BagMessage("/camera/observer/image_raw", _image(0), 108),
        BagMessage(
            "/camera/observer/frame_metadata",
            _custom_message(0, frame_id=0, stream="observer"),
            109,
        ),
    ]


def _metadata_for(messages, *, storage_id="mcap", type_overrides=None):
    type_overrides = type_overrides or {}
    return BagMetadata(
        storage_id=storage_id,
        topics=tuple(
            BagTopicMetadata(
                topic,
                type_overrides.get(topic, FIXED_TOPIC_TYPES[topic]),
                sum(message.topic == topic for message in messages),
            )
            for topic in FIXED_TOPICS
        ),
    )


class FakeBagBackend:
    def __init__(self, messages=None, metadata=None, *, read_error=None):
        self.messages = list(messages or _valid_messages())
        self.metadata = metadata or _metadata_for(self.messages)
        self.read_error = read_error

    def read_metadata(self, bag_directory):
        return self.metadata

    def read_messages(self, bag_directory, storage_id, topic_types):
        if self.read_error is not None:
            raise self.read_error
        return iter(self.messages)


def _bag_directory(tmp_path):
    bag = tmp_path / "rosbag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
    (bag / "bag_0.mcap").write_bytes(b"synthetic fixture")
    return bag


def _validate(tmp_path, backend):
    return RosbagValidator(RUN_ID, backend=backend).validate(tmp_path, "rosbag")


def _require_ros_module(name):
    try:
        return importlib.import_module(name)
    except ImportError as error:
        if os.environ.get("DRONE_SIM_REQUIRE_ROS_TESTS") == "1":
            pytest.fail(f"ROS test dependency is unavailable in the test image: {error}")
        pytest.skip(f"ROS test dependencies are unavailable on the host: {error}")


def test_valid_bag_returns_immutable_topic_count_type_and_timestamp_diagnostics(tmp_path):
    _bag_directory(tmp_path)

    result = _validate(tmp_path, FakeBagBackend())

    assert result.status is ValidationStatus.VALID
    assert tuple(topic.name for topic in result.topics) == FIXED_TOPICS
    assert all(topic.message_count == 1 for topic in result.topics)
    assert result.topics[0].message_type == "rosgraph_msgs/msg/Clock"
    assert result.topics[0].first_sim_timestamp_ns == 0
    assert result.topics[0].last_sim_timestamp_ns == 0
    with pytest.raises(FrozenInstanceError):
        result.topics[0].message_count = 2


def test_validation_rejects_missing_metadata_file_before_opening_bag(tmp_path):
    bag = tmp_path / "rosbag"
    bag.mkdir()
    (bag / "bag_0.mcap").write_bytes(b"bag")

    result = _validate(tmp_path, FakeBagBackend())

    assert result.status is ValidationStatus.INVALID
    assert "metadata" in result.detail


def test_validation_rejects_non_mcap_storage(tmp_path):
    _bag_directory(tmp_path)
    backend = FakeBagBackend()
    backend.metadata = replace(backend.metadata, storage_id="sqlite3")

    result = _validate(tmp_path, backend)

    assert result.status is ValidationStatus.INVALID
    assert "MCAP" in result.detail


@pytest.mark.parametrize("change", ["missing", "extra"])
def test_validation_rejects_any_topic_inventory_difference(tmp_path, change):
    _bag_directory(tmp_path)
    backend = FakeBagBackend()
    topics = list(backend.metadata.topics)
    if change == "missing":
        topics.pop()
    else:
        topics.append(BagTopicMetadata("/unexpected", "std_msgs/msg/String", 1))
    backend.metadata = replace(backend.metadata, topics=tuple(topics))

    result = _validate(tmp_path, backend)

    assert result.status is ValidationStatus.INVALID
    assert "topic inventory" in result.detail


def test_validation_rejects_wrong_topic_type(tmp_path):
    _bag_directory(tmp_path)
    backend = FakeBagBackend()
    topics = list(backend.metadata.topics)
    topics[0] = replace(topics[0], message_type="std_msgs/msg/String")
    backend.metadata = replace(backend.metadata, topics=tuple(topics))

    result = _validate(tmp_path, backend)

    assert result.status is ValidationStatus.INVALID
    assert "type" in result.detail


def test_validation_rejects_zero_required_topic_count(tmp_path):
    _bag_directory(tmp_path)
    backend = FakeBagBackend()
    topics = list(backend.metadata.topics)
    topics[0] = replace(topics[0], message_count=0)
    backend.metadata = replace(backend.metadata, topics=tuple(topics))

    result = _validate(tmp_path, backend)

    assert result.status is ValidationStatus.INVALID
    assert "zero messages" in result.detail


def test_validation_rejects_unreadable_serialized_message(tmp_path):
    _bag_directory(tmp_path)

    result = _validate(
        tmp_path,
        FakeBagBackend(read_error=RuntimeError("CDR deserialization failed")),
    )

    assert result.status is ValidationStatus.INVALID
    assert "serialized message" in result.detail


def test_validation_rejects_wrong_run_id(tmp_path):
    _bag_directory(tmp_path)
    backend = FakeBagBackend()
    backend.messages[1] = replace(
        backend.messages[1], message=_custom_message(0, run_id="stale-run")
    )

    result = _validate(tmp_path, backend)

    assert result.status is ValidationStatus.INVALID
    assert "run_id" in result.detail


def test_validation_rejects_nonmonotonic_custom_simulation_timestamps(tmp_path):
    _bag_directory(tmp_path)
    messages = _valid_messages()
    messages.insert(
        2,
        BagMessage("/simulation/run_state", _custom_message(10), 102),
    )
    messages.insert(
        3,
        BagMessage("/simulation/run_state", _custom_message(9), 103),
    )
    backend = FakeBagBackend(messages=messages, metadata=_metadata_for(messages))

    result = _validate(tmp_path, backend)

    assert result.status is ValidationStatus.INVALID
    assert "nonmonotonic" in result.detail


def test_validation_rejects_image_without_payload(tmp_path):
    _bag_directory(tmp_path)
    backend = FakeBagBackend()
    backend.messages[6] = replace(backend.messages[6], message=_image(0, data=b""))

    result = _validate(tmp_path, backend)

    assert result.status is ValidationStatus.INVALID
    assert "image payload" in result.detail


def test_validation_rejects_mismatched_image_and_metadata_counts(tmp_path):
    _bag_directory(tmp_path)
    messages = _valid_messages()
    messages.append(BagMessage("/camera/onboard/image_raw", _image(50_000_000), 110))
    backend = FakeBagBackend(messages=messages, metadata=_metadata_for(messages))

    result = _validate(tmp_path, backend)

    assert result.status is ValidationStatus.INVALID
    assert "image/metadata count" in result.detail


def test_validation_rejects_unpaired_image_and_metadata_timestamps(tmp_path):
    _bag_directory(tmp_path)
    backend = FakeBagBackend()
    backend.messages[7] = replace(
        backend.messages[7],
        message=_custom_message(50_000_000, frame_id=0, stream="onboard"),
    )

    result = _validate(tmp_path, backend)

    assert result.status is ValidationStatus.INVALID
    assert "image/metadata timestamps" in result.detail


def test_validation_rejects_wrong_frame_metadata_stream(tmp_path):
    _bag_directory(tmp_path)
    backend = FakeBagBackend()
    backend.messages[7] = replace(
        backend.messages[7],
        message=_custom_message(0, frame_id=0, stream="observer"),
    )

    result = _validate(tmp_path, backend)

    assert result.status is ValidationStatus.INVALID
    assert "wrong stream" in result.detail


def test_validation_rejects_noncontiguous_frame_ids(tmp_path):
    _bag_directory(tmp_path)
    backend = FakeBagBackend()
    backend.messages[7] = replace(
        backend.messages[7],
        message=_custom_message(0, frame_id=1, stream="onboard"),
    )

    result = _validate(tmp_path, backend)

    assert result.status is ValidationStatus.INVALID
    assert "frame IDs" in result.detail


def test_validation_rejects_frame_interval_other_than_50_ms(tmp_path):
    _bag_directory(tmp_path)
    messages = _valid_messages()
    messages.extend(
        [
            BagMessage("/camera/onboard/image_raw", _image(40_000_000), 110),
            BagMessage(
                "/camera/onboard/frame_metadata",
                _custom_message(
                    40_000_000, frame_id=1, stream="onboard"
                ),
                111,
            ),
        ]
    )
    backend = FakeBagBackend(messages=messages, metadata=_metadata_for(messages))

    result = _validate(tmp_path, backend)

    assert result.status is ValidationStatus.INVALID
    assert "50 ms" in result.detail


def test_real_jazzy_mcap_fixture_is_read_via_rosbag2_and_deserialized(tmp_path):
    rosbag2_py = _require_ros_module("rosbag2_py")
    _require_ros_module("simulation_interfaces")
    from builtin_interfaces.msg import Time
    from rclpy.serialization import serialize_message
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image
    from std_msgs.msg import Header
    from simulation_interfaces.msg import (
        ArtifactStatus,
        FrameMetadata,
        GroundTruth,
        RunState,
        ScenarioEvent,
        ScoreEvent,
    )

    bag_path = tmp_path / "rosbag"
    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="", output_serialization_format=""
    )
    writer.open(storage_options, converter_options)
    for index, topic in enumerate(FIXED_TOPICS):
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=index,
                name=topic,
                type=FIXED_TOPIC_TYPES[topic],
                serialization_format="cdr",
                offered_qos_profiles=[],
                type_description_hash="",
            )
        )

    stamp = Time(sec=0, nanosec=0)
    messages = {
        "/clock": Clock(clock=stamp),
        "/simulation/run_state": RunState(run_id=RUN_ID, sim_timestamp=stamp),
        "/simulation/artifact_status": ArtifactStatus(
            run_id=RUN_ID, sim_timestamp=stamp
        ),
        "/simulation/ground_truth": GroundTruth(run_id=RUN_ID, sim_timestamp=stamp),
        "/simulation/scenario_events": ScenarioEvent(
            run_id=RUN_ID, sim_timestamp=stamp
        ),
        "/simulation/score_events": ScoreEvent(run_id=RUN_ID, sim_timestamp=stamp),
        "/camera/onboard/image_raw": Image(
            header=Header(stamp=stamp),
            height=1,
            width=1,
            encoding="rgb8",
            step=3,
            data=[1, 2, 3],
        ),
        "/camera/onboard/frame_metadata": FrameMetadata(
            run_id=RUN_ID, sim_timestamp=stamp, frame_id=0, stream="onboard"
        ),
        "/camera/observer/image_raw": Image(
            header=Header(stamp=stamp),
            height=1,
            width=1,
            encoding="rgb8",
            step=3,
            data=[4, 5, 6],
        ),
        "/camera/observer/frame_metadata": FrameMetadata(
            run_id=RUN_ID, sim_timestamp=stamp, frame_id=0, stream="observer"
        ),
    }
    for recorded_timestamp, topic in enumerate(FIXED_TOPICS, start=1):
        writer.write(topic, serialize_message(messages[topic]), recorded_timestamp)
    writer.close()

    result = RosbagValidator(RUN_ID).validate(tmp_path, "rosbag")

    assert result.status is ValidationStatus.VALID
    assert [(topic.name, topic.message_count) for topic in result.topics] == [
        (topic, 1) for topic in FIXED_TOPICS
    ]


def test_real_jazzy_qos_api_rejects_best_effort_offer_for_reliable_request(tmp_path):
    qos = _require_ros_module("rclpy.qos")
    process_factory = FakeProcessFactory()
    recorder = RosbagRecorder(
        tmp_path,
        process_factory=process_factory,
        uuid_factory=lambda: RECORDER_UUID,
    )
    recorder.start()
    graph = FakeGraph(recorder.node_name)
    offered = qos.QoSProfile(
        depth=1,
        reliability=qos.ReliabilityPolicy.BEST_EFFORT,
    )
    requested = qos.QoSProfile(
        depth=1,
        reliability=qos.ReliabilityPolicy.RELIABLE,
    )
    graph.get_publishers_info_by_topic = lambda topic: [
        Endpoint("publisher", offered)
    ]
    graph.get_subscriptions_info_by_topic = lambda topic: [
        Endpoint(recorder.node_name, requested)
    ]

    assert recorder.is_ready(graph) is False
