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
from artifacts.validation import ValidationStatus, validate_tree


RECORDER_UUID = UUID("01234567-89ab-cdef-0123-456789abcdef")
RUN_ID = "run-7"
CONFIG_SHA256 = "a" * 64


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
    def __init__(
        self,
        node_name,
        topic_type,
        qos_profile="compatible",
        *,
        endpoint_type="subscription",
    ):
        self.node_name = node_name
        self.node_namespace = "/"
        self.topic_type = topic_type
        self.endpoint_type = endpoint_type
        self.endpoint_gid = bytes(24)
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
        return [Endpoint(self.recorder_node, FIXED_TOPIC_TYPES[topic])]

    def get_publishers_info_by_topic(self, topic):
        qos = "incompatible" if topic == self.incompatible_topic else "compatible"
        return [
            Endpoint(
                "publisher",
                FIXED_TOPIC_TYPES[topic],
                qos,
                endpoint_type="publisher",
            )
        ]


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


def test_private_recorder_qos_retains_both_artifact_startup_statuses():
    artifact_root = Path(__file__).parents[1]
    override_path = artifact_root / "recording-qos.yaml"
    dockerfile = (artifact_root / "Dockerfile").read_text(encoding="utf-8")
    override = override_path.read_text(encoding="utf-8")
    run_state = override.split("/simulation/run_state:", 1)[1].split(
        "/simulation/artifact_status:", 1
    )[0]
    artifact_status = override.split("/simulation/artifact_status:", 1)[1].split(
        "/simulation/ground_truth:", 1
    )[0]

    assert "history: keep_last" in artifact_status
    assert "depth: 2" in artifact_status
    assert "reliability: reliable" in artifact_status
    assert "durability: transient_local" in artifact_status
    assert "depth: 1" in run_state
    assert (
        "COPY artifacts/recording-qos.yaml /etc/drone_sim/recording-qos.yaml"
        in dockerfile
    )


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
    assert os.path.samefile(f"/proc/self/fd/{kwargs['stdout'].fileno()}", log_path)
    assert log_path.read_bytes() == b"existing diagnostics\n"


def test_process_health_detects_premature_recorder_exit_without_graph_access(tmp_path):
    process = FakeProcess()
    recorder = _recorder(tmp_path, process_factory=FakeProcessFactory(process))
    assert recorder.is_alive is False

    recorder.start()
    assert recorder.is_alive is True

    process.returncode = 7
    assert recorder.is_alive is False


def test_finalize_marks_a_recorder_that_already_exited_as_not_shutdown_requested(tmp_path):
    process = FakeProcess()
    recorder = _recorder(tmp_path, process_factory=FakeProcessFactory(process))
    recorder.start()
    process.returncode = 0

    result = recorder.finalize(deadline=10.0)

    assert result.exited is True
    assert result.returncode == 0
    assert result.shutdown_requested is False


def test_finalize_marks_exit_between_health_check_and_first_signal_as_premature(tmp_path):
    class ExitBeforeSignalProcess(FakeProcess):
        def __init__(self):
            super().__init__()
            self.poll_count = 0

        def poll(self):
            self.poll_count += 1
            if self.poll_count == 1:
                return None
            self.returncode = 0
            return self.returncode

    process = ExitBeforeSignalProcess()
    signals = []
    recorder = _recorder(
        tmp_path,
        process_factory=FakeProcessFactory(process),
        signal_sender=lambda proc, signum: signals.append(signum),
    )
    recorder.start()

    result = recorder.finalize(deadline=10.0)

    assert result.exited is True
    assert result.shutdown_requested is False
    assert signals == []


@pytest.mark.parametrize("symlink_component", ["logs", "docker", "log-file"])
def test_start_rejects_symlinked_log_path_without_writing_outside_run(
    tmp_path,
    symlink_component,
):
    outside = tmp_path.parent / f"outside-{symlink_component}"
    outside.mkdir()
    outside_log = outside / "rosbag2.log.partial"
    outside_log.write_bytes(b"outside must stay unchanged")

    if symlink_component == "logs":
        (tmp_path / "logs").symlink_to(outside, target_is_directory=True)
    elif symlink_component == "docker":
        (tmp_path / "logs").mkdir()
        (tmp_path / "logs/docker").symlink_to(outside, target_is_directory=True)
    else:
        log_directory = tmp_path / "logs/docker"
        log_directory.mkdir(parents=True)
        (log_directory / "rosbag2.log.partial").symlink_to(outside_log)

    factory = FakeProcessFactory()
    recorder = _recorder(tmp_path, process_factory=factory)

    with pytest.raises(RuntimeError, match="unsafe recorder log path"):
        recorder.start()

    assert factory.calls == []
    assert outside_log.read_bytes() == b"outside must stay unchanged"


def test_start_rejects_hardlinked_log_without_writing_outside_run(tmp_path):
    outside_log = tmp_path.parent / "outside-hardlinked.log"
    outside_log.write_bytes(b"outside must stay unchanged")
    log_directory = tmp_path / "logs/docker"
    log_directory.mkdir(parents=True)
    os.link(outside_log, log_directory / "rosbag2.log.partial")
    factory = FakeProcessFactory()
    recorder = _recorder(tmp_path, process_factory=factory)

    with pytest.raises(RuntimeError, match="unsafe recorder log path"):
        recorder.start()

    assert factory.calls == []
    assert outside_log.read_bytes() == b"outside must stay unchanged"


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


def test_readiness_is_false_when_recorder_subscription_has_wrong_topic_type(tmp_path):
    recorder = _recorder(tmp_path)
    recorder.start()
    graph = FakeGraph(recorder.node_name)
    original = graph.get_subscriptions_info_by_topic

    def subscriptions(topic):
        if topic == FIXED_TOPICS[0]:
            return [Endpoint(recorder.node_name, "std_msgs/msg/String")]
        return original(topic)

    graph.get_subscriptions_info_by_topic = subscriptions

    assert recorder.is_ready(graph) is False


def test_readiness_is_false_when_publisher_has_wrong_topic_type(tmp_path):
    recorder = _recorder(tmp_path)
    recorder.start()
    graph = FakeGraph(recorder.node_name)
    original = graph.get_publishers_info_by_topic

    def publishers(topic):
        if topic == FIXED_TOPICS[0]:
            return [
                Endpoint(
                    "publisher",
                    "std_msgs/msg/String",
                    endpoint_type="publisher",
                )
            ]
        return original(topic)

    graph.get_publishers_info_by_topic = publishers

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
    assert result.shutdown_requested is True
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
    assert result.shutdown_requested is True


def _stamp(ns):
    return SimpleNamespace(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def _custom_message(timestamp_ns, *, run_id=RUN_ID, **fields):
    return SimpleNamespace(run_id=run_id, sim_timestamp=_stamp(timestamp_ns), **fields)


def _image(
    timestamp_ns,
    *,
    data=b"rgb",
    height=None,
    width=None,
    encoding=None,
    step=None,
):
    fields = {"header": SimpleNamespace(stamp=_stamp(timestamp_ns)), "data": data}
    if height is not None:
        fields.update(height=height, width=width, encoding=encoding, step=step)
    return SimpleNamespace(**fields)


def _physical_image(timestamp_ns):
    return _image(
        timestamp_ns,
        data=b"\x00" * (320 * 240 * 3),
        height=240,
        width=320,
        encoding="rgb8",
        step=320 * 3,
    )


def _ground_truth(timestamp_ns, *, value=0.0, in_contact=False):
    return _custom_message(
        timestamp_ns,
        vehicle_id="iris",
        pose=SimpleNamespace(
            position=SimpleNamespace(x=value, y=value, z=value),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
        twist=SimpleNamespace(
            linear=SimpleNamespace(x=value, y=value, z=value),
            angular=SimpleNamespace(x=value, y=value, z=value),
        ),
        in_contact=in_contact,
    )


def _run_state(timestamp_ns, state, *, reason=""):
    return _custom_message(
        timestamp_ns,
        state=state,
        reason=reason,
        config_sha256=CONFIG_SHA256,
    )


def _artifact_status(*, ready, missing):
    return _custom_message(
        0,
        ready=ready,
        complete=False,
        missing=missing,
        manifest_path="",
    )


def _valid_messages():
    return [
        BagMessage(
            "/simulation/artifact_status",
            _artifact_status(
                ready=False,
                missing=["onboard", "observer", "rosbag"],
            ),
            100,
        ),
        BagMessage(
            "/simulation/artifact_status",
            _artifact_status(ready=True, missing=[]),
            101,
        ),
        BagMessage("/clock", SimpleNamespace(clock=_stamp(0)), 102),
        BagMessage("/simulation/run_state", _custom_message(0), 103),
        BagMessage("/simulation/ground_truth", _custom_message(0), 104),
        BagMessage("/simulation/scenario_events", _custom_message(0), 105),
        BagMessage("/simulation/score_events", _custom_message(0), 106),
        BagMessage("/camera/onboard/image_raw", _image(0), 107),
        BagMessage(
            "/camera/onboard/frame_metadata",
            _custom_message(0, frame_id=0, stream="onboard"),
            108,
        ),
        BagMessage("/camera/observer/image_raw", _image(0), 109),
        BagMessage(
            "/camera/observer/frame_metadata",
            _custom_message(0, frame_id=0, stream="observer"),
            110,
        ),
    ]


def _valid_physical_messages():
    rule_ids = (
        "airborne_then_contact",
        "touchdown_precision",
        "safe_preimpact_speed",
        "stable_contact",
    )
    score_events = [
        BagMessage(
            "/simulation/score_events",
            _custom_message(
                0,
                event_id=index,
                event_type=f"descent.{rule_id}",
                value=value,
                evidence_ref=f"scoring/events.jsonl#event-{index}",
            ),
            106 + index,
        )
        for index, (rule_id, value) in enumerate(
            zip(rule_ids, (20.0, 40.0, 0.0, 0.0), strict=True)
        )
    ]
    score_events.append(
        BagMessage(
            "/simulation/score_events",
            _custom_message(
                0,
                event_id=4,
                event_type="score.finalized",
                value=60.0,
                evidence_ref="scoring/events.jsonl#event-4",
            ),
            110,
        )
    )
    return [
        BagMessage(
            "/simulation/artifact_status",
            _artifact_status(
                ready=False,
                missing=["onboard", "observer", "rosbag"],
            ),
            100,
        ),
        BagMessage(
            "/simulation/artifact_status",
            _artifact_status(ready=True, missing=[]),
            101,
        ),
        BagMessage("/simulation/run_state", _run_state(0, 1), 102),
        BagMessage("/simulation/run_state", _run_state(0, 2), 103),
        BagMessage("/clock", SimpleNamespace(clock=_stamp(0)), 104),
        BagMessage("/simulation/run_state", _run_state(0, 3), 105),
        BagMessage("/simulation/ground_truth", _ground_truth(0), 106),
        BagMessage(
            "/simulation/scenario_events",
            _custom_message(
                0,
                event_id=0,
                magnet_id="descent-v1-magnet",
                state="INACTIVE",
            ),
            107,
        ),
        *score_events,
        BagMessage("/camera/onboard/image_raw", _physical_image(0), 120),
        BagMessage(
            "/camera/onboard/frame_metadata",
            _custom_message(0, frame_id=0, stream="onboard"),
            121,
        ),
        BagMessage("/camera/observer/image_raw", _physical_image(0), 122),
        BagMessage(
            "/camera/observer/frame_metadata",
            _custom_message(0, frame_id=0, stream="observer"),
            123,
        ),
        BagMessage(
            "/simulation/run_state",
            _run_state(0, 4, reason="mission_complete"),
            124,
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
    assert {
        topic.name: topic.message_count for topic in result.topics
    }["/simulation/artifact_status"] == 2
    assert all(
        topic.message_count == 1
        for topic in result.topics
        if topic.name != "/simulation/artifact_status"
    )
    assert result.topics[0].message_type == "rosgraph_msgs/msg/Clock"
    assert result.topics[0].first_sim_timestamp_ns == 0
    assert result.topics[0].last_sim_timestamp_ns == 0
    with pytest.raises(FrozenInstanceError):
        result.topics[0].message_count = 2


def test_physical_bag_requires_configured_camera_and_ground_truth_count(tmp_path):
    """A short but contiguous bag must not satisfy a longer production run."""
    _bag_directory(tmp_path)
    messages = _valid_physical_messages()
    backend = FakeBagBackend(messages=messages, metadata=_metadata_for(messages))

    result = RosbagValidator(
        RUN_ID,
        backend=backend,
        expected_camera_frames=2,
        physical_run=True,
        config_sha256=CONFIG_SHA256,
    ).validate(tmp_path, "rosbag")

    assert result.status is ValidationStatus.INVALID
    assert "configured frame count" in result.detail


def test_physical_bag_requires_ground_truth_aligned_to_both_cameras(tmp_path):
    """Matching only one camera could hide cross-stream physical evidence loss."""
    _bag_directory(tmp_path)
    messages = _valid_physical_messages()
    image_index = next(
        index for index, item in enumerate(messages)
        if item.topic == "/camera/observer/image_raw"
    )
    metadata_index = next(
        index for index, item in enumerate(messages)
        if item.topic == "/camera/observer/frame_metadata"
    )
    messages[image_index] = replace(
        messages[image_index], message=_physical_image(50_000_000)
    )
    messages[metadata_index] = replace(
        messages[metadata_index],
        message=_custom_message(50_000_000, frame_id=0, stream="observer"),
    )
    messages.append(
        BagMessage("/clock", SimpleNamespace(clock=_stamp(50_000_000)), 200)
    )
    backend = FakeBagBackend(messages=messages, metadata=_metadata_for(messages))

    result = RosbagValidator(
        RUN_ID,
        backend=backend,
        expected_camera_frames=1,
        physical_run=True,
        config_sha256=CONFIG_SHA256,
    ).validate(tmp_path, "rosbag")

    assert result.status is ValidationStatus.INVALID
    assert "aligned" in result.detail


def test_physical_bag_accepts_explicit_production_evidence_contract(tmp_path):
    _bag_directory(tmp_path)
    messages = _valid_physical_messages()
    backend = FakeBagBackend(messages=messages, metadata=_metadata_for(messages))

    result = RosbagValidator(
        RUN_ID,
        backend=backend,
        expected_camera_frames=1,
        physical_run=True,
        config_sha256=CONFIG_SHA256,
    ).validate(tmp_path, "rosbag")

    assert result.status is ValidationStatus.VALID


def test_physical_bag_returns_immutable_digest_bound_decoded_evidence(tmp_path):
    _bag_directory(tmp_path)
    messages = _valid_physical_messages()

    result = RosbagValidator(
        RUN_ID,
        backend=FakeBagBackend(messages=messages, metadata=_metadata_for(messages)),
        expected_camera_frames=1,
        physical_run=True,
        config_sha256=CONFIG_SHA256,
    ).validate(tmp_path, "rosbag")

    assert result.physical_evidence.bag_sha256 == result.sha256
    assert result.physical_evidence.config_sha256 == CONFIG_SHA256
    assert result.physical_evidence.lifecycle_states == (
        "STARTING",
        "READY",
        "RUNNING",
        "FINALIZING",
    )
    assert len(result.physical_evidence.ground_truth) == 1
    assert tuple(event.event_id for event in result.physical_evidence.score_events) == tuple(
        range(5)
    )
    with pytest.raises(FrozenInstanceError):
        result.physical_evidence.bag_sha256 = "0" * 64


def test_physical_bag_requires_exact_current_config_lifecycle(tmp_path):
    messages = _valid_physical_messages()
    ready_index = [
        index for index, item in enumerate(messages)
        if item.topic == "/simulation/run_state"
    ][1]
    messages[ready_index] = replace(
        messages[ready_index], message=_run_state(0, 3)
    )

    result = _validate_physical_messages(tmp_path, messages)

    assert result.status is ValidationStatus.INVALID
    assert "lifecycle" in result.detail


def test_physical_bag_requires_fixed_rgb8_image_shape(tmp_path):
    messages = _valid_physical_messages()
    image_index = next(
        index for index, item in enumerate(messages)
        if item.topic == "/camera/onboard/image_raw"
    )
    messages[image_index].message.width = 319

    result = _validate_physical_messages(tmp_path, messages)

    assert result.status is ValidationStatus.INVALID
    assert "image shape" in result.detail


def test_physical_bag_rejects_nonfinite_ground_truth(tmp_path):
    messages = _valid_physical_messages()
    ground_truth = next(
        item.message for item in messages
        if item.topic == "/simulation/ground_truth"
    )
    ground_truth.pose.position.z = float("nan")

    result = _validate_physical_messages(tmp_path, messages)

    assert result.status is ValidationStatus.INVALID
    assert "ground truth" in result.detail


def _validate_physical_messages(tmp_path, messages):
    _bag_directory(tmp_path)
    return RosbagValidator(
        RUN_ID,
        backend=FakeBagBackend(messages=messages, metadata=_metadata_for(messages)),
        expected_camera_frames=1,
        physical_run=True,
        config_sha256=CONFIG_SHA256,
    ).validate(tmp_path, "rosbag")


def test_physical_bag_requires_monotonic_clock_covering_frame_stamps(tmp_path):
    messages = _valid_physical_messages()
    clock_index = next(index for index, item in enumerate(messages) if item.topic == "/clock")
    messages[clock_index] = replace(
        messages[clock_index], message=SimpleNamespace(clock=_stamp(50_000_000))
    )
    messages.insert(
        clock_index + 1,
        BagMessage("/clock", SimpleNamespace(clock=_stamp(0)), 103),
    )

    result = _validate_physical_messages(tmp_path, messages)

    assert result.status is ValidationStatus.INVALID
    assert "clock" in result.detail


def test_physical_bag_requires_truthful_inactive_scenario_initialization(tmp_path):
    messages = _valid_physical_messages()
    scenario = next(
        index for index, item in enumerate(messages)
        if item.topic == "/simulation/scenario_events"
    )
    messages[scenario] = replace(
        messages[scenario],
        message=_custom_message(
            0,
            event_id=0,
            magnet_id="descent-v1-magnet",
            state="ACTIVE",
        ),
    )

    result = _validate_physical_messages(tmp_path, messages)

    assert result.status is ValidationStatus.INVALID
    assert "scenario" in result.detail


@pytest.mark.parametrize(
    "mutation",
    [
        lambda events: events.pop(),
        lambda events: setattr(events[1].message, "event_id", 8),
        lambda events: setattr(events[1].message, "event_type", "score.unknown"),
    ],
    ids=("missing", "noncontiguous", "wrong-order"),
)
def test_physical_bag_requires_exact_ordered_descent_score_events(
    tmp_path, mutation
):
    messages = _valid_physical_messages()
    score_events = [
        item for item in messages if item.topic == "/simulation/score_events"
    ]
    mutation(score_events)
    messages = [
        item for item in messages if item.topic != "/simulation/score_events"
    ] + score_events

    result = _validate_physical_messages(tmp_path, messages)

    assert result.status is ValidationStatus.INVALID
    assert "score events" in result.detail


def test_validation_rejects_bag_mutated_during_semantic_read(tmp_path):
    bag = _bag_directory(tmp_path)
    before = validate_tree(tmp_path, "rosbag")

    class MutatingBackend(FakeBagBackend):
        def read_metadata(self, bag_directory):
            with (bag_directory / "bag_0.mcap").open("ab") as stream:
                stream.write(b" changed during metadata read")
            return super().read_metadata(bag_directory)

    result = _validate(tmp_path, MutatingBackend())
    after = validate_tree(tmp_path, "rosbag")

    assert before.sha256 != after.sha256
    assert result.status is ValidationStatus.INVALID
    assert result.size_bytes is None
    assert result.sha256 is None
    assert "changed during semantic validation" in result.detail


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
    backend.messages[3] = replace(
        backend.messages[3], message=_custom_message(0, run_id="stale-run")
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
    backend.messages[7] = replace(backend.messages[7], message=_image(0, data=b""))

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
    backend.messages[8] = replace(
        backend.messages[8],
        message=_custom_message(50_000_000, frame_id=0, stream="onboard"),
    )

    result = _validate(tmp_path, backend)

    assert result.status is ValidationStatus.INVALID
    assert "image/metadata timestamps" in result.detail


def test_validation_rejects_wrong_frame_metadata_stream(tmp_path):
    _bag_directory(tmp_path)
    backend = FakeBagBackend()
    backend.messages[8] = replace(
        backend.messages[8],
        message=_custom_message(0, frame_id=0, stream="observer"),
    )

    result = _validate(tmp_path, backend)

    assert result.status is ValidationStatus.INVALID
    assert "wrong stream" in result.detail


def test_validation_rejects_noncontiguous_frame_ids(tmp_path):
    _bag_directory(tmp_path)
    backend = FakeBagBackend()
    backend.messages[8] = replace(
        backend.messages[8],
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


@pytest.mark.parametrize("mutation", ["missing-initial", "ready-first", "wrong-missing"])
def test_validation_requires_both_exact_startup_artifact_statuses_before_first_clock(
    tmp_path, mutation
):
    _bag_directory(tmp_path)
    messages = _valid_messages()
    if mutation == "missing-initial":
        messages.pop(0)
    elif mutation == "ready-first":
        messages[0], messages[2] = messages[2], messages[0]
    else:
        messages[0] = replace(
            messages[0],
            message=_artifact_status(ready=False, missing=["observer", "rosbag"]),
        )

    result = _validate(
        tmp_path,
        FakeBagBackend(messages=messages, metadata=_metadata_for(messages)),
    )

    assert result.status is ValidationStatus.INVALID
    assert "artifact status" in result.detail


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
            run_id=RUN_ID,
            sim_timestamp=stamp,
            ready=True,
            complete=False,
            missing=[],
            manifest_path="",
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
    initial_status = ArtifactStatus(
        run_id=RUN_ID,
        sim_timestamp=stamp,
        ready=False,
        complete=False,
        missing=["onboard", "observer", "rosbag"],
        manifest_path="",
    )
    writer.write(
        "/simulation/artifact_status",
        serialize_message(initial_status),
        1,
    )
    writer.write(
        "/simulation/artifact_status",
        serialize_message(messages["/simulation/artifact_status"]),
        2,
    )
    for recorded_timestamp, topic in enumerate(
        (topic for topic in FIXED_TOPICS if topic != "/simulation/artifact_status"),
        start=3,
    ):
        writer.write(topic, serialize_message(messages[topic]), recorded_timestamp)
    writer.close()

    result = RosbagValidator(RUN_ID).validate(tmp_path, "rosbag")

    assert result.status is ValidationStatus.VALID
    assert {topic.name: topic.message_count for topic in result.topics} == {
        topic: 2 if topic == "/simulation/artifact_status" else 1
        for topic in FIXED_TOPICS
    }


def test_real_jazzy_qos_api_rejects_best_effort_offer_for_reliable_request(tmp_path):
    qos = _require_ros_module("rclpy.qos")
    endpoint_info = _require_ros_module("rclpy.topic_endpoint_info")
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
        endpoint_info.TopicEndpointInfo(
            node_name="publisher",
            node_namespace="/",
            topic_type=FIXED_TOPIC_TYPES[topic],
            endpoint_type=endpoint_info.TopicEndpointTypeEnum.PUBLISHER,
            qos_profile=offered,
        )
    ]
    graph.get_subscriptions_info_by_topic = lambda topic: [
        endpoint_info.TopicEndpointInfo(
            node_name=recorder.node_name,
            node_namespace="/",
            topic_type=FIXED_TOPIC_TYPES[topic],
            endpoint_type=endpoint_info.TopicEndpointTypeEnum.SUBSCRIPTION,
            qos_profile=requested,
        )
    ]

    assert recorder.is_ready(graph) is False


def test_real_jazzy_endpoint_shape_rejects_wrong_recorder_topic_type(tmp_path):
    qos = _require_ros_module("rclpy.qos")
    endpoint_info = _require_ros_module("rclpy.topic_endpoint_info")
    recorder = RosbagRecorder(
        tmp_path,
        process_factory=FakeProcessFactory(),
        uuid_factory=lambda: RECORDER_UUID,
    )
    recorder.start()
    graph = FakeGraph(recorder.node_name)
    profile = qos.QoSProfile(depth=1)
    graph.get_publishers_info_by_topic = lambda topic: []
    graph.get_subscriptions_info_by_topic = lambda topic: [
        endpoint_info.TopicEndpointInfo(
            node_name=recorder.node_name,
            node_namespace="/",
            topic_type=(
                "std_msgs/msg/String"
                if topic == FIXED_TOPICS[0]
                else FIXED_TOPIC_TYPES[topic]
            ),
            endpoint_type=endpoint_info.TopicEndpointTypeEnum.SUBSCRIPTION,
            qos_profile=profile,
        )
    ]

    assert recorder.is_ready(graph) is False


def test_test_image_apt_dependencies_match_recorded_exact_versions():
    if os.environ.get("DRONE_SIM_REQUIRE_ROS_TESTS") != "1":
        pytest.skip("apt dependency lock is verified in the artifact test image")
    expected = {
        "python3-venv": "3.12.3-0ubuntu2.1",
        "python3.12-venv": "3.12.3-1ubuntu0.15",
        "python3-pip-whl": "24.0+dfsg-1ubuntu1.3",
        "python3-setuptools-whl": "68.1.2-2ubuntu1.2",
        "python3-wheel": "0.42.0-2",
        "ros-jazzy-rosbag2": "0.26.11-1noble.20260616.084050",
        "ros-jazzy-rosbag2-storage-mcap": "0.26.11-1noble.20260616.074830",
    }
    recorded_lock = os.environ.get("DRONE_SIM_APT_PACKAGE_LOCK")
    assert recorded_lock is not None, "artifact test image must record its apt package lock"
    recorded = dict(
        item.split("=", 1)
        for item in recorded_lock.split(",")
    )
    installed = {
        package: subprocess.check_output(
            ["dpkg-query", "-W", "-f=${Version}", package],
            text=True,
        )
        for package in expected
    }

    assert recorded == expected
    assert installed == expected
