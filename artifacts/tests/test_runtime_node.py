import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

import artifacts.runtime_node as runtime_node
from artifacts._adapters.rosbag import BASE_TOPICS
from artifacts.runtime_node import AggregateArtifactsRuntime, FaultAwareRecorder
from artifacts.validation import ValidationStatus


RUN_ID = "11111111-1111-4111-8111-111111111111"


@pytest.mark.parametrize(
    ("physical_run", "expected_depth", "geometry"),
    [(True, 100, (640, 480)), (False, 5, (320, 240))],
    ids=["physical-production", "synthetic-phase2"],
)
def test_production_runtime_requests_profile_specific_reliable_volatile_camera_qos(
    monkeypatch, tmp_path, physical_run, expected_depth, geometry
):
    reliability = SimpleNamespace(RELIABLE=object())
    durability = SimpleNamespace(TRANSIENT_LOCAL=object(), VOLATILE=object())

    class QoSProfile:
        def __init__(self, *, depth, reliability, durability):
            self.depth = depth
            self.reliability = reliability
            self.durability = durability

    class Publisher:
        def publish(self, _message):
            pass

        def wait_for_all_acked(self, *, timeout):
            del timeout
            return True

    class ProductionNode:
        instances = []

        def __init__(self, name):
            self.name = name
            self.subscriptions = []
            self.instances.append(self)

        def create_publisher(self, _message_type, _topic, _qos):
            return Publisher()

        def create_subscription(self, message_type, topic, callback, qos):
            subscription = SimpleNamespace(
                message_type=message_type,
                topic=topic,
                callback=callback,
                qos_profile=qos,
            )
            self.subscriptions.append(subscription)
            return subscription

        def destroy_node(self):
            pass

    class Protocol:
        def __init__(self, _run_directory, _run_id):
            pass

        def close(self):
            pass

    class BagRecorder:
        def __init__(self, _run_directory, *, topics):
            self.topics = topics

        def start(self):
            pass

    stream_recorder_arguments = []

    class StreamRecorder:
        def __init__(self, _run_directory, *, stream, **_kwargs):
            self.stream = stream
            self.frame_count = 0
            self.is_ready = False
            stream_recorder_arguments.append((stream, _kwargs))

        def start(self, *, deadline):
            del deadline
            self.is_ready = True

    validator_arguments = []

    class Validator:
        def __init__(self, *args, **kwargs):
            validator_arguments.append((args, kwargs))
            pass

    class ArtifactStatus:
        pass

    class FrameMetadata:
        pass

    class RunState:
        STARTING = 1

    monkeypatch.setitem(
        sys.modules,
        "rclpy",
        SimpleNamespace(init=lambda: None, ok=lambda: False, shutdown=lambda: None),
    )
    monkeypatch.setitem(
        sys.modules,
        "rclpy.duration",
        SimpleNamespace(Duration=lambda **_kwargs: object()),
    )
    monkeypatch.setitem(sys.modules, "rclpy.node", SimpleNamespace(Node=ProductionNode))
    monkeypatch.setitem(
        sys.modules,
        "rclpy.qos",
        SimpleNamespace(
            DurabilityPolicy=durability,
            QoSProfile=QoSProfile,
            ReliabilityPolicy=reliability,
        ),
    )
    monkeypatch.setitem(
        sys.modules,
        "simulation_interfaces.msg",
        SimpleNamespace(
            ArtifactStatus=ArtifactStatus,
            FrameMetadata=FrameMetadata,
            RunState=RunState,
        ),
    )
    monkeypatch.setitem(
        sys.modules, "sensor_msgs.msg", SimpleNamespace(Image=type("Image", (), {}))
    )
    monkeypatch.setattr(runtime_node, "RuntimeProtocol", Protocol)
    monkeypatch.setattr(runtime_node, "RosbagRecorder", BagRecorder)
    monkeypatch.setattr(runtime_node, "RosbagValidator", Validator)
    monkeypatch.setattr(runtime_node, "VideoStreamRecorder", StreamRecorder)
    monkeypatch.setattr(runtime_node, "VideoValidator", Validator)
    monkeypatch.setattr(
        runtime_node,
        "resolve_recording_runtime_config",
            lambda _config: SimpleNamespace(
                expected_camera_frames=1_200,
                physical_run=physical_run,
                synthetic_camera_ack=not physical_run,
                topics=BASE_TOPICS,
                ruleset_id="descent_v1",
                width_px=geometry[0],
                height_px=geometry[1],
                fps=20,
                encoding="rgb8",
            ),
    )
    monkeypatch.setattr(runtime_node, "write_event", lambda *_args, **_kwargs: None)

    config_path = tmp_path / "run.json"
    config_path.write_text(
        json.dumps(
            {
                "config_sha256": "a" * 64,
                "finalization_wall_seconds": 120,
                "startup_wall_seconds": 120,
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("SIM_RUN_ID", RUN_ID)
    monkeypatch.setenv("SIM_RUN_DIRECTORY", str(tmp_path))
    monkeypatch.setenv("SIM_CONFIG_PATH", str(config_path))

    runtime_node.main()

    camera_topics = {
        "/camera/onboard/image_raw",
        "/camera/onboard/frame_metadata",
        "/camera/observer/image_raw",
        "/camera/observer/frame_metadata",
    }
    subscriptions = [
        subscription
        for subscription in ProductionNode.instances[0].subscriptions
        if subscription.topic in camera_topics
    ]
    assert {subscription.topic for subscription in subscriptions} == camera_topics
    assert len(subscriptions) == 4
    assert all(
        subscription.qos_profile.depth == expected_depth
        and subscription.qos_profile.reliability is reliability.RELIABLE
        and subscription.qos_profile.durability is durability.VOLATILE
        for subscription in subscriptions
    )
    expected_geometry = {
        "width_px": geometry[0],
        "height_px": geometry[1],
        "fps": 20,
        "encoding": "rgb8",
    }
    assert [stream for stream, _arguments in stream_recorder_arguments] == [
        "onboard",
        "observer",
    ]
    assert all(
        arguments | expected_geometry == arguments
        for _stream, arguments in stream_recorder_arguments
    )
    video_validator_arguments = [kwargs for args, kwargs in validator_arguments if not args]
    assert video_validator_arguments == [
        {"width_px": geometry[0], "height_px": geometry[1], "fps": 20},
        {"width_px": geometry[0], "height_px": geometry[1], "fps": 20},
    ]


def test_durable_finalize_request_latches_one_nonrestarting_deadline_without_ros_event():
    now = {"value": 10.0}
    request = {
        "run_id": RUN_ID,
        "requested_terminal": "FAILED",
        "reason": "compose_child_exited",
    }

    class Protocol:
        def read_finalize_request(self):
            return request

    latch = runtime_node.FinalizationRequestLatch(
        120.0,
        monotonic=lambda: now["value"],
    )

    assert latch.poll(Protocol()) == ("FAILED", 130.0)
    now["value"] = 100.0
    assert latch.poll(Protocol()) == ("FAILED", 130.0)


class FakeProtocol:
    def __init__(self):
        self.statuses = []
        self.frozen = None
        self.terminal = None
        self.manifest_status = None

    def write_status(self, name, document):
        self.statuses.append((name, document))

    def read_status(self, name):
        assert name == "runtime-frozen"
        return self.frozen

    def read_terminal_committed(self):
        return self.terminal

    def read_manifest_status(self):
        return self.manifest_status


class FakeBag:
    def __init__(self, *, ready=True, alive=None, start_error=None, finalization=None):
        self.ready = ready
        self.alive = alive
        self.start_error = start_error
        self.finalization = finalization
        self.finalize_deadlines = []

    @property
    def is_alive(self):
        return self.ready if self.alive is None else self.alive

    def start(self):
        if self.start_error:
            raise self.start_error

    def is_ready(self, graph):
        return self.ready and graph == "graph"

    def finalize(self, deadline):
        self.finalize_deadlines.append(deadline)
        return self.finalization or SimpleNamespace(
            exited=True,
            returncode=0,
            detail="bag finalized",
            shutdown_requested=True,
        )


class FakeStream:
    def __init__(self, stream):
        self.stream = stream
        self.frame_count = 40
        self.is_ready = True
        self.finalize_calls = []
        self.accepted = []

    def start(self, *, deadline):
        self.started_deadline = deadline

    def accept_image(self, message):
        self.accepted.append(("image", message))

    def accept_metadata(self, message):
        self.accepted.append(("metadata", message))
        self.frame_count = message.frame_id + 1

    def finalize(self, deadline, *, outcome):
        self.finalize_calls.append((deadline, outcome))
        return SimpleNamespace(exited=True, returncode=0, output_published=True, detail="video finalized")


class FakeVideoNode:
    def __init__(self, onboard=None, observer=None, *, ready=True, start_error=None):
        self.recorders = {
            "onboard": onboard or FakeStream("onboard"),
            "observer": observer or FakeStream("observer"),
        }
        self._ready = ready
        self.start_error = start_error

    def start(self, *, deadline):
        if self.start_error:
            raise self.start_error
        for recorder in self.recorders.values():
            recorder.start(deadline=deadline)

    @property
    def is_ready(self):
        return self._ready and all(item.is_ready for item in self.recorders.values())


class FakeVideoValidator:
    def __init__(self, stream, *, status=ValidationStatus.VALID):
        self.stream = stream
        self.status = status
        self.calls = []

    def validate(self, run_directory, relative_path, **kwargs):
        self.calls.append((Path(run_directory), str(relative_path), kwargs))
        return SimpleNamespace(
            status=self.status,
            size_bytes=123 if self.status is ValidationStatus.VALID else None,
            sha256="a" * 64 if self.status is ValidationStatus.VALID else None,
            detail=f"{self.stream} result",
            codec_name="h264" if self.status is ValidationStatus.VALID else None,
            pix_fmt="yuv420p" if self.status is ValidationStatus.VALID else None,
            avg_frame_rate="20/1" if self.status is ValidationStatus.VALID else None,
            width=320 if self.status is ValidationStatus.VALID else None,
            height=240 if self.status is ValidationStatus.VALID else None,
            frame_count=40 if self.status is ValidationStatus.VALID else None,
            diagnostics=(SimpleNamespace(event="video_probe", detail=f"{self.stream} result"),),
        )


class FakeBagValidator:
    def __init__(self, *, status=ValidationStatus.VALID):
        self.status = status
        self.calls = []

    def validate(self, run_directory, relative_path="rosbag"):
        self.calls.append((Path(run_directory), str(relative_path)))
        topic = SimpleNamespace(
            name="/clock",
            message_type="rosgraph_msgs/msg/Clock",
            message_count=41,
            first_sim_timestamp_ns=0,
            last_sim_timestamp_ns=2_000_000_000,
        )
        return SimpleNamespace(
            status=self.status,
            size_bytes=456 if self.status is ValidationStatus.VALID else None,
            sha256="b" * 64 if self.status is ValidationStatus.VALID else None,
            detail="bag result",
            topics=(topic,) if self.status is ValidationStatus.VALID else (),
        )


def _runtime(tmp_path, **changes):
    protocol = changes.pop("protocol", FakeProtocol())
    bag = changes.pop("bag", FakeBag())
    video = changes.pop("video", FakeVideoNode())
    published = []
    values = dict(
        run_directory=tmp_path,
        run_id=RUN_ID,
        protocol=protocol,
        bag_recorder=bag,
        video_node=video,
        video_validators={
            "onboard": FakeVideoValidator("onboard"),
            "observer": FakeVideoValidator("observer"),
        },
        bag_validator=FakeBagValidator(),
        publish=published.append,
        monotonic=lambda: 0.0,
        expected_camera_frames=changes.pop("expected_camera_frames", 40),
    )
    values.update(changes)
    return AggregateArtifactsRuntime(**values), protocol, bag, video, published


def test_physical_runtime_validates_configured_camera_count(tmp_path):
    """Keeping the legacy 40 literal would reject a complete production recording."""
    validators = {
        "onboard": FakeVideoValidator("onboard"),
        "observer": FakeVideoValidator("observer"),
    }
    runtime, protocol, _, _, _ = _runtime(
        tmp_path,
        expected_camera_frames=600,
        video_validators=validators,
    )
    runtime.start(deadline=5.0)
    protocol.frozen = {"run_id": RUN_ID, "frozen": True}

    assert runtime.finalize("COMPLETED", deadline=99.0)["complete"] is True
    assert all(
        validator.calls[0][2]["expected_frame_count"] == 600
        for validator in validators.values()
    )


def test_startup_claims_ready_only_when_every_recorder_and_graph_endpoint_is_ready(tmp_path):
    runtime, protocol, _, _, published = _runtime(tmp_path)
    assert runtime.start(deadline=10.0) is True
    assert published == []
    assert runtime.check_ready("graph") is False
    assert runtime.check_ready("graph") is True
    assert published == [
        {
            "run_id": RUN_ID,
            "ready": False,
            "complete": False,
            "missing": ["onboard", "observer", "rosbag"],
            "manifest_path": "",
        },
        {"run_id": RUN_ID, "ready": True, "complete": False, "missing": [], "manifest_path": ""}
    ]
    assert protocol.statuses == [("artifacts-ready", {"run_id": RUN_ID, "ready": True})]


def test_startup_status_waits_for_rosbag_subscription_before_publication(tmp_path):
    bag = FakeBag(ready=True, alive=True)
    runtime, protocol, _, _, published = _runtime(tmp_path, bag=bag)
    runtime.start(deadline=10.0)

    bag.ready = False
    assert runtime.check_ready("graph") is False
    assert published == []

    bag.ready = True
    assert runtime.check_ready("graph") is False
    assert published == [
        {
            "run_id": RUN_ID,
            "ready": False,
            "complete": False,
            "missing": ["onboard", "observer", "rosbag"],
            "manifest_path": "",
        }
    ]
    assert protocol.statuses == []
    assert runtime.check_ready("graph") is True


def test_startup_ready_waits_for_initial_status_delivery_acknowledgement(tmp_path):
    delivered = {"initial_status": False}
    runtime, protocol, _, _, published = _runtime(
        tmp_path,
        initial_status_delivered=lambda: delivered["initial_status"],
    )
    runtime.start(deadline=10.0)

    assert runtime.check_ready("graph") is False
    assert runtime.check_ready("graph") is False
    assert len(published) == 1
    assert published[0]["ready"] is False
    assert protocol.statuses == []

    delivered["initial_status"] = True
    assert runtime.check_ready("graph") is True
    assert [document["ready"] for document in published] == [False, True]


def test_initial_status_delivery_timeout_reports_bounded_startup_failure(tmp_path):
    now = {"value": 0.0}
    runtime, protocol, _, _, published = _runtime(
        tmp_path,
        initial_status_delivered=lambda: False,
        monotonic=lambda: now["value"],
    )
    runtime.start(deadline=10.0)
    assert runtime.check_ready("graph") is False

    now["value"] = 10.0
    assert runtime.check_ready("graph") is False
    assert [document["ready"] for document in published] == [False]
    assert protocol.statuses == [
        (
            "runtime-failure",
            {
                "run_id": RUN_ID,
                "module": "artifacts",
                "reason": (
                    "initial artifact status delivery was not acknowledged "
                    "before startup deadline"
                ),
                "diagnostic_paths": ["logs/docker/rosbag2.log.partial"],
            },
        )
    ]


def test_startup_waits_for_camera_pair_ack_subscriber_discovery(tmp_path):
    discovered = {"ack": False}
    runtime, protocol, _, _, published = _runtime(
        tmp_path,
        backpressure_ready=lambda _graph: discovered["ack"],
    )
    runtime.start(deadline=10.0)

    assert runtime.check_ready("graph") is False
    assert published == [
        {
            "run_id": RUN_ID,
            "ready": False,
            "complete": False,
            "missing": ["onboard", "observer", "rosbag"],
            "manifest_path": "",
        }
    ]
    assert protocol.statuses == []

    discovered["ack"] = True
    assert runtime.check_ready("graph") is True


def test_startup_waits_until_current_run_starting_was_observed(tmp_path):
    observed = {"starting": False}
    runtime, protocol, _, _, published = _runtime(
        tmp_path,
        lifecycle_ready=lambda: observed["starting"],
    )
    runtime.start(deadline=10.0)

    assert runtime.check_ready("graph") is False
    assert published == [
        {
            "run_id": RUN_ID,
            "ready": False,
            "complete": False,
            "missing": ["onboard", "observer", "rosbag"],
            "manifest_path": "",
        }
    ]
    assert protocol.statuses == []

    observed["starting"] = True
    assert runtime.check_ready("graph") is True


def test_partial_startup_reports_failure_without_claiming_ready(tmp_path):
    runtime, protocol, _, _, published = _runtime(
        tmp_path, bag=FakeBag(start_error=RuntimeError("bag failed"))
    )
    assert runtime.start(deadline=10.0) is False
    assert runtime.check_ready("graph") is False
    assert published == []
    assert protocol.statuses == [
        (
            "runtime-failure",
            {
                "run_id": RUN_ID,
                "module": "artifacts",
                "reason": "recorder startup failed: RuntimeError: bag failed",
                "diagnostic_paths": ["logs/docker/rosbag2.log.partial"],
            },
        )
    ]


def test_finalization_waits_for_freeze_and_shares_one_deadline_for_all_recorders(tmp_path):
    runtime, protocol, bag, video, published = _runtime(tmp_path)
    runtime.start(deadline=5.0)
    runtime.check_ready("graph")
    runtime.check_ready("graph")
    assert runtime.finalize("COMPLETED", deadline=99.0) is None
    protocol.frozen = {"run_id": RUN_ID, "frozen": True}
    report = runtime.finalize("COMPLETED", deadline=99.0)

    assert bag.finalize_deadlines == [99.0]
    assert all(item.finalize_calls == [(99.0, "COMPLETED")] for item in video.recorders.values())
    assert report["complete"] is True
    assert [item["relative_path"] for item in report["records"]] == [
        "video/onboard.mp4", "video/observer.mp4", "rosbag"
    ]
    assert set(report["records"][0]["semantic"]) == {
        "codec_name", "pix_fmt", "avg_frame_rate", "width", "height", "frame_count", "diagnostics"
    }
    assert set(report["records"][2]["semantic"]) == {"storage_id", "topics"}
    assert protocol.statuses[-1] == ("artifacts-final", report)

    protocol.terminal = {
        "run_id": RUN_ID,
        "terminal_status": "COMPLETED",
        "reason": "validated",
        "manifest_path": "manifest.json",
    }
    protocol.manifest_status = {
        "run_id": RUN_ID,
        "complete": True,
        "missing": [],
        "manifest_path": "manifest.json",
    }
    assert runtime.poll_terminal() is True
    assert published == [
        {
            "run_id": RUN_ID,
            "ready": False,
            "complete": False,
            "missing": ["onboard", "observer", "rosbag"],
            "manifest_path": "",
        },
        {
            "run_id": RUN_ID,
            "ready": True,
            "complete": False,
            "missing": [],
            "manifest_path": "",
        },
        {
            "run_id": RUN_ID,
            "ready": True,
            "complete": True,
            "missing": [],
            "manifest_path": "manifest.json",
        },
    ]
    assert protocol.statuses[-1] == (
        "terminal-notified",
        {"run_id": RUN_ID, "notified": True},
    )


def test_post_manifest_status_sorts_invalid_paths_and_preserves_readiness(tmp_path):
    runtime, protocol, _, _, published = _runtime(tmp_path)
    runtime.start(deadline=5.0)
    runtime.check_ready("graph")
    runtime.check_ready("graph")
    protocol.terminal = {
        "run_id": RUN_ID,
        "terminal_status": "FAILED",
        "reason": "invalid artifacts",
        "manifest_path": "manifest.json",
    }
    protocol.manifest_status = {
        "run_id": RUN_ID,
        "complete": False,
        "missing": ["video/observer.mp4", "rosbag"],
        "manifest_path": "manifest.json",
    }
    runtime.final_report = {"run_id": RUN_ID, "complete": False, "records": []}

    assert runtime.poll_terminal() is True
    assert published[-1] == {
        "run_id": RUN_ID,
        "ready": True,
        "complete": False,
        "missing": ["rosbag", "video/observer.mp4"],
        "manifest_path": "manifest.json",
    }


def test_invalid_observer_still_produces_exact_three_record_failure_report(tmp_path):
    runtime, protocol, _, _, _ = _runtime(
        tmp_path,
        video_validators={
            "onboard": FakeVideoValidator("onboard"),
            "observer": FakeVideoValidator("observer", status=ValidationStatus.INVALID),
        },
    )
    runtime.start(deadline=5.0)
    protocol.frozen = {"run_id": RUN_ID, "frozen": True}
    report = runtime.finalize("FAILED", deadline=99.0)
    assert report["complete"] is False
    assert [item["status"] for item in report["records"]] == ["valid", "invalid", "valid"]
    assert report["records"][1]["semantic"]


def test_observer_fault_accepts_frame_four_then_disables_only_observer_and_reports_once():
    delegate = FakeStream("observer")
    failures = []
    recorder = FaultAwareRecorder(
        delegate,
        stream="observer",
        fault="observer_encoder_after_5",
        failure=lambda reason, paths: failures.append((reason, paths)),
    )
    for frame_id in range(5):
        recorder.accept_metadata(SimpleNamespace(frame_id=frame_id))
    recorder.accept_metadata(SimpleNamespace(frame_id=5))
    recorder.accept_image("late")
    assert [message.frame_id for kind, message in delegate.accepted if kind == "metadata"] == list(range(5))
    assert failures == [
        ("observer encoder fault injected after frame 4", ["logs/docker/ffmpeg-observer.log.partial"])
    ]


def test_runtime_pair_buffer_drains_reordered_dds_topics_in_frame_order():
    delegate = FakeStream("onboard")
    delegate.frame_count = 0
    failures = []
    recorder = FaultAwareRecorder(
        delegate,
        stream="onboard",
        fault="",
        failure=lambda reason, paths: failures.append((reason, paths)),
        pair_buffer_limit=4,
        paired=lambda stream, frame_id, timestamp_ns: failures.append(
            (stream, frame_id, timestamp_ns)
        ),
    )

    def stamp(value):
        return SimpleNamespace(sec=value // 1_000_000_000, nanosec=value % 1_000_000_000)

    metadata = [
        SimpleNamespace(frame_id=index, sim_timestamp=stamp(index * 50_000_000))
        for index in range(2)
    ]
    images = [
        SimpleNamespace(header=SimpleNamespace(stamp=stamp(index * 50_000_000)))
        for index in range(2)
    ]

    recorder.accept_metadata(metadata[0])
    recorder.accept_metadata(metadata[1])
    recorder.accept_image(images[1])
    assert delegate.accepted == []
    recorder.accept_image(images[0])

    assert delegate.accepted == [
        ("image", images[0]),
        ("metadata", metadata[0]),
        ("image", images[1]),
        ("metadata", metadata[1]),
    ]
    assert failures == [
        ("onboard", 0, 0),
        ("onboard", 1, 50_000_000),
    ]


def test_unknown_fault_is_rejected(tmp_path):
    with pytest.raises(ValueError, match="SIM_PHASE2_FAULT"):
        _runtime(tmp_path, fault="unknown")


def test_premature_ffmpeg_exit_after_readiness_writes_first_wins_runtime_failure(tmp_path):
    runtime, protocol, _, video, _ = _runtime(tmp_path)
    runtime.start(deadline=5.0)
    assert runtime.check_ready("graph") is False
    assert runtime.check_ready("graph") is True

    video.recorders["onboard"].is_ready = False
    assert runtime.check_health() is False
    assert runtime.check_health() is False
    assert protocol.statuses[-1] == (
        "runtime-failure",
        {
            "run_id": RUN_ID,
            "module": "artifacts",
            "reason": "onboard FFmpeg exited after recorder readiness",
            "diagnostic_paths": ["logs/docker/ffmpeg-onboard.log.partial"],
        },
    )
    assert [name for name, _document in protocol.statuses].count("runtime-failure") == 1


def test_video_diagnostic_writes_durable_failure_before_structured_output(tmp_path):
    runtime, protocol, _, _, _ = _runtime(tmp_path)
    observed = []
    diagnostic = SimpleNamespace(
        stream="observer", event="recorder_callback_failed", detail="broken pipe"
    )

    runtime.report_video_diagnostic(diagnostic, structured=lambda: observed.append("logged"))
    runtime.report_video_diagnostic(diagnostic, structured=lambda: observed.append("duplicate"))

    assert protocol.statuses[0] == (
        "runtime-failure",
        {
            "run_id": RUN_ID,
            "module": "artifacts",
            "reason": "observer recorder_callback_failed: broken pipe",
            "diagnostic_paths": ["logs/docker/ffmpeg-observer.log.partial"],
        },
    )
    assert observed == ["logged", "duplicate"]


def test_stubborn_rosbag_never_publishes_final_report_or_runs_validators(tmp_path):
    stubborn = SimpleNamespace(
        exited=False,
        returncode=None,
        detail="recorder did not exit before deadline after SIGINT, SIGTERM, SIGKILL",
    )
    bag_validator = FakeBagValidator()
    video_validators = {
        "onboard": FakeVideoValidator("onboard"),
        "observer": FakeVideoValidator("observer"),
    }
    runtime, protocol, bag, _, _ = _runtime(
        tmp_path,
        bag=FakeBag(finalization=stubborn),
        bag_validator=bag_validator,
        video_validators=video_validators,
    )
    runtime.start(deadline=5.0)
    protocol.frozen = {"run_id": RUN_ID, "frozen": True}

    assert runtime.finalize("FAILED", deadline=99.0) is None
    assert runtime.finalize("FAILED", deadline=99.0) is None
    assert runtime.finalization_started is True
    assert runtime.finalization_blocked is True
    assert bag.finalize_deadlines == [99.0]
    assert bag_validator.calls == []
    assert all(validator.calls == [] for validator in video_validators.values())
    assert not any(name == "artifacts-final" for name, _document in protocol.statuses)
    assert protocol.statuses[-1][0] == "runtime-failure"
    assert "did not exit" in protocol.statuses[-1][1]["reason"]


def test_rosbag_death_after_finalizing_before_aggregate_freeze_blocks_success(tmp_path):
    bag_validator = FakeBagValidator()
    video_validators = {
        "onboard": FakeVideoValidator("onboard"),
        "observer": FakeVideoValidator("observer"),
    }
    runtime, protocol, bag, video, _ = _runtime(
        tmp_path,
        bag_validator=bag_validator,
        video_validators=video_validators,
    )
    runtime.start(deadline=5.0)
    assert runtime.check_ready("graph") is False
    assert runtime.check_ready("graph") is True

    # FINALIZING has been requested, but orchestration has not yet published
    # the aggregate freeze. The recorder dies in that exact wait window.
    bag.ready = False
    assert runtime.check_health() is False
    assert protocol.statuses[-1] == (
        "runtime-failure",
        {
            "run_id": RUN_ID,
            "module": "artifacts",
            "reason": "rosbag recorder exited after recorder readiness",
            "diagnostic_paths": ["logs/docker/rosbag2.log.partial"],
        },
    )
    assert runtime.finalize("COMPLETED", deadline=99.0) is None
    protocol.frozen = {"run_id": RUN_ID, "frozen": True}
    assert runtime.finalize("COMPLETED", deadline=99.0) is None

    assert runtime.finalization_started is True
    assert runtime.finalization_blocked is True
    assert bag.finalize_deadlines == []
    assert all(stream.finalize_calls == [(99.0, "COMPLETED")] for stream in video.recorders.values())
    assert bag_validator.calls == []
    assert all(validator.calls == [] for validator in video_validators.values())
    assert not any(name == "artifacts-final" for name, _document in protocol.statuses)


def test_rosbag_exit_racing_expected_shutdown_blocks_final_report(tmp_path):
    premature = SimpleNamespace(
        exited=True,
        returncode=0,
        detail="recorder already exited with return code 0",
        shutdown_requested=False,
    )
    runtime, protocol, bag, _, _ = _runtime(
        tmp_path, bag=FakeBag(finalization=premature)
    )
    runtime.start(deadline=5.0)
    protocol.frozen = {"run_id": RUN_ID, "frozen": True}

    assert runtime.finalize("COMPLETED", deadline=99.0) is None

    assert bag.finalize_deadlines == [99.0]
    assert runtime.finalization_blocked is True
    assert protocol.statuses[-1][0] == "runtime-failure"
    assert "before shutdown was requested" in protocol.statuses[-1][1]["reason"]
    assert not any(name == "artifacts-final" for name, _document in protocol.statuses)
