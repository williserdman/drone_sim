#!/usr/bin/env python3
"""Deterministic synthetic camera/clock source for Phase 2 infrastructure tests."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import time
from typing import Any, Callable


FRAME_INTERVAL_NS = 50_000_000
FRAME_COUNT = 40
PAYLOAD_BYTES = 320 * 240 * 3
ACCEPTED_FAULTS = frozenset({"", "clock_stall_after_5", "observer_encoder_after_5"})
FRAME_PUBLICATION_ORDER = (
    "clock",
    "onboard_image",
    "onboard_metadata",
    "observer_image",
    "observer_metadata",
    "ground_truth",
)


class BoundedPublicationQueue:
    """Publish at most one deterministic frame item per ROS executor turn."""

    def __init__(self, *, limit: int) -> None:
        if isinstance(limit, bool) or not isinstance(limit, int) or limit <= 0:
            raise ValueError("publication queue limit must be a positive integer")
        self._limit = limit
        self._items: deque[tuple[str, Callable[[], None]]] = deque()
        self._preempted = False

    @property
    def pending(self) -> bool:
        return bool(self._items)

    def enqueue(self, label: str, publish: Callable[[], None]) -> None:
        if self._preempted:
            raise RuntimeError("publication queue was preempted")
        if len(self._items) >= self._limit:
            raise RuntimeError("publication queue exceeded its fixed limit")
        self._items.append((label, publish))

    def drain_one(self) -> str | None:
        if not self._items or self._preempted:
            return None
        label, publish = self._items.popleft()
        publish()
        return label

    def preempt(self) -> None:
        self._items.clear()
        self._preempted = True


class CameraTransportBarrier:
    """Bounded infrastructure barrier for the four reliable archival publishers."""

    def __init__(
        self,
        publishers: dict[str, Any],
        *,
        deadline: float,
        failure: Callable[[str], None],
    ) -> None:
        self._publishers = publishers
        self._deadline = deadline
        self._failure = failure
        self.ready = False
        self.preempted = False
        self.failed = False

    def poll(self, *, now: float, finalizing: bool) -> bool:
        if self.ready:
            return True
        if self.preempted or self.failed:
            return False
        if finalizing:
            self.preempted = True
            return False
        if all(
            publisher.get_subscription_count() >= 2
            for publisher in self._publishers.values()
        ):
            self.ready = True
            return True
        if now >= self._deadline:
            self.failed = True
            self._failure("camera transport discovery deadline expired")
        return False


@dataclass(frozen=True)
class SyntheticFrame:
    stream: str
    frame_id: int
    sim_timestamp_ns: int
    width: int
    height: int
    encoding: str
    payload: bytes


@dataclass(frozen=True)
class SyntheticGroundTruth:
    sim_timestamp_ns: int
    vehicle_id: str = "synthetic-vehicle"
    in_contact: bool = False


def frame_payload(stream: str, frame_id: int) -> bytes:
    if stream not in {"onboard", "observer"}:
        raise ValueError("stream must be onboard or observer")
    marker = 17 if stream == "onboard" else 193
    pixel = bytes(((marker + frame_id) % 256, (frame_id * 7) % 256, marker))
    return pixel * (320 * 240)


class SyntheticGazeboModel:
    """Pure event sequencer whose output never depends on wall time."""

    def __init__(
        self,
        run_id: str,
        *,
        fault: str = "",
        wall_delay_ms: int = 0,
        publish: Callable[[str, Any], None],
        sleep: Callable[[float], None] = time.sleep,
        source_finished: Callable[[int], None] = lambda _stamp: None,
    ) -> None:
        if fault not in ACCEPTED_FAULTS:
            raise ValueError("SIM_PHASE2_FAULT is not one of the accepted Phase 2 faults")
        if isinstance(wall_delay_ms, bool) or not isinstance(wall_delay_ms, int) or wall_delay_ms < 0:
            raise ValueError("SIM_SYNTHETIC_WALL_DELAY_MS must be a nonnegative integer")
        self.run_id = run_id
        self.fault = fault
        self.wall_delay_ms = wall_delay_ms
        self.publish = publish
        self.sleep = sleep
        self.source_finished = source_finished
        self.ready = False
        self.running = False
        self.initial_clock_published = False
        self.next_frame_id = 0
        self.last_pair_ack = -1
        self.finished = False
        self.stalled = False
        self.finalizing = False

    def accept_run_state(self, run_id: str, state: str) -> None:
        if run_id != self.run_id:
            return
        if state == "READY":
            self.ready = True
        elif state == "RUNNING":
            self.running = True
        elif state == "FINALIZING":
            self.finalizing = True

    def accept_pair_ack(
        self,
        run_id: str,
        frame_id: int,
        sim_timestamp_ns: int,
        stream: str,
    ) -> None:
        if run_id != self.run_id:
            return
        if stream != "aggregate":
            raise ValueError("camera pair acknowledgement stream must be aggregate")
        if (
            not isinstance(frame_id, int)
            or isinstance(frame_id, bool)
            or frame_id < 0
            or sim_timestamp_ns != (frame_id + 1) * FRAME_INTERVAL_NS
        ):
            raise ValueError("camera pair acknowledgement timestamp is invalid")
        if frame_id <= self.last_pair_ack:
            return
        if frame_id != self.last_pair_ack + 1 or frame_id >= self.next_frame_id:
            raise ValueError("camera pair acknowledgement is not contiguous")
        self.last_pair_ack = frame_id

    def step(self) -> bool:
        if self.finalizing or self.finished or self.stalled:
            return False
        if not self.ready:
            return False
        if not self.initial_clock_published:
            self.publish("clock", 0)
            self.initial_clock_published = True
            return True
        if not self.running:
            return False
        if self.fault == "clock_stall_after_5" and self.next_frame_id >= 5:
            self.stalled = True
            return False
        if self.next_frame_id >= FRAME_COUNT:
            if self.last_pair_ack != FRAME_COUNT - 1:
                return False
            self.finished = True
            self.source_finished(2_000_000_000)
            return False
        if self.next_frame_id > self.last_pair_ack + 1:
            return False
        frame_id = self.next_frame_id
        timestamp_ns = (frame_id + 1) * FRAME_INTERVAL_NS
        if self.wall_delay_ms:
            self.sleep(self.wall_delay_ms / 1000.0)
        self.publish("clock", timestamp_ns)
        for stream in ("onboard", "observer"):
            self.publish(
                stream,
                SyntheticFrame(
                    stream,
                    frame_id,
                    timestamp_ns,
                    320,
                    240,
                    "rgb8",
                    frame_payload(stream, frame_id),
                ),
            )
        self.publish("ground_truth", SyntheticGroundTruth(timestamp_ns))
        self.next_frame_id += 1
        return True


def apply_durable_lifecycle(
    model: SyntheticGazeboModel,
    running_status: dict[str, Any] | None,
    finalize_request: dict[str, Any] | None,
) -> None:
    """Apply current-run durable evidence when a one-shot ROS sample is missed."""
    if running_status is not None:
        if running_status["state"] == "RUNNING":
            model.accept_run_state(running_status["run_id"], "READY")
        model.accept_run_state(running_status["run_id"], running_status["state"])
    if finalize_request is not None:
        model.accept_run_state(finalize_request["run_id"], "FINALIZING")


def _assign_stamp(stamp: Any, timestamp_ns: int) -> None:
    stamp.sec = timestamp_ns // 1_000_000_000
    stamp.nanosec = timestamp_ns % 1_000_000_000


def _write_fixture_files(run_directory: Path, run_id: str, frame_count: int) -> None:
    from module_stub import write_bytes_atomic

    write_bytes_atomic(
        run_directory / "gazebo/server.log",
        (
            "SYNTHETIC PHASE 2 FIXTURE — NOT GAZEBO PHYSICS\n"
            f"run_id={run_id}\nframes={frame_count}\n"
        ).encode("utf-8"),
    )
    write_bytes_atomic(
        run_directory / "gazebo/state/synthetic-state.json",
        (
            json.dumps(
                {
                    "run_id": run_id,
                    "fixture": True,
                    "validity": "synthetic infrastructure evidence only",
                    "frame_count": frame_count,
                    "last_sim_timestamp_ns": frame_count * FRAME_INTERVAL_NS,
                },
                sort_keys=True,
                separators=(",", ":"),
            )
            + "\n"
        ).encode("utf-8"),
    )


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image
    from simulation_interfaces.msg import FrameMetadata, GroundTruth, RunState
    from artifacts.runtime_protocol import RuntimeProtocol
    from artifacts.structured_log import StructuredEvent, write_event
    from module_stub import QuiescenceBoundary

    run_id = os.environ["SIM_RUN_ID"]
    run_directory = Path(os.environ["SIM_RUN_DIRECTORY"])
    config = json.loads(Path(os.environ["SIM_CONFIG_PATH"]).read_text(encoding="utf-8"))
    fault = os.environ.get("SIM_PHASE2_FAULT", "")
    try:
        delay_ms = int(os.environ.get("SIM_SYNTHETIC_WALL_DELAY_MS", "0"))
    except ValueError as error:
        raise ValueError("SIM_SYNTHETIC_WALL_DELAY_MS must be an integer") from error
    protocol = RuntimeProtocol(run_directory, run_id)
    boundary = QuiescenceBoundary(protocol, "gazebo")
    rclpy.init()
    node = Node("synthetic_gazebo")
    clock_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
    frame_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
    ground_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    state_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    clock_publisher = node.create_publisher(Clock, "/clock", clock_qos)
    image_publishers = {
        stream: node.create_publisher(Image, f"/camera/{stream}/image_raw", frame_qos)
        for stream in ("onboard", "observer")
    }
    metadata_publishers = {
        stream: node.create_publisher(
            FrameMetadata, f"/camera/{stream}/frame_metadata", frame_qos
        )
        for stream in ("onboard", "observer")
    }
    ground_publisher = node.create_publisher(
        GroundTruth, "/simulation/ground_truth", ground_qos
    )
    publication_queue = BoundedPublicationQueue(limit=len(FRAME_PUBLICATION_ORDER))
    last_sim_timestamp_ns = 0

    def emit(event: str, **fields: Any) -> None:
        if not boundary.output_allowed:
            return
        write_event(
            sys.stdout,
            StructuredEvent(
                run_id=run_id,
                module="gazebo",
                severity="INFO",
                event=event,
                sim_timestamp=last_sim_timestamp_ns / 1e9,
                wall_timestamp=datetime.now(timezone.utc),
                fields=fields,
            ),
        )

    def publish(kind: str, value: Any) -> None:
        nonlocal last_sim_timestamp_ns
        if kind == "clock":
            last_sim_timestamp_ns = value
            message = Clock()
            _assign_stamp(message.clock, value)
            publication_queue.enqueue(
                "clock", lambda value=message: clock_publisher.publish(value)
            )
            return
        if kind in {"onboard", "observer"}:
            image = Image()
            _assign_stamp(image.header.stamp, value.sim_timestamp_ns)
            image.height = value.height
            image.width = value.width
            image.encoding = value.encoding
            image.is_bigendian = 0
            image.step = value.width * 3
            image.data = value.payload
            metadata = FrameMetadata()
            metadata.run_id = run_id
            _assign_stamp(metadata.sim_timestamp, value.sim_timestamp_ns)
            metadata.frame_id = value.frame_id
            metadata.stream = value.stream
            publication_queue.enqueue(
                f"{kind}_image",
                lambda value=image, publisher=image_publishers[kind]: publisher.publish(
                    value
                ),
            )
            publication_queue.enqueue(
                f"{kind}_metadata",
                lambda value=metadata, publisher=metadata_publishers[kind]: publisher.publish(
                    value
                ),
            )
            return
        message = GroundTruth()
        message.run_id = run_id
        _assign_stamp(message.sim_timestamp, value.sim_timestamp_ns)
        message.vehicle_id = value.vehicle_id
        message.in_contact = value.in_contact
        publication_queue.enqueue(
            "ground_truth", lambda value=message: ground_publisher.publish(value)
        )

    model = SyntheticGazeboModel(
        run_id,
        fault=fault,
        wall_delay_ms=delay_ms,
        publish=publish,
        source_finished=lambda stamp: protocol.write_status(
            "source-finished",
            {"run_id": run_id, "finished": True, "sim_timestamp_ns": stamp},
        ),
    )
    transport_barrier = CameraTransportBarrier(
        {
            f"{stream}_{kind}": publishers[stream]
            for kind, publishers in (
                ("image", image_publishers),
                ("metadata", metadata_publishers),
            )
            for stream in ("onboard", "observer")
        },
        deadline=time.monotonic() + float(config["startup_wall_seconds"]),
        failure=lambda reason: protocol.write_status(
            "runtime-failure",
            {
                "run_id": run_id,
                "module": "gazebo",
                "reason": reason,
                "diagnostic_paths": ["logs/docker/gazebo.log.partial"],
            },
        ),
    )

    state_names = {
        RunState.READY: "READY",
        RunState.RUNNING: "RUNNING",
        RunState.FINALIZING: "FINALIZING",
    }

    deferred_pair_acks: deque[Any] = deque(maxlen=1)

    def accept_pair_ack(message: Any) -> None:
        previous = model.last_pair_ack
        model.accept_pair_ack(
            message.run_id,
            message.frame_id,
            int(message.sim_timestamp.sec) * 1_000_000_000
            + int(message.sim_timestamp.nanosec),
            message.stream,
        )
        if model.last_pair_ack != previous:
            emit("camera_pair_ack_received", frame_id=model.last_pair_ack)

    def pair_ack_callback(message: Any) -> None:
        if publication_queue.pending:
            if deferred_pair_acks:
                raise RuntimeError("more than one pair acknowledgement is pending")
            deferred_pair_acks.append(message)
            return
        accept_pair_ack(message)

    def state_callback(message: Any) -> None:
        if message.run_id != run_id or message.state not in state_names:
            return
        before_ready = model.ready
        model.accept_run_state(run_id, state_names[message.state])
        if model.ready and not before_ready:
            emit("ready")

    node.create_subscription(RunState, "/simulation/run_state", state_callback, state_qos)
    node.create_subscription(
        FrameMetadata,
        "/simulation/camera_pair_ack",
        pair_ack_callback,
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    emit("starting")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            was_running = model.running
            apply_durable_lifecycle(
                model,
                None
                if model.running
                else protocol.read_status("runtime-running"),
                None
                if model.finalizing
                else protocol.read_finalize_request(),
            )
            if model.running and not was_running:
                emit("running")
            if model.finalizing:
                publication_queue.preempt()
                deferred_pair_acks.clear()
                transport_barrier.poll(now=time.monotonic(), finalizing=True)
            elif publication_queue.pending:
                publication_queue.drain_one()
            elif deferred_pair_acks:
                accept_pair_ack(deferred_pair_acks.popleft())
            elif model.ready and transport_barrier.poll(
                now=time.monotonic(), finalizing=model.finalizing
            ):
                next_frame_before_step = model.next_frame_id
                model.step()
                if model.next_frame_id != next_frame_before_step:
                    emit(
                        "frame_enqueued",
                        frame_id=next_frame_before_step,
                        sim_timestamp_ns=(next_frame_before_step + 1)
                        * FRAME_INTERVAL_NS,
                    )
            if model.finalizing and boundary.output_allowed:
                boundary.enter(
                    lambda: (
                        emit("finalizing"),
                        _write_fixture_files(run_directory, run_id, model.next_frame_id),
                    )
                )
            if not boundary.output_allowed and protocol.read_terminal_committed() is not None:
                break
    finally:
        node.destroy_node()
        protocol.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
