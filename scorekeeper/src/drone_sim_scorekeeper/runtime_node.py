"""ROS 2 adapter and durable lifecycle for the production scorekeeper."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from decimal import Decimal, InvalidOperation
import json
import os
from pathlib import Path
from types import SimpleNamespace
import sys
from typing import Any, Callable, Protocol
from uuid import UUID

from .descent import DescentScorer, GroundTruthSample, ScoreEvent, load_descent_rules
from .runtime import ScenarioSample, ScorekeeperRuntime


_FRAME_INTERVAL_NS = 50_000_000


def _canonical_run_id(value: object) -> str:
    if not isinstance(value, str):
        raise ValueError("run_id must be a canonical UUID")
    try:
        parsed = UUID(value)
    except (TypeError, ValueError, AttributeError) as error:
        raise ValueError("run_id must be a canonical UUID") from error
    if str(parsed) != value:
        raise ValueError("run_id must be a canonical UUID")
    return value


@dataclass(frozen=True)
class RuntimeSettings:
    expected_ground_truth_samples: int


def load_runtime_settings(path: Path | str, run_id: str) -> RuntimeSettings:
    """Read only the resolved fields that define score input cardinality."""
    canonical = _canonical_run_id(run_id)
    try:
        document = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("resolved scorekeeper configuration is invalid") from error
    if not isinstance(document, dict) or document.get("run_id") != canonical:
        raise ValueError("resolved scorekeeper configuration has the wrong run_id")
    if document.get("scenario") != "descent_v1":
        raise ValueError("scorekeeper requires scenario descent_v1")
    recording = document.get("recording")
    simulation = document.get("simulation")
    if (
        not isinstance(recording, dict)
        or recording.get("fps") != 20
        or isinstance(recording.get("fps"), bool)
        or not isinstance(simulation, dict)
    ):
        raise ValueError("scorekeeper requires 20 Hz resolved simulation configuration")
    duration = simulation.get("duration_sim_seconds")
    if isinstance(duration, bool) or not isinstance(duration, (int, float)):
        raise ValueError("simulation duration must be a positive exact-grid number")
    try:
        seconds = Decimal(str(duration))
    except InvalidOperation as error:
        raise ValueError("simulation duration must be a positive exact-grid number") from error
    if not seconds.is_finite() or seconds <= 0:
        raise ValueError("simulation duration must be a positive exact-grid number")
    duration_ns = seconds * Decimal(1_000_000_000)
    if duration_ns != duration_ns.to_integral_value():
        raise ValueError("simulation duration must resolve to integer nanoseconds")
    duration_ns_int = int(duration_ns)
    if duration_ns_int % _FRAME_INTERVAL_NS:
        raise ValueError("simulation duration must lie on the 50 ms score grid")
    return RuntimeSettings(duration_ns_int // _FRAME_INTERVAL_NS)


def _timestamp_ns(stamp: Any) -> int:
    seconds = stamp.sec
    nanoseconds = stamp.nanosec
    if (
        not isinstance(seconds, int)
        or isinstance(seconds, bool)
        or seconds < 0
        or not isinstance(nanoseconds, int)
        or isinstance(nanoseconds, bool)
        or not 0 <= nanoseconds < 1_000_000_000
    ):
        raise ValueError("ROS simulation timestamp is invalid")
    return seconds * 1_000_000_000 + nanoseconds


def _assign_stamp(stamp: Any, timestamp_ns: int) -> None:
    stamp.sec = timestamp_ns // 1_000_000_000
    stamp.nanosec = timestamp_ns % 1_000_000_000


def ground_truth_from_message(message: Any) -> GroundTruthSample:
    return GroundTruthSample(
        run_id=message.run_id,
        sim_timestamp_ns=_timestamp_ns(message.sim_timestamp),
        position_xyz=(message.pose.position.x, message.pose.position.y, message.pose.position.z),
        orientation_xyzw=(
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        ),
        linear_velocity_xyz=(
            message.twist.linear.x,
            message.twist.linear.y,
            message.twist.linear.z,
        ),
        angular_velocity_xyz=(
            message.twist.angular.x,
            message.twist.angular.y,
            message.twist.angular.z,
        ),
        in_contact=message.in_contact,
    )


def scenario_from_message(message: Any) -> ScenarioSample:
    return ScenarioSample(
        run_id=message.run_id,
        sim_timestamp_ns=_timestamp_ns(message.sim_timestamp),
        event_id=message.event_id,
        magnet_id=message.magnet_id,
        state=message.state,
    )


def score_event_message(event: ScoreEvent, message_type: Callable[[], Any]) -> Any:
    message = message_type()
    message.run_id = event.run_id
    if not hasattr(message, "sim_timestamp"):
        message.sim_timestamp = SimpleNamespace(sec=0, nanosec=0)
    _assign_stamp(message.sim_timestamp, event.sim_timestamp_ns)
    message.event_id = event.event_id
    message.event_type = event.event_type
    message.value = event.value
    message.evidence_ref = event.evidence_ref
    return message


class DriverProtocol(Protocol):
    def read_status(self, name: str) -> dict[str, Any] | None: ...
    def read_finalize_request(self) -> dict[str, Any] | None: ...
    def read_terminal_committed(self) -> dict[str, Any] | None: ...


class ScorekeeperDriver:
    """Idempotently bridge durable source/finalization facts into the runtime."""

    def __init__(
        self,
        run_id: str,
        runtime: ScorekeeperRuntime,
        protocol: DriverProtocol,
        *,
        on_scored: Callable[[bool, float, int], None] = lambda _complete, _score, _stamp: None,
        before_quiescence: Callable[[], None] = lambda: None,
    ) -> None:
        self.run_id = _canonical_run_id(run_id)
        self.runtime = runtime
        self.protocol = protocol
        self._source_seen = False
        self._source_timestamp_ns: int | None = None
        self._finalize_seen = False
        self._on_scored = on_scored
        self._before_quiescence = before_quiescence

    def poll(self) -> bool:
        if not self._source_seen:
            source = (
                self.protocol.read_status("source-finished")
                if self._source_timestamp_ns is None
                else None
            )
            if source is not None:
                if (
                    set(source) != {"run_id", "finished", "sim_timestamp_ns"}
                    or source.get("run_id") != self.run_id
                    or source.get("finished") is not True
                    or not isinstance(source.get("sim_timestamp_ns"), int)
                    or isinstance(source["sim_timestamp_ns"], bool)
                    or source["sim_timestamp_ns"] < 0
                ):
                    raise ValueError("source-finished status is invalid")
                self._source_timestamp_ns = source["sim_timestamp_ns"]
            if (
                self._source_timestamp_ns is not None
                and self.runtime.source_inputs_observed_through(
                    self._source_timestamp_ns
                )
            ):
                result = self.runtime.accept_source_finished(self._source_timestamp_ns)
                self._source_seen = True
                self._on_scored(
                    result.complete,
                    result.achieved_score,
                    result.events[-1].sim_timestamp_ns,
                )
        if not self._finalize_seen:
            request = self.protocol.read_finalize_request()
            if request is not None:
                if request.get("run_id") != self.run_id:
                    raise ValueError("finalize request belongs to another run")
                self._finalize_seen = True
                self._before_quiescence()
                self.runtime.begin_finalization()
        if not self.runtime.quiescent:
            return False
        committed = self.protocol.read_terminal_committed()
        return committed is not None and committed.get("run_id") == self.run_id


class _StructuredLogger:
    def __init__(self, run_id: str) -> None:
        self.run_id = run_id
        self.silent = False

    def emit(self, event: str, *, sim_timestamp_ns: int | None = None, **fields: Any) -> None:
        if self.silent:
            return
        from artifacts.structured_log import StructuredEvent, write_event

        write_event(
            sys.stdout,
            StructuredEvent(
                run_id=self.run_id,
                module="scorekeeper",
                severity="INFO",
                event=event,
                sim_timestamp=(
                    None
                    if sim_timestamp_ns is None
                    else Decimal(sim_timestamp_ns) / Decimal(1_000_000_000)
                ),
                wall_timestamp=datetime.now(timezone.utc),
                fields=fields,
            ),
        )


@dataclass
class _RosBoundary:
    node: Any
    publisher: Any
    errors: list[BaseException]

    def publish(self, event: ScoreEvent) -> None:
        from simulation_interfaces.msg import ScoreEvent as ScoreEventMessage

        self.publisher.publish(score_event_message(event, ScoreEventMessage))

    def flush(self) -> None:
        if self.publisher.wait_for_all_acked(timeout_sec=5.0) is not True:
            raise TimeoutError("score event acknowledgement deadline expired")


def _create_ros_boundary(
    run_id: str,
    runtime_ref: list[ScorekeeperRuntime],
    logger: _StructuredLogger,
) -> _RosBoundary:
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from simulation_interfaces.msg import GroundTruth, RunState, ScenarioEvent, ScoreEvent

    node = Node("drone_sim_scorekeeper")
    publisher = node.create_publisher(
        ScoreEvent,
        "/simulation/score_events",
        QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
    )
    errors: list[BaseException] = []

    def accept_ground_truth(message: Any) -> None:
        try:
            runtime_ref[0].accept_ground_truth(ground_truth_from_message(message))
        except BaseException as error:
            errors.append(error)

    def accept_scenario(message: Any) -> None:
        try:
            sample = scenario_from_message(message)
            runtime_ref[0].accept_scenario(sample)
            if sample.run_id == run_id:
                logger.emit(
                    "scenario_observed",
                    sim_timestamp_ns=sample.sim_timestamp_ns,
                    event_id=sample.event_id,
                    magnet_id=sample.magnet_id,
                    state=sample.state,
                )
        except BaseException as error:
            errors.append(error)

    def accept_clock(_message: Any) -> None:
        return

    def accept_run_state(_message: Any) -> None:
        return

    node.create_subscription(
        GroundTruth,
        "/simulation/ground_truth",
        accept_ground_truth,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    node.create_subscription(
        ScenarioEvent,
        "/simulation/scenario_events",
        accept_scenario,
        QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
    )
    node.create_subscription(
        Clock,
        "/clock",
        accept_clock,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    node.create_subscription(
        RunState,
        "/simulation/run_state",
        accept_run_state,
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    return _RosBoundary(node, publisher, errors)


def main() -> int:
    import rclpy
    from artifacts.runtime_protocol import RuntimeProtocol

    run_id = _canonical_run_id(os.environ["SIM_RUN_ID"])
    run_directory = Path(os.environ["SIM_RUN_DIRECTORY"]).resolve(strict=True)
    config_path = Path(
        os.environ.get(
            "SIM_CONFIG_PATH", str(run_directory / "configuration/run.json")
        )
    )
    rules_path = Path(
        os.environ.get(
            "SIM_SCORE_RULES_PATH",
            "/opt/drone_sim/scorekeeper/rules/descent_v1.json",
        )
    )
    settings = load_runtime_settings(config_path, run_id)
    rules = load_descent_rules(rules_path)
    protocol = RuntimeProtocol(run_directory, run_id)
    logger = _StructuredLogger(run_id)
    runtime_ref: list[ScorekeeperRuntime] = []
    rclpy.init()
    boundary = _create_ros_boundary(run_id, runtime_ref, logger)
    runtime = ScorekeeperRuntime(
        run_id,
        DescentScorer(
            run_id,
            rules,
            expected_ground_truth_samples=settings.expected_ground_truth_samples,
        ),
        run_directory=run_directory,
        protocol=protocol,
        publish=boundary.publish,
        flush=boundary.flush,
    )
    runtime_ref.append(runtime)
    driver = ScorekeeperDriver(
        run_id,
        runtime,
        protocol,
        on_scored=lambda complete, score, stamp: logger.emit(
            "score_finalized",
            sim_timestamp_ns=stamp,
            complete=complete,
            achieved_score=score,
            maximum_available_score=100.0,
            scoring_checksum=rules.scoring_checksum,
        ),
        before_quiescence=lambda: logger.emit(
            "finalizing",
            sim_timestamp_ns=(runtime.result.events[-1].sim_timestamp_ns if runtime.result else None),
        ),
    )
    exit_code = 0
    try:
        logger.emit(
            "runtime_ready",
            expected_ground_truth_samples=settings.expected_ground_truth_samples,
            ruleset_id=rules.ruleset_id,
            scoring_checksum=rules.scoring_checksum,
        )
        while rclpy.ok():
            rclpy.spin_once(boundary.node, timeout_sec=0.05)
            if boundary.errors:
                raise boundary.errors[0]
            if driver.poll():
                break
            if runtime.quiescent:
                logger.silent = True
    except BaseException as error:
        exit_code = 1
        logger.emit("runtime_exception", error_type=type(error).__name__, detail=str(error))
        try:
            if not runtime.quiescent:
                runtime.begin_finalization()
        except BaseException:
            pass
        logger.silent = True
    finally:
        boundary.node.destroy_node()
        protocol.close()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "RuntimeSettings",
    "ScorekeeperDriver",
    "ground_truth_from_message",
    "load_runtime_settings",
    "main",
    "scenario_from_message",
    "score_event_message",
]
