"""ROS 2 adapter and durable lifecycle for the production scorekeeper."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from decimal import Decimal, InvalidOperation
import hashlib
import json
import os
from pathlib import Path
from types import SimpleNamespace
import sys
from typing import Any, Callable, Protocol

from artifacts.runtime_status import SourceFinishedStatus, canonical_run_id
from .competition import (
    CompetitionScorer,
    MissionEventSample,
    PayloadEventSample,
    PayloadStateSample,
    load_competition_rules,
)
from .competition_runtime import CompetitionScorekeeperRuntime
from .descent import DescentScorer, GroundTruthSample, load_descent_rules
from .models import ScoreEvent, ScoreResult
from .runtime import ScenarioSample, ScorekeeperRuntime
from .vfs import (
    AttemptStatus,
    JudgeDecision,
    VehicleStatusSample,
    VfsJudge,
    VfsScorer,
    load_vfs_rules,
)
from .vfs_runtime import VfsScorekeeperRuntime


_FRAME_INTERVAL_NS = 50_000_000
_SCORING_SCENARIOS = frozenset(
    {"descent_v1", "competition_v1", "competition_2027_v1"}
)


@dataclass(frozen=True)
class RuntimeSettings:
    scenario: str
    expected_ground_truth_samples: int


def load_runtime_settings(path: Path | str, run_id: str) -> RuntimeSettings:
    """Read only the resolved fields that define score input cardinality."""
    canonical = canonical_run_id(run_id)
    try:
        document = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("resolved scorekeeper configuration is invalid") from error
    if not isinstance(document, dict) or document.get("run_id") != canonical:
        raise ValueError("resolved scorekeeper configuration has the wrong run_id")
    scenario = document.get("scenario")
    if scenario not in _SCORING_SCENARIOS:
        raise ValueError("scorekeeper scenario is unsupported")
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
    return RuntimeSettings(scenario, duration_ns_int // _FRAME_INTERVAL_NS)


def rules_path_for_scenario(path: Path | str, scenario: str) -> Path:
    """Resolve the selected ruleset beside either a rules directory or old file path."""
    if scenario not in _SCORING_SCENARIOS:
        raise ValueError("unsupported scoring scenario")
    configured = Path(path)
    directory = configured.parent if configured.suffix == ".json" else configured
    return directory / f"{scenario}.json"


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


def payload_state_from_message(message: Any) -> PayloadStateSample:
    return PayloadStateSample(
        run_id=message.run_id,
        sim_timestamp_ns=_timestamp_ns(message.sim_timestamp),
        aruco_id=message.aruco_id,
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
        grounded=message.grounded,
        attached=message.attached,
    )


def payload_event_from_message(message: Any) -> PayloadEventSample:
    return PayloadEventSample(
        run_id=message.run_id,
        sim_timestamp_ns=_timestamp_ns(message.sim_timestamp),
        event_id=message.event_id,
        aruco_id=message.aruco_id,
        command_id=message.command_id,
        action=message.action,
        state=message.state,
        code=message.code,
    )


def mission_event_from_message(message: Any) -> MissionEventSample:
    return MissionEventSample(
        run_id=message.run_id,
        sim_timestamp_ns=_timestamp_ns(message.sim_timestamp),
        event_id=message.event_id,
        phase=message.phase,
        state=message.state,
        detail=message.detail,
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


def vehicle_status_from_message(message: Any) -> VehicleStatusSample:
    return VehicleStatusSample(
        message.run_id,
        _timestamp_ns(message.sim_timestamp),
        message.sequence_id,
        message.vehicle_id,
        message.armed,
        message.mode,
        message.failsafe_active,
        message.failsafe_reason,
    )


def judge_decision_message(
    decision: JudgeDecision, message_type: Callable[[], Any]
) -> Any:
    message = message_type()
    message.run_id = decision.run_id
    _assign_stamp(message.sim_timestamp, decision.sim_timestamp_ns)
    message.decision_id = decision.decision_id
    message.request_id = decision.request_id
    message.request_event_id = decision.request_event_id
    message.phase = decision.phase
    message.checkpoint = decision.checkpoint
    message.outcome = int(decision.outcome)
    message.eligible_aruco_ids = list(decision.eligible_aruco_ids)
    message.active_elapsed_ns = decision.active_elapsed_ns
    message.reason_code = decision.reason_code
    message.evidence_refs = list(decision.evidence_refs)
    return message


def attempt_status_message(
    status: AttemptStatus, message_type: Callable[[], Any]
) -> Any:
    message = message_type()
    message.run_id = status.run_id
    _assign_stamp(message.sim_timestamp, status.sim_timestamp_ns)
    message.sequence_id = status.sequence_id
    message.state = int(status.state)
    message.active_elapsed_ns = status.active_elapsed_ns
    message.approval_timestamp_ns = status.approval_timestamp_ns
    message.started = status.started
    return message


class DriverProtocol(Protocol):
    def read_status(
        self, status_type: type[SourceFinishedStatus]
    ) -> SourceFinishedStatus | None: ...
    def read_finalize_request(self) -> dict[str, Any] | None: ...
    def read_terminal_committed(self) -> dict[str, Any] | None: ...


class ScoreRuntimeProtocol(Protocol):
    quiescent: bool
    result: ScoreResult | None

    def source_inputs_observed_through(self, timestamp_ns: int) -> bool: ...
    def accept_source_finished(self, sim_timestamp_ns: int) -> ScoreResult: ...
    def fail(self, reason: str) -> None: ...
    def begin_finalization(self) -> None: ...


class ScorekeeperDriver:
    """Idempotently bridge durable source/finalization facts into the runtime."""

    def __init__(
        self,
        run_id: str,
        runtime: ScoreRuntimeProtocol,
        protocol: DriverProtocol,
        *,
        on_scored: Callable[[bool, float, int], None] = lambda _complete, _score, _stamp: None,
        before_quiescence: Callable[[], None] = lambda: None,
    ) -> None:
        self.run_id = canonical_run_id(run_id)
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
                self.protocol.read_status(SourceFinishedStatus)
                if self._source_timestamp_ns is None
                else None
            )
            if source is not None:
                self._source_timestamp_ns = source.sim_timestamp_ns
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
                self._finalize_seen = True
                self._before_quiescence()
                self.runtime.begin_finalization()
        if not self.runtime.quiescent:
            return False
        committed = self.protocol.read_terminal_committed()
        return committed is not None


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
    judge_publisher: Any | None = None
    attempt_publisher: Any | None = None

    def publish(self, event: ScoreEvent) -> None:
        from simulation_interfaces.msg import ScoreEvent as ScoreEventMessage

        self.publisher.publish(score_event_message(event, ScoreEventMessage))

    def flush(self) -> None:
        from rclpy.duration import Duration

        for publisher in (self.publisher, self.judge_publisher, self.attempt_publisher):
            if publisher is not None and publisher.wait_for_all_acked(
                timeout=Duration(seconds=5.0)
            ) is not True:
                raise TimeoutError("scorekeeper publisher acknowledgement deadline expired")

    def publish_judge(self, decision: JudgeDecision) -> None:
        if self.judge_publisher is None:
            raise RuntimeError("judge publisher is unavailable")
        from simulation_interfaces.msg import JudgeDecision as JudgeDecisionMessage

        self.judge_publisher.publish(
            judge_decision_message(decision, JudgeDecisionMessage)
        )

    def publish_attempt(self, status: AttemptStatus) -> None:
        if self.attempt_publisher is None:
            raise RuntimeError("attempt publisher is unavailable")
        from simulation_interfaces.msg import AttemptStatus as AttemptStatusMessage

        self.attempt_publisher.publish(
            attempt_status_message(status, AttemptStatusMessage)
        )


def _create_ros_boundary(
    run_id: str,
    runtime_ref: list[Any],
    logger: _StructuredLogger,
    *,
    scenario: str = "descent_v1",
) -> _RosBoundary:
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from simulation_interfaces.msg import (
        GroundTruth,
        MissionEvent,
        PayloadEvent,
        PayloadState,
        RunState,
        ScenarioEvent,
        ScoreEvent,
    )

    if scenario not in _SCORING_SCENARIOS:
        raise ValueError("unsupported scoring scenario")

    node = Node("drone_sim_scorekeeper")
    publisher = node.create_publisher(
        ScoreEvent,
        "/simulation/score_events",
        QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
    )
    judge_publisher = None
    attempt_publisher = None
    if scenario == "competition_2027_v1":
        from simulation_interfaces.msg import (
            AttemptStatus as AttemptStatusMessage,
            JudgeDecision as JudgeDecisionMessage,
            VehicleStatus,
        )

        durable_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        judge_publisher = node.create_publisher(
            JudgeDecisionMessage, "/simulation/judge_decisions", durable_qos
        )
        attempt_publisher = node.create_publisher(
            AttemptStatusMessage,
            "/simulation/attempt_status",
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
        )
    errors: list[BaseException] = []

    def accept_ground_truth(message: Any) -> None:
        try:
            if message.run_id != run_id:
                return
            runtime_ref[0].accept_ground_truth(ground_truth_from_message(message))
        except BaseException as error:
            runtime_ref[0].fail("ros_evidence_invalid")
            errors.append(error)

    def accept_scenario(message: Any) -> None:
        try:
            if message.run_id != run_id:
                return
            sample = scenario_from_message(message)
            runtime_ref[0].accept_scenario(sample)
        except BaseException as error:
            runtime_ref[0].fail("ros_evidence_invalid")
            errors.append(error)
            return
        try:
            logger.emit(
                "scenario_observed",
                sim_timestamp_ns=sample.sim_timestamp_ns,
                event_id=sample.event_id,
                magnet_id=sample.magnet_id,
                state=sample.state,
            )
        except BaseException as error:
            errors.append(error)

    def accept_payload_state(message: Any) -> None:
        try:
            if message.run_id != run_id:
                return
            runtime_ref[0].accept_payload_state(payload_state_from_message(message))
        except BaseException as error:
            runtime_ref[0].fail("ros_evidence_invalid")
            errors.append(error)

    def accept_payload_event(message: Any) -> None:
        try:
            if message.run_id != run_id:
                return
            sample = payload_event_from_message(message)
            runtime_ref[0].accept_payload_event(sample)
        except BaseException as error:
            runtime_ref[0].fail("ros_evidence_invalid")
            errors.append(error)
            return
        try:
            logger.emit(
                "payload_event_observed",
                sim_timestamp_ns=sample.sim_timestamp_ns,
                event_id=sample.event_id,
                aruco_id=sample.aruco_id,
                action=sample.action,
                state=sample.state,
            )
        except BaseException as error:
            errors.append(error)

    def accept_mission_event(message: Any) -> None:
        try:
            if message.run_id != run_id:
                return
            sample = mission_event_from_message(message)
            runtime_ref[0].accept_mission_event(sample)
        except BaseException as error:
            runtime_ref[0].fail("ros_evidence_invalid")
            errors.append(error)
            return
        try:
            logger.emit(
                "mission_event_observed",
                sim_timestamp_ns=sample.sim_timestamp_ns,
                event_id=sample.event_id,
                phase=sample.phase,
                state=sample.state,
            )
        except BaseException as error:
            errors.append(error)

    def accept_clock(_message: Any) -> None:
        return

    def accept_run_state(_message: Any) -> None:
        return

    def accept_vehicle_status(message: Any) -> None:
        try:
            if message.run_id != run_id:
                return
            runtime_ref[0].accept_vehicle_status(vehicle_status_from_message(message))
        except BaseException as error:
            runtime_ref[0].fail("ros_evidence_invalid")
            errors.append(error)

    node.create_subscription(
        GroundTruth,
        "/simulation/ground_truth",
        accept_ground_truth,
        QoSProfile(
            depth=10,
            reliability=(
                ReliabilityPolicy.RELIABLE
                if scenario in {"competition_v1", "competition_2027_v1"}
                else ReliabilityPolicy.BEST_EFFORT
            ),
        ),
    )
    if scenario in {"competition_v1", "competition_2027_v1"}:
        node.create_subscription(
            PayloadState,
            "/simulation/payload_state",
            accept_payload_state,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
        )
        event_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(
            PayloadEvent,
            "/simulation/payload_events",
            accept_payload_event,
            event_qos,
        )
        node.create_subscription(
            MissionEvent,
            "/simulation/mission_events",
            accept_mission_event,
            event_qos,
        )
        if scenario == "competition_2027_v1":
            from simulation_interfaces.msg import VehicleStatus

            node.create_subscription(
                VehicleStatus,
                "/simulation/vehicle_status",
                accept_vehicle_status,
                QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
            )
    else:
        node.create_subscription(
            ScenarioEvent,
            "/simulation/scenario_events",
            accept_scenario,
            QoSProfile(
                depth=100,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
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
    return _RosBoundary(node, publisher, errors, judge_publisher, attempt_publisher)


def main() -> int:
    import rclpy
    from artifacts.runtime_protocol import RuntimeProtocol

    run_id = canonical_run_id(os.environ["SIM_RUN_ID"])
    run_directory = Path(os.environ["SIM_RUN_DIRECTORY"]).resolve(strict=True)
    config_path = Path(
        os.environ.get(
            "SIM_CONFIG_PATH", str(run_directory / "configuration/run.json")
        )
    )
    settings = load_runtime_settings(config_path, run_id)
    rules_path = rules_path_for_scenario(
        os.environ.get(
            "SIM_SCORE_RULES_PATH",
            "/opt/drone_sim/scorekeeper/rules",
        ),
        settings.scenario,
    )
    protocol = RuntimeProtocol(run_directory, run_id)
    logger = _StructuredLogger(run_id)
    runtime_ref: list[Any] = []
    rclpy.init()
    boundary = _create_ros_boundary(
        run_id,
        runtime_ref,
        logger,
        scenario=settings.scenario,
    )
    if settings.scenario == "competition_2027_v1":
        from artifacts.vfs_contract import (
            load_vfs_course,
            load_vfs_scenario,
            validate_vfs_configuration,
        )

        resolved_document = json.loads(config_path.read_text(encoding="utf-8"))
        competition = resolved_document.get("competition")
        if not isinstance(competition, dict):
            raise ValueError("2027 scorekeeper requires resolved competition configuration")
        if (competition.get("course"), competition.get("scenario")) != (
            "course.yaml",
            "scenario.yaml",
        ):
            raise ValueError("2027 scorekeeper configuration paths are invalid")
        course_path = config_path.parent / competition.get("course", "")
        scenario_path = config_path.parent / competition.get("scenario", "")
        if (
            hashlib.sha256(course_path.read_bytes()).hexdigest()
            != competition.get("course_sha256")
            or hashlib.sha256(scenario_path.read_bytes()).hexdigest()
            != competition.get("scenario_sha256")
        ):
            raise ValueError("2027 scorekeeper configuration checksum mismatch")
        course = load_vfs_course(course_path)
        scenario = load_vfs_scenario(scenario_path)
        validate_vfs_configuration(course, scenario)
        vehicle_mass_kg = scenario.aircraft_empty_mass_kg
        unloaded_weight_lb = (
            vehicle_mass_kg + scenario.payload_geometry.tare_mass_kg
        ) * 2.2046226218487757
        rules = load_vfs_rules(rules_path)
        runtime = VfsScorekeeperRuntime(
            run_id,
            VfsJudge(run_id, course, scenario),
            VfsScorer(
                run_id,
                rules,
                course,
                scenario,
                unloaded_weight_lb=unloaded_weight_lb,
            ),
            publish_decision=boundary.publish_judge,
            publish_attempt=boundary.publish_attempt,
            run_directory=run_directory,
            protocol=protocol,
            publish_score=boundary.publish,
            flush=boundary.flush,
        )
    elif settings.scenario == "competition_v1":
        rules = load_competition_rules(rules_path)
        runtime = CompetitionScorekeeperRuntime(
            run_id,
            CompetitionScorer(run_id, rules),
            run_directory=run_directory,
            protocol=protocol,
            publish=boundary.publish,
            flush=boundary.flush,
        )
    else:
        rules = load_descent_rules(rules_path)
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
            maximum_available_score=rules.maximum_available_score,
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
    "mission_event_from_message",
    "payload_event_from_message",
    "payload_state_from_message",
    "rules_path_for_scenario",
    "scenario_from_message",
    "score_event_message",
    "vehicle_status_from_message",
    "judge_decision_message",
    "attempt_status_message",
]
