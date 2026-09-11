"""QGC admission and the sole synchronous command execution loop.

This module is hardware-independent. A live composition injects a transport
that proves its actual MAVLink source identity before construction succeeds.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from collections import deque
import hashlib
import math
import os
from pathlib import Path
import queue
import signal
import threading
import time
from types import MappingProxyType
from typing import Callable, Mapping

from .. import timebase
from ..auto_attempt import PHYSICAL_EVIDENCE_INTERVAL_SECONDS
from .attempt_setup import (
    DeploymentProfile,
    PreparedAttempt,
    load_deployment_profile_bytes,
    load_prepared_session_bytes,
)
from .flight_profile import FlightProfile, load_flight_profile_bytes
from .flight_state import SourceIdentity
from .mission_supervisor import (
    ABORT_AND_RECOVER,
    CLEAR_ALL,
    CLEAR_PICKUPS,
    PinnedAttemptLedger,
    AuthorityLost,
    FlightOperationError,
    FM1,
    FM2,
    FM3,
    MUTATION_COMMANDS,
    PHASE_COMMANDS,
    UPDATE_L,
    UPDATE_TARGET,
    UPDATE_WA,
    UPDATE_WM1,
    UPDATE_WM2,
    UPDATE_WM3,
    UPDATE_WM4,
    UPDATE_WM5,
    UPDATE_WM6,
    CommandEnvelope,
    CommandRejected,
    MissionAbort,
    MissionSupervisor,
)
from .listener_runtime import AutopilotVersionContract, TelemetryStartupPolicy, _verified_text


ACK_ACCEPTED = 0
ACK_TEMPORARILY_REJECTED = 1
ACK_DENIED = 2
ACK_UNSUPPORTED = 3
ACK_FAILED = 4
ACK_IN_PROGRESS = 5
ACK_CANCELLED = 6

_RESULT_CODES = {
    "ACCEPTED": ACK_ACCEPTED,
    "TEMPORARILY_REJECTED": ACK_TEMPORARILY_REJECTED,
    "DENIED": ACK_DENIED,
    "UNSUPPORTED": ACK_UNSUPPORTED,
    "IN_PROGRESS": ACK_IN_PROGRESS,
    "SUCCEEDED": ACK_ACCEPTED,
    "FAILED": ACK_FAILED,
    "ABORTED": ACK_CANCELLED,
}


@dataclass(frozen=True)
class CommandAck:
    command: int
    result: int
    progress: int
    target_system: int | None
    target_component: int | None


class _TargetPreservingMessage:
    """Hide target fields from DroneKit's FC-only ``fix_targets`` wrapper."""

    __slots__ = ("_message", "_force_mavlink1")

    def __init__(self, message: object, *, force_mavlink1: bool) -> None:
        self._message = message
        self._force_mavlink1 = force_mavlink1

    def pack(self, mav: object, *, force_mavlink1: bool = False) -> bytes:
        pack = getattr(self._message, "pack")
        return pack(mav, force_mavlink1=self._force_mavlink1)


class DroneKitQGCAckTransport:
    """Queue QGC ACKs through DroneKit without retargeting them to the FC.

    This adapter uses DroneKit's one existing MAVLink encoder and MAVWriter
    queue. It creates no connection, output thread, or flight-command seam.
    """

    def __init__(
        self,
        vehicle: object,
        *,
        expected_source: SourceIdentity,
        wire_protocol: str,
    ) -> None:
        if not isinstance(expected_source, SourceIdentity):
            raise TypeError("expected_source must be a SourceIdentity")
        if wire_protocol not in ("1.0", "2.0"):
            raise ValueError("MAVLink wire protocol is unsupported")
        add_listener = getattr(vehicle, "add_message_listener", None)
        send_mavlink = getattr(vehicle, "send_mavlink", None)
        master = getattr(vehicle, "_master", None)
        encoder = getattr(master, "mav", None)
        if not callable(add_listener) or not callable(send_mavlink) or encoder is None:
            raise TypeError("vehicle must expose the DroneKit MAVLink interface")
        actual_wire = getattr(master, "WIRE_PROTOCOL_VERSION", None)
        if actual_wire != wire_protocol:
            raise ValueError("actual MAVLink wire protocol does not match configuration")
        actual_source = SourceIdentity(
            getattr(encoder, "srcSystem", None),
            getattr(encoder, "srcComponent", None),
        )
        if actual_source != expected_source:
            raise ValueError("actual MAVLink source does not match configuration")
        writer = getattr(encoder, "file", None)
        validated_writer = getattr(writer, "delegate", writer)
        if (
            validated_writer is None
            or type(validated_writer).__module__ != "dronekit.mavlink"
            or type(validated_writer).__name__ != "MAVWriter"
        ):
            raise ValueError("encoder must use DroneKit MAVWriter")
        outbound = getattr(validated_writer, "queue", None)
        if not isinstance(outbound, queue.Queue) or outbound.maxsize != 0:
            raise ValueError("DroneKit MAVWriter must use its unbounded output queue")
        encode = getattr(encoder, "command_ack_encode", None)
        if not callable(encode):
            raise TypeError("MAVLink encoder does not support COMMAND_ACK")
        probe = (
            encode(0, ACK_ACCEPTED)
            if wire_protocol == "1.0"
            else encode(
                0,
                ACK_ACCEPTED,
                0,
                0,
                expected_source.system_id,
                expected_source.component_id,
            )
        )
        packet = probe.pack(encoder, force_mavlink1=wire_protocol == "1.0")
        expected_magic = 0xFE if wire_protocol == "1.0" else 0xFD
        source_offset = 3 if wire_protocol == "1.0" else 5
        if (
            not isinstance(packet, bytes)
            or len(packet) <= source_offset + 1
            or packet[0] != expected_magic
            or packet[source_offset] != expected_source.system_id
            or packet[source_offset + 1] != expected_source.component_id
        ):
            raise ValueError("encoded MAVLink header does not prove source and wire protocol")

        self.source_identity = actual_source
        self.wire_protocol = wire_protocol
        self._vehicle = vehicle
        self._encoder = encoder
        self._encode_ack = encode

    def install_message_callback(
        self,
        message_names: tuple[str, ...],
        callback: Callable[[object], None],
    ) -> None:
        if (
            not isinstance(message_names, tuple)
            or not message_names
            or any(not isinstance(name, str) or not name for name in message_names)
            or not callable(callback)
        ):
            raise TypeError("message names and callback must be explicit")

        def deliver(_vehicle: object, _name: str, message: object) -> None:
            callback(message)

        for name in message_names:
            self._vehicle.add_message_listener(name, deliver)

    def send_command_ack(self, ack: CommandAck) -> None:
        if not isinstance(ack, CommandAck):
            raise TypeError("ack must be a CommandAck")
        if (
            _exact_command(ack.command) is None
            or not isinstance(ack.result, int)
            or isinstance(ack.result, bool)
            or not 0 <= ack.result <= 255
            or not isinstance(ack.progress, int)
            or isinstance(ack.progress, bool)
            or not 0 <= ack.progress <= 255
        ):
            raise ValueError("COMMAND_ACK fields are invalid")
        if self.wire_protocol == "1.0":
            if ack.target_system is not None or ack.target_component is not None:
                raise ValueError("MAVLink 1 COMMAND_ACK cannot carry target extensions")
            message = self._encode_ack(ack.command, ack.result)
        else:
            if _exact_id(ack.target_system) is None or _exact_id(ack.target_component) is None:
                raise ValueError("MAVLink 2 COMMAND_ACK requires exact target identities")
            message = self._encode_ack(
                ack.command,
                ack.result,
                ack.progress,
                0,
                ack.target_system,
                ack.target_component,
            )
        wrapped = _TargetPreservingMessage(
            message,
            force_mavlink1=self.wire_protocol == "1.0",
        )
        self._vehicle.send_mavlink(wrapped)


@dataclass(frozen=True)
class TelemetryStartupEvidence:
    received_at_by_message_id: Mapping[int, tuple[float, ...]]
    autopilot_version_received_at: float


@dataclass
class _TelemetryStartupVerification:
    collector: object
    evidence: TelemetryStartupEvidence | None = None

    def accept(self, evidence: object) -> None:
        if not isinstance(evidence, TelemetryStartupEvidence):
            raise RuntimeError("startup telemetry returned invalid evidence")
        self.evidence = evidence

    def verify_after_guided(self) -> None:
        verify = getattr(self.collector, "verify_after_guided", None)
        if not callable(verify):
            raise RuntimeError("staged telemetry verifier is unavailable")
        self.accept(verify())


class _TelemetryStartupCleanupError(RuntimeError):
    def __init__(self, errors: list[tuple[str, BaseException]]) -> None:
        self.errors = tuple(errors)
        details = "; ".join(
            f"{label}: {_error_detail(error)}" for label, error in self.errors
        )
        super().__init__(f"telemetry startup cleanup failed: {details}")


class TelemetryStartupCollector:
    """Request and prove source-filtered startup telemetry before admission."""

    def __init__(
        self,
        *,
        controller: object,
        flight_profile: FlightProfile,
        policy: TelemetryStartupPolicy,
        autopilot_version: AutopilotVersionContract,
        clock: Callable[[], float] = timebase.monotonic,
    ) -> None:
        if not isinstance(flight_profile, FlightProfile):
            raise TypeError("flight_profile must be a FlightProfile")
        if not isinstance(policy, TelemetryStartupPolicy):
            raise TypeError("policy must be a TelemetryStartupPolicy")
        if not isinstance(autopilot_version, AutopilotVersionContract):
            raise TypeError("autopilot_version must be an AutopilotVersionContract")
        if clock is not timebase.monotonic:
            raise ValueError("telemetry collector must use shared timebase.monotonic")
        vehicle = getattr(controller, "vehicle", None)
        if not callable(getattr(vehicle, "add_message_listener", None)):
            raise TypeError("controller vehicle must install message listeners")
        for method in (
            "set_telemetry_interval",
            "request_autopilot_version",
            "close_startup_telemetry",
        ):
            if not callable(getattr(controller, method, None)):
                raise TypeError(f"controller must provide {method}")
        self._controller = controller
        self._vehicle = vehicle
        self._profile = flight_profile
        self._policy = policy
        self._version_contract = autopilot_version
        self._clock = clock
        self._lock = threading.Lock()
        self._request_started: dict[int, float] = {}
        self._received: dict[int, deque[float]] = {
            request.message_id: deque(maxlen=policy.minimum_distinct_samples)
            for request in flight_profile.telemetry_requests
        }
        self._version_request_started: float | None = None
        self._version_received_at: float | None = None
        self._callback_error: BaseException | None = None
        self._started = False
        self._active = False
        self._listener_installed = False
        self._closed = False
        self._startup_requests_closed = False
        self._staged_prepared = False
        self._staged_verification_started = False
        self._verification_gate_time: float | None = None

    def configure_and_collect(self) -> TelemetryStartupEvidence:
        self._begin_collection()
        try:
            self._issue_requests()
            deadline = self._now() + self._policy.collection_timeout_s
            while True:
                evidence = self._current_evidence()
                if evidence is not None:
                    break
                if self._now() >= deadline:
                    with self._lock:
                        version_missing = self._version_received_at is None
                    if version_missing:
                        raise TimeoutError("AUTOPILOT_VERSION was not observed")
                    raise TimeoutError("requested telemetry cadence was not achieved")
                timebase.sleep(self._policy.poll_interval_s)
        except BaseException as primary:
            self._close_after_failure(primary)
            raise
        else:
            self.close()
            return evidence

    def prepare(self) -> None:
        """Install observation and prove requests/version without awaiting cadence."""
        self._begin_collection()
        self._staged_prepared = True
        try:
            self._issue_requests()
            deadline = time.monotonic() + self._policy.collection_timeout_s
            while True:
                with self._lock:
                    if self._callback_error is not None:
                        raise self._callback_error
                    if self._version_received_at is not None:
                        break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("AUTOPILOT_VERSION was not observed")
                time.sleep(min(self._policy.poll_interval_s, remaining))
        except BaseException as primary:
            self._close_after_failure(primary)
            raise
        else:
            try:
                self._cleanup(remove_listener=False)
            except BaseException as cleanup_error:
                self._close_after_failure(cleanup_error)
                raise

    def verify_after_guided(self) -> TelemetryStartupEvidence:
        """Prove post-GUIDED cadence on strictly advancing shared simulation time."""
        if not self._staged_prepared or not self._active:
            raise RuntimeError("staged telemetry was not prepared")
        if self._staged_verification_started:
            raise RuntimeError("staged telemetry verification is one-shot")
        self._staged_verification_started = True
        try:
            with self._lock:
                self._verification_gate_time = self._now()
                for samples in self._received.values():
                    samples.clear()
            deadline = time.monotonic() + self._policy.collection_timeout_s
            while True:
                self._controller.check_permission()
                self._now()
                evidence = self._current_evidence()
                if evidence is not None:
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise FlightOperationError(
                        "requested telemetry cadence was not achieved"
                    )
                time.sleep(min(self._policy.poll_interval_s, remaining))
        except BaseException as primary:
            self._close_after_failure(primary)
            raise
        else:
            self.close()
            return evidence

    def close(self) -> None:
        """Close the listener and startup request capability idempotently."""
        self._cleanup(remove_listener=True)

    def _close_after_failure(self, primary: BaseException) -> None:
        try:
            self.close()
        except BaseException as cleanup_error:
            primary.__cause__ = cleanup_error
            primary.__suppress_context__ = True

    def _cleanup(self, *, remove_listener: bool) -> None:
        with self._lock:
            if remove_listener:
                if self._closed:
                    return
                self._closed = True
                self._active = False
                remove_installed_listener = self._listener_installed
                self._listener_installed = False
            else:
                remove_installed_listener = False
            close_startup_requests = not self._startup_requests_closed
            self._startup_requests_closed = True
        errors: list[tuple[str, BaseException]] = []
        if remove_installed_listener:
            try:
                remove = getattr(self._vehicle, "remove_message_listener", None)
                if callable(remove):
                    remove("*", self._observe)
            except BaseException as error:
                errors.append(("listener removal", error))
        if close_startup_requests:
            try:
                self._controller.close_startup_telemetry()
            except BaseException as error:
                errors.append(("startup request cleanup", error))
        if errors:
            failure = _TelemetryStartupCleanupError(errors)
            raise failure from errors[0][1]

    def _begin_collection(self) -> None:
        if self._started:
            raise RuntimeError("telemetry startup collection is one-shot")
        self._started = True
        self._active = True
        self._vehicle.add_message_listener("*", self._observe)
        self._listener_installed = True

    def _issue_requests(self) -> None:
        for request in self._profile.telemetry_requests:
            with self._lock:
                self._request_started[request.message_id] = self._now()
            self._controller.set_telemetry_interval(
                request.message_id,
                request.interval_us,
                timeout_s=self._policy.command_ack_timeout_s,
                poll_interval_s=self._policy.poll_interval_s,
            )
        with self._lock:
            self._version_request_started = self._now()
        self._controller.request_autopilot_version(
            timeout_s=self._policy.command_ack_timeout_s,
            poll_interval_s=self._policy.poll_interval_s,
        )

    def _observe(self, _vehicle: object, _name: str, message: object) -> None:
        try:
            with self._lock:
                if not self._active:
                    return
            if (
                message.get_srcSystem() != self._profile.flight_controller.system_id  # type: ignore[attr-defined]
                or message.get_srcComponent()
                != self._profile.flight_controller.component_id  # type: ignore[attr-defined]
            ):
                return
            message_id = message.get_msgId()  # type: ignore[attr-defined]
            if not isinstance(message_id, int) or isinstance(message_id, bool):
                return
            observed_at = self._now()
            with self._lock:
                if not self._active:
                    return
                if message_id == 148:
                    if (
                        self._version_request_started is None
                        or observed_at < self._version_request_started
                    ):
                        return
                    if not self._version_matches(message):
                        self._callback_error = RuntimeError(
                            "actual firmware metadata does not match configuration"
                        )
                        return
                    self._version_received_at = observed_at
                    return
                started = self._request_started.get(message_id)
                gate = self._verification_gate_time
                if (
                    started is not None
                    and observed_at >= started
                    and (gate is None or observed_at > gate)
                ):
                    self._received[message_id].append(observed_at)
        except BaseException as error:
            with self._lock:
                if self._callback_error is None:
                    self._callback_error = error

    def _version_matches(self, message: object) -> bool:
        raw_custom = getattr(message, "flight_custom_version", None)
        try:
            custom = bytes(raw_custom)
        except (TypeError, ValueError):
            return False
        return (
            getattr(message, "flight_sw_version", None)
            == self._version_contract.flight_sw_version
            and custom == self._version_contract.flight_custom_version
        )

    def _current_evidence(self) -> TelemetryStartupEvidence | None:
        with self._lock:
            if self._callback_error is not None:
                raise self._callback_error
            version_received_at = self._version_received_at
            snapshots = {
                message_id: tuple(times) for message_id, times in self._received.items()
            }
        if version_received_at is None:
            return None
        minimum = self._policy.minimum_distinct_samples
        intervals = {
            request.message_id: request.interval_us / 1_000_000
            for request in self._profile.telemetry_requests
        }
        for message_id, times in snapshots.items():
            if len(times) < minimum:
                return None
            recent = times[-minimum:]
            if any(later <= earlier for earlier, later in zip(recent, recent[1:])):
                return None
            error = self._policy.maximum_interval_error_fraction
            minimum_gap = intervals[message_id] * (1 - error)
            maximum_gap = intervals[message_id] * (
                1 + error
            )
            if any(
                not minimum_gap <= later - earlier <= maximum_gap
                for earlier, later in zip(recent, recent[1:])
            ):
                return None
            now = self._now()
            if not 0 <= now - recent[-1] <= maximum_gap:
                return None
        return TelemetryStartupEvidence(
            received_at_by_message_id=MappingProxyType(snapshots),
            autopilot_version_received_at=version_received_at,
        )

    def _now(self) -> float:
        value = self._clock()
        if not _finite_number(value):
            raise timebase.ClockError("telemetry clock returned a nonfinite value")
        return float(value)


@dataclass(frozen=True)
class ListenerStartupFiles:
    profile_path: os.PathLike[str] | str
    session_path: os.PathLike[str] | str
    actions_path: os.PathLike[str] | str
    ledger_path: os.PathLike[str] | str

    def __post_init__(self) -> None:
        paths = {
            name: Path(getattr(self, name)).resolve()
            for name in (
                "profile_path",
                "session_path",
                "actions_path",
                "ledger_path",
            )
        }
        if len(set(paths.values())) != len(paths):
            raise ValueError("listener startup files must be distinct")
        for name, path in paths.items():
            object.__setattr__(self, name, path)


class ValidatedListenerArtifacts:
    """Opaque immutable snapshot issued only by the exact-byte loader."""

    __slots__ = (
        "_deployment_profile",
        "_flight_profile",
        "_prepared_attempt",
        "_ledger_path",
        "_bound_ledger_path",
        "_ledger",
        "_ledger_identity",
        "_lock_identity",
        "_profile_bytes",
        "_session_bytes",
        "_actions_bytes",
    )

    def __init__(self, *_args: object, **_kwargs: object) -> None:
        raise TypeError("validated listener artifacts must come from a loader")

    def __setattr__(self, _name: str, _value: object) -> None:
        raise TypeError("validated listener artifacts are immutable")

    def __copy__(self) -> object:
        raise TypeError("validated listener artifacts cannot be copied")

    def __deepcopy__(self, _memo: object) -> object:
        raise TypeError("validated listener artifacts cannot be copied")

    def __del__(self) -> None:
        ledger = getattr(self, "_ledger", None)
        if ledger is not None:
            try:
                ledger.close()
            except BaseException:
                pass

    @property
    def deployment_profile(self) -> DeploymentProfile:
        return self._deployment_profile

    @property
    def flight_profile(self) -> FlightProfile:
        return self._flight_profile

    @property
    def prepared_attempt(self) -> PreparedAttempt:
        return self._prepared_attempt

    @property
    def ledger(self) -> PinnedAttemptLedger:
        return self._ledger

    def close(self) -> None:
        self._ledger.close()

    @property
    def wire_protocol(self) -> str:
        return self._deployment_profile.mavlink_wire_protocol

    @property
    def profile_sha256(self) -> str:
        return hashlib.sha256(self._profile_bytes).hexdigest()

    @property
    def session_sha256(self) -> str:
        return hashlib.sha256(self._session_bytes).hexdigest()

    @property
    def actions_sha256(self) -> str:
        return hashlib.sha256(self._actions_bytes).hexdigest()


def _new_validated_listener_artifacts(
    deployment: DeploymentProfile,
    flight_profile: FlightProfile,
    prepared: PreparedAttempt,
    *,
    ledger_path: os.PathLike[str] | str,
    profile_bytes: bytes,
    session_bytes: bytes,
    actions_bytes: bytes,
) -> ValidatedListenerArtifacts:
    ledger = PinnedAttemptLedger(ledger_path)
    artifacts = object.__new__(ValidatedListenerArtifacts)
    object.__setattr__(artifacts, "_deployment_profile", deployment)
    object.__setattr__(artifacts, "_flight_profile", flight_profile)
    object.__setattr__(artifacts, "_prepared_attempt", prepared)
    object.__setattr__(artifacts, "_ledger_path", ledger.path)
    object.__setattr__(artifacts, "_bound_ledger_path", ledger.path)
    object.__setattr__(artifacts, "_ledger", ledger)
    object.__setattr__(artifacts, "_ledger_identity", ledger.ledger_identity)
    object.__setattr__(artifacts, "_lock_identity", ledger.lock_identity)
    object.__setattr__(artifacts, "_profile_bytes", profile_bytes)
    object.__setattr__(artifacts, "_session_bytes", session_bytes)
    object.__setattr__(artifacts, "_actions_bytes", actions_bytes)
    return artifacts


def _require_runtime_modes(profile: FlightProfile) -> None:
    if profile.copter_modes.get(6) != "RTL" or profile.copter_modes.get(9) != "LAND":
        raise ValueError("selected Copter mode contract must define RTL=6 and LAND=9")


def load_listener_artifacts(files: ListenerStartupFiles) -> ValidatedListenerArtifacts:
    """Fix one validated offline session before any transport is constructed."""
    if not isinstance(files, ListenerStartupFiles):
        raise TypeError("files must be ListenerStartupFiles")
    try:
        profile_bytes = Path(files.profile_path).read_bytes()
        session_bytes = Path(files.session_path).read_bytes()
        actions_bytes = Path(files.actions_path).read_bytes()
    except OSError as error:
        raise ValueError(f"listener startup files are unreadable: {error}") from error
    artifacts = load_listener_artifacts_bytes(
        profile_bytes,
        session_bytes,
        actions_bytes,
        ledger_path=files.ledger_path,
    )
    try:
        _require_current_listener_attempt(artifacts)
    except BaseException:
        artifacts.close()
        raise
    return artifacts


def load_listener_artifacts_bytes(
    profile_bytes: bytes,
    session_bytes: bytes,
    actions_bytes: bytes,
    *,
    ledger_path: os.PathLike[str] | str,
) -> ValidatedListenerArtifacts:
    """Bind validated listener objects to exact immutable source bytes."""
    deployment, flight_profile, prepared = _parse_listener_artifact_bytes(
        profile_bytes, session_bytes, actions_bytes
    )
    return _new_validated_listener_artifacts(
        deployment,
        flight_profile,
        prepared,
        ledger_path=ledger_path,
        profile_bytes=profile_bytes,
        session_bytes=session_bytes,
        actions_bytes=actions_bytes,
    )


def _parse_listener_artifact_bytes(
    profile_bytes: bytes, session_bytes: bytes, actions_bytes: bytes
) -> tuple[DeploymentProfile, FlightProfile, PreparedAttempt]:
    deployment = load_deployment_profile_bytes(profile_bytes)
    flight_profile = load_flight_profile_bytes(profile_bytes)
    _require_runtime_modes(flight_profile)
    prepared = load_prepared_session_bytes(
        session_bytes,
        actions_bytes,
        deployment,
    )
    if (
        prepared.profile_id != flight_profile.profile_id
        or flight_profile.raw_sha256 != deployment.raw_sha256
    ):
        raise ValueError("prepared listener profile identity is inconsistent")
    return deployment, flight_profile, prepared


def _require_current_listener_attempt(artifacts: ValidatedListenerArtifacts) -> None:
    artifacts.ledger.require_current_unconsumed(artifacts.prepared_attempt.attempt_id)


def _validated_artifacts(
    source: ListenerStartupFiles | ValidatedListenerArtifacts,
) -> ValidatedListenerArtifacts:
    if isinstance(source, ListenerStartupFiles):
        return load_listener_artifacts(source)
    if not isinstance(source, ValidatedListenerArtifacts):
        raise TypeError("listener source must be startup files or validated artifacts")
    return _revalidate_artifact_relationships(source)


def _revalidate_artifact_relationships(
    source: ValidatedListenerArtifacts,
) -> ValidatedListenerArtifacts:
    deployment, flight_profile, prepared = _parse_listener_artifact_bytes(
        source._profile_bytes,
        source._session_bytes,
        source._actions_bytes,
    )
    if (
        deployment != source.deployment_profile
        or flight_profile != source.flight_profile
        or prepared != source.prepared_attempt
        or source._ledger_path != source._bound_ledger_path
        or source.ledger.path != source._bound_ledger_path
        or source.ledger.ledger_identity != source._ledger_identity
        or source.ledger.lock_identity != source._lock_identity
        or hashlib.sha256(source._profile_bytes).hexdigest() != source.profile_sha256
        or hashlib.sha256(source._session_bytes).hexdigest() != source.session_sha256
        or hashlib.sha256(source._actions_bytes).hexdigest() != source.actions_sha256
    ):
        raise ValueError("validated listener artifact relationships were altered")
    _require_current_listener_attempt(source)
    return source


def construct_after_listener_validation(
    files: ListenerStartupFiles | ValidatedListenerArtifacts,
    component_factory: Callable[[ValidatedListenerArtifacts], object],
) -> object:
    """Call a hardware/runtime factory only after all offline files validate."""
    if not callable(component_factory):
        raise TypeError("component_factory must be callable")
    owns_artifacts = isinstance(files, ListenerStartupFiles)
    artifacts = _validated_artifacts(files)
    try:
        return component_factory(artifacts)
    except BaseException:
        if owns_artifacts:
            try:
                artifacts.close()
            except BaseException:
                pass
        raise


def construct_after_full_validation(
    files: ListenerStartupFiles | ValidatedListenerArtifacts,
    runtime_config: object,
    *,
    component_factory: Callable[[ValidatedListenerArtifacts, object], object],
) -> object:
    """Validate every offline startup input before invoking a live factory."""
    if not callable(component_factory):
        raise TypeError("component_factory must be callable")
    owns_artifacts = isinstance(files, ListenerStartupFiles)
    artifacts = _validated_artifacts(files)
    from .listener_runtime import construct_after_runtime_validation

    try:
        return construct_after_runtime_validation(
            runtime_config,
            deployment_profile=artifacts.deployment_profile,
            flight_profile=artifacts.flight_profile,
            component_factory=lambda validated: component_factory(artifacts, validated),
        )
    except BaseException:
        if owns_artifacts:
            try:
                artifacts.close()
            except BaseException:
                pass
        raise


def _exact_id(value: object) -> int | None:
    if isinstance(value, int) and not isinstance(value, bool) and 1 <= value <= 255:
        return value
    return None


def _exact_command(value: object) -> int | None:
    if isinstance(value, int) and not isinstance(value, bool) and 0 <= value <= 0xFFFF:
        return value
    return None


def _finite_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


class QGCCommandListener:
    """Decode trusted QGC packets and reserve commands before acknowledging them."""

    def __init__(
        self,
        *,
        supervisor: MissionSupervisor,
        profile: object,
        command_queue: queue.Queue[CommandEnvelope],
        clock: Callable[[], float] = timebase.monotonic,
        ack_transport: object,
        wire_protocol: str,
        diagnostics: Callable[[str], None] | None = None,
    ) -> None:
        if not isinstance(supervisor, MissionSupervisor):
            raise TypeError("supervisor must be a MissionSupervisor")
        if not isinstance(profile, FlightProfile):
            raise TypeError("profile must be a validated FlightProfile")
        if not callable(clock):
            raise TypeError("clock must be callable")
        if not all(
            callable(getattr(command_queue, method, None))
            for method in ("put", "get", "task_done")
        ):
            raise TypeError("command_queue must provide put, get, and task_done")
        if not isinstance(command_queue, queue.Queue) or command_queue.maxsize != 0:
            raise ValueError("command_queue must be an unbounded queue.Queue")

        qgc_source = getattr(profile, "qgc_source", None)
        companion_target = getattr(profile, "companion_target", None)
        if not isinstance(qgc_source, SourceIdentity) or not isinstance(
            companion_target, SourceIdentity
        ):
            raise ValueError("profile must provide explicit QGC and companion identities")
        if wire_protocol not in ("1.0", "2.0"):
            raise ValueError("profile MAVLink wire protocol is unsupported")
        _require_runtime_modes(profile)
        if getattr(ack_transport, "source_identity", None) != companion_target:
            raise ValueError("ACK transport outbound source does not match companion target")
        if getattr(ack_transport, "wire_protocol", None) != wire_protocol:
            raise ValueError("ACK transport wire protocol does not match the profile")
        for method in ("install_message_callback", "send_command_ack"):
            if not callable(getattr(ack_transport, method, None)):
                raise TypeError(f"ACK transport must provide {method}")
        if diagnostics is not None and not callable(diagnostics):
            raise TypeError("diagnostics must be callable")

        self.supervisor = supervisor
        self.profile = profile
        self.command_queue = command_queue
        self._wire_protocol = wire_protocol
        self._clock = clock
        self._transport = ack_transport
        self._diagnostics = diagnostics or (lambda _message: None)
        self._admission_lock = threading.RLock()
        self._accepted: dict[int, CommandEnvelope] = {}
        self._installed = False

    def install(self) -> None:
        if self._installed:
            raise RuntimeError("QGC command listener is already installed")
        self._transport.install_message_callback(
            ("COMMAND_LONG", "COMMAND_INT"), self.handle_message
        )
        self._installed = True

    def handle_message(self, message: object) -> None:
        """Receive one packet. Bad or foreign input never escapes the callback."""
        try:
            self._handle_message(message)
        except Exception as error:
            self._diagnose(
                f"QGC command callback rejected packet: {_error_detail(error)}"
            )

    def _handle_message(self, message: object) -> None:
        identity = self._trusted_identity(message)
        if identity is None:
            return
        command = _exact_command(getattr(message, "command", None))
        if command is None:
            return
        message_type = self._message_type(message)
        if message_type == "COMMAND_INT":
            self._ack(command, "UNSUPPORTED")
            return
        if message_type != "COMMAND_LONG":
            return

        try:
            envelope = self._decode_long(message, command, identity)
        except Exception:
            self._ack(command, "DENIED")
            return

        # MissionSupervisor checks new-work prerequisites before duplicate
        # lookup. Serialize this short snapshot-only section so duplicates do
        # not rerun those checks.
        result = self._admit_or_status(envelope)
        self._ack(command, result)

    def _admit_or_status(self, envelope: CommandEnvelope) -> str:
        if envelope.command == ABORT_AND_RECOVER:
            prior_result = self.supervisor.admission_result(envelope.command)
            reserved = self.supervisor.reserved_envelope(envelope.command)
            if prior_result is not None and reserved is not None:
                if not self._same_request(reserved, envelope):
                    return "DENIED"
                return prior_result
        with self._admission_lock:
            prior_result = self.supervisor.admission_result(envelope.command)
            prior = self._accepted.get(envelope.command)
            if prior_result is not None:
                if prior is None or not self._same_request(prior, envelope):
                    return "DENIED"
                return prior_result
            try:
                enqueue = self.supervisor.admit(envelope)
            except CommandRejected as error:
                return error.result
            except AuthorityLost:
                return "DENIED"
            self._accepted[envelope.command] = envelope
            if enqueue is True:
                self.command_queue.put(envelope)
            result = self.supervisor.admission_result(envelope.command)
            return "DENIED" if result is None else result

    def _trusted_identity(
        self, message: object
    ) -> tuple[int, int, int, int] | None:
        try:
            source_system = _exact_id(message.get_srcSystem())  # type: ignore[attr-defined]
            source_component = _exact_id(message.get_srcComponent())  # type: ignore[attr-defined]
        except (AttributeError, TypeError, ValueError):
            return None
        target_system = _exact_id(getattr(message, "target_system", None))
        target_component = _exact_id(getattr(message, "target_component", None))
        source = self.profile.qgc_source
        target = self.profile.companion_target
        if (
            source_system != source.system_id
            or source_component != source.component_id
            or target_system != target.system_id
            or target_component != target.component_id
        ):
            return None
        return source_system, source_component, target_system, target_component

    @staticmethod
    def _message_type(message: object) -> str | None:
        try:
            value = message.get_type()  # type: ignore[attr-defined]
        except (AttributeError, TypeError, ValueError):
            return None
        return value if isinstance(value, str) else None

    def _decode_long(
        self,
        message: object,
        command: int,
        identity: tuple[int, int, int, int],
    ) -> CommandEnvelope:
        missing = object()
        raw_params = tuple(
            getattr(message, f"param{index}", missing) for index in range(1, 8)
        )
        if not all(_finite_number(value) for value in raw_params):
            raise ValueError("COMMAND_LONG requires seven finite parameters")
        params = tuple(float(value) for value in raw_params)
        token = params[0]
        if not token.is_integer():
            raise ValueError("attempt token must be an exact integer")
        if params[1:] != (0.0, 0.0, 0.0, 0.0, 0.0, 0.0):
            raise ValueError("command parameters do not match the prepared action")
        received_at = self._clock()
        if not _finite_number(received_at):
            raise ValueError("command receipt clock is invalid")
        return CommandEnvelope(
            source_system=identity[0],
            source_component=identity[1],
            target_system=identity[2],
            target_component=identity[3],
            command=command,
            attempt_id=int(token),
            params=params,
            received_at=float(received_at),
        )

    @staticmethod
    def _same_request(first: CommandEnvelope, second: CommandEnvelope) -> bool:
        return (
            first.source_system == second.source_system
            and first.source_component == second.source_component
            and first.target_system == second.target_system
            and first.target_component == second.target_component
            and first.command == second.command
            and first.attempt_id == second.attempt_id
            and first.params == second.params
        )

    def acknowledge_terminal(self, envelope: CommandEnvelope, result: str) -> None:
        accepted = self._accepted.get(envelope.command)
        if accepted is not None and self._same_request(accepted, envelope):
            self._ack(envelope.command, result)

    def request_abort(self, reason: str = "SIGINT") -> None:
        """Latch cooperative cancellation without starting recovery here."""
        self.supervisor.request_abort(reason)

    def _ack(self, command: int, result: str) -> None:
        result_code = _RESULT_CODES.get(result, ACK_DENIED)
        qgc = self.profile.qgc_source
        ack = CommandAck(
            command=command,
            result=result_code,
            progress=0,
            target_system=qgc.system_id if self._wire_protocol == "2.0" else None,
            target_component=qgc.component_id if self._wire_protocol == "2.0" else None,
        )
        try:
            self._transport.send_command_ack(ack)
        except Exception as error:
            self._diagnose(
                f"QGC ACK failed for command {command}: {_error_detail(error)}"
            )

    def _diagnose(self, message: str) -> None:
        try:
            self._diagnostics(message)
        except BaseException:
            pass


class _PhaseObserverFailure(Exception):
    def __init__(self, error: BaseException) -> None:
        self.error = error
        super().__init__(_error_detail(error))


class CommandExecutionOwner:
    """Run queued commands and recovery synchronously from one calling thread."""

    def __init__(
        self,
        *,
        supervisor: MissionSupervisor,
        command_queue: queue.Queue[CommandEnvelope],
        handlers: Mapping[int, Callable[[CommandEnvelope], bool]],
        terminal_ack: Callable[[CommandEnvelope, str], None],
        recovery: Callable[[], object],
        attempt_timeout_s: float,
        idle_poll_s: float,
        clock: Callable[[], float] = timebase.monotonic,
        phase_observer: Callable[[str, str], None] | None = None,
    ) -> None:
        if not isinstance(supervisor, MissionSupervisor):
            raise TypeError("supervisor must be a MissionSupervisor")
        if not callable(clock) or not callable(terminal_ack) or not callable(recovery):
            raise TypeError("clock, terminal_ack, and recovery must be callable")
        if not _finite_number(attempt_timeout_s) or attempt_timeout_s <= 0:
            raise ValueError("attempt timeout must be finite and positive")
        if not _finite_number(idle_poll_s) or idle_poll_s <= 0:
            raise ValueError("idle poll must be finite and positive")
        if any(
            not isinstance(command, int) or not callable(handler)
            for command, handler in handlers.items()
        ):
            raise TypeError("handlers must map integer commands to callables")
        if phase_observer is not None and not callable(phase_observer):
            raise TypeError("phase_observer must be callable or None")
        self.supervisor = supervisor
        self.command_queue = command_queue
        self._handlers = dict(handlers)
        self._terminal_ack = terminal_ack
        self._recovery = recovery
        self._attempt_timeout_s = float(attempt_timeout_s)
        self._idle_poll_s = float(idle_poll_s)
        self._clock = clock
        self._phase_observer = phase_observer
        self._mission_start: float | None = None
        self._deadline: float | None = None
        self._recovery_started = False

    def process_next(self, *, timeout_s: float | None = None) -> str | None:
        wait = self._idle_poll_s if timeout_s is None else timeout_s
        if self._deadline is not None:
            try:
                wait = min(wait, max(0.0, self._deadline - self._now()))
            except timebase.ClockError as error:
                self.supervisor.request_abort("listener clock unavailable")
                self._recover_preserving(error)
        try:
            envelope = self.command_queue.get(timeout=wait)
        except queue.Empty:
            return self._process_idle()
        result = "FAILED"
        began = False
        cancellation: BaseException | None = None
        start_observer_error: BaseException | None = None
        completion_observer_error: BaseException | None = None
        try:
            if not isinstance(envelope, CommandEnvelope):
                raise TypeError("command queue contained a non-envelope item")
            interruption = self._interruption_result(recover=False)
            if interruption is not None:
                result = interruption
                self._send_terminal_ack(envelope, result)
                self._recover_once()
                return result
            self.supervisor.begin(envelope.command)
            began = True
            if envelope.command == FM1 and self._deadline is None:
                self._mission_start = self._now()
                self._deadline = self._mission_start + self._attempt_timeout_s
            self._require_before_deadline()
            handler = self._handlers.get(envelope.command)
            if handler is None:
                raise RuntimeError(f"no handler for command {envelope.command}")
            if self._mission_start is None:
                self._observe_phase_start(envelope.command)
                handler_result = handler(envelope)
            else:
                with timebase._deadline(
                    self._mission_start, self._attempt_timeout_s
                ):
                    self._observe_phase_start(envelope.command)
                    handler_result = handler(envelope)
            if handler_result is not True:
                raise RuntimeError("command handler must return literal True")
            if (
                envelope.command == FM2
                and self.supervisor.enabled_phases[-1] == FM3
            ):
                self.await_physical_evidence()
            self._require_before_deadline()
            if (
                envelope.command == FM2
                and self.supervisor.enabled_phases[-1] == FM2
            ):
                try:
                    self._recover_once()
                except TimeoutError as error:
                    cancellation = error
                else:
                    if self.supervisor.recovery_outcome != "HOME_LANDED":
                        raise RuntimeError(
                            "final FM2 did not recover to original home"
                        )
            if cancellation is None:
                result = "SUCCEEDED"
        except MissionAbort:
            result = "ABORTED"
        except TimeoutError:
            self.supervisor.request_abort("attempt deadline expired")
            result = "ABORTED"
        except timebase.ClockError as error:
            cancellation = error
            result = "FAILED"
        except _PhaseObserverFailure as error:
            start_observer_error = error.error
            result = (
                "ABORTED" if self.supervisor.abort_reason is not None else "FAILED"
            )
        except Exception:
            result = (
                "ABORTED" if self.supervisor.abort_reason is not None else "FAILED"
            )
        finally:
            if began:
                try:
                    self.supervisor.finish(envelope.command, result)
                except CommandRejected:
                    pass
                finalized = self.supervisor.admission_result(envelope.command)
                if finalized in ("SUCCEEDED", "FAILED", "ABORTED"):
                    result = finalized
                if (
                    envelope.command in (FM1, FM2)
                    and result == "SUCCEEDED"
                    and self._phase_observer is not None
                ):
                    try:
                        self._phase_observer(
                            "FM1" if envelope.command == FM1 else "FM2",
                            "COMPLETE",
                        )
                    except BaseException as error:
                        completion_observer_error = error
                self._send_terminal_ack(envelope, result)
            self.command_queue.task_done()
        if start_observer_error is not None:
            self._recover_preserving(start_observer_error)
        if completion_observer_error is not None:
            self._recover_preserving(completion_observer_error)
        if cancellation is not None:
            self._recover_preserving(cancellation)
        if envelope.command in PHASE_COMMANDS and result != "SUCCEEDED":
            self._recover_once()
        return result

    def _process_idle(self) -> str | None:
        try:
            result = self._interruption_result(recover=False)
            if result is None and self._deadline is not None:
                self.supervisor.check_permission()
        except AuthorityLost as error:
            self.supervisor.request_abort(f"authority lost: {error.reason}")
            result = "ABORTED"
        except MissionAbort:
            result = "ABORTED"
        except (TimeoutError, timebase.ClockError) as error:
            reason = (
                "listener clock unavailable"
                if isinstance(error, timebase.ClockError)
                else "listener timing cancelled"
            )
            self.supervisor.request_abort(reason)
            self._recover_preserving(error)
        if result is None:
            return None
        self._recover_once()
        return result

    def run(self) -> str:
        while True:
            result = self.process_next()
            terminal = self.supervisor.terminal_result
            if terminal is not None:
                if terminal != "SUCCEEDED":
                    self._recover_once()
                return terminal
            if result is None:
                continue

    def _interruption_result(self, *, recover: bool = True) -> str | None:
        if self.supervisor.abort_reason is not None:
            if recover:
                self._recover_once()
            return "ABORTED"
        if self._deadline is not None and self._now() >= self._deadline:
            self.supervisor.request_abort("attempt deadline expired")
            if recover:
                self._recover_once()
            return "ABORTED"
        return None

    def _require_before_deadline(self) -> None:
        if self._deadline is not None and self._now() >= self._deadline:
            self.supervisor.request_abort("attempt deadline expired")
            raise MissionAbort("attempt deadline expired")

    def _now(self) -> float:
        value = self._clock()
        if not _finite_number(value):
            raise timebase.ClockError("listener clock returned a nonfinite value")
        return float(value)

    def _send_terminal_ack(self, envelope: CommandEnvelope, result: str) -> None:
        try:
            self._terminal_ack(envelope, result)
        except BaseException:
            pass

    def _observe_phase_start(self, command: int) -> None:
        if command in (FM1, FM2) and self._phase_observer is not None:
            self.observe_phase(
                "FM1" if command == FM1 else "FM2",
                "STARTED",
            )

    def observe_phase(self, phase: str, state: str) -> None:
        if self._phase_observer is None:
            return
        try:
            self._phase_observer(phase, state)
        except BaseException as error:
            raise _PhaseObserverFailure(error) from error

    def await_physical_evidence(self) -> None:
        """Hold one shared-time interval before certifying payload completion."""
        if self._mission_start is None or self._deadline is None:
            raise RuntimeError("physical evidence wait requires an active attempt")
        with timebase._deadline(self._mission_start, self._attempt_timeout_s):
            self._require_before_deadline()
            self.supervisor.check_permission()
            self._require_before_deadline()
            timebase.sleep(PHYSICAL_EVIDENCE_INTERVAL_SECONDS)
            self._require_before_deadline()
            self.supervisor.check_permission()
            self._require_before_deadline()

    def _recover_once(self) -> None:
        if self._recovery_started:
            return
        if self.supervisor.recovery_outcome is not None:
            self._recovery_started = True
            return
        self._recovery_started = True
        self._recovery()

    def _recover_preserving(self, cancellation: BaseException) -> None:
        try:
            self._recover_once()
        except BaseException as recovery_error:
            raise cancellation from recovery_error
        raise cancellation


@dataclass(frozen=True)
class LiveComponentFactories:
    """Explicit constructors for one live composition.

    Defaults are selected only by :func:`build_live_listener`; tests and a
    separately validated simulator composition inject labeled inert adapters.
    """

    backend: str
    controller_factory: Callable[..., object]
    ack_transport_factory: Callable[..., object]
    telemetry_collector_factory: Callable[..., object]
    lidar_factory: Callable[..., object]
    dropper_factory: Callable[..., object]
    camera_factory: Callable[..., object]
    supports_attachment: bool
    telemetry_startup_mode: str = "complete"

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "backend",
            _verified_text("component factory backend", self.backend),
        )
        for name in (
            "controller_factory",
            "ack_transport_factory",
            "telemetry_collector_factory",
            "lidar_factory",
            "dropper_factory",
            "camera_factory",
        ):
            if not callable(getattr(self, name)):
                raise TypeError(f"{name} must be callable")
        if not isinstance(self.supports_attachment, bool):
            raise TypeError("supports_attachment must be an explicit Boolean")
        if self.telemetry_startup_mode not in {"complete", "staged_simulation"}:
            raise ValueError("telemetry_startup_mode is invalid")


@dataclass(frozen=True)
class LiveCleanupReport:
    lidar_stopped: bool
    lidar_cleanup_completed: bool
    camera_stopped: bool
    recordings_completed: bool
    payload_closed: bool
    vehicle_closed: bool
    diagnostics: tuple[str, ...]


@dataclass(frozen=True)
class LiveRunResult:
    mission_result: str
    recovery_outcome: str | None
    monitoring_exit_reason: str
    cleanup_report: LiveCleanupReport | None = None


class _DeferredDropper:
    """Construct inert physical actuators only after FM1 authority acquisition."""

    def __init__(self, factory: Callable[..., object], config: object, permission, *, supports_attachment: bool):
        self._factory = factory
        self._config = config
        self._permission = permission
        self._delegate: object | None = None
        self.supports_attachment = supports_attachment

    def initialize(self) -> None:
        if self._delegate is not None:
            return
        candidate = self._factory(config=self._config, permission=self._permission)
        if getattr(candidate, "supports_attachment", None) is not self.supports_attachment:
            raise ValueError("payload capability does not match its composition declaration")
        self._delegate = candidate

    @property
    def servos(self):
        return () if self._delegate is None else getattr(self._delegate, "servos", ())

    def drop(self) -> None:
        if self._delegate is None:
            raise RuntimeError("payload hardware is not initialized")
        self._delegate.drop()

    def drop_with_guard(self, release_actuate, continuation_actuate) -> None:
        if self._delegate is None:
            raise RuntimeError("payload hardware is not initialized")
        guarded = getattr(self._delegate, "drop_with_guard", None)
        if callable(guarded):
            guarded(release_actuate, continuation_actuate)
            return
        release_actuate(self._delegate.drop)

    def attach(self, target_id: int) -> bool:
        if self._delegate is None:
            raise RuntimeError("payload hardware is not initialized")
        return self._delegate.attach(target_id)

    def cleanup_passive(self, timeout_s: float) -> bool:
        if self._delegate is None:
            return True
        cleanup = getattr(self._delegate, "cleanup_passive", None)
        if not callable(cleanup):
            return False
        outcome: list[bool] = []

        def invoke() -> None:
            try:
                outcome.append(cleanup() is True)
            except BaseException:
                outcome.append(False)

        worker = threading.Thread(target=invoke, daemon=True)
        worker.start()
        worker.join(timeout_s)
        return not worker.is_alive() and outcome == [True]


@dataclass(frozen=True)
class LiveListenerRuntime:
    listener: QGCCommandListener
    owner: CommandExecutionOwner
    supervisor: MissionSupervisor
    controller: object
    flight_state: object
    tracker: object
    lidar: object
    camera: object | None
    dropper: object
    cleanup_timeout_s: float
    diagnostics: Callable[[str], None]
    monitoring_stop: threading.Event
    manage_signals: bool = True
    terminal_monitoring: threading.Event = field(default_factory=threading.Event)
    _abort_delivered: threading.Event = field(default_factory=threading.Event)
    _abort_lock: threading.RLock = field(default_factory=threading.RLock)
    _artifacts: ValidatedListenerArtifacts | None = field(default=None, repr=False)
    _telemetry_collector: object | None = field(default=None, repr=False)
    _telemetry_verification: _TelemetryStartupVerification | None = field(
        default=None, repr=False
    )

    def run(self) -> LiveRunResult:
        def abort_on_sigint(_signum: int, _frame: object) -> None:
            self.request_abort("SIGINT")

        def execute() -> LiveRunResult:
            mission_result = self.owner.run()
            self.terminal_monitoring.set()
            recovery_outcome = self.supervisor.recovery_outcome
            monitoring_exit_reason = self._monitor_if_required(recovery_outcome)
            return LiveRunResult(
                mission_result=mission_result,
                recovery_outcome=recovery_outcome,
                monitoring_exit_reason=monitoring_exit_reason,
            )

        if not self.manage_signals:
            return execute()
        previous = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, abort_on_sigint)
        try:
            return execute()
        finally:
            signal.signal(signal.SIGINT, previous)

    def request_abort(self, reason: str = "SIGINT") -> None:
        if self.terminal_monitoring.is_set():
            self.stop_monitoring()
            return
        with self._abort_lock:
            if self._abort_delivered.is_set():
                return
            self._abort_delivered.set()
        self.listener.request_abort(reason)

    def stop_monitoring(self) -> None:
        """Stop an already command-silent monitoring phase without recovery."""
        self.monitoring_stop.set()

    @staticmethod
    def _field_sequence(field) -> int:
        return -1 if field is None else field.observation.sequence

    @staticmethod
    def _fresh_ground_disarmed(snapshot, *, after_landed: int, after_armed: int) -> bool:
        landed = getattr(snapshot, "landed_state", None)
        armed = getattr(snapshot, "armed", None)
        return bool(
            landed is not None
            and landed.fresh is True
            and type(landed.observation.value) is int
            and landed.observation.value == 1
            and landed.observation.sequence > after_landed
            and armed is not None
            and armed.fresh is True
            and armed.observation.value is False
            and armed.observation.sequence > after_armed
        )

    def _monitor_if_required(self, recovery_outcome: str | None) -> str:
        try:
            terminal = self.flight_state.snapshot()
        except (TimeoutError, timebase.ClockError):
            return "CLOCK_FAILURE_UNCONFIRMED"
        landed_sequence = self._field_sequence(terminal.landed_state)
        armed_sequence = self._field_sequence(terminal.armed)
        ground_now = bool(
            terminal.landed_state is not None
            and terminal.landed_state.fresh is True
            and type(terminal.landed_state.observation.value) is int
            and terminal.landed_state.observation.value == 1
            and terminal.armed is not None
            and terminal.armed.fresh is True
            and terminal.armed.observation.value is False
        )
        if recovery_outcome not in {"PILOT", "UNCONFIRMED"} and ground_now:
            return "NOT_REQUIRED"
        while True:
            if self.monitoring_stop.wait(self.owner._idle_poll_s):
                return "EXPLICIT_STOP_UNCONFIRMED"
            try:
                snapshot = self.flight_state.snapshot()
            except (TimeoutError, timebase.ClockError):
                return "CLOCK_FAILURE_UNCONFIRMED"
            if self._fresh_ground_disarmed(
                snapshot,
                after_landed=landed_sequence,
                after_armed=armed_sequence,
            ):
                return "OBSERVED_GROUND_DISARMED"

    def close(self) -> LiveCleanupReport:
        notes: list[str] = []

        def note(label: str, error: BaseException) -> None:
            notes.append(f"{label}: {_error_detail(error)}")

        try:
            telemetry_collector = self._telemetry_collector
            if telemetry_collector is not None:
                close_telemetry = getattr(self._telemetry_collector, "close", None)
                if callable(close_telemetry):
                    close_telemetry()
        except BaseException as error:
            note("startup telemetry cleanup failed", error)
        lidar_status = None
        try:
            lidar = self.lidar
            stop_lidar = getattr(lidar, "stop", None)
            if callable(stop_lidar):
                lidar_status = stop_lidar(timeout_seconds=self.cleanup_timeout_s)
        except BaseException as error:
            note("LiDAR cleanup failed", error)

        camera_stopped = False
        recordings_completed = False
        try:
            camera = self.camera
            if camera is None:
                camera_stopped = True
            else:
                manager = getattr(camera, "cm", camera)
                stop = getattr(manager, "stop_acquisition", None)
                camera_stopped = bool(
                    callable(stop) and stop(timeout_s=self.cleanup_timeout_s) is True
                )
        except BaseException as error:
            note("camera cleanup failed", error)
        try:
            camera = self.camera
            if camera is None:
                recordings_completed = True
            else:
                wait = getattr(camera, "wait_for_recordings", None)
                recordings_completed = bool(
                    callable(wait) and wait(self.cleanup_timeout_s) is True
                )
        except BaseException as error:
            note("camera recording cleanup failed", error)
        try:
            camera = self.camera
            if camera is not None:
                errors = getattr(camera, "recording_errors", None)
                if isinstance(errors, Mapping):
                    notes.extend(
                        f"camera recording {name}: {reason}"
                        for name, reason in errors.items()
                    )
        except BaseException as error:
            note("camera recording diagnostics failed", error)

        payload_closed = False
        try:
            dropper = self.dropper
            cleanup_payload = getattr(dropper, "cleanup_passive", None)
            if callable(cleanup_payload):
                payload_closed = cleanup_payload(self.cleanup_timeout_s)
        except BaseException as error:
            note("payload cleanup failed", error)
        if not payload_closed:
            if not any(note.startswith("payload cleanup failed:") for note in notes):
                notes.append("payload cleanup incomplete or unconfirmed")

        try:
            controller = self.controller
            vehicle = getattr(controller, "vehicle", None)
            vehicle_closed = _bounded_close(
                vehicle,
                self.cleanup_timeout_s,
                notes,
            )
        except BaseException as error:
            note("vehicle cleanup failed", error)
            vehicle_closed = False
        try:
            artifacts = self._artifacts
            if artifacts is not None:
                close_artifacts = getattr(artifacts, "close", None)
                if callable(close_artifacts):
                    close_artifacts()
        except BaseException as error:
            note("attempt-state descriptor cleanup failed", error)
        try:
            lidar_stopped = bool(
                lidar_status is not None
                and getattr(lidar_status, "worker_stopped", False) is True
            )
            lidar_cleanup_completed = bool(
                lidar_status is not None
                and getattr(lidar_status, "cleanup_completed", False) is True
            )
        except BaseException as error:
            note("LiDAR cleanup status failed", error)
            lidar_stopped = False
            lidar_cleanup_completed = False
        for cleanup_note in notes:
            try:
                diagnostics = getattr(self, "diagnostics")
                diagnostics(cleanup_note)
            except BaseException:
                pass
        return LiveCleanupReport(
            lidar_stopped=lidar_stopped,
            lidar_cleanup_completed=lidar_cleanup_completed,
            camera_stopped=camera_stopped,
            recordings_completed=recordings_completed,
            payload_closed=payload_closed,
            vehicle_closed=vehicle_closed,
            diagnostics=tuple(notes),
        )


def _error_detail(error: BaseException) -> str:
    try:
        return str(error) or type(error).__name__
    except BaseException:
        return type(error).__name__


def _bounded_close(resource: object, timeout_s: float, notes: list[str]) -> bool:
    try:
        close = getattr(resource, "close", None)
    except BaseException as error:
        notes.append(f"vehicle close worker setup failed: {_error_detail(error)}")
        return False
    if not callable(close):
        return True
    errors: list[BaseException] = []

    def invoke() -> None:
        try:
            close()
        except BaseException as error:
            errors.append(error)

    try:
        worker = threading.Thread(target=invoke, daemon=True)
        worker.start()
        worker.join(timeout_s)
        alive = worker.is_alive()
    except BaseException as error:
        notes.append(f"vehicle close worker failed: {_error_detail(error)}")
        return False
    if alive:
        notes.append("vehicle close did not finish within the configured bound")
        return False
    if errors:
        notes.append(f"vehicle close failed: {_error_detail(errors[0])}")
        return False
    return True


def _cleanup_failed_startup(
    *,
    controller: object,
    lidar: object | None,
    camera: object | None,
    timeout_s: float,
    telemetry_collector: object | None = None,
) -> None:
    try:
        close_telemetry = getattr(telemetry_collector, "close", None)
        if callable(close_telemetry):
            close_telemetry()
    except BaseException:
        pass
    if camera is not None:
        try:
            stop = getattr(getattr(camera, "cm", camera), "stop_acquisition", None)
            if callable(stop):
                stop(timeout_s=timeout_s)
        except BaseException:
            pass
    if lidar is not None:
        try:
            stop = getattr(lidar, "stop", None)
            if callable(stop):
                stop(timeout_seconds=timeout_s)
        except BaseException:
            pass
    try:
        vehicle = getattr(controller, "vehicle", None)
        _bounded_close(vehicle, timeout_s, [])
    except BaseException:
        pass


def _default_live_factories() -> LiveComponentFactories:
    from .drone_control import DroneControl
    from ..sensors.camera._camera_manager import CameraManager
    from ..sensors.camera.camera import Camera
    from ..sensors.lidar.lidar import Lidar
    from ..sensors.servo.servo import Dropper

    def controller_factory(*, config, flight_state, permission_guard, decoders):
        return DroneControl(
            config.connection.endpoint,
            source_identity=config.connection.source_identity,
            flight_controller_target=config.connection.target_identity,
            wire_protocol=config.connection.wire_protocol,
            wait_ready=config.connection.wait_ready,
            heartbeat_timeout=config.connection.heartbeat_timeout_s,
            flight_state=flight_state,
            permission_guard=permission_guard,
            heartbeat_mode_decoder=decoders.heartbeat_mode_decoder,
            rc_health_decoder=decoders.rc_health_decoder,
            sys_status_observer=decoders.observe_sys_status,
            failsafe_decoders={
                "HEARTBEAT": decoders.heartbeat_failsafe_decoder,
            },
            mission_home_check=config.operating_site.mission_home_check,
            fc_home_position_tolerance_m=config.fc_home_position_tolerance_m,
            fc_home_altitude_tolerance_m=config.fc_home_altitude_tolerance_m,
            clearance_calibration=config.clearance_calibration,
            release_stability_config=config.release_stability,
            home_request_timeout_s=config.telemetry.home_request_timeout_s,
            telemetry_poll_interval_s=config.telemetry.poll_interval_s,
        )

    def ack_transport_factory(*, controller, config):
        return DroneKitQGCAckTransport(
            controller.vehicle,
            expected_source=config.connection.source_identity,
            wire_protocol=config.connection.wire_protocol,
        )

    def telemetry_collector_factory(*, controller, profile, config):
        return TelemetryStartupCollector(
            controller=controller,
            flight_profile=profile,
            policy=config.telemetry,
            autopilot_version=config.autopilot_version,
        )

    def lidar_factory(*, config):
        values = config.components.range_sensor
        return Lidar(
            raw_min_cm=values.raw_min_cm,
            raw_max_cm=values.raw_max_cm,
            mounting_offset_cm=values.mounting_offset_cm,
            stale_after_seconds=values.stale_after_seconds,
            startup_timeout_seconds=values.startup_timeout_seconds,
            poll_interval_seconds=values.poll_interval_seconds,
            clock=timebase.monotonic,
        )

    def dropper_factory(*, config, permission):
        values = config.components.payload
        return Dropper(
            pins=values.pins,
            permission=permission,
            release_hold_seconds=values.release_hold_seconds,
            permission_check_interval_seconds=values.permission_check_interval_seconds,
            min_pulse_width=values.min_pulse_width_seconds,
            max_pulse_width=values.max_pulse_width_seconds,
        )

    def camera_factory(*, config, **_unused):
        values = config.vision
        if values is None:
            raise ValueError("camera construction requires vision configuration")
        manager = CameraManager(
            calibration_path=values.calibration_path,
            clock=values.receipt_clock_ns,
            max_exposure_age_ns=values.max_exposure_age_ns,
        )
        return Camera(
            values.marker_size_mm,
            manager=manager,
            mounting_path=values.mounting_path,
        )

    return LiveComponentFactories(
        backend="physical-hardware-v1",
        controller_factory=controller_factory,
        ack_transport_factory=ack_transport_factory,
        telemetry_collector_factory=telemetry_collector_factory,
        lidar_factory=lidar_factory,
        dropper_factory=dropper_factory,
        camera_factory=camera_factory,
        supports_attachment=Dropper.supports_attachment,
    )


_WAYPOINT_BY_UPDATE = {
    UPDATE_WA: "WA",
    UPDATE_WM1: "WM1",
    UPDATE_WM2: "WM2",
    UPDATE_WM3: "WM3",
    UPDATE_WM4: "WM4",
    UPDATE_WM5: "WM5",
    UPDATE_WM6: "WM6",
    UPDATE_L: "L",
    UPDATE_TARGET: "TARGET",
}
_PICKUP_WAYPOINTS = ("WA", "WM1", "WM2", "WM3", "WM4", "WM5", "WM6")
_ALL_WAYPOINTS = ("L", "TARGET", *_PICKUP_WAYPOINTS)


def _require_fresh_ground_snapshot(flight_state: object):
    from .flight_state import FailsafeEvidence, RCInput

    snapshot = flight_state.snapshot()
    required = (
        "heartbeat",
        "mode",
        "location",
        "velocity",
        "attitude",
        "landed_state",
        "armed",
        "home",
        "rc_input",
        "failsafe",
    )
    if any(
        getattr(snapshot, name) is None or getattr(snapshot, name).fresh is not True
        for name in required
    ):
        raise CommandRejected("safe-ground telemetry is absent or stale")
    if snapshot.landed_state.observation.value != 1:
        raise CommandRejected("safe-ground admission requires exact ON_GROUND")
    if snapshot.armed.observation.value is not False:
        raise CommandRejected("safe-ground admission requires disarmed state")
    rc_input = snapshot.rc_input.observation.value
    if not isinstance(rc_input, RCInput) or rc_input.healthy is not True:
        raise CommandRejected("safe-ground admission requires healthy RC evidence")
    failsafe = snapshot.failsafe.observation.value
    failsafe_clear = (
        isinstance(failsafe, FailsafeEvidence)
        and failsafe.active is False
        and isinstance(failsafe.reason, str)
        and bool(failsafe.reason)
    ) or (
        isinstance(failsafe, tuple)
        and len(failsafe) == 2
        and failsafe[0] is False
        and isinstance(failsafe[1], str)
        and bool(failsafe[1])
    )
    if not failsafe_clear:
        raise CommandRejected("safe-ground admission requires clear failsafe evidence")
    return snapshot


def _require_fresh_vital_permission(flight_state: object) -> None:
    from .flight_state import FailsafeEvidence, RCInput

    snapshot = flight_state.snapshot()
    invalid = []
    for name in ("heartbeat", "mode", "rc_input", "failsafe"):
        field = getattr(snapshot, name)
        if field is None or field.fresh is not True:
            invalid.append(name)
    rc_input = None if snapshot.rc_input is None else snapshot.rc_input.observation.value
    if not isinstance(rc_input, RCInput) or rc_input.healthy is not True:
        invalid.append("rc_input")
    failsafe = None if snapshot.failsafe is None else snapshot.failsafe.observation.value
    failsafe_clear = (
        isinstance(failsafe, FailsafeEvidence) and failsafe.active is False
    ) or (
        isinstance(failsafe, tuple)
        and len(failsafe) == 2
        and failsafe[0] is False
    )
    if not failsafe_clear:
        invalid.append("failsafe")
    if invalid:
        source = flight_state.source
        flight_state.invalidate_observations(
            tuple(dict.fromkeys(invalid)),
            source_system=source.system_id,
            source_component=source.component_id,
        )
        raise AuthorityLost("vital flight evidence is absent, stale, or unsafe")


def _require_cached_waypoint_ages(snapshot: object) -> None:
    from .mission_info import MAX_WAYPOINT_AGE_SECONDS

    now = timebase.epoch()
    if not _finite_number(now):
        raise timebase.ClockError("waypoint age clock returned a nonfinite value")
    for name in snapshot.names:
        record = snapshot.get_record(name)
        if record is None or not _finite_number(record.loaded_at):
            raise CommandRejected(f"waypoint {name!r} has invalid loaded_at")
        age = float(now) - float(record.loaded_at)
        if not math.isfinite(age) or age < 0 or age > MAX_WAYPOINT_AGE_SECONDS:
            raise CommandRejected(f"waypoint {name!r} is outside the allowed age")


def _mission_home_from_snapshot(snapshot: object):
    from ..common_types import MissionHome

    value = snapshot.location.observation.value
    if not isinstance(value, tuple) or len(value) != 4:
        raise CommandRejected("launch location is invalid")
    lat, lon, amsl_mm, _relative_mm = value
    if not all(_finite_number(item) for item in value):
        raise CommandRejected("launch location is invalid")
    home = MissionHome(float(lat) / 1e7, float(lon) / 1e7, float(amsl_mm) / 1000)
    if not -90 <= home.lat <= 90 or not -180 <= home.lon <= 180:
        raise CommandRejected("launch location is invalid")
    return home


def build_live_listener(
    artifacts: ValidatedListenerArtifacts,
    config: object,
    **kwargs: object,
) -> LiveListenerRuntime:
    """Validate the pinned attempt binding before constructing live components."""
    validated = _validated_artifacts(artifacts)
    try:
        return _build_live_listener_validated(validated, config, **kwargs)
    except BaseException:
        validated.close()
        raise


def _build_live_listener_validated(
    artifacts: ValidatedListenerArtifacts,
    config: object,
    *,
    factories: LiveComponentFactories | None = None,
    diagnostics: Callable[[str], None] | None = None,
    monitoring_stop: threading.Event | None = None,
    startup_admission_check: Callable[[], None] | None = None,
    phase_observer: Callable[[str, str], None] | None = None,
    manage_signals: bool = True,
) -> LiveListenerRuntime:
    """Construct one guarded listener after all offline validation has passed."""
    from .flight_state import FlightState
    from .listener_runtime import (
        HardwareComponentConfig,
        InjectedComponentConfig,
        RuntimeConfiguration,
        construct_after_runtime_validation,
    )
    from .mission_info import MissonTracker

    if not isinstance(config, RuntimeConfiguration):
        raise TypeError("config must be a RuntimeConfiguration")
    construct_after_runtime_validation(
        config,
        deployment_profile=artifacts.deployment_profile,
        flight_profile=artifacts.flight_profile,
        component_factory=lambda validated: validated,
    )
    if startup_admission_check is not None and not callable(startup_admission_check):
        raise TypeError("startup_admission_check must be callable or None")
    if phase_observer is not None and not callable(phase_observer):
        raise TypeError("phase_observer must be callable or None")
    if not isinstance(manage_signals, bool):
        raise TypeError("manage_signals must be a Boolean")
    if factories is None:
        if not isinstance(config.components, HardwareComponentConfig):
            raise ValueError("default factories require hardware component configuration")
        selected_factories = _default_live_factories()
    else:
        if not isinstance(factories, LiveComponentFactories):
            raise TypeError("factories must be LiveComponentFactories")
        if not isinstance(config.components, InjectedComponentConfig):
            raise ValueError("explicit factories require injected component configuration")
        if factories.backend != config.components.backend:
            raise ValueError("component factory backend does not match configuration backend")
        selected_factories = factories
    if not isinstance(selected_factories, LiveComponentFactories):
        raise TypeError("factories must be LiveComponentFactories")
    if FM3 in config.enabled_phases and selected_factories.supports_attachment is not True:
        raise ValueError("enabled FM3 requires a validated attachment capability")
    if selected_factories.telemetry_startup_mode == "staged_simulation" and (
        not isinstance(config.components, InjectedComponentConfig)
        or selected_factories.backend != "drone-sim-ros-confirmed-v1"
    ):
        raise ValueError(
            "staged telemetry requires the explicit simulator composition"
        )
    report = diagnostics or (lambda _message: None)
    if not callable(report):
        raise TypeError("diagnostics must be callable")
    if monitoring_stop is not None and not isinstance(monitoring_stop, threading.Event):
        raise TypeError("monitoring_stop must be a threading.Event or None")

    tracker = MissonTracker(
        mission_time_seconds=config.attempt_timeout_s,
        waypoint_path=config.waypoint_path,
    )
    cached_snapshot = tracker.snapshot_for_attempt(
        (), config.operating_site.waypoint_check, optional_names=_ALL_WAYPOINTS
    )
    snapshot_lock = threading.RLock()
    attempt_snapshot = [None]
    pending_home = [None]

    profile = artifacts.flight_profile
    flight_state = FlightState(
        source_system=profile.flight_controller.system_id,
        source_component=profile.flight_controller.component_id,
        freshness_bounds=profile.freshness_bounds,
        rc_channel=profile.rc_channel,
        rc_mode_mapping=profile.rc_mode_bands,
        clock=timebase.monotonic,
    )
    decoders = profile.decoders(clock=timebase.monotonic, flight_state=flight_state)
    controller_ref: list[object | None] = [None]
    dropper_ref: list[_DeferredDropper | None] = [None]
    lidar_ref: list[object | None] = [None]
    camera_ref: list[object | None] = [None]

    def permission_check() -> None:
        decoders.check_authority_dependency()
        _require_fresh_vital_permission(flight_state)

    def admission_check(envelope: CommandEnvelope) -> None:
        nonlocal cached_snapshot
        if startup_admission_check is not None:
            startup_result = startup_admission_check()
            if startup_result is not None:
                raise CommandRejected("startup_admission_check must return None")
        config.require_command_enabled(envelope.command)
        if envelope.command == ABORT_AND_RECOVER:
            return
        if envelope.command in MUTATION_COMMANDS or envelope.command == FM1:
            ground = _require_fresh_ground_snapshot(flight_state)
            decoders.check_rc_health()
            if envelope.command == FM1:
                with snapshot_lock:
                    required = {"L", "TARGET"}
                    if FM3 in config.enabled_phases:
                        required.update(("WA", "WM1"))
                    missing = required - cached_snapshot.names
                    if missing:
                        raise CommandRejected(
                            f"required waypoint {sorted(missing)[0]!r} is missing"
                        )
                    _require_cached_waypoint_ages(cached_snapshot)
                    attempt_snapshot[0] = cached_snapshot
                home = _mission_home_from_snapshot(ground)
                if config.operating_site.mission_home_check(home) is not None:
                    raise CommandRejected("mission-home check violated its success contract")
                pending_home[0] = home
            return
        permission_check()
        if envelope.command in (FM2, FM3):
            lidar = lidar_ref[0]
            if lidar is None or not callable(getattr(lidar, "get_sample", None)):
                raise CommandRejected("current LiDAR evidence is unavailable")
            try:
                lidar.get_sample()
            except Exception as error:
                raise CommandRejected(f"current LiDAR evidence is unavailable: {error}") from error
        if envelope.command == FM3:
            camera = camera_ref[0]
            readiness = getattr(camera, "precision_readiness", None)
            if not callable(readiness):
                raise CommandRejected("camera precision readiness is unavailable")
            try:
                status = readiness()
            except Exception as error:
                raise CommandRejected(f"camera precision readiness is unavailable: {error}") from error
            if getattr(status, "ready", None) is not True:
                raise CommandRejected("camera precision readiness is not satisfied")

    def consume_and_acquire(attempt_id: int) -> None:
        artifacts.ledger.consume(attempt_id)
        home = pending_home[0]
        controller = controller_ref[0]
        if home is None or controller is None:
            raise RuntimeError("FM1 launch snapshot is unavailable")
        if flight_state.acquire_initial_companion_authority() is not True:
            raise AuthorityLost("initial companion authority was not acquired")
        controller.set_mission_home(home)
        dropper = dropper_ref[0]
        if dropper is None:
            raise RuntimeError("payload composition is unavailable")
        dropper.initialize()

    supervisor = MissionSupervisor(
        artifacts.prepared_attempt.attempt_id,
        admission_check=admission_check,
        attempt_consumer=consume_and_acquire,
        permission_check=permission_check,
        recovery_policy=config.recovery_policy,
        enabled_phases=tuple(
            command for command in PHASE_COMMANDS if command in config.enabled_phases
        ),
    )
    controller = selected_factories.controller_factory(
        config=config,
        flight_state=flight_state,
        permission_guard=supervisor.check_permission,
        decoders=decoders,
    )
    controller_ref[0] = controller
    install_output_transactions = getattr(controller, "install_output_transactions", None)
    if not callable(install_output_transactions):
        raise TypeError("controller does not expose guarded output transactions")
    install_output_transactions(
        dependency_transaction=decoders.output_transaction,
        supervisor_transaction=supervisor.output_transaction,
    )
    telemetry_collector = None
    telemetry_verification = None
    try:
        ack_transport = selected_factories.ack_transport_factory(
            controller=controller, config=config
        )
        telemetry_collector = selected_factories.telemetry_collector_factory(
            controller=controller, profile=profile, config=config
        )
        telemetry_verification = _TelemetryStartupVerification(telemetry_collector)
        install_telemetry_verifier = getattr(
            controller, "install_startup_telemetry_verifier", None
        )
        if not callable(install_telemetry_verifier):
            raise TypeError("controller does not expose startup telemetry verification")
        if selected_factories.telemetry_startup_mode == "complete":
            evidence = telemetry_collector.configure_and_collect()
            telemetry_verification.accept(evidence)
            install_telemetry_verifier(None, verified=True)
        else:
            telemetry_collector.prepare()

            def verify_staged_after_guided() -> None:
                telemetry_verification.verify_after_guided()
                if FM3 not in config.enabled_phases:
                    return
                camera = camera_ref[0]
                prepare = getattr(camera, "prepare_precision_readiness", None)
                if not callable(prepare):
                    raise RuntimeError("camera readiness preparation is unavailable")
                prepared = prepare(timeout_s=config.precision_policy.frame_timeout_s)
                if getattr(prepared, "ready", None) is not True:
                    raise RuntimeError("camera readiness preparation is not satisfied")

            install_telemetry_verifier(
                verify_staged_after_guided, verified=False
            )
        lidar = selected_factories.lidar_factory(config=config)
        lidar_ref[0] = lidar
    except BaseException:
        _cleanup_failed_startup(
            controller=controller,
            lidar=locals().get("lidar"),
            camera=None,
            timeout_s=config.cleanup_timeout_s,
            telemetry_collector=telemetry_collector,
        )
        raise

    def payload_permission() -> bool:
        supervisor.check_permission()
        return True

    def payload_actuation(output) -> None:
        controller = controller_ref[0]
        if controller is None:
            raise RuntimeError("payload controller is unavailable")
        controller._actuate_guarded(output)

    payload_permission.actuate = payload_actuation

    dropper = _DeferredDropper(
        selected_factories.dropper_factory,
        config,
        payload_permission,
        supports_attachment=selected_factories.supports_attachment,
    )
    dropper_ref[0] = dropper
    try:
        camera = (
            selected_factories.camera_factory(
                config=config, lidar=lidar, controller=controller
            )
            if FM3 in config.enabled_phases
            else None
        )
        camera_ref[0] = camera
        if (
            camera is not None
            and selected_factories.telemetry_startup_mode == "complete"
        ):
            prepare = getattr(camera, "prepare_precision_readiness", None)
            if not callable(prepare):
                raise RuntimeError("camera readiness preparation is unavailable")
            prepared = prepare(timeout_s=config.precision_policy.frame_timeout_s)
            if getattr(prepared, "ready", None) is not True:
                raise RuntimeError("camera readiness preparation is not satisfied")
        command_queue: queue.Queue[CommandEnvelope] = queue.Queue()
        listener = QGCCommandListener(
            supervisor=supervisor,
            profile=profile,
            command_queue=command_queue,
            ack_transport=ack_transport,
            wire_protocol=artifacts.wire_protocol,
            diagnostics=report,
        )
    except BaseException:
        _cleanup_failed_startup(
            controller=controller,
            lidar=lidar,
            camera=locals().get("camera"),
            timeout_s=config.cleanup_timeout_s,
            telemetry_collector=telemetry_collector,
        )
        raise

    def refresh_snapshot() -> None:
        nonlocal cached_snapshot
        candidate = tracker.snapshot_for_attempt(
            (), config.operating_site.waypoint_check, optional_names=_ALL_WAYPOINTS
        )
        with snapshot_lock:
            cached_snapshot = candidate

    def mutation_handler(envelope: CommandEnvelope) -> bool:
        ground = _require_fresh_ground_snapshot(flight_state)
        decoders.check_rc_health()
        if envelope.command in _WAYPOINT_BY_UPDATE:
            from ..common_types import GPSCoord

            raw = ground.location.observation.value
            tracker.set_waypoint(
                _WAYPOINT_BY_UPDATE[envelope.command],
                GPSCoord(raw[0] / 1e7, raw[1] / 1e7, raw[3] / 1000),
            )
        elif envelope.command == CLEAR_PICKUPS:
            for name in _PICKUP_WAYPOINTS:
                tracker.clear_waypoint(name)
        elif envelope.command == CLEAR_ALL:
            for name in _ALL_WAYPOINTS:
                tracker.clear_waypoint(name)
        else:
            raise RuntimeError("unsupported mutation handler")
        refresh_snapshot()
        return True

    def frozen_waypoints():
        snapshot = attempt_snapshot[0]
        if snapshot is None:
            raise RuntimeError("attempt waypoints were not frozen")
        return snapshot

    def fm1_handler(_envelope: CommandEnvelope) -> bool:
        from ..missions.fm1 import fm1

        tracker.begin_mission()
        return fm1(
            tracker,
            controller,
            config.cruise_altitude_m,
            frozen_waypoints().get_waypoint("L"),
        ) is True

    def recover() -> str:
        home = controller.mission_home
        if home is None:
            raise RuntimeError("recovery has no pinned mission home")
        return supervisor.recover(controller, home, config.cruise_altitude_m)

    def fm2_handler(_envelope: CommandEnvelope) -> bool:
        from ..missions.fm2 import fm2

        return fm2(
            tracker,
            controller,
            config.cruise_altitude_m,
            frozen_waypoints().get_waypoint("TARGET"),
            dropper,
            lidar,
            desired_drop_height_m=config.desired_drop_height_m,
        ) is True

    def fm3_handler(_envelope: CommandEnvelope) -> bool:
        from ..common_types import GPSCoord, MissionHome
        from ..mock_mission import fm3

        snapshot = frozen_waypoints()
        delivery = snapshot.get_waypoint("TARGET")
        pickup_sites = (
            (snapshot.get_waypoint("WA"), 3, "FM3_3"),
            (snapshot.get_waypoint("WM1"), 4, "FM3_4"),
        )
        for pickup, marker_id, phase in pickup_sites:
            owner.observe_phase(phase, "STARTED")
            if fm3(
                tracker,
                controller,
                camera,
                lidar,
                dropper,
                {marker_id},
                pickup,
                delivery,
                precision_policy=config.precision_policy,
            ) is not True:
                return False
            owner.await_physical_evidence()
            owner.observe_phase(phase, "COMPLETE")
        home = controller.mission_home
        if not isinstance(home, MissionHome):
            raise RuntimeError("terminal landing has no pinned mission home")
        owner.observe_phase("HOME", "STARTED")
        controller.check_permission()
        if controller.goto_waypoint(
            GPSCoord(home.lat, home.lon, config.cruise_altitude_m)
        ) != 0:
            return False
        controller.check_permission()
        if controller.simple_land() != 0:
            return False
        controller.check_permission()
        if controller.disarm() != 0:
            return False
        owner.observe_phase("HOME", "DISARMED")
        if supervisor.confirm_original_home_landing(controller, home) != "HOME_LANDED":
            return False
        owner.observe_phase("HOME", "COMPLETE")
        return True

    handlers = {command: mutation_handler for command in MUTATION_COMMANDS}
    handlers.update({FM1: fm1_handler, FM2: fm2_handler, FM3: fm3_handler})

    owner = CommandExecutionOwner(
        supervisor=supervisor,
        command_queue=command_queue,
        handlers=handlers,
        terminal_ack=listener.acknowledge_terminal,
        recovery=recover,
        attempt_timeout_s=config.attempt_timeout_s,
        idle_poll_s=config.idle_poll_s,
        phase_observer=phase_observer,
    )
    try:
        listener.install()
    except BaseException:
        _cleanup_failed_startup(
            controller=controller,
            lidar=lidar,
            camera=camera,
            timeout_s=config.cleanup_timeout_s,
            telemetry_collector=telemetry_collector,
        )
        raise
    return LiveListenerRuntime(
        listener=listener,
        owner=owner,
        supervisor=supervisor,
        controller=controller,
        flight_state=flight_state,
        tracker=tracker,
        lidar=lidar,
        camera=camera,
        dropper=dropper,
        cleanup_timeout_s=config.cleanup_timeout_s,
        diagnostics=report,
        monitoring_stop=monitoring_stop or threading.Event(),
        manage_signals=manage_signals,
        _artifacts=artifacts,
        _telemetry_collector=telemetry_collector,
        _telemetry_verification=telemetry_verification,
    )


def start_repl(
    files: ListenerStartupFiles | ValidatedListenerArtifacts,
    runtime_config: object,
    *,
    factories: LiveComponentFactories | None = None,
    diagnostics: Callable[[str], None] | None = None,
    monitoring_stop: threading.Event | None = None,
    startup_admission_check: Callable[[], None] | None = None,
    on_listener_ready: Callable[[LiveListenerRuntime], None] | None = None,
    manage_signals: bool = True,
    phase_observer: Callable[[str, str], None] | None = None,
) -> LiveRunResult:
    """Validate, construct, and synchronously own one QGC attempt."""

    def build_validated_runtime(artifacts, config):
        if on_listener_ready is not None and not callable(on_listener_ready):
            raise TypeError("on_listener_ready must be callable or None")
        return build_live_listener(
            artifacts,
            config,
            factories=factories,
            diagnostics=diagnostics,
            monitoring_stop=monitoring_stop,
            startup_admission_check=startup_admission_check,
            manage_signals=manage_signals,
            phase_observer=phase_observer,
        )

    runtime = construct_after_full_validation(
        files,
        runtime_config,
        component_factory=build_validated_runtime,
    )
    if on_listener_ready is not None:
        try:
            ready_result = on_listener_ready(runtime)
            if ready_result is not None:
                raise TypeError("on_listener_ready must return None")
        except BaseException as ready_error:
            try:
                runtime.close()
            except BaseException as cleanup_error:
                raise ready_error from cleanup_error
            raise
    try:
        result = runtime.run()
    except BaseException as run_error:
        try:
            runtime.close()
        except BaseException as cleanup_error:
            raise run_error from cleanup_error
        raise
    else:
        cleanup = runtime.close()
    return replace(result, cleanup_report=cleanup)
