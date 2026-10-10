from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import pytest

from artifacts.operator_protocol import OperatorProtocol
from drone_sim_companion.operator_control import (
    CompanionOperatorControl,
    RunningStateBinding,
    dronekit_heartbeat_is_fresh,
    running_state_binding,
)


RUN_ID = "00000000-0000-4000-8000-000000000001"
CONFIG_SHA256 = "a" * 64
REQUEST_SHA256 = "b" * 64
ADMISSION_SHA256 = "c" * 64
START_REQUEST_ID = "00000000-0000-4000-8000-000000000002"
ABORT_REQUEST_ID = "00000000-0000-4000-8000-000000000003"


@dataclass(frozen=True)
class StartRequest:
    run_id: str = RUN_ID
    config_sha256: str = CONFIG_SHA256
    request_id: str = START_REQUEST_ID
    sha256: str = REQUEST_SHA256


@dataclass(frozen=True)
class Admission:
    run_id: str = RUN_ID
    config_sha256: str = CONFIG_SHA256
    request_id: str = START_REQUEST_ID
    request_sha256: str = REQUEST_SHA256
    native_epoch_ns: int = 10_000_000_000
    public_epoch_ns: int = 0
    sha256: str = ADMISSION_SHA256


@dataclass(frozen=True)
class AbortRequest:
    run_id: str = RUN_ID
    config_sha256: str = CONFIG_SHA256
    request_id: str = ABORT_REQUEST_ID
    admission_sha256: str = ADMISSION_SHA256
    reason: str = "operator requested recovery"
    sha256: str = "d" * 64


class Protocol:
    def __init__(self) -> None:
        self.start_request = None
        self.admission = None
        self.abort_request = None
        self.preflight = []
        self.readiness = []
        self.abort_statuses = []

    def write_preflight(self, status) -> None:
        self.preflight.append(status)

    def read_preflight(self, _component):
        return self.preflight[-1] if self.preflight else None

    def read_start_request(self):
        return self.start_request

    def write_start_readiness(self, status) -> None:
        self.readiness.append(status)

    def read_start_admission(self):
        return self.admission

    def read_flight_abort_request(self):
        return self.abort_request

    def write_flight_abort_status(self, status) -> None:
        self.abort_statuses.append(status)

    def read_flight_abort_statuses(self):
        return tuple(self.abort_statuses)


def _control(protocol: Protocol, wall_ns: list[int]) -> CompanionOperatorControl:
    return CompanionOperatorControl(
        protocol=protocol,
        run_id=RUN_ID,
        config_sha256=CONFIG_SHA256,
        launch_mode="operator",
        preparation_wall_seconds=1_800,
        mission_wall_seconds=120,
        recovery_wall_seconds=10,
        wall_time_ns=lambda: wall_ns[0],
        monotonic_ns=lambda: wall_ns[0],
    )


def _running_state(
    *,
    run_id: str = RUN_ID,
    config_sha256: str = CONFIG_SHA256,
    admission_sha256: str = ADMISSION_SHA256,
) -> RunningStateBinding:
    return RunningStateBinding(run_id, config_sha256, admission_sha256)


def test_dronekit_preflight_requires_heartbeat_within_snapshot_freshness() -> None:
    assert dronekit_heartbeat_is_fresh(0.0) is True
    assert dronekit_heartbeat_is_fresh(2.0) is True
    assert dronekit_heartbeat_is_fresh(2.000_001) is False
    assert dronekit_heartbeat_is_fresh(True) is False


def test_running_state_binding_captures_the_wire_admission_digest() -> None:
    binding = running_state_binding(
        SimpleNamespace(
            state=3,
            run_id=RUN_ID,
            config_sha256=CONFIG_SHA256,
            admission_sha256=ADMISSION_SHA256,
        ),
        running_state=3,
    )
    missing = running_state_binding(
        SimpleNamespace(
            state=3,
            run_id=RUN_ID,
            config_sha256=CONFIG_SHA256,
        ),
        running_state=3,
    )

    assert binding == _running_state()
    assert missing == _running_state(admission_sha256="")


def test_operator_start_requires_fresh_disarmed_snapshot_and_matching_admission() -> None:
    protocol = Protocol()
    wall_ns = [1_000_000_000]
    control = _control(protocol, wall_ns)

    control.observe_vehicle(
        native_timestamp_ns=8_000_000_000,
        heartbeat_live=True,
        prearm_checks_healthy=True,
        armed=False,
        landed=True,
        mode="STABILIZE",
    )
    protocol.start_request = StartRequest()

    decision = control.poll_start(run_state=_running_state())

    assert decision.command_allowed is False
    assert protocol.readiness[-1].ready is True
    protocol.admission = Admission()
    decision = control.poll_start(run_state=_running_state())
    assert decision.command_allowed is True
    assert decision.admitted_now is True
    assert decision.admission_sha256 == ADMISSION_SHA256
    assert control.wall_deadline_ns == wall_ns[0] + 120_000_000_000


def test_operator_readiness_rejects_armed_and_stale_vehicle() -> None:
    protocol = Protocol()
    wall_ns = [1_000_000_000]
    control = _control(protocol, wall_ns)
    protocol.start_request = StartRequest()
    control.observe_vehicle(
        native_timestamp_ns=8_000_000_000,
        heartbeat_live=True,
        prearm_checks_healthy=True,
        armed=True,
        landed=False,
        mode="GUIDED",
    )

    control.poll_start(run_state=None)
    assert protocol.readiness[-1].ready is False
    assert "armed" in protocol.readiness[-1].detail

    protocol.readiness.clear()
    protocol.start_request = StartRequest(
        request_id="00000000-0000-4000-8000-000000000004",
        sha256="e" * 64,
    )
    wall_ns[0] += 2_000_000_001
    control.poll_start(run_state=None)
    assert protocol.readiness[-1].ready is False
    assert "stale" in protocol.readiness[-1].detail


def test_qgc_readiness_requires_fresh_confirmed_ground_state() -> None:
    protocol = Protocol()
    wall_ns = [1_000_000_000]
    control = _control(protocol, wall_ns)

    assert "telemetry" in " ".join(control.qgc_readiness_reasons())
    control.observe_vehicle(
        native_timestamp_ns=8_000_000_000,
        heartbeat_live=True,
        prearm_checks_healthy=True,
        armed=False,
        landed=False,
        mode="STABILIZE",
    )
    assert control.qgc_readiness_reasons() == ("vehicle is not confirmed landed",)
    control.observe_vehicle(native_timestamp_ns=8_050_000_000, landed=True)
    assert control.qgc_readiness_reasons() == ()
    wall_ns[0] += 2_000_000_001
    assert "telemetry" in " ".join(control.qgc_readiness_reasons())


def test_receipt_timestamp_preserves_callback_age_on_owner_loop() -> None:
    protocol = Protocol()
    wall_ns = [10_000_000_000]
    control = _control(protocol, wall_ns)

    control.observe_vehicle(
        native_timestamp_ns=8_000_000_000,
        received_wall_time_ns=7_000_000_000,
        heartbeat_live=True,
        prearm_checks_healthy=True,
        armed=False,
        landed=True,
        mode="STABILIZE",
    )

    assert "telemetry" in " ".join(control.qgc_readiness_reasons())
    assert protocol.preflight[-1].ready is False


@pytest.mark.parametrize("received", (True, -1, 12_000_000_001))
def test_receipt_timestamp_rejects_malformed_or_unbounded_values(received) -> None:
    protocol = Protocol()
    wall_ns = [10_000_000_000]
    control = _control(protocol, wall_ns)

    with pytest.raises(ValueError, match="bounded wall timestamp"):
        control.observe_vehicle(
            native_timestamp_ns=0,
            received_wall_time_ns=received,
            heartbeat_live=True,
        )


def test_unattended_mode_preserves_running_gate_without_operator_files() -> None:
    protocol = Protocol()
    wall_ns = [1_000_000_000]
    control = CompanionOperatorControl(
        protocol=protocol,
        run_id=RUN_ID,
        config_sha256=CONFIG_SHA256,
        launch_mode="unattended",
        preparation_wall_seconds=1_800,
        mission_wall_seconds=120,
        recovery_wall_seconds=10,
        wall_time_ns=lambda: wall_ns[0],
        monotonic_ns=lambda: wall_ns[0],
    )

    assert control.poll_start(run_state=None).command_allowed is False
    assert control.poll_start(
        run_state=_running_state(
            run_id="00000000-0000-4000-8000-000000000099",
            admission_sha256="",
        )
    ).command_allowed is False
    assert control.poll_start(
        run_state=_running_state(admission_sha256=ADMISSION_SHA256)
    ).command_allowed is False
    assert control.poll_start(
        run_state=_running_state(admission_sha256="")
    ).command_allowed is True
    assert protocol.readiness == []


def test_running_state_binding_rejects_foreign_or_unbound_operator_running() -> None:
    protocol = Protocol()
    wall_ns = [1_000_000_000]
    control = _control(protocol, wall_ns)
    protocol.start_request = StartRequest()
    protocol.admission = Admission()
    initial_deadline = control.wall_deadline_ns

    rejected = (
        _running_state(run_id="00000000-0000-4000-8000-000000000099"),
        _running_state(config_sha256="f" * 64),
        _running_state(admission_sha256=""),
        _running_state(admission_sha256="e" * 64),
    )
    for run_state in rejected:
        assert control.poll_start(run_state=run_state).command_allowed is False
        assert control.admission_sha256 is None
        assert control.wall_deadline_ns == initial_deadline

    assert control.poll_start(run_state=_running_state()).command_allowed is True


def test_equivalent_abort_retries_start_one_recovery_and_confirm_observed_landing() -> None:
    protocol = Protocol()
    wall_ns = [1_000_000_000]
    control = _control(protocol, wall_ns)
    protocol.start_request = StartRequest()
    protocol.admission = Admission()
    control.observe_vehicle(
        native_timestamp_ns=10_000_000_000,
        heartbeat_live=True,
        prearm_checks_healthy=True,
        armed=False,
        landed=True,
        mode="STABILIZE",
    )
    control.poll_start(run_state=_running_state())
    control.observe_vehicle(
        native_timestamp_ns=12_000_000_000,
        heartbeat_live=True,
        prearm_checks_healthy=True,
        armed=True,
        landed=False,
        mode="GUIDED",
    )
    protocol.abort_request = AbortRequest()
    recoveries = []

    for _ in range(3):
        control.poll_abort(
            finalizing=False,
            begin_recovery=lambda reason: recoveries.append(reason) or True,
        )

    assert recoveries == ["operator requested recovery"]
    assert [status.state for status in protocol.abort_statuses] == [
        "accepted",
        "recovering",
    ]

    wall_ns[0] += 500_000_000
    control.observe_vehicle(
        native_timestamp_ns=12_500_000_000,
        heartbeat_live=True,
        prearm_checks_healthy=True,
        armed=False,
        landed=True,
        mode="LAND",
    )
    control.poll_abort(finalizing=False, begin_recovery=lambda _reason: True)
    assert protocol.abort_statuses[-1].state == "landed_disarmed"


def test_abort_reports_rejection_missing_telemetry_timeout_and_finalization() -> None:
    protocol = Protocol()
    wall_ns = [1_000_000_000]
    control = _control(protocol, wall_ns)
    protocol.start_request = StartRequest()
    protocol.admission = Admission()
    control.observe_vehicle(
        native_timestamp_ns=10_000_000_000,
        heartbeat_live=True,
        prearm_checks_healthy=True,
        armed=False,
        landed=True,
        mode="STABILIZE",
    )
    control.poll_start(run_state=_running_state())
    control.observe_vehicle(
        native_timestamp_ns=12_000_000_000,
        heartbeat_live=True,
        prearm_checks_healthy=True,
        armed=True,
        landed=False,
        mode="GUIDED",
    )
    protocol.abort_request = AbortRequest()
    control.poll_abort(finalizing=False, begin_recovery=lambda _reason: False)
    assert protocol.abort_statuses[-1].state == "failed"
    assert "rejected" in protocol.abort_statuses[-1].detail

    protocol = Protocol()
    control = _control(protocol, wall_ns)
    protocol.start_request = StartRequest()
    protocol.admission = Admission()
    control.observe_vehicle(
        native_timestamp_ns=10_000_000_000,
        heartbeat_live=True,
        prearm_checks_healthy=True,
        armed=False,
        landed=True,
        mode="STABILIZE",
    )
    control.poll_start(run_state=_running_state())
    control.observe_vehicle(
        native_timestamp_ns=12_000_000_000,
        heartbeat_live=True,
        prearm_checks_healthy=True,
        armed=True,
        landed=False,
        mode="GUIDED",
    )
    protocol.abort_request = AbortRequest()
    mission_wall_deadline_ns = control.wall_deadline_ns
    control.poll_abort(finalizing=False, begin_recovery=lambda _reason: True)
    wall_ns[0] += 10_000_000_001
    control.poll_abort(finalizing=False, begin_recovery=lambda _reason: True)
    assert protocol.abort_statuses[-1].state == "failed"
    assert "telemetry" in protocol.abort_statuses[-1].detail
    assert control.wall_deadline_ns == mission_wall_deadline_ns

    protocol = Protocol()
    control = _control(protocol, wall_ns)
    protocol.start_request = StartRequest()
    protocol.admission = Admission()
    control.observe_vehicle(
        native_timestamp_ns=10_000_000_000,
        heartbeat_live=True,
        prearm_checks_healthy=True,
        armed=False,
        landed=True,
        mode="STABILIZE",
    )
    control.poll_start(run_state=_running_state())
    control.observe_vehicle(
        native_timestamp_ns=12_000_000_000,
        heartbeat_live=True,
        prearm_checks_healthy=True,
        armed=True,
        landed=False,
        mode="GUIDED",
    )
    protocol.abort_request = AbortRequest()
    control.poll_abort(finalizing=False, begin_recovery=lambda _reason: True)
    control.poll_abort(finalizing=True, begin_recovery=lambda _reason: True)
    assert protocol.abort_statuses[-1].state == "interrupted"


def test_recovery_reports_qgc_authority_change_without_retrying_land() -> None:
    protocol = Protocol()
    wall_ns = [1_000_000_000]
    control = _control(protocol, wall_ns)
    protocol.start_request = StartRequest()
    protocol.admission = Admission()
    control.observe_vehicle(
        native_timestamp_ns=10_000_000_000,
        heartbeat_live=True,
        prearm_checks_healthy=True,
        armed=False,
        landed=True,
        mode="STABILIZE",
    )
    control.poll_start(run_state=_running_state())
    control.observe_vehicle(
        native_timestamp_ns=12_000_000_000,
        heartbeat_live=True,
        prearm_checks_healthy=True,
        armed=True,
        landed=False,
        mode="GUIDED",
    )
    protocol.abort_request = AbortRequest()
    recoveries = []
    control.poll_abort(
        finalizing=False,
        begin_recovery=lambda reason: recoveries.append(reason) or True,
    )
    wall_ns[0] += 100_000_000
    control.observe_vehicle(
        native_timestamp_ns=12_100_000_000,
        heartbeat_live=True,
        prearm_checks_healthy=True,
        armed=True,
        landed=False,
        mode="LAND",
    )
    control.poll_abort(finalizing=False, begin_recovery=lambda _reason: True)
    wall_ns[0] += 100_000_000
    control.observe_vehicle(
        native_timestamp_ns=12_200_000_000,
        heartbeat_live=True,
        prearm_checks_healthy=True,
        armed=True,
        landed=False,
        mode="LOITER",
    )
    control.poll_abort(finalizing=False, begin_recovery=lambda _reason: True)

    assert recoveries == ["operator requested recovery"]
    assert protocol.abort_statuses[-1].state == "failed"
    assert "LOITER" in protocol.abort_statuses[-1].detail


def test_companion_control_crosses_durable_operator_protocol(tmp_path: Path) -> None:
    run_directory = tmp_path / RUN_ID
    for relative in (".control", ".status", "configuration"):
        (run_directory / relative).mkdir(parents=True, exist_ok=True)
    wall_ns = [1_000_000_000]

    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        request = protocol.write_start_request(START_REQUEST_ID)
        control = _control(protocol, wall_ns)
        control.observe_vehicle(
            native_timestamp_ns=9_000_000_000,
            heartbeat_live=True,
            prearm_checks_healthy=True,
            armed=False,
            landed=True,
            mode="STABILIZE",
        )
        assert control.poll_start(run_state=None).command_allowed is False
        readiness = protocol.read_start_readiness()
        assert readiness is not None and readiness.ready is True
        admission = protocol.write_start_admission(
            request,
            native_epoch_ns=10_000_000_000,
        )
        assert control.poll_start(
            run_state=_running_state(admission_sha256=admission.sha256)
        ).admission_sha256 == admission.sha256
        protocol.write_flight_abort_request(
            ABORT_REQUEST_ID,
            admission.sha256,
            "operator requested recovery",
        )
        control.poll_abort(finalizing=False, begin_recovery=lambda _reason: True)
        assert [status.state for status in protocol.read_flight_abort_statuses()] == [
            "accepted",
            "landed_disarmed",
        ]
