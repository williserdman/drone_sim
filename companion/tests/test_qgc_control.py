from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
import json
from pathlib import Path

import pytest

from artifacts.operator_protocol import (
    MissionStartRequest,
    OperatorProtocol,
    StartAdmission,
)
from artifacts.qgc_operator import (
    MAX_ATTEMPT_TOKEN,
    MAV_RESULT_ACCEPTED,
    MAV_RESULT_CANCELLED,
    MAV_RESULT_DENIED,
    MAV_RESULT_IN_PROGRESS,
    MAV_RESULT_TEMPORARILY_REJECTED,
    MAV_RESULT_UNSUPPORTED,
    MavlinkIdentity,
    QGC_ABORT_COMMAND,
    QGC_ACTIONS_KIND,
    QGC_COMPANION_IDENTITY,
    QGC_FLIGHT_CONTROLLER_IDENTITY,
    QGC_START_COMMAND,
    QgcCommandEnvelope,
    QgcOperatorDescriptor,
)
from drone_sim_companion.operator_control import (
    CompanionOperatorControl,
    RunningStateBinding,
)
from drone_sim_companion.qgc_control import QgcMissionControl


RUN_ID = "00000000-0000-4000-8000-000000000001"
CONFIG_SHA256 = "a" * 64
START_REQUEST_ID = "00000000-0000-4000-8000-000000000002"
ABORT_REQUEST_ID = "00000000-0000-4000-8000-000000000003"
FOREIGN_REQUEST_ID = "00000000-0000-4000-8000-000000000004"
QGC = MavlinkIdentity(255, 190)


class CommandIO:
    def __init__(self) -> None:
        self.receive = None
        self.responses = []

    def install(self, receive) -> None:
        assert self.receive is None
        self.receive = receive

    def send_ack(self, response) -> None:
        self.responses.append(response)


@pytest.fixture
def run_directory(tmp_path: Path) -> Path:
    run = tmp_path / RUN_ID
    for relative in (".control", ".status", "configuration"):
        (run / relative).mkdir(parents=True, exist_ok=True)
    return run


def _descriptor() -> QgcOperatorDescriptor:
    return QgcOperatorDescriptor(
        kind=QGC_ACTIONS_KIND,
        start_command=QGC_START_COMMAND,
        abort_command=QGC_ABORT_COMMAND,
        attempt_token=17,
        start_request_id=START_REQUEST_ID,
        abort_request_id=ABORT_REQUEST_ID,
        qgc_source=QGC,
        companion=QGC_COMPANION_IDENTITY,
        flight_controller=QGC_FLIGHT_CONTROLLER_IDENTITY,
        actions_path="configuration/qgc-actions.json",
        actions_sha256="b" * 64,
    )


def _envelope(
    *,
    command: int = QGC_START_COMMAND,
    wire_version: int = 2,
    source: MavlinkIdentity = QGC,
    target: MavlinkIdentity = QGC_COMPANION_IDENTITY,
    parameters: tuple[int | float, ...] = (17, 0, 0, 0, 0, 0, 0),
    received_wall_time_ns: int = 1_000_000_000,
) -> QgcCommandEnvelope:
    return QgcCommandEnvelope(
        wire_version,
        source,
        target,
        command,
        parameters,
        received_wall_time_ns,
    )


def _parameters_with(index: int, value: int | float) -> tuple[int | float, ...]:
    parameters: list[int | float] = [17, 0, 0, 0, 0, 0, 0]
    parameters[index] = value
    return tuple(parameters)


def _owner(
    protocol,
    io,
    wall_ns,
    readiness=lambda: (),
    finalizing=lambda: False,
):
    operator = CompanionOperatorControl(
        protocol=protocol,
        run_id=RUN_ID,
        config_sha256=CONFIG_SHA256,
        launch_mode="operator",
        preparation_wall_seconds=1800,
        mission_wall_seconds=120,
        recovery_wall_seconds=10,
        wall_time_ns=lambda: wall_ns[0],
        monotonic_ns=lambda: wall_ns[0],
    )
    return operator, QgcMissionControl(
        protocol,
        operator,
        _descriptor(),
        io,
        readiness,
        finalizing,
        wall_time_ns=lambda: wall_ns[0],
    )


_READY_VEHICLE = {
    "native_timestamp_ns": 10_000_000_000,
    "heartbeat_live": True,
    "prearm_checks_healthy": True,
    "armed": False,
    "landed": True,
    "mode": "STABILIZE",
}


@pytest.mark.parametrize(
    ("command", "vehicle", "host_reasons", "advance_ns", "expected"),
    (
        pytest.param(
            _envelope(command=999),
            None,
            (),
            0,
            MAV_RESULT_UNSUPPORTED,
            id="unsupported",
        ),
        pytest.param(
            _envelope(wire_version=1),
            None,
            (),
            0,
            MAV_RESULT_DENIED,
            id="mavlink-1",
        ),
        pytest.param(
            _envelope(source=MavlinkIdentity(254, 190)),
            None,
            (),
            0,
            MAV_RESULT_DENIED,
            id="wrong-source",
        ),
        pytest.param(
            _envelope(target=QGC_FLIGHT_CONTROLLER_IDENTITY),
            None,
            (),
            0,
            MAV_RESULT_DENIED,
            id="wrong-target",
        ),
        pytest.param(
            _envelope(parameters=(0, 0, 0, 0, 0, 0, 0)),
            None,
            (),
            0,
            MAV_RESULT_DENIED,
            id="token-zero",
        ),
        pytest.param(
            _envelope(parameters=(MAX_ATTEMPT_TOKEN + 1, 0, 0, 0, 0, 0, 0)),
            None,
            (),
            0,
            MAV_RESULT_DENIED,
            id="token-above-boundary",
        ),
        pytest.param(
            _envelope(parameters=(17.5, 0, 0, 0, 0, 0, 0)),
            None,
            (),
            0,
            MAV_RESULT_DENIED,
            id="fractional-token",
        ),
        pytest.param(
            _envelope(parameters=(float("nan"), 0, 0, 0, 0, 0, 0)),
            None,
            (),
            0,
            MAV_RESULT_DENIED,
            id="nan-token",
        ),
        pytest.param(
            _envelope(parameters=(float("inf"), 0, 0, 0, 0, 0, 0)),
            None,
            (),
            0,
            MAV_RESULT_DENIED,
            id="infinite-token",
        ),
        *(
            pytest.param(
                _envelope(parameters=_parameters_with(index, 1)),
                None,
                (),
                0,
                MAV_RESULT_DENIED,
                id=f"nonzero-param-{index + 1}",
            )
            for index in range(1, 7)
        ),
        pytest.param(
            _envelope(),
            {**_READY_VEHICLE, "armed": True},
            (),
            0,
            MAV_RESULT_TEMPORARILY_REJECTED,
            id="armed",
        ),
        pytest.param(
            _envelope(),
            {**_READY_VEHICLE, "landed": False},
            (),
            0,
            MAV_RESULT_TEMPORARILY_REJECTED,
            id="airborne",
        ),
        pytest.param(
            _envelope(),
            _READY_VEHICLE,
            (),
            2_000_000_001,
            MAV_RESULT_TEMPORARILY_REJECTED,
            id="stale-vehicle",
        ),
        pytest.param(
            _envelope(),
            {**_READY_VEHICLE, "prearm_checks_healthy": False},
            (),
            0,
            MAV_RESULT_TEMPORARILY_REJECTED,
            id="unhealthy-vehicle",
        ),
        pytest.param(
            _envelope(),
            _READY_VEHICLE,
            ("calibration pending",),
            0,
            MAV_RESULT_TEMPORARILY_REJECTED,
            id="calibration",
        ),
        pytest.param(
            _envelope(),
            _READY_VEHICLE,
            ("native camera producer is stale",),
            0,
            MAV_RESULT_TEMPORARILY_REJECTED,
            id="producer",
        ),
        pytest.param(
            _envelope(),
            _READY_VEHICLE,
            ("native clock is unavailable",),
            0,
            MAV_RESULT_TEMPORARILY_REJECTED,
            id="native-clock",
        ),
    ),
)
def test_invalid_or_early_start_does_not_reserve(
    run_directory: Path,
    command: QgcCommandEnvelope,
    vehicle: dict | None,
    host_reasons: tuple[str, ...],
    advance_ns: int,
    expected: int,
) -> None:
    wall_ns = [command.received_wall_time_ns]
    io = CommandIO()
    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        operator, owner = _owner(protocol, io, wall_ns, lambda: host_reasons)
        if vehicle is not None:
            operator.observe_vehicle(**vehicle)
        wall_ns[0] += advance_ns
        owner.accept(command)
        assert owner.poll_start(run_state=None).command_allowed is False
        assert protocol.read_start_request() is None
        assert protocol.read_start_admission() is None
        assert operator.admission_sha256 is None
        assert io.responses[-1].result == expected
        assert io.responses[-1].status_text
        assert len(protocol.read_qgc_command_statuses()) == 1


def test_temporary_readiness_failure_does_not_reserve(run_directory: Path) -> None:
    wall_ns = [1_000_000_000]
    io = CommandIO()
    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        operator, owner = _owner(protocol, io, wall_ns, lambda: ("calibration pending",))
        operator.observe_vehicle(
            native_timestamp_ns=10_000_000_000,
            heartbeat_live=True,
            prearm_checks_healthy=True,
            armed=False,
            landed=True,
            mode="STABILIZE",
        )
        owner.accept(_envelope())
        owner.poll_start(run_state=None)
        assert io.responses[-1].result == MAV_RESULT_TEMPORARILY_REJECTED
        assert "calibration" in io.responses[-1].status_text
        assert protocol.read_start_request() is None


def test_start_retry_uses_one_request_and_admission(run_directory: Path) -> None:
    wall_ns = [1_000_000_000]
    io = CommandIO()
    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        operator, owner = _owner(protocol, io, wall_ns)
        operator.observe_vehicle(
            native_timestamp_ns=10_000_000_000,
            heartbeat_live=True,
            prearm_checks_healthy=True,
            armed=False,
            landed=True,
            mode="STABILIZE",
        )
        for _ in range(2):
            owner.accept(_envelope())
            owner.poll_start(run_state=None)
        request = protocol.read_start_request()
        assert request is not None and request.request_id == START_REQUEST_ID
        assert [response.result for response in io.responses] == [
            MAV_RESULT_IN_PROGRESS,
            MAV_RESULT_IN_PROGRESS,
        ]

        admission = protocol.write_start_admission(
            request, native_epoch_ns=10_050_000_000
        )
        running = RunningStateBinding(RUN_ID, CONFIG_SHA256, admission.sha256)
        decision = owner.poll_start(run_state=running)
        assert decision.command_allowed and decision.admitted_now
        assert io.responses[-1].result == MAV_RESULT_ACCEPTED

        operator.observe_vehicle(
            native_timestamp_ns=10_100_000_000,
            armed=True,
            landed=False,
            mode="GUIDED",
        )
        owner.accept(_envelope())
        replay = owner.poll_start(run_state=running)
        assert replay.command_allowed
        assert io.responses[-1].result == MAV_RESULT_ACCEPTED
        accepted = [
            row for row in protocol.read_qgc_command_statuses()
            if row.response.result == MAV_RESULT_ACCEPTED
        ]
        assert len(accepted) == 2
        assert all(row.admission_sha256 == admission.sha256 for row in accepted)


def test_start_rechecks_readiness_before_execution_release(run_directory: Path) -> None:
    wall_ns = [1_000_000_000]
    reasons: list[str] = []
    io = CommandIO()
    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        operator, owner = _owner(protocol, io, wall_ns, lambda: tuple(reasons))
        operator.observe_vehicle(
            native_timestamp_ns=10_000_000_000,
            heartbeat_live=True,
            prearm_checks_healthy=True,
            armed=False,
            landed=True,
        )
        owner.accept(_envelope())
        owner.poll_start(run_state=None)
        request = protocol.read_start_request()
        assert request is not None
        admission = protocol.write_start_admission(
            request, native_epoch_ns=10_050_000_000
        )
        running = RunningStateBinding(RUN_ID, CONFIG_SHA256, admission.sha256)

        reasons.append("native camera producer is stale")
        blocked = owner.poll_start(run_state=running)
        assert blocked.command_allowed is False
        assert all(
            response.result != MAV_RESULT_ACCEPTED for response in io.responses
        )

        reasons.clear()
        released = owner.poll_start(run_state=running)
        assert released.command_allowed is True
        assert io.responses[-1].result == MAV_RESULT_ACCEPTED


def test_callback_ingress_is_bounded_and_reservation_runs_on_owner_poll(
    run_directory: Path,
) -> None:
    wall_ns = [1_000_000_000]
    io = CommandIO()
    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        operator, owner = _owner(protocol, io, wall_ns)
        operator.observe_vehicle(
            native_timestamp_ns=10_000_000_000,
            heartbeat_live=True,
            prearm_checks_healthy=True,
            armed=False,
            landed=True,
        )
        assert io.receive is not None
        with ThreadPoolExecutor(max_workers=8) as executor:
            tuple(executor.map(io.receive, (_envelope() for _ in range(20))))

        assert protocol.read_start_request() is None
        assert protocol.read_qgc_command_statuses() == ()
        owner.poll_start(run_state=None)
        assert protocol.read_start_request().request_id == START_REQUEST_ID
        assert len(io.responses) == 20


def test_overflow_command_keeps_its_validation_result(run_directory: Path) -> None:
    wall_ns = [1_000_000_000]
    io = CommandIO()
    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        operator, owner = _owner(protocol, io, wall_ns)
        operator.observe_vehicle(
            native_timestamp_ns=10_000_000_000,
            heartbeat_live=True,
            prearm_checks_healthy=True,
            armed=False,
            landed=True,
        )
        for _ in range(64):
            owner.accept(_envelope())
        owner.accept(_envelope(parameters=(float("nan"), 0, 0, 0, 0, 0, 0)))

        owner.poll_start(run_state=None)

        assert io.responses[-1].result == MAV_RESULT_DENIED
        assert protocol.read_qgc_command_statuses()[-1].envelope.parameters[0] == "NaN"


def _seed_restart_state(run_directory: Path, state: str) -> None:
    request = MissionStartRequest(RUN_ID, CONFIG_SHA256, START_REQUEST_ID)
    foreign_request = MissionStartRequest(RUN_ID, CONFIG_SHA256, FOREIGN_REQUEST_ID)
    admission_request = foreign_request if state == "mismatched-admission" else request
    admission = StartAdmission(
        RUN_ID,
        CONFIG_SHA256,
        admission_request.request_id,
        admission_request.sha256,
        10_050_000_000,
    )
    if state in {"request-only", "matched-admission", "mismatched-admission"}:
        (run_directory / ".control/mission-start-request.json").write_text(
            json.dumps(asdict(request)), encoding="utf-8"
        )
    elif state == "foreign-request":
        (run_directory / ".control/mission-start-request.json").write_text(
            json.dumps(asdict(foreign_request)), encoding="utf-8"
        )
    if state in {"admission-only", "matched-admission", "mismatched-admission"}:
        (run_directory / "configuration/start-admission.json").write_text(
            json.dumps(asdict(admission)), encoding="utf-8"
        )


@pytest.mark.parametrize(
    "durable_state",
    (
        "request-only",
        "admission-only",
        "foreign-request",
        "matched-admission",
        "mismatched-admission",
    ),
)
def test_restart_after_reservation_requires_new_prepare(
    run_directory: Path, durable_state: str
) -> None:
    _seed_restart_state(run_directory, durable_state)
    wall_ns = [1_000_000_000]
    restarted_io = CommandIO()
    request_path = run_directory / ".control/mission-start-request.json"
    request_before = request_path.read_bytes() if request_path.exists() else None
    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        operator, restarted = _owner(protocol, restarted_io, wall_ns)
        operator.observe_vehicle(
            native_timestamp_ns=10_050_000_000,
            heartbeat_live=True,
            prearm_checks_healthy=True,
            armed=False,
            landed=True,
        )
        restarted.accept(_envelope())
        assert restarted.poll_start(run_state=None).command_allowed is False
        assert restarted_io.responses[-1].result == MAV_RESULT_DENIED
        assert "new Prepare" in restarted_io.responses[-1].status_text
        assert operator.admission_sha256 is None
    request_after = request_path.read_bytes() if request_path.exists() else None
    assert request_after == request_before


def test_abort_cancels_pending_start_without_inventing_admission(
    run_directory: Path,
) -> None:
    wall_ns = [1_000_000_000]
    io = CommandIO()
    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        operator, owner = _owner(protocol, io, wall_ns)
        operator.observe_vehicle(
            native_timestamp_ns=10_000_000_000,
            heartbeat_live=True,
            prearm_checks_healthy=True,
            armed=False,
            landed=True,
        )
        owner.accept(_envelope())
        owner.poll_start(run_state=None)
        owner.accept(_envelope(command=QGC_ABORT_COMMAND))
        owner.poll_abort(finalizing=False, begin_recovery=lambda _reason: True)

        assert protocol.read_start_admission() is None
        assert protocol.read_flight_abort_request() is None
        assert io.responses[-2].result == MAV_RESULT_CANCELLED
        assert io.responses[-1].result == MAV_RESULT_ACCEPTED
        assert owner.poll_start(run_state=None).command_allowed is False


def test_abort_is_once_and_finalization_wins(run_directory: Path) -> None:
    wall_ns = [1_000_000_000]
    io = CommandIO()
    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        operator, owner = _owner(protocol, io, wall_ns)
        operator.observe_vehicle(
            native_timestamp_ns=10_000_000_000,
            heartbeat_live=True,
            prearm_checks_healthy=True,
            armed=False,
            landed=True,
            mode="STABILIZE",
        )
        owner.accept(_envelope())
        owner.poll_start(run_state=None)
        request = protocol.read_start_request()
        assert request is not None
        admission = protocol.write_start_admission(
            request, native_epoch_ns=10_050_000_000
        )
        owner.poll_start(
            run_state=RunningStateBinding(RUN_ID, CONFIG_SHA256, admission.sha256)
        )
        operator.observe_vehicle(
            native_timestamp_ns=10_100_000_000,
            heartbeat_live=True,
            prearm_checks_healthy=True,
            armed=True,
            landed=False,
            mode="GUIDED",
        )
        owner.accept(_envelope(command=QGC_ABORT_COMMAND))
        recoveries = []
        owner.poll_abort(
            finalizing=False,
            begin_recovery=lambda reason: recoveries.append(reason) or True,
        )
        owner.accept(_envelope(command=QGC_ABORT_COMMAND))
        owner.poll_abort(
            finalizing=True,
            begin_recovery=lambda reason: recoveries.append(reason) or True,
        )

        assert len(recoveries) == 1
        assert protocol.read_latest_flight_abort_status().state == "interrupted"
        assert io.responses[-1].result == MAV_RESULT_CANCELLED


def test_finalization_blocks_new_abort_reservation(run_directory: Path) -> None:
    wall_ns = [1_000_000_000]
    io = CommandIO()
    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        operator, owner = _owner(protocol, io, wall_ns)
        operator.observe_vehicle(
            native_timestamp_ns=10_000_000_000,
            heartbeat_live=True,
            prearm_checks_healthy=True,
            armed=False,
            landed=True,
            mode="STABILIZE",
        )
        owner.accept(_envelope())
        owner.poll_start(run_state=None)
        request = protocol.read_start_request()
        assert request is not None
        admission = protocol.write_start_admission(
            request, native_epoch_ns=10_050_000_000
        )
        owner.poll_start(
            run_state=RunningStateBinding(RUN_ID, CONFIG_SHA256, admission.sha256)
        )
        owner.accept(_envelope(command=QGC_ABORT_COMMAND))
        recoveries: list[str] = []

        owner.poll_abort(
            finalizing=True,
            begin_recovery=lambda reason: recoveries.append(reason) or True,
        )

        assert protocol.read_flight_abort_request() is None
        assert recoveries == []
        assert io.responses[-1].result == MAV_RESULT_CANCELLED
        assert io.responses[-1].status_text == "Simulation finalization blocks Abort recovery"
        owner.accept(_envelope(command=QGC_ABORT_COMMAND))
        owner.poll_abort(finalizing=False, begin_recovery=lambda _reason: True)
        assert protocol.read_flight_abort_request() is None
        assert io.responses[-1].result == MAV_RESULT_CANCELLED


def test_callback_finalization_blocks_queued_start_and_stays_latched(
    run_directory: Path,
) -> None:
    wall_ns = [1_000_000_000]
    finalizing = [True]
    io = CommandIO()
    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        operator, owner = _owner(
            protocol, io, wall_ns, finalizing=lambda: finalizing[0]
        )
        operator.observe_vehicle(
            native_timestamp_ns=10_000_000_000,
            heartbeat_live=True,
            prearm_checks_healthy=True,
            armed=False,
            landed=True,
        )
        owner.accept(_envelope())

        owner.poll_abort(finalizing=False, begin_recovery=lambda _reason: True)

        assert protocol.read_start_request() is None
        assert io.responses[-1].result == MAV_RESULT_DENIED
        finalizing[0] = False
        owner.accept(_envelope())
        assert owner.poll_start(run_state=None).command_allowed is False
        assert protocol.read_start_request() is None
        assert io.responses[-1].result == MAV_RESULT_DENIED


def test_abort_reports_only_observed_landed_disarmed_success(
    run_directory: Path,
) -> None:
    wall_ns = [1_000_000_000]
    io = CommandIO()
    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        operator, owner = _owner(protocol, io, wall_ns)
        operator.observe_vehicle(
            native_timestamp_ns=10_000_000_000,
            heartbeat_live=True,
            prearm_checks_healthy=True,
            armed=False,
            landed=True,
            mode="STABILIZE",
        )
        owner.accept(_envelope())
        owner.poll_start(run_state=None)
        request = protocol.read_start_request()
        assert request is not None
        admission = protocol.write_start_admission(
            request, native_epoch_ns=10_050_000_000
        )
        owner.poll_start(
            run_state=RunningStateBinding(RUN_ID, CONFIG_SHA256, admission.sha256)
        )
        operator.observe_vehicle(
            native_timestamp_ns=10_100_000_000,
            heartbeat_live=True,
            prearm_checks_healthy=True,
            armed=True,
            landed=False,
            mode="GUIDED",
        )
        owner.accept(_envelope(command=QGC_ABORT_COMMAND))
        owner.poll_abort(finalizing=False, begin_recovery=lambda _reason: True)
        assert all(
            response.result != MAV_RESULT_ACCEPTED
            for response in io.responses
            if response.command == QGC_ABORT_COMMAND
        )

        wall_ns[0] += 100_000_000
        operator.observe_vehicle(
            native_timestamp_ns=10_200_000_000,
            heartbeat_live=True,
            prearm_checks_healthy=True,
            armed=False,
            landed=True,
            mode="LAND",
        )
        owner.poll_abort(finalizing=False, begin_recovery=lambda _reason: True)
        assert io.responses[-1].result == MAV_RESULT_ACCEPTED
        assert protocol.read_qgc_command_statuses()[-1].observed_outcome == "landed_disarmed"


def test_terminal_run_rejects_new_start_without_reservation(run_directory: Path) -> None:
    wall_ns = [1_000_000_000]
    io = CommandIO()
    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        _operator, owner = _owner(protocol, io, wall_ns)
        owner.observe_mission_terminal("FAILED")
        owner.accept(_envelope())
        owner.poll_start(run_state=None)

        assert protocol.read_start_request() is None
        assert io.responses[-1].result == MAV_RESULT_DENIED
        assert "Terminal" in io.responses[-1].status_text
