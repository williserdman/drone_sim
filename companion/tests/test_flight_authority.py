from __future__ import annotations

import queue

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from drone_sim_companion.flight_authority import (
    DroneKitAuthorityWriter,
    ExpectedTransition,
    FlightAuthorityGate,
    FlightAuthorityLost,
)
from drone_sim_companion.mission import Telemetry


def heartbeat(stamp: int, *, mode: str, armed: bool) -> Telemetry:
    return Telemetry(stamp, heartbeat=True, mode=mode, armed=armed)


def admitted_gate() -> FlightAuthorityGate:
    gate = FlightAuthorityGate()
    gate.observe(heartbeat(1, mode="STABILIZE", armed=False))
    gate.admit("a" * 64)
    return gate


def test_expected_transition_must_describe_a_mode_or_arm_change() -> None:
    with pytest.raises(ValueError, match="mode or armed"):
        ExpectedTransition()


def test_expected_mode_and_arm_changes_are_owned_only_after_enqueue() -> None:
    gate = admitted_gate()

    with gate.output_transaction(ExpectedTransition(mode="GUIDED")):
        pass
    gate.observe(heartbeat(2, mode="GUIDED", armed=False))

    with gate.output_transaction(ExpectedTransition(armed=True)):
        pass
    gate.observe(heartbeat(3, mode="GUIDED", armed=True))

    assert gate.revoked is False


def test_back_to_back_mode_and_arm_expectations_survive_until_observed() -> None:
    gate = admitted_gate()

    with gate.output_transaction(ExpectedTransition(mode="GUIDED")):
        pass
    with gate.output_transaction(ExpectedTransition(armed=True)):
        pass
    gate.observe(heartbeat(2, mode="GUIDED", armed=False))
    gate.observe(heartbeat(3, mode="GUIDED", armed=True))

    assert gate.revoked is False


@pytest.mark.parametrize("external_mode", ("LAND", "BRAKE", "LOITER"))
def test_pre_enqueue_qgc_mode_change_revokes_pending_output(
    external_mode: str,
) -> None:
    gate = admitted_gate()
    gate.observe(heartbeat(2, mode=external_mode, armed=False))

    with pytest.raises(FlightAuthorityLost, match=external_mode):
        with gate.output_transaction(ExpectedTransition(mode="GUIDED")):
            pytest.fail("revoked output reached the enqueue callback")

    gate.observe(heartbeat(3, mode="GUIDED", armed=False))
    with pytest.raises(FlightAuthorityLost):
        with gate.output_transaction(ExpectedTransition(mode="GUIDED")):
            pass


def test_stale_observation_cannot_revoke_or_restore_authority() -> None:
    gate = admitted_gate()
    with gate.output_transaction(ExpectedTransition(mode="GUIDED")):
        pass
    gate.observe(heartbeat(2, mode="GUIDED", armed=False))

    gate.observe(heartbeat(1, mode="LOITER", armed=False))

    assert gate.revoked is False
    with gate.output_transaction():
        pass


def test_distinct_observations_with_the_same_receipt_stamp_are_not_lost() -> None:
    gate = admitted_gate()
    with gate.output_transaction(ExpectedTransition(mode="GUIDED")):
        pass
    gate.observe(heartbeat(2, mode="GUIDED", armed=False))
    gate.observe(heartbeat(2, mode="LOITER", armed=False))

    assert gate.reason == "unexpected flight-controller mode transition to LOITER"


def test_operator_wait_accepts_only_its_active_native_guided_arm_transition() -> None:
    gate = admitted_gate()
    expected = ExpectedTransition(mode="GUIDED", armed=True)

    with gate.operator_wait(expected):
        gate.observe(heartbeat(2, mode="GUIDED", armed=False))
        gate.observe(heartbeat(3, mode="GUIDED", armed=True))
        assert gate.revoked is False

    another = admitted_gate()
    another.observe(heartbeat(2, mode="GUIDED", armed=False))
    assert another.revoked is True


@pytest.mark.parametrize(
    "transition",
    (ExpectedTransition(mode="LAND"), ExpectedTransition(armed=False)),
)
def test_operator_wait_rejects_non_start_transitions(
    transition: ExpectedTransition,
) -> None:
    gate = admitted_gate()

    with pytest.raises(ValueError, match="GUIDED and arm"):
        with gate.operator_wait(transition):
            pass


def test_revoke_is_irreversible_and_preserves_first_reason() -> None:
    gate = admitted_gate()
    gate.revoke("external mode change")
    gate.revoke("later failure")

    assert gate.revoked is True
    assert gate.reason == "external mode change"
    gate.observe(heartbeat(2, mode="GUIDED", armed=False))
    with pytest.raises(FlightAuthorityLost, match="external mode change"):
        with gate.output_transaction():
            pass


class QueueWriter:
    def __init__(self) -> None:
        self.queue: queue.Queue[bytes] = queue.Queue()

    def write(self, packet: bytes) -> int:
        self.queue.put(packet)
        return len(packet)

    def read(self) -> bytes:
        return b""


def test_dronekit_writer_blocks_mutation_at_queue_and_keeps_observation_output() -> None:
    gate = admitted_gate()
    delegate = QueueWriter()
    writer = DroneKitAuthorityWriter(delegate, gate, mavlink)
    encoder = mavlink.MAVLink(writer, srcSystem=1, srcComponent=191)

    encoder.command_long_send(
        1, 1, mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
    )
    gate.observe(heartbeat(2, mode="STABILIZE", armed=True))
    gate.revoke("external authority")
    queued_before = delegate.queue.qsize()

    blocked = (
        lambda: encoder.command_long_send(
            1, 1, mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 4, 0, 0, 0, 0, 0
        ),
        lambda: encoder.set_position_target_global_int_send(
            0, 1, 1, mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0, 0, 0, 1,
            0, 0, 0, 0, 0, 0, 0, 0,
        ),
        lambda: encoder.rc_channels_override_send(1, 1, *([1500] * 18)),
        lambda: encoder.command_long_send(
            1, 1, mavlink.MAV_CMD_DO_AUX_FUNCTION, 0, 180, 2, 0, 0, 0, 0, 0
        ),
        lambda: encoder.param_set_send(1, 1, b"PLND_ENABLED", 0.0, 9),
    )
    for output in blocked:
        with pytest.raises(FlightAuthorityLost):
            output()

    encoder.heartbeat_send(18, 8, 0, 0, 4)
    encoder.param_request_list_send(1, 1)
    assert delegate.queue.qsize() == queued_before + 2


def test_dronekit_writer_owns_expected_autotune_and_loiter_transitions() -> None:
    gate = admitted_gate()
    delegate = QueueWriter()
    writer = DroneKitAuthorityWriter(delegate, gate, mavlink)
    encoder = mavlink.MAVLink(writer, srcSystem=1, srcComponent=191)

    encoder.set_mode_send(
        1, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 15
    )
    gate.observe(heartbeat(2, mode="AUTOTUNE", armed=False))
    encoder.set_mode_send(
        1, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 5
    )
    gate.observe(heartbeat(3, mode="LOITER", armed=False))

    assert gate.revoked is False


def test_dronekit_land_mode_owns_entry_and_eventual_disarm_separately() -> None:
    gate = FlightAuthorityGate()
    gate.observe(heartbeat(1, mode="GUIDED", armed=True))
    gate.admit("a" * 64)
    delegate = QueueWriter()
    writer = DroneKitAuthorityWriter(delegate, gate, mavlink)
    encoder = mavlink.MAVLink(writer, srcSystem=1, srcComponent=191)

    encoder.set_mode_send(
        1, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9
    )
    gate.observe(heartbeat(2, mode="LAND", armed=True))
    gate.observe(heartbeat(3, mode="LAND", armed=False))

    assert gate.revoked is False


def test_dronekit_land_permission_survives_intervening_landing_target_writes() -> None:
    gate = FlightAuthorityGate()
    gate.observe(heartbeat(1, mode="GUIDED", armed=True))
    gate.admit("a" * 64)
    delegate = QueueWriter()
    writer = DroneKitAuthorityWriter(delegate, gate, mavlink)
    encoder = mavlink.MAVLink(writer, srcSystem=1, srcComponent=191)

    encoder.set_mode_send(1, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9)
    encoder.landing_target_send(0, 0, mavlink.MAV_FRAME_BODY_FRD, 0.0, 0.0, 0.0, 0.0, 0.0)
    gate.observe(heartbeat(2, mode="LAND", armed=True))
    encoder.landing_target_send(0, 0, mavlink.MAV_FRAME_BODY_FRD, 0.0, 0.0, 0.0, 0.0, 0.0)
    gate.observe(heartbeat(3, mode="LAND", armed=False))

    assert gate.revoked is False


def test_clearing_operation_expectations_makes_late_transition_external() -> None:
    gate = admitted_gate()
    with gate.output_transaction(ExpectedTransition(mode="LAND")):
        pass

    gate.clear_expected()
    gate.observe(heartbeat(2, mode="LAND", armed=False))

    assert gate.reason == "unexpected flight-controller mode transition to LAND"


def test_finalization_latch_blocks_selected_output_at_actual_queue_write() -> None:
    gate = admitted_gate()
    delegate = QueueWriter()
    writer = DroneKitAuthorityWriter(delegate, gate, mavlink)
    encoder = mavlink.MAVLink(writer, srcSystem=1, srcComponent=191)
    selected = lambda: encoder.set_mode_send(
        1, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4
    )

    gate.begin_finalizing()

    with pytest.raises(FlightAuthorityLost, match="finalizing"):
        selected()
    assert delegate.queue.empty()
