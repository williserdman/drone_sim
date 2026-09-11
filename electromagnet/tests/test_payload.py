from __future__ import annotations

from dataclasses import replace

import pytest

from drone_sim_electromagnet.payload import (
    PayloadAuthority,
    PayloadDecision,
    PayloadRequest,
    PayloadWorld,
    PickupZone,
)


RUN_ID = "00000000-0000-4000-8000-000000000001"


def authority() -> PayloadAuthority:
    return PayloadAuthority(
        run_id=RUN_ID,
        pickup_zones={
            "WA": PickupZone(-45.72, -9.144, 6.096, 6.096),
            "WM": PickupZone(-45.72, 9.144, 6.096, 6.096),
        },
        payload_zones={2: None, 3: "WA", 4: "WM"},
        payload_capacity=1,
        max_center_error_m=0.075,
    )


def centered_state() -> PayloadWorld:
    return PayloadWorld(
        vehicle_xy=(-45.72, -9.144),
        vehicle_grounded=True,
        payload_xy=(-45.72, -9.144),
        payload_grounded=True,
        attached_id=None,
    )


def attach_request(marker: int = 3, command_id: str = "run:3:attach:1") -> PayloadRequest:
    return PayloadRequest(RUN_ID, marker, "attach", command_id)


@pytest.mark.parametrize(
    ("mutation", "code"),
    [
        (lambda state: replace(state, vehicle_grounded=False), "NOT_LANDED"),
        (
            lambda state: replace(state, vehicle_xy=(-40.0, -9.144)),
            "OUTSIDE_PICKUP_ZONE",
        ),
        (
            lambda state: replace(state, payload_grounded=False),
            "PAYLOAD_NOT_GROUNDED",
        ),
        (lambda state: replace(state, attached_id=2), "CAPACITY_OCCUPIED"),
        (
            lambda state: replace(state, vehicle_xy=(-45.60, -9.144)),
            "CENTER_ERROR",
        ),
    ],
)
def test_attach_rejections_have_no_command(mutation, code: str) -> None:
    decision = authority().decide(mutation(centered_state()), attach_request())
    assert decision == PayloadDecision(False, code, None)


def test_centered_grounded_attach_emits_one_wire_command() -> None:
    decision = authority().decide(centered_state(), attach_request())
    assert decision == PayloadDecision(
        True,
        "OK",
        "payload-command-v1|run:3:attach:1|attach",
    )


def test_attach_rejects_stale_run_unknown_marker_and_wrong_payload_zone() -> None:
    stale = replace(attach_request(), run_id="00000000-0000-4000-8000-000000000002")
    assert authority().decide(centered_state(), stale) == PayloadDecision(
        False, "STALE_RUN", None
    )
    assert authority().decide(centered_state(), attach_request(9)) == PayloadDecision(
        False, "UNKNOWN_MARKER", None
    )
    wrong_zone = replace(centered_state(), payload_xy=(-45.72, 9.144))
    assert authority().decide(wrong_zone, attach_request()) == PayloadDecision(
        False, "WRONG_PICKUP_ZONE", None
    )


def test_center_tolerance_boundary_is_inclusive() -> None:
    edge = replace(centered_state(), vehicle_xy=(-45.645, -9.144))
    assert authority().decide(edge, attach_request()).accepted is True


def test_release_requires_requested_marker_and_emits_detach_wire() -> None:
    request = PayloadRequest(RUN_ID, 3, "release", "run:3:release:1")
    assert authority().decide(
        replace(centered_state(), attached_id=2), request
    ) == PayloadDecision(False, "NOT_ATTACHED", None)
    assert authority().decide(
        replace(centered_state(), attached_id=3), request
    ) == PayloadDecision(
        True,
        "OK",
        "payload-command-v1|run:3:release:1|detach",
    )


def test_authority_revalidates_reused_command_without_retaining_history() -> None:
    policy = authority()
    request = attach_request()
    assert policy.decide(centered_state(), request).wire_command is not None
    assert policy.decide(
        replace(centered_state(), attached_id=2), request
    ) == PayloadDecision(
        False, "CAPACITY_OCCUPIED", None
    )


@pytest.mark.parametrize("command_id", ["", "bad/id", "x" * 129])
def test_invalid_command_identity_never_reaches_the_coordinator(command_id: str) -> None:
    assert authority().decide(
        centered_state(), attach_request(command_id=command_id)
    ) == PayloadDecision(False, "INVALID_COMMAND_ID", None)
