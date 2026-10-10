import copy
from dataclasses import FrozenInstanceError

import pytest

from artifacts.qgc_operator import (
    MavlinkIdentity,
    QgcOperatorDescriptor,
    parse_qgc_operator_descriptor,
    qgc_operator_descriptor_document,
)


def _document() -> dict[str, object]:
    return {
        "kind": "qgc_actions_v1",
        "start_command": 31020,
        "abort_command": 31021,
        "attempt_token": 7,
        "start_request_id": "11111111-1111-4111-8111-111111111111",
        "abort_request_id": "22222222-2222-4222-8222-222222222222",
        "qgc_source": {"system_id": 255, "component_id": 190},
        "companion": {"system_id": 1, "component_id": 191},
        "flight_controller": {"system_id": 1, "component_id": 1},
        "actions_path": "configuration/qgc-actions.json",
        "actions_sha256": "a" * 64,
    }


def test_operator_descriptor_round_trip_preserves_protocol_values() -> None:
    document = _document()

    descriptor = parse_qgc_operator_descriptor(document)

    assert descriptor == QgcOperatorDescriptor(
        kind="qgc_actions_v1",
        start_command=31020,
        abort_command=31021,
        attempt_token=7,
        start_request_id="11111111-1111-4111-8111-111111111111",
        abort_request_id="22222222-2222-4222-8222-222222222222",
        qgc_source=MavlinkIdentity(255, 190),
        companion=MavlinkIdentity(1, 191),
        flight_controller=MavlinkIdentity(1, 1),
        actions_path="configuration/qgc-actions.json",
        actions_sha256="a" * 64,
    )
    assert qgc_operator_descriptor_document(descriptor) == document
    with pytest.raises(FrozenInstanceError):
        descriptor.attempt_token = 8  # type: ignore[misc]


@pytest.mark.parametrize(
    "mutate, message",
    [
        (lambda value: value.update(kind="qgc_actions_v2"), "kind"),
        (lambda value: value.update(start_command=31019), "start_command"),
        (lambda value: value.update(abort_command=True), "abort_command"),
        (lambda value: value.update(attempt_token=0), "attempt_token"),
        (lambda value: value.update(attempt_token=16_777_216), "attempt_token"),
        (lambda value: value.update(start_request_id="not-a-uuid"), "start_request_id"),
        (
            lambda value: value.update(
                abort_request_id=value["start_request_id"]
            ),
            "distinct",
        ),
        (lambda value: value.update(qgc_source={"system_id": True, "component_id": 190}), "system_id"),
        (lambda value: value.update(companion={"system_id": 1, "component_id": 256}), "component_id"),
        (lambda value: value.update(companion={"system_id": 1, "component_id": 190}), "companion"),
        (lambda value: value.update(flight_controller={"system_id": 2, "component_id": 1}), "flight_controller"),
        (lambda value: value.update(flight_controller=value["companion"]), "distinct"),
        (lambda value: value.update(actions_path="/tmp/qgc-actions.json"), "actions_path"),
        (lambda value: value.update(actions_path="configuration/../qgc-actions.json"), "actions_path"),
        (lambda value: value.update(actions_sha256="A" * 64), "actions_sha256"),
        (lambda value: value.update(extra=True), "keys"),
    ],
)
def test_operator_descriptor_rejects_values_outside_frozen_protocol(
    mutate, message: str,
) -> None:
    document = _document()
    mutate(document)

    with pytest.raises(ValueError, match=message):
        parse_qgc_operator_descriptor(copy.deepcopy(document))
