from __future__ import annotations

import json
from pathlib import Path
from uuid import UUID

import pytest
from artifacts.qgc_operator import MavlinkIdentity

from orchestration.host_integration import ControllerRegistration
from orchestration.qgc_actions import QgcActionStore


class Lease:
    def __init__(self, registration: ControllerRegistration | None = None) -> None:
        self.calls: list[tuple[object, bool]] = []
        self.registration = registration or ControllerRegistration(None, False)

    def register_controller(self, identity, *, pristine: bool):
        self.calls.append((identity, pristine))
        return self.registration


def test_pristine_enrollment_creates_identity_and_zero_token_ledger(tmp_path: Path) -> None:
    from orchestration.controller_identity import load_or_enroll_identity

    state = tmp_path / "qgc"
    lease = Lease()

    enrollment = load_or_enroll_identity(state, lease)

    assert enrollment.registration == ControllerRegistration(None, False)
    assert lease.calls == [(enrollment.identity, True)]
    assert enrollment.identity.installation_id != UUID(int=0)
    assert enrollment.identity.ledger_id != UUID(int=0)
    ledger = json.loads((state / "attempt-ledger.json").read_text())
    assert ledger == {"schema_version": 1, "last_allocated_token": 0}
    assert (state / "initialized").read_bytes() == b"qgc_actions_v1\n"
    assert not list(state.glob("*.tmp-*"))

    store = QgcActionStore(state, MavlinkIdentity(255, 190))
    assert store.allocate("00000000-0000-4000-8000-000000000001").descriptor.attempt_token == 1


def test_initialized_state_reuses_identity_without_resetting_counter(tmp_path: Path) -> None:
    from orchestration.controller_identity import load_or_enroll_identity

    state = tmp_path / "qgc"
    first_lease = Lease()
    first = load_or_enroll_identity(state, first_lease)
    store = QgcActionStore(state, MavlinkIdentity(255, 190))
    store.allocate("00000000-0000-4000-8000-000000000001")

    second_lease = Lease(ControllerRegistration(None, False))
    second = load_or_enroll_identity(state, second_lease)

    assert second.identity == first.identity
    assert second_lease.calls == [(first.identity, False)]
    assert store.allocate("00000000-0000-4000-8000-000000000002").descriptor.attempt_token == 2


@pytest.mark.parametrize("missing", ["controller-identity.json", "attempt-ledger.json", "initialized"])
def test_partial_initialized_state_fails_closed(tmp_path: Path, missing: str) -> None:
    from orchestration.controller_identity import load_or_enroll_identity

    state = tmp_path / "qgc"
    load_or_enroll_identity(state, Lease())
    (state / missing).unlink()
    lease = Lease()

    with pytest.raises(RuntimeError, match="recovery"):
        load_or_enroll_identity(state, lease)

    assert lease.calls == []


def test_registration_failure_leaves_explicit_initialized_state(tmp_path: Path) -> None:
    from orchestration.controller_identity import load_or_enroll_identity

    class FailingLease(Lease):
        def register_controller(self, identity, *, pristine: bool):
            super().register_controller(identity, pristine=pristine)
            raise RuntimeError("native registration failed")

    state = tmp_path / "qgc"
    with pytest.raises(RuntimeError, match="native registration failed"):
        load_or_enroll_identity(state, FailingLease())

    assert (state / "controller-identity.json").is_file()
    assert (state / "attempt-ledger.json").is_file()
    retry = Lease()
    load_or_enroll_identity(state, retry)
    assert retry.calls[0][1] is False
