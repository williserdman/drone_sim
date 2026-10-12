from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

from orchestration.host_integration import ControllerRegistration


class Lease:
    def __init__(
        self,
        events: list[str],
        registration: ControllerRegistration = ControllerRegistration(None, False),
    ) -> None:
        self.events = events
        self.registration = registration

    def register_controller(self, identity, *, pristine: bool):
        self.events.append(f"register:{pristine}")
        return self.registration

    def begin_run(self, run_id: str) -> None:
        self.events.append(f"begin:{run_id}")

    def verify_resume(self, run_id, publication) -> None:
        self.events.append(f"resume:{run_id}")

    def finish_run(self, run_id: str, *, verified_cleanup: bool) -> None:
        self.events.append(f"finish:{run_id}:{verified_cleanup}")


class Host:
    def __init__(
        self,
        state_directory: Path,
        events: list[str],
        *,
        kind: str = "linux",
        registration: ControllerRegistration = ControllerRegistration(None, False),
        fail_acquire: bool = False,
    ) -> None:
        self.state_directory = state_directory
        self.facts = SimpleNamespace(kind=kind)
        self.events = events
        self.lease = Lease(events, registration)
        self.fail_acquire = fail_acquire

    @contextmanager
    def acquire_native_lease(self):
        self.events.append("native-enter")
        if self.fail_acquire:
            raise RuntimeError("native lease unavailable")
        try:
            yield self.lease
        finally:
            self.events.append("native-exit")


def test_workstation_is_reentrant_only_for_same_owner_and_blocks_other_process(
    tmp_path: Path,
) -> None:
    from orchestration.workstation import Workstation

    lock_path = tmp_path / "workstation.lock"
    workstation = Workstation(Host(tmp_path / "state", []), lock_path=lock_path)
    with workstation.hold() as lease:
        assert workstation.owned() is lease
        with workstation.hold(allow_nested=True) as nested:
            assert nested is lease
            code = (
                "from contextlib import contextmanager\n"
                "from pathlib import Path\n"
                "from types import SimpleNamespace\n"
                "from orchestration.workstation import Workstation\n"
                "class H:\n"
                "  facts=SimpleNamespace(kind='linux')\n"
                "  state_directory=Path('/unused')\n"
                "  @contextmanager\n"
                "  def acquire_native_lease(self): yield object()\n"
                f"with Workstation(H(), lock_path=Path({str(lock_path)!r})).hold(): pass\n"
            )
            result = subprocess.run(
                [sys.executable, "-c", code], capture_output=True, text=True
            )
            assert result.returncode != 0
            assert "already owns" in result.stderr
    with pytest.raises(RuntimeError, match="not owned"):
        workstation.owned()


def test_native_acquisition_and_reverse_release_leave_posix_reusable(tmp_path: Path) -> None:
    from orchestration.workstation import Workstation

    events: list[str] = []
    workstation = Workstation(Host(tmp_path / "state", events), lock_path=tmp_path / "lock")

    with workstation.hold():
        events.append("body")

    assert events == ["native-enter", "body", "native-exit"]
    with workstation.hold():
        pass


def test_native_acquisition_failure_releases_posix_lease(tmp_path: Path) -> None:
    from orchestration.workstation import Workstation

    lock_path = tmp_path / "lock"
    with pytest.raises(RuntimeError, match="native lease unavailable"):
        with Workstation(
            Host(tmp_path / "state", [], fail_acquire=True), lock_path=lock_path
        ).hold():
            pass
    with Workstation(Host(tmp_path / "state", []), lock_path=lock_path).hold():
        pass


def test_wsl_enrolls_once_and_nested_scope_reuses_exact_lease(tmp_path: Path) -> None:
    from orchestration.workstation import Workstation

    events: list[str] = []
    workstation = Workstation(
        Host(tmp_path / "state", events, kind="wsl2"), lock_path=tmp_path / "lock"
    )

    with workstation.hold() as lease:
        with workstation.hold(allow_nested=True) as nested:
            assert nested is lease

    assert events == ["native-enter", "register:True", "native-exit"]


def test_retained_native_owner_blocks_normal_entry_and_allows_exact_cleanup(
    tmp_path: Path,
) -> None:
    from orchestration.workstation import Workstation

    run_id = "00000000-0000-4000-8000-000000000111"
    host = Host(
        tmp_path / "state",
        [],
        kind="wsl2",
        registration=ControllerRegistration(run_id, True),
    )
    workstation = Workstation(host, lock_path=tmp_path / "lock")

    with pytest.raises(RuntimeError, match=run_id):
        with workstation.hold():
            pass
    with pytest.raises(RuntimeError, match="does not match"):
        with workstation.hold(
            recovery_run_id="00000000-0000-4000-8000-000000000112"
        ):
            pass
    with workstation.hold(recovery_run_id=run_id) as lease:
        assert lease is host.lease


def test_recovery_without_active_run_fails_closed(tmp_path: Path) -> None:
    from orchestration.workstation import Workstation

    workstation = Workstation(
        Host(
            tmp_path / "state",
            [],
            kind="wsl2",
            registration=ControllerRegistration(None, True),
        ),
        lock_path=tmp_path / "lock",
    )
    with pytest.raises(RuntimeError, match="inspection"):
        with workstation.hold():
            pass


def test_persistent_active_owner_survives_mutex_release_until_exact_cleanup(
    tmp_path: Path,
) -> None:
    from orchestration.workstation import Workstation

    run_id = "00000000-0000-4000-8000-000000000301"
    events: list[str] = []

    class PersistentLease(Lease):
        def register_controller(self, identity, *, pristine: bool):
            super().register_controller(identity, pristine=pristine)
            return ControllerRegistration(self.active_run_id, self.active_run_id is not None)

        def begin_run(self, value: str) -> None:
            self.active_run_id = value

        def finish_run(self, value: str, *, verified_cleanup: bool) -> None:
            assert value == self.active_run_id
            assert verified_cleanup is True
            self.active_run_id = None

    host = Host(tmp_path / "state", events, kind="wsl2")
    host.lease = PersistentLease(events)
    host.lease.active_run_id = None
    lock_path = tmp_path / "lock"

    with Workstation(host, lock_path=lock_path).hold() as lease:
        lease.begin_run(run_id)

    with pytest.raises(RuntimeError, match=run_id):
        with Workstation(host, lock_path=lock_path).hold():
            pass
    with Workstation(host, lock_path=lock_path).hold(
        recovery_run_id=run_id
    ) as lease:
        lease.finish_run(run_id, verified_cleanup=True)
    with Workstation(host, lock_path=lock_path).hold():
        pass


def test_worktrees_share_monotonic_ledger(tmp_path: Path) -> None:
    from artifacts.qgc_operator import MavlinkIdentity
    from orchestration.qgc_actions import QgcActionStore

    shared_state = tmp_path / "native-state"
    first_worktree = QgcActionStore(shared_state, MavlinkIdentity(255, 190))
    second_worktree = QgcActionStore(shared_state, MavlinkIdentity(255, 190))

    first_token = first_worktree.allocate(
        "00000000-0000-4000-8000-000000000401"
    ).descriptor.attempt_token
    second_token = second_worktree.allocate(
        "00000000-0000-4000-8000-000000000402"
    ).descriptor.attempt_token

    assert second_token == first_token + 1


def test_lock_rejects_symlink_without_touching_target(tmp_path: Path) -> None:
    from orchestration.workstation import Workstation

    target = tmp_path / "target"
    target.write_text("unchanged")
    link = tmp_path / "lock"
    link.symlink_to(target)
    with pytest.raises(OSError):
        with Workstation(Host(tmp_path / "state", []), lock_path=link).hold():
            pass
    assert target.read_text() == "unchanged"
