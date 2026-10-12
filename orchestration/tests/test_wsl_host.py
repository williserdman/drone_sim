from __future__ import annotations

import hashlib
from pathlib import Path
from types import SimpleNamespace
from uuid import UUID

import pytest

from artifacts.qgc_operator import MavlinkIdentity
from orchestration._adapters.wsl_host import WslHost
from orchestration.host_config import HostConfig, HostFacts
from orchestration.host_integration import (
    ControllerIdentity,
    HostHelperInfo,
    HostTransportError,
    PublicationReceipt,
    QgcInstallation,
)
from orchestration.qgc_actions import QgcActionMaterial
from orchestration.windows_helper import WindowsHelperError


RUN_ID = "11111111-1111-4111-8111-111111111111"
INSTALLATION = QgcInstallation(
    executable=r"C:\Program Files\QGroundControl\bin\QGroundControl.exe",
    settings_file=r"C:\Users\pilot\AppData\Roaming\QGroundControl\QGroundControl.ini",
    save_path=r"C:\Users\pilot\Documents\QGroundControl",
    version="5.0.8",
    source=MavlinkIdentity(255, 190),
)
RECEIPT = PublicationReceipt(
    native_path=r"C:\Users\pilot\Documents\QGroundControl\MavlinkActions\drone-sim-11111111-1111-4111-8111-111111111111.json",
    filename=f"drone-sim-{RUN_ID}.json",
    sha256="a" * 64,
)


class FakeHelper:
    def __init__(self, responses: dict[str, object] | None = None) -> None:
        self.responses = responses or {}
        self.requests: list[tuple[str, dict[str, object]]] = []
        self.sent: list[bytes] = []
        self.received: list[bytes] = []
        self.closed = False
        self.info = HostHelperInfo("1", "a" * 64)

    def request(self, operation: str, payload: dict[str, object], *, timeout_s: float = 10.0):
        del timeout_s
        self.requests.append((operation, payload))
        value = self.responses.get(operation, {})
        if isinstance(value, Exception):
            raise value
        return value

    def send(self, payload: bytes) -> None:
        if self.closed:
            raise WindowsHelperError("helper_closed", "closed")
        self.sent.append(payload)

    def receive(self, timeout_s: float) -> bytes | None:
        del timeout_s
        if self.closed:
            raise WindowsHelperError("helper_closed", "closed")
        return self.received.pop(0) if self.received else None

    def close(self) -> None:
        self.closed = True


def _facts() -> HostFacts:
    return HostFacts("wsl2", "amd64", "linux/amd64", "docker-desktop")


def _host(tmp_path: Path, helpers: list[FakeHelper], **kwargs) -> WslHost:
    def factory(*, session_id: str) -> FakeHelper:
        UUID(session_id)
        return helpers.pop(0)

    return WslHost(
        tmp_path,
        HostConfig("wsl2", INSTALLATION.executable, INSTALLATION.settings_file),
        _facts(),
        helper_factory=factory,
        home=tmp_path / "home",
        environ={"WSL_DISTRO_NAME": "Ubuntu-24.04"},
        **kwargs,
    )


def test_disabled_interop_or_policy_is_actionable(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    helper = FakeHelper()
    host = _host(tmp_path, [helper])
    monkeypatch.setattr(
        host,
        "_start_helper",
        lambda: (_ for _ in ()).throw(
            WindowsHelperError("wsl_interop_unavailable", "powershell.exe is unavailable")
        ),
    )
    flight_commands: list[str] = []

    issues = host.preflight([])

    assert len(issues) == 1
    issue = issues[0]
    assert issue.code in {"wsl_interop_unavailable", "helper_policy_blocked"}
    assert flight_commands == []


def test_preflight_uses_a_short_lived_probe(tmp_path: Path) -> None:
    helper = FakeHelper({"probe": {"helper_version": "1", "source_sha256": "a" * 64}})
    host = _host(tmp_path, [helper])

    assert host.preflight([]) == ()
    assert helper.requests == [("probe", {})]
    assert helper.closed is True


def test_helper_info_uses_active_lease_identity_or_short_lived_probe(tmp_path: Path) -> None:
    active = FakeHelper()
    probe = FakeHelper({"probe": {"helper_version": "1", "source_sha256": "a" * 64}})
    host = _host(tmp_path, [active, probe])

    with host.acquire_native_lease():
        assert host.helper_info() == active.info
        assert probe.requests == []
        with pytest.raises(RuntimeError, match="already has an active"):
            with host.acquire_native_lease():
                pass
        assert host.helper_info() == active.info
    assert host.helper_info() == probe.info
    assert probe.requests == [("probe", {})]
    assert probe.closed is True


def test_inspection_preserves_native_windows_paths(tmp_path: Path) -> None:
    helper = FakeHelper(
        {
            "inspect_qgc": {
                "executable": INSTALLATION.executable,
                "settings_file": INSTALLATION.settings_file,
                "save_path": INSTALLATION.save_path,
                "version": "5.0.8",
                "source": {"system_id": 255, "component_id": 190},
            }
        }
    )
    host = _host(tmp_path, [helper])

    assert host.inspect_qgc() == INSTALLATION
    assert helper.requests == [
        (
            "inspect_qgc",
            {
                "executable": INSTALLATION.executable,
                "settings_file": INSTALLATION.settings_file,
            },
        )
    ]
    assert helper.closed is True


def test_lease_keeps_one_wire_session_for_run_operations(tmp_path: Path) -> None:
    helper = FakeHelper()
    host = _host(tmp_path, [helper])
    identity = ControllerIdentity(
        UUID("22222222-2222-4222-8222-222222222222"),
        UUID("33333333-3333-4333-8333-333333333333"),
    )

    with host.acquire_native_lease() as lease:
        lease.register_controller(identity, pristine=True)
        lease.begin_run(RUN_ID)
        lease.finish_run(RUN_ID, verified_cleanup=True)
        assert helper.closed is False

    operations = [operation for operation, _ in helper.requests]
    assert operations == [
        "acquire_lease",
        "register_controller",
        "begin_run",
        "finish_run",
        "release_lease",
    ]
    assert helper.closed is True


def test_recovery_uses_replacement_helper_after_old_helper_exits(
    tmp_path: Path,
) -> None:
    action_bytes = b'{"run":"recovery"}'
    digest = hashlib.sha256(action_bytes).hexdigest()
    receipt = PublicationReceipt(
        native_path=rf"C:\recovery\drone-sim-{RUN_ID}.json",
        filename=f"drone-sim-{RUN_ID}.json",
        sha256=digest,
    )
    first = FakeHelper(
        {
            "publish_actions": {
                "native_path": receipt.native_path,
                "filename": receipt.filename,
                "sha256": receipt.sha256,
            }
        }
    )
    replacement = FakeHelper()
    host = _host(tmp_path, [first, replacement])
    identity = ControllerIdentity(
        UUID("22222222-2222-4222-8222-222222222222"),
        UUID("33333333-3333-4333-8333-333333333333"),
    )
    material = SimpleNamespace(
        published_name=receipt.filename,
        action_bytes=action_bytes,
    )

    with host.acquire_native_lease() as lease:
        lease.register_controller(identity, pristine=True)
        lease.begin_run(RUN_ID)
        assert host.publish_actions(material, INSTALLATION, lease) == receipt

    assert first.closed is True
    assert [operation for operation, _ in first.requests] == [
        "acquire_lease",
        "register_controller",
        "begin_run",
        "publish_actions",
        "release_lease",
    ]

    with host.acquire_native_lease() as replacement_lease:
        assert first.closed is True
        replacement_lease.register_controller(identity, pristine=False)
        replacement_lease.verify_resume(RUN_ID, receipt)

    assert [operation for operation, _ in replacement.requests] == [
        "acquire_lease",
        "register_controller",
        "verify_resume",
        "release_lease",
    ]
    assert first.requests[1][1] == {
        "installation_id": "22222222-2222-4222-8222-222222222222",
        "ledger_id": "33333333-3333-4333-8333-333333333333",
        "distro_name": "Ubuntu-24.04",
        "pristine": True,
    }
    assert replacement.requests[1][1] == {
        **first.requests[1][1],
        "pristine": False,
    }
    assert replacement.requests[2][1] == {
        "run_id": RUN_ID,
        "publication": {
            "native_path": receipt.native_path,
            "filename": receipt.filename,
            "sha256": receipt.sha256,
        },
    }


def test_publication_uses_lease_helper_and_exact_material(tmp_path: Path) -> None:
    material = SimpleNamespace(
        published_name=RECEIPT.filename,
        action_bytes=b"{\"binary\":\"\\u0000\"}",
    )
    digest = __import__("hashlib").sha256(material.action_bytes).hexdigest()
    expected = PublicationReceipt(RECEIPT.native_path, RECEIPT.filename, digest)
    helper = FakeHelper({"publish_actions": {"native_path": expected.native_path, "filename": expected.filename, "sha256": expected.sha256}})
    host = _host(tmp_path, [helper])

    with host.acquire_native_lease() as lease:
        assert host.publish_actions(material, INSTALLATION, lease) == expected

    operation, payload = helper.requests[1]
    assert operation == "publish_actions"
    assert payload["directory"] == INSTALLATION.save_path + r"\MavlinkActions"
    assert payload["filename"] == RECEIPT.filename
    assert payload["sha256"] == digest
    assert payload["data_base64"] == "eyJiaW5hcnkiOiJcdTAwMDAifQ=="


def test_transport_close_does_not_close_lease_helper(tmp_path: Path) -> None:
    helper = FakeHelper({"open_transport": {"local_port": 49152, "peer": "127.0.0.1:14550"}})
    helper.received.append(b"reply")
    host = _host(tmp_path, [helper])

    with host.acquire_native_lease() as lease:
        transport = host.open_datagrams(lease, RUN_ID, RECEIPT)
        transport.send(b"request")
        assert transport.receive(0.2) == b"reply"
        transport.close()
        transport.close()
        assert helper.closed is False
        lease.finish_run(RUN_ID, verified_cleanup=True)

    assert helper.sent == [b"request"]
    assert [operation for operation, _ in helper.requests].count("close_transport") == 1
    assert helper.closed is True


def test_transport_maps_helper_failure_and_stays_closed(tmp_path: Path) -> None:
    helper = FakeHelper({"open_transport": {"local_port": 49152, "peer": "127.0.0.1:14550"}})
    host = _host(tmp_path, [helper])

    with host.acquire_native_lease() as lease:
        transport = host.open_datagrams(lease, RUN_ID, RECEIPT)
        helper.closed = True
        with pytest.raises(HostTransportError, match="closed"):
            transport.send(b"x")


def test_host_rejects_a_lease_owned_by_another_adapter(tmp_path: Path) -> None:
    first_helper = FakeHelper()
    second_helper = FakeHelper()
    first = _host(tmp_path / "first", [first_helper])
    second = _host(tmp_path / "second", [second_helper])

    with first.acquire_native_lease() as lease:
        with pytest.raises(RuntimeError, match="its active native lease"):
            second.open_datagrams(lease, RUN_ID, RECEIPT)


def test_launch_operations_use_short_lived_helpers(tmp_path: Path) -> None:
    qgc = FakeHelper({"open_qgc": {"opened": True, "detail": "started"}})
    browser = FakeHelper({"open_browser": {"opened": True, "detail": "opened"}})
    host = _host(tmp_path, [qgc, browser])

    assert host.open_qgc(INSTALLATION).opened is True
    assert host.open_browser("http://127.0.0.1:8080/").opened is True
    assert qgc.closed is True
    assert browser.closed is True
