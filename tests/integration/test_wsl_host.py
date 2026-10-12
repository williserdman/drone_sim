from __future__ import annotations

import base64
import json
import os
from pathlib import Path
import platform
import select
import shutil
import subprocess
from typing import IO, Mapping, cast
from uuid import UUID, uuid4

import pytest

from artifacts.qgc_operator import MavlinkIdentity
from orchestration._adapters.wsl_host import WslHost
from orchestration.host_config import HostConfig, HostFacts
from orchestration.host_integration import ControllerIdentity, QgcInstallation
from orchestration.qgc_actions import QgcActionStore
from orchestration.windows_helper import WindowsHelper, WindowsHelperError


_IS_WSL2 = (
    platform.system() == "Linux"
    and "microsoft-standard" in platform.release().lower()
)
_HAS_NATIVE_INTEROP = (
    _IS_WSL2
    and shutil.which("powershell.exe") is not None
    and shutil.which("wslpath") is not None
)
_REQUIRES_NATIVE_WSL = pytest.mark.skipif(
    not _HAS_NATIVE_INTEROP,
    reason="requires WSL2 interoperability with Windows PowerShell 5.1",
)


def _windows_path(path: Path) -> str:
    return subprocess.run(
        ["wslpath", "-w", str(path.resolve())],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=10,
    ).stdout.strip()


def _read_json_line(stream: IO[bytes], timeout_s: float) -> Mapping[str, object]:
    readable, _, _ = select.select([stream], [], [], timeout_s)
    if not readable:
        raise TimeoutError("Windows UDP peer did not produce a protocol line")
    line = stream.readline()
    if not line:
        raise EOFError("Windows UDP peer exited before producing a protocol line")
    value = json.loads(line.decode("utf-8"))
    if not isinstance(value, dict):
        raise RuntimeError("Windows UDP peer protocol line must be an object")
    return value


class _WindowsUdpPeer:
    def __init__(self, repository: Path, distro_name: str) -> None:
        fixture = repository / "tests/windows/wsl_integration_peer.ps1"
        self.test_id = uuid4().hex
        self.process = subprocess.Popen(
            [
                "powershell.exe",
                "-NoLogo",
                "-NoProfile",
                "-NonInteractive",
                "-File",
                _windows_path(fixture),
                "-TestId",
                self.test_id,
                "-DistroName",
                distro_name,
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        if (
            self.process.stdin is None
            or self.process.stdout is None
            or self.process.stderr is None
        ):
            raise AssertionError("Windows UDP peer pipes were not created")
        self._stdin: IO[bytes] = self.process.stdin
        self._stdout: IO[bytes] = self.process.stdout
        self._stderr: IO[bytes] = self.process.stderr
        try:
            self.info = _read_json_line(self._stdout, 15.0)
        except BaseException:
            self._stop()
            raise RuntimeError(
                "Windows UDP peer failed to start: " + self._diagnostics()
            )

    def request(self, operation: str, **payload: object) -> Mapping[str, object]:
        request_id = str(uuid4())
        frame = json.dumps(
            {"request_id": request_id, "operation": operation, **payload},
            separators=(",", ":"),
        ).encode("utf-8") + b"\n"
        self._stdin.write(frame)
        self._stdin.flush()
        response = _read_json_line(self._stdout, 15.0)
        if response.get("request_id") != request_id:
            raise RuntimeError("Windows UDP peer response ID mismatched")
        if response.get("ok") is not True:
            raise RuntimeError(str(response.get("error", "Windows UDP peer failed")))
        result = response.get("result")
        if not isinstance(result, dict):
            raise RuntimeError("Windows UDP peer result must be an object")
        return result

    def receive(self) -> tuple[bytes, str, int]:
        result = self.request("receive")
        encoded = result.get("data_base64")
        address = result.get("sender_address")
        port = result.get("sender_port")
        if (
            not isinstance(encoded, str)
            or not isinstance(address, str)
            or type(port) is not int
        ):
            raise RuntimeError("Windows UDP peer returned an invalid datagram")
        return base64.b64decode(encoded, validate=True), address, cast(int, port)

    def send(self, payload: bytes, *, spoof: bool = False) -> None:
        self.request(
            "spoof" if spoof else "send",
            data_base64=base64.b64encode(payload).decode("ascii"),
        )

    def terminate_process(self, process_id: int, process_start_time_utc: str) -> None:
        self.request(
            "terminate_process",
            process_id=process_id,
            process_start_time_utc=process_start_time_utc,
        )

    def close(self) -> None:
        if self.process.poll() is None:
            try:
                self.request("shutdown")
            finally:
                self._stop()
        if self.process.returncode != 0:
            raise RuntimeError(
                f"Windows UDP peer exited {self.process.returncode}: {self._diagnostics()}"
            )

    def _stop(self) -> None:
        try:
            self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)

    def _diagnostics(self) -> str:
        if self.process.poll() is None:
            return "process is still running"
        return self._stderr.read().decode("utf-8", errors="replace").strip()


def _pass_test_environment(
    monkeypatch: pytest.MonkeyPatch, peer: _WindowsUdpPeer
) -> None:
    values = {
        "DRONE_SIM_WINDOWS_NATIVE_TEST_MODE": "1",
        "DRONE_SIM_TEST_LOCALAPPDATA": peer.info["temp_root"],
        "DRONE_SIM_TEST_LXSS_ROOT": peer.info["registry_root"],
        "DRONE_SIM_TEST_MUTEX_SUFFIX": peer.test_id,
    }
    for name, value in values.items():
        if not isinstance(value, str):
            raise RuntimeError(f"Windows UDP peer field for {name} must be a string")
        monkeypatch.setenv(name, value)
    inherited = [entry for entry in os.environ.get("WSLENV", "").split(":") if entry]
    inherited_names = {entry.split("/", 1)[0] for entry in inherited}
    inherited.extend(name for name in values if name not in inherited_names)
    monkeypatch.setenv("WSLENV", ":".join(inherited))


@_REQUIRES_NATIVE_WSL
def test_native_wsl_adapter_pipe_udp_eof_and_process_lifecycle(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    repository = Path(__file__).resolve().parents[2]
    distro_name = os.environ.get("WSL_DISTRO_NAME", "")
    if not distro_name:
        pytest.fail("WSL_DISTRO_NAME is required for the native WSL integration gate")
    peer = _WindowsUdpPeer(repository, distro_name)
    helpers: list[WindowsHelper] = []

    def start_helper(*, session_id: str) -> WindowsHelper:
        helper = WindowsHelper.start(session_id=session_id)
        helpers.append(helper)
        return helper

    try:
        version = peer.info.get("powershell_version")
        assert isinstance(version, str) and version.startswith("5.1.")
        assert peer.info["filesystem"] == "NTFS"
        _pass_test_environment(monkeypatch, peer)
        host = WslHost(
            tmp_path,
            HostConfig("wsl2", None, None),
            HostFacts("wsl2", "amd64", "linux/amd64", "docker-desktop"),
            environ=os.environ,
            helper_factory=start_helper,
        )
        info = host.helper_info()
        assert info.version == "1"
        assert len(info.source_sha256) == 64
        assert helpers[0]._process.poll() is not None

        run_id = str(uuid4())
        material = QgcActionStore(
            tmp_path / "qgc-actions", MavlinkIdentity(255, 190)
        ).allocate(run_id)
        installation = QgcInstallation(
            executable=str(peer.info["temp_root"]) + r"\QGroundControl.exe",
            settings_file=str(peer.info["temp_root"]) + r"\QGroundControl.ini",
            save_path=str(peer.info["save_path"]),
            version="5.0.8",
            source=MavlinkIdentity(255, 190),
        )
        identity = ControllerIdentity(
            UUID("22222222-2222-4222-8222-222222222222"),
            UUID("33333333-3333-4333-8333-333333333333"),
        )

        with host.acquire_native_lease() as lease:
            lease.register_controller(identity, pristine=True)
            lease.begin_run(run_id)
            receipt = host.publish_actions(material, installation, lease)
            assert receipt.sha256 == material.descriptor.actions_sha256
            transport = host.open_datagrams(lease, run_id, receipt)
            try:
                outbound = b"\x00\xfd\xfeWSL-to-Windows\r\n\xff"
                transport.send(outbound)
                received, address, sender_port = peer.receive()
                assert received == outbound
                assert address == "127.0.0.1"
                assert 0 < sender_port < 65536

                peer.send(b"spoofed", spoof=True)
                assert transport.receive(0.25) is None
                inbound = b"\xff\x00Windows-to-WSL\xfe\xfd"
                peer.send(inbound)
                assert transport.receive(5.0) == inbound
            finally:
                transport.close()
            lease.finish_run(run_id, verified_cleanup=True)

        active_process = helpers[-1]._process
        assert active_process.poll() is not None

        eof_helper = WindowsHelper.start(session_id=str(uuid4()))
        eof_process = eof_helper._process
        try:
            lease_result = eof_helper.request("acquire_lease", {})
            process_id = lease_result.get("helper_process_id")
            process_start = lease_result.get("helper_process_start_time_utc")
            assert type(process_id) is int
            assert isinstance(process_start, str)
            peer.terminate_process(cast(int, process_id), process_start)
            with pytest.raises(WindowsHelperError, match="EOF|exited"):
                eof_helper.request("probe", {}, timeout_s=5.0)
            assert eof_process.wait(timeout=5) is not None
        finally:
            eof_helper.close()
    finally:
        for helper in helpers:
            helper.close()
        peer.close()
