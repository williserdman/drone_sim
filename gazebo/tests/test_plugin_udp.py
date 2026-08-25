"""Real Gazebo-plugin UDP regressions for one-for-one lockstep packets."""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time
from uuid import uuid4

import pytest


pytestmark = pytest.mark.skipif(
    os.environ.get("DRONE_SIM_REAL_PLUGIN_TEST") != "1",
    reason="requires the built Gazebo image and real ArduPilotPlugin",
)

_STATUS_SERVICE = "/model/iris/ardupilot/status"
_CONTROL_SERVICE = "/world/vertical_descent/control"
_WORLD = Path("/opt/drone_sim/gazebo/resources/worlds/vertical_descent.sdf")
_PLUGIN = Path("/opt/drone_sim/gazebo/plugins/libArduPilotPlugin.so")
_SERVO_PACKET = struct.Struct("<HHI16H")


def _packet(frame: int) -> bytes:
    return _SERVO_PACKET.pack(18458, 1000, frame, *([1100] * 16))


class PluginHarness:
    def __init__(self) -> None:
        self._temporary = tempfile.TemporaryDirectory(
            prefix="drone-sim-plugin-test-"
        )
        self._log_path = Path(self._temporary.name) / "gz.log"
        self._log = self._log_path.open("w+b")
        self._peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._peer.bind(("127.0.0.1", 0))
        self._peer.settimeout(0.05)
        self._environment = dict(os.environ)
        self._environment.update(
            {
                "GZ_PARTITION": f"plugin_test_{uuid4().hex}",
                "GZ_SIM_RESOURCE_PATH": "/opt/drone_sim/gazebo/resources/models",
                "GZ_SIM_SYSTEM_PLUGIN_PATH": "/opt/drone_sim/gazebo/plugins",
            }
        )
        self._server = subprocess.Popen(
            (
                "gz",
                "sim",
                "-s",
                "--headless-rendering",
                str(_WORLD),
            ),
            env=self._environment,
            stdin=subprocess.DEVNULL,
            stdout=self._log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def __enter__(self) -> PluginHarness:
        assert _WORLD.is_file()
        assert _PLUGIN.is_file()
        try:
            self._wait_for_status(lambda _status: True)
        except BaseException:
            self.close()
            raise
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        del exc_type, exc, traceback
        self.close()

    def close(self) -> None:
        self._peer.close()
        if self._server.poll() is None:
            os.killpg(self._server.pid, signal.SIGTERM)
            try:
                self._server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self._server.pid, signal.SIGKILL)
                self._server.wait(timeout=5)
        self._log.close()
        self._temporary.cleanup()

    def _command(self, *argv: str, timeout: float = 8.0) -> subprocess.CompletedProcess:
        result = subprocess.run(
            argv,
            check=False,
            capture_output=True,
            env=self._environment,
            text=True,
            timeout=timeout,
        )
        if result.returncode != 0:
            raise AssertionError(
                f"command failed: {argv!r}\nstdout={result.stdout}\nstderr={result.stderr}"
            )
        return result

    def status(self) -> dict[str, bool | int]:
        result = self._command(
            "gz",
            "service",
            "-s",
            _STATUS_SERVICE,
            "--reqtype",
            "gz.msgs.Empty",
            "--reptype",
            "gz.msgs.StringMsg",
            "--timeout",
            "1000",
            "--req",
            "",
            timeout=3.0,
        )
        output = result.stdout.strip()
        assert output.startswith('data: "'), output
        return json.loads(json.loads(output.removeprefix("data: ")))

    def _wait_for_status(self, predicate, timeout: float = 15.0) -> dict[str, bool | int]:
        deadline = time.monotonic() + timeout
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            if self._server.poll() is not None:
                raise AssertionError(
                    f"Gazebo exited with {self._server.returncode}: {self.log_text()}"
                )
            try:
                status = self.status()
            except (AssertionError, subprocess.TimeoutExpired) as error:
                last_error = error
                time.sleep(0.05)
                continue
            if predicate(status):
                return status
            time.sleep(0.01)
        raise AssertionError(
            f"status predicate timed out: {last_error}; log={self.log_text()}"
        )

    def send(self, *frames: int) -> None:
        for frame in frames:
            self._peer.sendto(_packet(frame), ("127.0.0.1", 9002))

    def receive_json(self, minimum: int, timeout: float = 2.0) -> list[bytes]:
        messages: list[bytes] = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and len(messages) < minimum:
            try:
                payload, _address = self._peer.recvfrom(65535)
            except socket.timeout:
                continue
            if payload.strip().startswith(b"{"):
                messages.append(payload)
        return messages

    def drain_json(self) -> None:
        while True:
            try:
                self._peer.recvfrom(65535)
            except socket.timeout:
                return

    def bootstrap(self) -> dict[str, bool | int]:
        self.send(0)
        assert self.receive_json(1, timeout=5.0), self.log_text()
        status = self._wait_for_status(
            lambda value: value["motor_updates"] >= 1
            and value["last_servo_frame"] == 0
        )
        self.drain_json()
        return status

    def step(self) -> None:
        result = self._command(
            "gz",
            "service",
            "-s",
            _CONTROL_SERVICE,
            "--reqtype",
            "gz.msgs.WorldControl",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            "pause: true, multi_step: 1",
            timeout=8.0,
        )
        assert "true" in result.stdout.lower(), result.stdout

    def wait_for_motor_update(self, baseline: int) -> dict[str, bool | int]:
        return self._wait_for_status(
            lambda value: value["motor_updates"] >= baseline + 1
        )

    def log_text(self) -> str:
        self._log.flush()
        return self._log_path.read_text(encoding="utf-8", errors="replace")


def test_one_controlled_step_consumes_only_the_first_sequential_servo_packet():
    with PluginHarness() as harness:
        baseline = harness.bootstrap()
        current = int(baseline["last_servo_frame"])

        harness.send(current + 1, current + 2, current + 3)
        harness.step()
        status = harness.wait_for_motor_update(int(baseline["motor_updates"]))

        assert status["last_servo_frame"] == current + 1
        assert status["motor_updates"] == int(baseline["motor_updates"]) + 1
        assert status["servo_frame_gaps"] == baseline["servo_frame_gaps"] == 0


def test_duplicate_then_sequential_packet_resends_json_and_updates_once():
    with PluginHarness() as harness:
        baseline = harness.bootstrap()
        current = int(baseline["last_servo_frame"])

        harness.send(current, current + 1)
        harness.step()
        status = harness.wait_for_motor_update(int(baseline["motor_updates"]))
        recovery_and_step_json = harness.receive_json(2)

        assert status["duplicate_servo_packets"] == (
            int(baseline["duplicate_servo_packets"]) + 1
        )
        assert status["motor_updates"] == int(baseline["motor_updates"]) + 1
        assert status["last_servo_frame"] == current + 1
        assert status["servo_frame_gaps"] == baseline["servo_frame_gaps"] == 0
        assert len(recovery_and_step_json) == 2


def test_lone_forward_jump_remains_diagnosed_as_a_real_gap():
    with PluginHarness() as harness:
        baseline = harness.bootstrap()
        current = int(baseline["last_servo_frame"])

        harness.send(current + 3)
        harness.step()
        status = harness.wait_for_motor_update(int(baseline["motor_updates"]))

        assert status["last_servo_frame"] == current + 3
        assert status["motor_updates"] == int(baseline["motor_updates"]) + 1
        assert status["servo_frame_gaps"] == int(baseline["servo_frame_gaps"]) + 2
