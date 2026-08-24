from __future__ import annotations

from dataclasses import FrozenInstanceError, replace
import json
import math
import os
from pathlib import Path
import signal
import subprocess
from types import MappingProxyType
from typing import Any

import pytest

from drone_sim_gazebo.server import (
    GazeboServer,
    NativeArtifactSummary,
    ServerProcessError,
    ServerSpec,
    server_spec,
)
from drone_sim_gazebo.worlds import ResolvedWorld
from orchestration.config import SimulationConfig


RUN_ID = "00000000-0000-4000-8000-000000000505"
WORLD_DIGEST = "a" * 64
MODEL_DIGEST = "b" * 64


def _run_directory(tmp_path: Path) -> Path:
    path = tmp_path / RUN_ID
    path.mkdir()
    return path


def _resolved_world(tmp_path: Path) -> ResolvedWorld:
    resources = tmp_path / "image resources"
    models = resources / "models"
    worlds = resources / "worlds"
    models.mkdir(parents=True)
    worlds.mkdir()
    world = worlds / "phase3_foundation.sdf"
    world.write_text("<sdf version='1.10'/>", encoding="utf-8")
    return ResolvedWorld(
        path=world.resolve(),
        world_name="phase3_foundation",
        vehicle_id="iris",
        resource_path=models.resolve(),
        world_sha256=WORLD_DIGEST,
        resource_sha256s=(
            ("models/iris_phase3/model.sdf", MODEL_DIGEST),
            ("worlds/phase3_foundation.sdf", WORLD_DIGEST),
        ),
    )


def _spec(tmp_path: Path, *, seed: int = 9) -> ServerSpec:
    return server_spec(
        run_id=RUN_ID,
        run_directory=_run_directory(tmp_path),
        resolved_world=_resolved_world(tmp_path),
        config=SimulationConfig(seed, 2_000_000_000, 0.1),
    )


class FakeProcess:
    def __init__(
        self,
        *,
        pid: int = 4242,
        returncode: int | None = None,
        waits: list[int | str] | None = None,
    ) -> None:
        self.pid = pid
        self.returncode = returncode
        self.waits = list(waits or [0])
        self.wait_timeouts: list[float] = []

    def poll(self) -> int | None:
        return self.returncode

    def wait(self, timeout: float) -> int:
        self.wait_timeouts.append(timeout)
        outcome = self.waits.pop(0)
        if outcome == "timeout":
            raise subprocess.TimeoutExpired("gz sim", timeout)
        assert isinstance(outcome, int) and not isinstance(outcome, bool)
        self.returncode = outcome
        return outcome


class PopenFactory:
    def __init__(self, process: FakeProcess | None = None, error: Exception | None = None):
        self.process = process or FakeProcess()
        self.error = error
        self.calls: list[tuple[tuple[str, ...], dict[str, Any]]] = []
        self.stream = None

    def __call__(self, argv: tuple[str, ...], **kwargs: Any) -> FakeProcess:
        self.calls.append((argv, kwargs))
        self.stream = kwargs["stdout"]
        if self.error is not None:
            raise self.error
        return self.process


def _start(
    spec: ServerSpec,
    *,
    process: FakeProcess | None = None,
    monotonic=lambda: 10.0,
):
    factory = PopenFactory(process)
    signals: list[tuple[int, int]] = []
    server = GazeboServer(
        spec,
        popen_factory=factory,
        monotonic=monotonic,
        signal_process_group=lambda pid, signum: signals.append((pid, signum)),
    )
    server.start()
    return server, factory, signals


def test_server_spec_is_paused_local_partitioned_and_records_native_state(tmp_path: Path):
    run_directory = _run_directory(tmp_path)
    resolved = _resolved_world(tmp_path)

    spec = server_spec(
        run_id=RUN_ID,
        run_directory=run_directory,
        resolved_world=resolved,
        config=SimulationConfig(9, 2_000_000_000, 0.1),
    )

    assert spec.argv == (
        "gz",
        "sim",
        "-s",
        "--headless-rendering",
        "--seed",
        "9",
        "--record-path",
        str(run_directory / "gazebo/state"),
        str(resolved.path),
    )
    assert "-r" not in spec.argv
    assert dict(spec.environment) == {
        "GZ_PARTITION": f"drone_sim_{RUN_ID.replace('-', '_')}",
        "GZ_SIM_RESOURCE_PATH": str(resolved.resource_path),
    }
    assert isinstance(spec.environment, MappingProxyType)
    assert spec.partial_log_path == run_directory / "gazebo/server.log.partial"
    assert spec.final_log_path == run_directory / "gazebo/server.log"
    assert spec.native_state_path == run_directory / "gazebo/state/state.tlog"
    with pytest.raises(TypeError):
        spec.environment["GZ_PARTITION"] = "hostile"  # type: ignore[index]
    with pytest.raises(FrozenInstanceError):
        spec.argv = ()  # type: ignore[misc]


def test_server_spec_ignores_hostile_ambient_environment(tmp_path: Path, monkeypatch):
    monkeypatch.setenv("GZ_PARTITION", "ambient")
    monkeypatch.setenv("GZ_SIM_RESOURCE_PATH", "/tmp/remote")
    monkeypatch.setenv("SDF_PATH", "/tmp/hostile")
    monkeypatch.setenv("LD_PRELOAD", "/tmp/injected.so")
    spec = _spec(tmp_path)

    server, factory, _signals = _start(spec)

    assert factory.calls[0][1]["env"] == dict(spec.environment)
    assert set(factory.calls[0][1]["env"]) == {
        "GZ_PARTITION",
        "GZ_SIM_RESOURCE_PATH",
    }
    spec.native_state_path.write_bytes(b"native")
    server.stop(20.0)


@pytest.mark.parametrize(
    "change",
    [
        {"argv": ("sh", "-c", "echo compromised")},
        {"environment": MappingProxyType({"GZ_PARTITION": "foreign", "GZ_SIM_RESOURCE_PATH": "/tmp"})},
    ],
    ids=["command", "authority-environment"],
)
def test_server_spec_cannot_be_forged_around_exact_command_authority(tmp_path: Path, change):
    spec = _spec(tmp_path)

    with pytest.raises((TypeError, ValueError)):
        replace(spec, **change)


@pytest.mark.parametrize(
    ("mutation", "match"),
    [
        ("relative-run", "absolute"),
        ("wrong-run-name", "run_id"),
        ("run-symlink", "symlink"),
        ("world-symlink", "symlink"),
        ("resource-symlink", "symlink"),
        ("mutable-checksums", "immutable tuple"),
        ("boolean-seed", "seed"),
    ],
)
def test_server_spec_rejects_malformed_or_unsafe_inputs(tmp_path: Path, mutation: str, match: str):
    run_directory = _run_directory(tmp_path)
    resolved = _resolved_world(tmp_path)
    config = SimulationConfig(9, 2_000_000_000, 0.1)
    if mutation == "relative-run":
        run_directory = Path(RUN_ID)
    elif mutation == "wrong-run-name":
        other = tmp_path / "wrong"
        other.mkdir()
        run_directory = other
    elif mutation == "run-symlink":
        real = run_directory
        alias = tmp_path / "alias"
        alias.symlink_to(real, target_is_directory=True)
        run_directory = alias
    elif mutation == "world-symlink":
        target = resolved.path
        alias = target.parent / "alias.sdf"
        alias.symlink_to(target)
        resolved = ResolvedWorld(
            alias,
            resolved.world_name,
            resolved.vehicle_id,
            resolved.resource_path,
            resolved.world_sha256,
            resolved.resource_sha256s,
        )
    elif mutation == "resource-symlink":
        alias = resolved.resource_path.parent / "model-alias"
        alias.symlink_to(resolved.resource_path, target_is_directory=True)
        resolved = ResolvedWorld(
            resolved.path,
            resolved.world_name,
            resolved.vehicle_id,
            alias,
            resolved.world_sha256,
            resolved.resource_sha256s,
        )
    elif mutation == "mutable-checksums":
        resolved = ResolvedWorld(
            resolved.path,
            resolved.world_name,
            resolved.vehicle_id,
            resolved.resource_path,
            resolved.world_sha256,
            list(resolved.resource_sha256s),  # type: ignore[arg-type]
        )
    else:
        config = SimulationConfig(True, 2_000_000_000, 0.1)  # type: ignore[arg-type]

    with pytest.raises((TypeError, ValueError), match=match):
        server_spec(
            run_id=RUN_ID,
            run_directory=run_directory,
            resolved_world=resolved,
            config=config,
        )


@pytest.mark.parametrize("run_id", ["bad", "AAAAAAAA-AAAA-4AAA-8AAA-AAAAAAAAAAAA", True])
def test_server_spec_rejects_noncanonical_run_ids(tmp_path: Path, run_id: object):
    run_directory = tmp_path / str(run_id)
    run_directory.mkdir()
    with pytest.raises(ValueError, match="canonical UUID"):
        server_spec(
            run_id=run_id,  # type: ignore[arg-type]
            run_directory=run_directory,
            resolved_world=_resolved_world(tmp_path),
            config=SimulationConfig(9, 2_000_000_000, 0.1),
        )


def test_start_uses_shell_free_new_session_and_one_shared_append_log(tmp_path: Path):
    spec = _spec(tmp_path)
    server, factory, _signals = _start(spec)

    argv, kwargs = factory.calls[0]
    assert argv == spec.argv
    assert kwargs["shell"] is False
    assert kwargs["start_new_session"] is True
    assert kwargs["stdin"] is subprocess.DEVNULL
    assert kwargs["stdout"] is kwargs["stderr"]
    assert kwargs["stdout"].closed is False
    kwargs["stdout"].write(b"child output\n")
    spec.native_state_path.write_bytes(b"native")
    summary = server.stop(20.0)
    assert summary.server_log_path.read_bytes().endswith(b"child output\n")


def test_start_preamble_is_one_escaped_data_only_json_record(tmp_path: Path):
    hostile_root = tmp_path / "parent\nwith spaces"
    hostile_root.mkdir()
    run_directory = hostile_root / RUN_ID
    run_directory.mkdir()
    resolved = _resolved_world(hostile_root)
    spec = server_spec(
        run_id=RUN_ID,
        run_directory=run_directory,
        resolved_world=resolved,
        config=SimulationConfig(23, 2_000_000_000, 0.1),
    )
    factory = PopenFactory(error=OSError("spawn refused"))
    server = GazeboServer(spec, popen_factory=factory)

    with pytest.raises(ServerProcessError, match="spawn refused"):
        server.start()

    raw = spec.partial_log_path.read_bytes()
    assert raw.count(b"\n") == 1
    preamble = json.loads(raw)
    assert preamble == {
        "argv": list(spec.argv),
        "event": "gazebo_server_start",
        "partition": spec.environment["GZ_PARTITION"],
        "resource_sha256s": [list(item) for item in spec.resource_sha256s],
        "run_id": RUN_ID,
        "seed": 23,
        "world_sha256": WORLD_DIGEST,
    }
    assert factory.stream.closed is True


@pytest.mark.parametrize("collision", ["gazebo-file", "state-symlink", "partial", "final", "native"])
def test_start_rejects_unsafe_preexisting_owned_paths(tmp_path: Path, collision: str):
    spec = _spec(tmp_path)
    gazebo = spec.final_log_path.parent
    if collision == "gazebo-file":
        gazebo.write_bytes(b"collision")
    else:
        gazebo.mkdir()
        state = gazebo / "state"
        if collision == "state-symlink":
            outside = tmp_path / "outside-state"
            outside.mkdir()
            state.symlink_to(outside, target_is_directory=True)
        else:
            state.mkdir()
            if collision == "partial":
                spec.partial_log_path.write_bytes(b"old")
            elif collision == "final":
                spec.final_log_path.write_bytes(b"old")
            else:
                spec.native_state_path.write_bytes(b"old")
    factory = PopenFactory()
    server = GazeboServer(spec, popen_factory=factory)

    with pytest.raises(ServerProcessError, match="unsafe|already exists|collision"):
        server.start()

    assert factory.calls == []


def test_spawn_failure_closes_parent_descriptor_preserves_diagnostic_and_latches(tmp_path: Path):
    spec = _spec(tmp_path)
    factory = PopenFactory(error=OSError("spawn exploded"))
    server = GazeboServer(spec, popen_factory=factory)

    with pytest.raises(ServerProcessError, match="spawn exploded") as first:
        server.start()
    assert spec.partial_log_path.is_file()
    assert spec.partial_log_path.stat().st_size > 0
    assert factory.stream.closed is True
    with pytest.raises(ServerProcessError) as repeated:
        server.stop(20.0)
    assert str(repeated.value) == str(first.value)


def test_stop_gracefully_terminates_group_reaps_and_publishes_native_artifacts(tmp_path: Path):
    spec = _spec(tmp_path)
    process = FakeProcess(waits=[0])
    server, _factory, signals = _start(spec, process=process)
    spec.native_state_path.write_bytes(b"native state")

    summary = server.stop(20.0)

    assert signals == [(process.pid, signal.SIGTERM)]
    assert process.wait_timeouts == [pytest.approx(5.0)]
    assert summary == NativeArtifactSummary(
        server_log_path=spec.final_log_path,
        state_log_path=spec.native_state_path,
        server_returncode=0,
        graceful=True,
    )
    assert spec.final_log_path.is_file()
    assert not spec.partial_log_path.exists()


def test_stop_of_already_exited_child_never_signals_reused_pid(tmp_path: Path):
    spec = _spec(tmp_path)
    process = FakeProcess(returncode=17, waits=[])
    server, _factory, signals = _start(spec, process=process)
    spec.native_state_path.write_bytes(b"native state")

    summary = server.stop(20.0)

    assert signals == []
    assert process.wait_timeouts == []
    assert summary.server_returncode == 17
    assert summary.graceful is False


def test_stop_escalates_once_and_recomputes_remaining_absolute_deadline(tmp_path: Path):
    spec = _spec(tmp_path)
    process = FakeProcess(waits=["timeout", -signal.SIGKILL])
    times = iter((10.0, 12.0, *([12.0] * 20)))
    server, _factory, signals = _start(
        spec,
        process=process,
        monotonic=lambda: next(times),
    )
    spec.native_state_path.write_bytes(b"native state")

    summary = server.stop(20.0)

    assert signals == [
        (process.pid, signal.SIGTERM),
        (process.pid, signal.SIGKILL),
    ]
    assert process.wait_timeouts == [pytest.approx(5.0), pytest.approx(8.0)]
    assert summary.server_returncode == -signal.SIGKILL
    assert summary.graceful is False


def test_expired_absolute_deadline_never_publishes_artifacts(tmp_path: Path, monkeypatch):
    spec = _spec(tmp_path)
    process = FakeProcess(waits=[0])
    times = iter((10.0, 20.0))
    server, _factory, _signals = _start(
        spec,
        process=process,
        monotonic=lambda: next(times),
    )
    spec.native_state_path.write_bytes(b"native")
    fsync_calls: list[int] = []
    from drone_sim_gazebo.server import process as process_module

    monkeypatch.setattr(process_module.os, "fsync", fsync_calls.append)

    with pytest.raises(ServerProcessError, match="deadline"):
        server.stop(20.0)

    assert fsync_calls == []
    assert spec.partial_log_path.is_file()
    assert not spec.final_log_path.exists()


@pytest.mark.parametrize("deadline", [True, 0, -1.0, math.inf, math.nan, "20"])
def test_stop_rejects_invalid_deadline_without_mutating_lifecycle(tmp_path: Path, deadline: object):
    spec = _spec(tmp_path)
    server, _factory, signals = _start(spec)

    with pytest.raises(ValueError, match="deadline"):
        server.stop(deadline)  # type: ignore[arg-type]

    assert signals == []
    spec.native_state_path.write_bytes(b"native")
    assert server.stop(20.0).server_returncode == 0


@pytest.mark.parametrize("native", ["missing", "empty", "symlink", "hardlink"])
def test_stop_rejects_invalid_native_state_and_preserves_partial_log(tmp_path: Path, native: str):
    spec = _spec(tmp_path)
    server, _factory, _signals = _start(spec)
    if native == "empty":
        spec.native_state_path.touch()
    elif native == "symlink":
        outside = tmp_path / "outside.tlog"
        outside.write_bytes(b"outside")
        spec.native_state_path.symlink_to(outside)
    elif native == "hardlink":
        outside = tmp_path / "outside.tlog"
        outside.write_bytes(b"outside")
        os.link(outside, spec.native_state_path)

    with pytest.raises(ServerProcessError, match="state.tlog"):
        server.stop(20.0)

    assert spec.partial_log_path.is_file()
    assert not spec.final_log_path.exists()


def test_stop_never_clobbers_a_late_final_log_collision(tmp_path: Path):
    spec = _spec(tmp_path)
    server, _factory, _signals = _start(spec)
    spec.native_state_path.write_bytes(b"native")
    spec.final_log_path.write_bytes(b"attacker")

    with pytest.raises(ServerProcessError, match="server.log"):
        server.stop(20.0)

    assert spec.final_log_path.read_bytes() == b"attacker"
    assert spec.partial_log_path.is_file()


def test_stop_fsyncs_files_and_directories_before_returning_summary(tmp_path: Path, monkeypatch):
    spec = _spec(tmp_path)
    server, _factory, _signals = _start(spec)
    spec.native_state_path.write_bytes(b"native")
    real_fsync = os.fsync
    fsync_kinds: list[str] = []

    def recording_fsync(descriptor: int) -> None:
        metadata = os.fstat(descriptor)
        fsync_kinds.append("directory" if os.path.isdir(f"/proc/self/fd/{descriptor}") else "file")
        real_fsync(descriptor)

    from drone_sim_gazebo.server import process as process_module

    monkeypatch.setattr(process_module.os, "fsync", recording_fsync)

    server.stop(20.0)

    assert fsync_kinds.count("file") >= 2
    assert fsync_kinds[-1] == "directory"


def test_repeated_stop_returns_same_frozen_summary_without_more_process_actions(tmp_path: Path):
    spec = _spec(tmp_path)
    process = FakeProcess(waits=[0])
    server, _factory, signals = _start(spec, process=process)
    spec.native_state_path.write_bytes(b"native")

    first = server.stop(20.0)
    first_signal_count = len(signals)
    first_wait_count = len(process.wait_timeouts)
    repeated = server.stop(999.0)

    assert repeated is first
    assert len(signals) == first_signal_count
    assert len(process.wait_timeouts) == first_wait_count
    with pytest.raises(FrozenInstanceError):
        first.graceful = False  # type: ignore[misc]
