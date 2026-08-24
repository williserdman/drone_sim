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
        self.exited = returncode is not None
        self.exit_returncode = returncode
        self.waits = list(waits or [0])
        self.wait_timeouts: list[float] = []
        self.poll_calls = 0
        self.observations = 0

    def poll(self) -> int | None:
        self.poll_calls += 1
        return self.returncode

    def observe_exit(self, pid: int) -> bool:
        assert pid == self.pid
        self.observations += 1
        return self.exited

    def mark_exited(self, returncode: int) -> None:
        self.exited = True
        self.exit_returncode = returncode

    def receive_signal(self, signum: int) -> None:
        if self.exited:
            return
        if signum == signal.SIGTERM and self.waits and self.waits[0] == "timeout":
            self.waits.pop(0)
            return
        if self.waits:
            outcome = self.waits.pop(0)
            assert isinstance(outcome, int) and not isinstance(outcome, bool)
        else:
            outcome = -signum
        self.mark_exited(outcome)

    def wait(self, timeout: float) -> int:
        self.wait_timeouts.append(timeout)
        if not self.exited:
            raise subprocess.TimeoutExpired("gz sim", timeout)
        assert self.exit_returncode is not None
        self.returncode = self.exit_returncode
        return self.exit_returncode


class FakeProcessGroup:
    def __init__(
        self,
        process: FakeProcess,
        *,
        descendant_alive: bool = False,
        ignore_sigterm: bool = False,
    ) -> None:
        self.process = process
        self.descendant_alive = descendant_alive
        self.ignore_sigterm = ignore_sigterm
        self.signals: list[tuple[int, int]] = []
        self.probes: list[tuple[int, int]] = []

    def exists(self, process_group_id: int, session_id: int) -> bool:
        self.probes.append((process_group_id, session_id))
        return self.descendant_alive or not self.process.exited

    def signal(self, process_group_id: int, signum: int) -> None:
        self.signals.append((process_group_id, signum))
        self.process.receive_signal(signum)
        if signum == signal.SIGKILL or not self.ignore_sigterm:
            self.descendant_alive = False


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
    sleep=lambda _seconds: None,
    group: FakeProcessGroup | None = None,
    observe_leader_exit=None,
):
    process = process or FakeProcess()
    factory = PopenFactory(process)
    group = group or FakeProcessGroup(process)
    options = {
        "popen_factory": factory,
        "monotonic": monotonic,
        "sleep": sleep,
        "get_process_group": lambda pid: pid,
        "process_group_exists": group.exists,
        "signal_process_group": group.signal,
    }
    options["observe_leader_exit"] = observe_leader_exit or process.observe_exit
    server = GazeboServer(spec, **options)
    server.start()
    spec.native_state_path.parent.mkdir()
    return server, factory, group.signals


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
        "PATH": (
            "/opt/ros/jazzy/opt/gz_msgs_vendor/bin:"
            "/opt/ros/jazzy/opt/gz_tools_vendor/bin:"
            "/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:"
            "/opt/ros/jazzy/bin:/opt/drone_sim/venv/bin:"
            "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
        ),
        "GZ_CONFIG_PATH": (
            "/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:"
            "/opt/ros/jazzy/opt/sdformat_vendor/share/gz:"
            "/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:"
            "/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:"
            "/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:"
            "/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:"
            "/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:"
            "/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:"
            "/opt/ros/jazzy/opt/gz_common_vendor/share/gz"
        ),
        "LD_LIBRARY_PATH": (
            "/opt/ros/jazzy/opt/gz_sim_vendor/lib:"
            "/opt/ros/jazzy/opt/gz_sensors_vendor/lib:"
            "/opt/ros/jazzy/opt/gz_physics_vendor/lib:"
            "/opt/ros/jazzy/opt/sdformat_vendor/lib:"
            "/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:"
            "/opt/ros/jazzy/lib/x86_64-linux-gnu:"
            "/opt/ros/jazzy/opt/gz_gui_vendor/lib:"
            "/opt/ros/jazzy/opt/gz_transport_vendor/lib:"
            "/opt/ros/jazzy/opt/gz_rendering_vendor/lib:"
            "/opt/ros/jazzy/opt/gz_plugin_vendor/lib:"
            "/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:"
            "/opt/ros/jazzy/opt/gz_msgs_vendor/lib:"
            "/opt/ros/jazzy/opt/gz_common_vendor/lib:"
            "/opt/ros/jazzy/opt/gz_math_vendor/lib:"
            "/opt/ros/jazzy/opt/gz_utils_vendor/lib:"
            "/opt/ros/jazzy/opt/gz_tools_vendor/lib:"
            "/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:"
            "/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:"
            "/opt/ros/jazzy/opt/gz_cmake_vendor/lib:"
            "/opt/ros/jazzy/lib"
        ),
        "HOME": "/tmp",
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
    monkeypatch.setenv("PATH", "/tmp/ambient-bin")
    monkeypatch.setenv("GZ_CONFIG_PATH", "/tmp/ambient-gz")
    monkeypatch.setenv("LD_LIBRARY_PATH", "/tmp/ambient-lib")
    monkeypatch.setenv("HOME", "/tmp/ambient-home")
    spec = _spec(tmp_path)

    server, factory, _signals = _start(spec)

    assert factory.calls[0][1]["env"] == dict(spec.environment)
    assert set(factory.calls[0][1]["env"]) == {
        "PATH",
        "GZ_CONFIG_PATH",
        "LD_LIBRARY_PATH",
        "HOME",
        "GZ_PARTITION",
        "GZ_SIM_RESOURCE_PATH",
    }
    assert factory.calls[0][1]["env"]["HOME"] == "/tmp"
    spec.native_state_path.write_bytes(b"native")
    server.stop(20.0)


@pytest.mark.parametrize(
    "change",
    [
        {"argv": ("sh", "-c", "echo compromised")},
        {"environment": MappingProxyType({"GZ_PARTITION": "foreign"})},
    ],
    ids=["command", "authority-environment"],
)
def test_server_spec_cannot_be_forged_around_exact_command_authority(tmp_path: Path, change):
    spec = _spec(tmp_path)

    with pytest.raises((TypeError, ValueError)):
        replace(spec, **change)


def test_server_spec_copies_environment_authority_and_server_spec_is_read_only(
    tmp_path: Path,
):
    spec = _spec(tmp_path)
    source = dict(spec.environment)
    copied = replace(spec, environment=MappingProxyType(source))
    source["PATH"] = "/tmp/hostile"
    server = GazeboServer(copied)

    assert copied.environment["PATH"] == spec.environment["PATH"]
    with pytest.raises(AttributeError):
        server.spec = spec  # type: ignore[misc]
    assert server.spec is copied


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


def test_start_leaves_record_path_absent_for_gazebo_to_create(tmp_path: Path):
    spec = _spec(tmp_path)
    process = FakeProcess()
    factory = PopenFactory(process)
    group = FakeProcessGroup(process)
    server = GazeboServer(
        spec,
        popen_factory=factory,
        monotonic=lambda: 10.0,
        sleep=lambda _seconds: None,
        get_process_group=lambda pid: pid,
        process_group_exists=group.exists,
        observe_leader_exit=process.observe_exit,
        signal_process_group=group.signal,
    )

    server.start()

    assert spec.final_log_path.parent.is_dir()
    assert not spec.native_state_path.parent.exists()
    spec.native_state_path.parent.mkdir()
    spec.native_state_path.write_bytes(b"native")
    server.stop(20.0)


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
    assert process.poll_calls == 0
    assert process.wait_timeouts == [pytest.approx(10.0)]
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
    assert process.poll_calls == 0
    assert process.wait_timeouts == [pytest.approx(10.0)]
    assert summary.server_returncode == 17
    assert summary.graceful is False


def test_clean_unreaped_leader_exit_retains_identity_until_one_final_reap(
    tmp_path: Path,
):
    spec = _spec(tmp_path)
    process = FakeProcess(returncode=17, waits=[])
    server, _factory, signals = _start(
        spec,
        process=process,
        observe_leader_exit=process.observe_exit,
    )
    spec.native_state_path.write_bytes(b"native state")

    summary = server.stop(20.0)

    assert signals == []
    assert process.poll_calls == 0
    assert len(process.wait_timeouts) == 1
    assert summary.server_returncode == 17


def test_concurrent_leader_exit_between_observation_and_group_probe_is_clean(
    tmp_path: Path,
):
    spec = _spec(tmp_path)
    process = FakeProcess(waits=[])
    group = FakeProcessGroup(process)

    def exit_during_probe(process_group_id: int, session_id: int) -> bool:
        group.probes.append((process_group_id, session_id))
        process.mark_exited(23)
        return False

    group.exists = exit_during_probe  # type: ignore[method-assign]
    server, _factory, signals = _start(
        spec,
        process=process,
        group=group,
        observe_leader_exit=process.observe_exit,
    )
    spec.native_state_path.write_bytes(b"native state")

    summary = server.stop(20.0)

    assert signals == []
    assert process.poll_calls == 0
    assert len(process.wait_timeouts) == 1
    assert summary.server_returncode == 23


def test_concurrent_exit_after_group_probe_never_signals_reused_numeric_group(
    tmp_path: Path,
):
    spec = _spec(tmp_path)
    process = FakeProcess(waits=[])
    group = FakeProcessGroup(process)
    attempted_signals: list[tuple[int, int]] = []

    def exit_before_signal(process_group_id: int, signum: int) -> None:
        attempted_signals.append((process_group_id, signum))
        process.mark_exited(31)
        raise ProcessLookupError("original group exited")

    group.signal = exit_before_signal  # type: ignore[method-assign]
    server, _factory, _signals = _start(
        spec,
        process=process,
        group=group,
        observe_leader_exit=process.observe_exit,
    )
    spec.native_state_path.write_bytes(b"native state")

    summary = server.stop(20.0)

    assert attempted_signals == [(process.pid, signal.SIGTERM)]
    assert process.poll_calls == 0
    assert len(process.wait_timeouts) == 1
    assert summary.server_returncode == 31


def test_group_can_disappear_just_before_leader_exit_becomes_waitable(
    tmp_path: Path,
):
    spec = _spec(tmp_path)
    process = FakeProcess(waits=[])
    group = FakeProcessGroup(process)
    termination_started = False
    post_signal_observations = 0

    def delayed_exit_observation(pid: int) -> bool:
        nonlocal post_signal_observations
        assert pid == process.pid
        if not termination_started:
            return False
        post_signal_observations += 1
        if post_signal_observations == 1:
            return False
        process.mark_exited(29)
        return True

    def group_exits_before_waitable(process_group_id: int, signum: int) -> None:
        nonlocal termination_started
        group.signals.append((process_group_id, signum))
        termination_started = True
        group.descendant_alive = False

    def transition_probe(process_group_id: int, session_id: int) -> bool:
        group.probes.append((process_group_id, session_id))
        return not termination_started

    group.signal = group_exits_before_waitable  # type: ignore[method-assign]
    group.exists = transition_probe  # type: ignore[method-assign]
    server, _factory, signals = _start(
        spec,
        process=process,
        group=group,
        observe_leader_exit=delayed_exit_observation,
    )
    spec.native_state_path.write_bytes(b"native state")

    summary = server.stop(20.0)

    assert signals == [(process.pid, signal.SIGTERM)]
    assert post_signal_observations == 2
    assert process.poll_calls == 0
    assert len(process.wait_timeouts) == 1
    assert summary.server_returncode == 29


def test_stop_terminates_surviving_descendant_after_group_leader_exited(
    tmp_path: Path,
):
    spec = _spec(tmp_path)
    process = FakeProcess(returncode=0, waits=[])
    group = FakeProcessGroup(process, descendant_alive=True)
    server, _factory, signals = _start(spec, process=process, group=group)
    spec.native_state_path.write_bytes(b"native state")

    summary = server.stop(20.0)

    assert signals == [(process.pid, signal.SIGTERM)]
    assert group.descendant_alive is False
    assert group.probes
    assert all(identity == (process.pid, process.pid) for identity in group.probes)
    assert summary.server_returncode == 0
    assert summary.graceful is True


def test_stop_escalates_when_surviving_descendant_ignores_sigterm(tmp_path: Path):
    spec = _spec(tmp_path)
    process = FakeProcess(returncode=0, waits=[])
    group = FakeProcessGroup(
        process,
        descendant_alive=True,
        ignore_sigterm=True,
    )
    clock = {"now": 10.0}

    def advance(seconds: float) -> None:
        clock["now"] += seconds

    server, _factory, signals = _start(
        spec,
        process=process,
        monotonic=lambda: clock["now"],
        sleep=advance,
        group=group,
    )
    spec.native_state_path.write_bytes(b"native state")

    summary = server.stop(10.2)

    assert signals == [
        (process.pid, signal.SIGTERM),
        (process.pid, signal.SIGKILL),
    ]
    assert group.descendant_alive is False
    assert summary.graceful is False


def test_group_probe_crossing_absolute_deadline_starts_no_sleep_or_signal(
    tmp_path: Path,
):
    spec = _spec(tmp_path)
    process = FakeProcess(returncode=0, waits=[])
    group = FakeProcessGroup(
        process,
        descendant_alive=True,
        ignore_sigterm=True,
    )
    clock = {"now": 10.0}
    probe_count = 0
    sleeps: list[float] = []

    def expiring_probe(process_group_id: int, session_id: int) -> bool:
        nonlocal probe_count
        probe_count += 1
        group.probes.append((process_group_id, session_id))
        if probe_count == 3:
            clock["now"] = 20.0
        return group.descendant_alive

    group.exists = expiring_probe  # type: ignore[method-assign]
    server, _factory, signals = _start(
        spec,
        process=process,
        monotonic=lambda: clock["now"],
        sleep=sleeps.append,
        group=group,
        observe_leader_exit=process.observe_exit,
    )
    spec.native_state_path.write_bytes(b"native state")

    with pytest.raises(ServerProcessError, match="deadline"):
        server.stop(20.0)

    assert signals == [(process.pid, signal.SIGTERM)]
    assert sleeps == []
    assert process.wait_timeouts == []


def test_stop_escalates_once_and_recomputes_remaining_absolute_deadline(tmp_path: Path):
    spec = _spec(tmp_path)
    process = FakeProcess(waits=["timeout", -signal.SIGKILL])
    clock = {"now": 10.0}

    def finish_grace_phase(_seconds: float) -> None:
        clock["now"] = 15.0

    server, _factory, signals = _start(
        spec,
        process=process,
        monotonic=lambda: clock["now"],
        sleep=finish_grace_phase,
    )
    spec.native_state_path.write_bytes(b"native state")

    summary = server.stop(20.0)

    assert signals == [
        (process.pid, signal.SIGTERM),
        (process.pid, signal.SIGKILL),
    ]
    assert process.poll_calls == 0
    assert process.wait_timeouts == [pytest.approx(5.0)]
    assert summary.server_returncode == -signal.SIGKILL
    assert summary.graceful is False


def test_expired_absolute_deadline_never_publishes_artifacts(tmp_path: Path, monkeypatch):
    spec = _spec(tmp_path)
    process = FakeProcess(waits=[0])
    server, _factory, _signals = _start(
        spec,
        process=process,
        monotonic=lambda: 20.0,
    )
    spec.native_state_path.write_bytes(b"native")
    fsync_calls: list[int] = []
    from drone_sim_gazebo.server import process as process_module

    monkeypatch.setattr(process_module.os, "fsync", fsync_calls.append)

    with pytest.raises(ServerProcessError, match="deadline"):
        server.stop(20.0)

    assert fsync_calls == []
    assert _signals == []
    assert process.wait_timeouts == []
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


def test_state_rename_and_replacement_during_validation_never_returns_summary(
    tmp_path: Path,
    monkeypatch,
):
    spec = _spec(tmp_path)
    server, _factory, _signals = _start(spec)
    original = b"original native state"
    replacement = b"replacement native state"
    spec.native_state_path.write_bytes(original)
    retained_state = spec.native_state_path.parent.with_name("state.retained")
    from drone_sim_gazebo.server import process as process_module

    real_fsync = process_module.os.fsync
    original_identity = spec.native_state_path.stat()
    swapped = False

    def swap_state_on_file_fsync(descriptor: int) -> None:
        nonlocal swapped
        real_fsync(descriptor)
        opened = os.fstat(descriptor)
        if swapped or (opened.st_dev, opened.st_ino) != (
            original_identity.st_dev,
            original_identity.st_ino,
        ):
            return
        swapped = True
        spec.native_state_path.parent.rename(retained_state)
        spec.native_state_path.parent.mkdir()
        spec.native_state_path.write_bytes(replacement)

    monkeypatch.setattr(process_module.os, "fsync", swap_state_on_file_fsync)

    with pytest.raises(ServerProcessError, match="state|directory|identity|inventory"):
        server.stop(20.0)

    assert swapped is True
    assert (retained_state / "state.tlog").read_bytes() == original
    assert spec.native_state_path.read_bytes() == replacement
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


def test_stop_rejects_replaced_partial_and_never_publishes_replacement_bytes(
    tmp_path: Path,
):
    spec = _spec(tmp_path)
    server, _factory, _signals = _start(spec)
    spec.native_state_path.write_bytes(b"native")
    retained = spec.partial_log_path.with_name("server.log.retained")
    spec.partial_log_path.rename(retained)
    replacement = b"attacker replacement\n"
    spec.partial_log_path.write_bytes(replacement)

    with pytest.raises(ServerProcessError, match="identity|changed"):
        server.stop(20.0)

    assert retained.is_file()
    assert b'"event":"gazebo_server_start"' in retained.read_bytes()
    assert spec.partial_log_path.read_bytes() == replacement
    assert not spec.final_log_path.exists()


def test_deadline_at_publication_commit_edge_preserves_both_named_diagnostics(
    tmp_path: Path,
    monkeypatch,
):
    spec = _spec(tmp_path)
    clock = {"now": 10.0}
    server, _factory, _signals = _start(spec, monotonic=lambda: clock["now"])
    spec.native_state_path.write_bytes(b"native")
    from drone_sim_gazebo.server import process as process_module

    real_link = process_module.os.link

    def expiring_link(*args, **kwargs):
        real_link(*args, **kwargs)
        clock["now"] = 20.0

    monkeypatch.setattr(process_module.os, "link", expiring_link)

    with pytest.raises(ServerProcessError, match="deadline"):
        server.stop(20.0)

    assert spec.partial_log_path.is_file()
    assert spec.final_log_path.is_file()
    assert spec.partial_log_path.samefile(spec.final_log_path)


def test_successful_no_clobber_commit_is_not_reversed_by_later_deadline(
    tmp_path: Path,
    monkeypatch,
):
    spec = _spec(tmp_path)
    clock = {"now": 10.0}
    server, _factory, _signals = _start(spec, monotonic=lambda: clock["now"])
    spec.native_state_path.write_bytes(b"native")
    from drone_sim_gazebo.server import process as process_module

    real_unlink = process_module.os.unlink

    def expiring_unlink(*args, **kwargs):
        real_unlink(*args, **kwargs)
        clock["now"] = 20.0

    monkeypatch.setattr(process_module.os, "unlink", expiring_unlink)

    summary = server.stop(20.0)

    assert summary.server_log_path == spec.final_log_path
    assert spec.final_log_path.is_file()
    assert not spec.partial_log_path.exists()


@pytest.mark.parametrize(
    "close_error",
    [OSError("close failed"), RuntimeError("close failed")],
)
def test_close_failure_after_log_commit_is_non_authoritative_and_idempotent(
    tmp_path: Path,
    close_error: Exception,
):
    spec = _spec(tmp_path)
    process = FakeProcess(waits=[0])
    server, factory, signals = _start(spec, process=process)
    spec.native_state_path.write_bytes(b"native")
    stream = factory.stream

    class CloseFailsAfterClosing:
        def flush(self) -> None:
            stream.flush()

        def fileno(self) -> int:
            return stream.fileno()

        def close(self) -> None:
            stream.close()
            raise close_error

    server._log_stream = CloseFailsAfterClosing()  # type: ignore[attr-defined]

    summary = server.stop(20.0)
    repeated = server.stop(999.0)

    assert repeated is summary
    assert signals == [(process.pid, signal.SIGTERM)]
    assert spec.final_log_path.is_file()
    assert not spec.partial_log_path.exists()


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
