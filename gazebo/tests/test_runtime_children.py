from pathlib import Path
from types import MappingProxyType

import pytest

from drone_sim_gazebo.runtime.children import (
    ChildProcessError,
    ChildSpec,
    ChildSupervisor,
    gazebo_child_specs,
)


class FakeProcess:
    _next_pid = 100

    def __init__(self, returncode=None):
        self.pid = self._next_pid
        FakeProcess._next_pid += 1
        self.returncode = returncode
        self.terminated = 0
        self.killed = 0

    def poll(self):
        return self.returncode

    def terminate(self):
        self.terminated += 1

    def kill(self):
        self.killed += 1

    def wait(self, timeout=None):
        if self.returncode is None:
            if self.terminated:
                self.returncode = 0
            else:
                raise TimeoutError
        return self.returncode


def test_child_specs_are_one_way_private_bridges_and_no_ack_consumer(tmp_path):
    specs = gazebo_child_specs(
        bridge_config=tmp_path / "bridge.yaml",
        environment={"ROS_DOMAIN_ID": "7", "GZ_PARTITION": "run_partition"},
        world_name="competition_mission",
    )

    assert tuple(spec.name for spec in specs) == (
        "bridge",
        "image_bridge_onboard",
        "image_bridge_observer",
    )
    assert specs[0].argv == (
        "ros2", "run", "ros_gz_bridge", "parameter_bridge",
        "--ros-args", "-p", f"config_file:={tmp_path / 'bridge.yaml'}",
    )
    assert tuple(spec.argv[-1] for spec in specs[1:]) == (
        "/gazebo/private/camera/competition_onboard/image",
        "/gazebo/private/camera/observer/image",
    )
    assert all(spec.argv[-2] == "image_bridge" for spec in specs[1:])
    flattened = " ".join(part for spec in specs for part in spec.argv)
    assert "camera_pair_ack" not in flattened
    assert isinstance(specs[0].environment, MappingProxyType)


def test_foundation_image_bridge_keeps_its_existing_private_source(tmp_path):
    specs = gazebo_child_specs(
        bridge_config=tmp_path / "bridge.yaml",
        environment={"GZ_PARTITION": "foundation"},
        world_name="phase3_foundation",
    )

    assert tuple(spec.argv[-1] for spec in specs[1:]) == (
        "/gazebo/private/camera/onboard/image",
        "/gazebo/private/camera/observer/image",
    )


def test_supervisor_reports_first_unexpected_child_exit():
    processes = iter((FakeProcess(), FakeProcess(17)))
    supervisor = ChildSupervisor(
        popen=lambda *_args, **_kwargs: next(processes)
    )
    specs = (
        ChildSpec("bridge", ("bridge",), {}),
        ChildSpec("image_bridge", ("image",), {}),
    )
    supervisor.start(specs)

    assert supervisor.poll_failure() == ("image_bridge", 17)
    assert supervisor.poll_failure() == ("image_bridge", 17)


def test_supervisor_stops_all_children_with_one_absolute_deadline():
    clock_values = iter((10.0, 10.0, 10.1, 10.1))
    processes = [FakeProcess(), FakeProcess()]
    process_iter = iter(processes)
    supervisor = ChildSupervisor(
        popen=lambda *_args, **_kwargs: next(process_iter),
        monotonic=lambda: next(clock_values),
        signal_process_group=lambda pid, _signal: next(
            process for process in processes if process.pid == pid
        ).terminate(),
    )
    supervisor.start((ChildSpec("bridge", ("bridge",), {}), ChildSpec("image_bridge", ("image",), {})))

    supervisor.stop(11.0)

    assert [process.terminated for process in processes] == [1, 1]
    assert [process.killed for process in processes] == [0, 0]


def test_supervisor_rejects_expired_deadline_before_signaling():
    process = FakeProcess()
    supervisor = ChildSupervisor(
        popen=lambda *_args, **_kwargs: process,
        monotonic=lambda: 12.0,
        signal_process_group=lambda _pid, _signal: process.terminate(),
    )
    supervisor.start((ChildSpec("bridge", ("bridge",), {}),))

    with pytest.raises(ChildProcessError, match="deadline"):
        supervisor.stop(11.0)
    assert process.terminated == 0


def test_supervisor_signals_each_new_child_process_group_not_only_wrapper():
    process = FakeProcess()
    signals = []
    supervisor = ChildSupervisor(
        popen=lambda *_args, **_kwargs: process,
        monotonic=lambda: 1.0,
        signal_process_group=lambda pid, signum: (signals.append((pid, signum)), process.terminate()),
    )
    supervisor.start((ChildSpec("bridge", ("ros2", "run", "bridge"), {}),))

    supervisor.stop(2.0)

    assert signals[0][0] == process.pid
    assert process.terminated == 1


def test_partial_child_start_failure_quiesces_already_started_group():
    process = FakeProcess()
    calls = iter((process, OSError("image launch failed")))
    signals = []

    def popen(*_args, **_kwargs):
        result = next(calls)
        if isinstance(result, Exception):
            raise result
        return result

    supervisor = ChildSupervisor(
        popen=popen,
        signal_process_group=lambda pid, signum: (signals.append((pid, signum)), process.terminate()),
    )

    with pytest.raises(ChildProcessError, match="image launch failed"):
        supervisor.start((
            ChildSpec("bridge", ("bridge",), {}),
            ChildSpec("image_bridge", ("image",), {}),
        ))

    assert signals and signals[0][0] == process.pid
    assert process.returncode == 0
