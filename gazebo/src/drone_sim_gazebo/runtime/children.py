"""Small shell-free supervisor for the private ROS bridge children."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
import math
import os
import signal
import subprocess
import time

from ..ros_adapter.topics import camera_topics_for_world


class ChildProcessError(RuntimeError):
    """A bridge child could not start or quiesce safely."""


@dataclass(frozen=True)
class ChildSpec:
    name: str
    argv: tuple[str, ...]
    environment: Mapping[str, str]

    def __post_init__(self) -> None:
        if not self.name or type(self.argv) is not tuple or not self.argv:
            raise ValueError("child name and argv must be nonempty")
        object.__setattr__(self, "environment", MappingProxyType(dict(self.environment)))


def gazebo_child_specs(
    *,
    bridge_config: Path,
    environment: Mapping[str, str],
    world_name: str,
) -> tuple[ChildSpec, ...]:
    bridge = ChildSpec(
        "bridge",
        (
            "ros2",
            "run",
            "ros_gz_bridge",
            "parameter_bridge",
            "--ros-args",
            "-p",
            f"config_file:={bridge_config}",
        ),
        environment,
    )
    image_bridges = tuple(
        ChildSpec(
            f"image_bridge_{stream}",
            (
                "ros2",
                "run",
                "ros_gz_image",
                "image_bridge",
                topic,
            ),
            environment,
        )
        for stream, topic in zip(
            ("onboard", "observer"),
            camera_topics_for_world(world_name),
            strict=True,
        )
    )
    return (bridge, *image_bridges)


class ChildSupervisor:
    def __init__(
        self,
        *,
        popen: Callable[..., object] = subprocess.Popen,
        monotonic: Callable[[], float] = time.monotonic,
        signal_process_group: Callable[[int, int], None] = os.killpg,
    ) -> None:
        self._popen = popen
        self._monotonic = monotonic
        self._signal_process_group = signal_process_group
        self._children: list[tuple[str, object]] = []
        self._first_failure: tuple[str, int] | None = None
        self._stopped = False

    def start(self, specs: tuple[ChildSpec, ...]) -> None:
        if self._children:
            raise ChildProcessError("bridge children already started")
        try:
            for spec in specs:
                process = self._popen(
                    spec.argv,
                    env=dict(spec.environment),
                    shell=False,
                    start_new_session=True,
                )
                self._children.append((spec.name, process))
        except Exception as error:
            cleanup_deadline = float(self._monotonic()) + 5.0
            for _name, process in self._children:
                try:
                    if process.poll() is None:
                        self._signal_process_group(process.pid, signal.SIGTERM)
                        process.wait(timeout=max(0.0, self._remaining(cleanup_deadline)))
                except Exception:
                    try:
                        self._signal_process_group(process.pid, signal.SIGKILL)
                    except Exception:
                        pass
            self._stopped = True
            raise ChildProcessError(f"could not start bridge child: {error}") from error

    def poll_failure(self) -> tuple[str, int] | None:
        if self._first_failure is not None:
            return self._first_failure
        if self._stopped:
            return None
        for name, process in self._children:
            returncode = process.poll()
            if returncode is not None:
                self._first_failure = (name, int(returncode))
                return self._first_failure
        return None

    def _remaining(self, deadline: float) -> float:
        now = float(self._monotonic())
        if not math.isfinite(now):
            raise ChildProcessError("monotonic clock is invalid")
        return deadline - now

    def stop(self, deadline_monotonic: float) -> None:
        if self._stopped:
            return
        if self._remaining(deadline_monotonic) <= 0:
            raise ChildProcessError("child stop deadline expired")
        for _name, process in self._children:
            if process.poll() is None:
                self._signal_process_group(process.pid, signal.SIGTERM)
        for _name, process in self._children:
            if process.poll() is not None:
                continue
            remaining = self._remaining(deadline_monotonic)
            if remaining <= 0:
                raise ChildProcessError("child stop deadline expired")
            try:
                process.wait(timeout=remaining)
            except (subprocess.TimeoutExpired, TimeoutError):
                if self._remaining(deadline_monotonic) <= 0:
                    raise ChildProcessError("child stop deadline expired")
                self._signal_process_group(process.pid, signal.SIGKILL)
                remaining = self._remaining(deadline_monotonic)
                if remaining <= 0:
                    raise ChildProcessError("child stop deadline expired")
                process.wait(timeout=remaining)
        self._stopped = True
