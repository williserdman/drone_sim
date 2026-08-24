#!/usr/bin/env python3
"""No-control Phase 2 companion and ArduPilot process fixtures."""

from __future__ import annotations

from datetime import datetime, timezone
import os
from pathlib import Path
import sys
from typing import Any


def write_bytes_atomic(path: Path, payload: bytes) -> None:
    """Durably replace one owned fixture file through a sibling temporary."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    descriptor: int | None = None
    try:
        descriptor = os.open(
            temporary,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0),
            # Synthetic fixtures are consumed by the non-root host manifest
            # builder after being created by root-run containers.
            0o644,
        )
        written = 0
        while written < len(payload):
            count = os.write(descriptor, payload[written:])
            if count <= 0:
                raise OSError("fixture write made no progress")
            written += count
        os.fsync(descriptor)
        os.close(descriptor)
        descriptor = None
        os.replace(temporary, path)
        directory_fd = os.open(path.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if descriptor is not None:
            os.close(descriptor)
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from simulation_interfaces.msg import RunState
    from artifacts.runtime_protocol import RuntimeProtocol
    from artifacts.structured_log import StructuredEvent, write_event

    run_id = os.environ["SIM_RUN_ID"]
    module = os.environ["SIM_MODULE"]
    if module not in {"companion", "ardupilot_sitl"}:
        raise ValueError("module_stub owns only companion or ardupilot_sitl")
    protocol = RuntimeProtocol(Path(os.environ["SIM_RUN_DIRECTORY"]), run_id)
    rclpy.init()
    node = Node(f"synthetic_{module}")
    finalizing = False

    def emit(event: str) -> None:
        write_event(
            sys.stdout,
            StructuredEvent(
                run_id=run_id,
                module=module,
                severity="INFO",
                event=event,
                sim_timestamp=0.0,
                wall_timestamp=datetime.now(timezone.utc),
                fields={"fixture": True},
            ),
        )

    def callback(message: Any) -> None:
        nonlocal finalizing
        if message.run_id == run_id and message.state == RunState.FINALIZING and not finalizing:
            emit("finalizing")
            finalizing = True

    node.create_subscription(
        RunState,
        "/simulation/run_state",
        callback,
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    emit("starting")
    emit("ready")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if finalizing and protocol.read_terminal_committed() is not None:
                break
    finally:
        node.destroy_node()
        protocol.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
