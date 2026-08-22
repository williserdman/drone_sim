#!/usr/bin/env python3
"""Observe the synthetic foundation topics through DDS and record the result."""

import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from simulation_interfaces.msg import RunState


OBSERVATION_TIMEOUT_SECONDS = 30.0
CLOCK_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
RUN_STATE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _qos_dict(profile: QoSProfile) -> dict[str, object]:
    return {
        "history": profile.history.name,
        "depth": profile.depth,
        "reliability": profile.reliability.name,
        "durability": profile.durability.name,
    }


def _offered_qos_dict(profile: QoSProfile) -> dict[str, str]:
    return {
        "reliability": profile.reliability.name,
        "durability": profile.durability.name,
    }


def _is_compatible(publisher: QoSProfile, subscriber: QoSProfile) -> bool:
    reliability_compatible = (
        subscriber.reliability == ReliabilityPolicy.BEST_EFFORT
        or publisher.reliability == ReliabilityPolicy.RELIABLE
    )
    durability_compatible = (
        subscriber.durability == DurabilityPolicy.VOLATILE
        or publisher.durability == DurabilityPolicy.TRANSIENT_LOCAL
    )
    return reliability_compatible and durability_compatible


def _time_dict(message) -> dict[str, int]:
    return {"sec": message.sec, "nanosec": message.nanosec}


class FoundationObserver(Node):
    def __init__(self, output_path: Path) -> None:
        super().__init__("synthetic_foundation_observer")
        self.output_path = output_path
        self.clock_messages: list[dict[str, int]] = []
        self.run_state_messages: list[dict[str, object]] = []
        self.publisher_qos: dict[str, QoSProfile] = {}
        self.create_subscription(Clock, "/clock", self._observe_clock, CLOCK_QOS)
        self.create_subscription(
            RunState,
            "/simulation/run_state",
            self._observe_run_state,
            RUN_STATE_QOS,
        )

    def _observe_clock(self, message: Clock) -> None:
        self.clock_messages.append(_time_dict(message.clock))
        self.write_report()

    def _observe_run_state(self, message: RunState) -> None:
        observed = {
            "run_id": message.run_id,
            "sim_timestamp": _time_dict(message.sim_timestamp),
            "state": message.state,
            "reason": message.reason,
            "config_sha256": message.config_sha256,
        }
        self.run_state_messages.append(observed)
        self.write_report()

    def discover_publishers(self) -> None:
        for topic in ("/clock", "/simulation/run_state"):
            endpoint_info = self.get_publishers_info_by_topic(topic)
            if endpoint_info:
                self.publisher_qos[topic] = endpoint_info[0].qos_profile

    @property
    def discovered(self) -> bool:
        return len(self.publisher_qos) == 2

    @property
    def complete(self) -> bool:
        return len(self.clock_messages) == 3 and len(self.run_state_messages) == 5

    def write_report(self, error: str | None = None) -> None:
        report = {
            "discovered": self.discovered,
            "complete": self.complete,
            "clock": self.clock_messages,
            "run_state": self.run_state_messages,
            "qos": {
                "/clock": {
                    "publisher": (
                        _offered_qos_dict(self.publisher_qos["/clock"])
                        if "/clock" in self.publisher_qos
                        else None
                    ),
                    "subscriber": _qos_dict(CLOCK_QOS),
                    "compatible": (
                        _is_compatible(self.publisher_qos["/clock"], CLOCK_QOS)
                        if "/clock" in self.publisher_qos
                        else False
                    ),
                },
                "/simulation/run_state": {
                    "publisher": (
                        _offered_qos_dict(self.publisher_qos["/simulation/run_state"])
                        if "/simulation/run_state" in self.publisher_qos
                        else None
                    ),
                    "subscriber": _qos_dict(RUN_STATE_QOS),
                    "compatible": (
                        _is_compatible(
                            self.publisher_qos["/simulation/run_state"],
                            RUN_STATE_QOS,
                        )
                        if "/simulation/run_state" in self.publisher_qos
                        else False
                    ),
                },
            },
            "error": error,
        }
        temporary = self.output_path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(report, allow_nan=False, sort_keys=True, separators=(",", ":")),
            encoding="utf-8",
        )
        os.replace(temporary, self.output_path)


def main() -> None:
    output_path = Path(os.environ["SIM_OUTPUT_ROOT"]) / "foundation-observation.json"
    rclpy.init()
    observer = FoundationObserver(output_path)
    deadline = time.monotonic() + OBSERVATION_TIMEOUT_SECONDS
    try:
        observer.write_report()
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.1)
            observer.discover_publishers()
            observer.write_report()
            if observer.discovered and observer.complete:
                return
        error = (
            "timed out observing foundation topics: "
            f"discovered={observer.discovered}, clocks={len(observer.clock_messages)}, "
            f"run_states={len(observer.run_state_messages)}"
        )
        observer.write_report(error=error)
        raise RuntimeError(error)
    finally:
        observer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
