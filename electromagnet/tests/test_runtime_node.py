from __future__ import annotations

from types import SimpleNamespace

from drone_sim_electromagnet.runtime_node import assign_stamp, stamp_ns


def test_ros_timestamp_conversion_is_exact() -> None:
    source = SimpleNamespace(sec=12, nanosec=345)
    destination = SimpleNamespace(sec=0, nanosec=0)
    assert stamp_ns(source) == 12_000_000_345
    assign_stamp(destination, 12_000_000_345)
    assert (destination.sec, destination.nanosec) == (12, 345)
