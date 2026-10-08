"""Capability declarations for the native-Linux repository test harness."""

from __future__ import annotations

import os

import pytest


_LIVE_DOCKER_MODULE = "tests/integration/test_phase2_compose.py::"
_LIVE_FOUNDATION_TEST = (
    "tests/integration/test_foundation_compose.py::"
    "test_foundation_compose_emits_an_ordered_structured_lifecycle"
)
_DOCKER_SKIP_REASON = (
    "requires a live Docker daemon; scripts/test-linux.sh deliberately does not "
    "mount the host Docker socket"
)


def pytest_report_header() -> list[str]:
    return [
        "Linux harness filesystem: Docker named volume",
        "Linux harness Docker daemon: unavailable (live Compose tests skip explicitly)",
        "Linux harness ROS 2: unavailable (ROS tests retain their explicit import skips)",
    ]


def pytest_collection_modifyitems(items: list[pytest.Item]) -> None:
    if os.environ.get("DRONE_SIM_LINUX_DOCKER_DAEMON") == "1":
        return
    skip = pytest.mark.skip(reason=_DOCKER_SKIP_REASON)
    for item in items:
        if item.nodeid.startswith(_LIVE_DOCKER_MODULE) or item.nodeid.startswith(
            _LIVE_FOUNDATION_TEST
        ):
            item.add_marker(skip)
