"""Fresh observed preflight facts shared by host run control and its console."""

from __future__ import annotations

from dataclasses import asdict
import time
from typing import Any


def preflight_snapshot(protocol: Any) -> dict[str, Any]:
    now = time.time_ns()
    facts: dict[str, Any] = {}
    reasons: list[str] = []
    for component in ("gazebo", "companion"):
        value = protocol.read_preflight(component)
        if value is None:
            reasons.append(f"waiting for {component} observations")
            continue
        facts[component] = asdict(value)
        age = (now - value.observed_wall_time_ns) / 1e9
        facts[component]["age_seconds"] = age
        if not -2 <= age <= 5:
            reasons.append(f"{component} observations are stale")
        if not value.ready:
            reasons.append(value.detail or f"{component} is not ready")
        if component == "companion":
            if value.armed is not False:
                reasons.append("vehicle must be observed disarmed")
            if value.heartbeat_live is not True or value.prearm_checks_healthy is not True:
                reasons.append("waiting for healthy live vehicle telemetry")
        elif value.physics_advancing is not True or value.camera_native_timestamp_ns is None:
            reasons.append("waiting for advancing physics and camera")
    return {"ready": not reasons, "reasons": reasons, "components": facts}
