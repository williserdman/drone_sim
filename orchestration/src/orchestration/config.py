"""Immutable run configuration loading and validation."""

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from uuid import UUID


_FIELDS = {
    "run_id",
    "world",
    "vehicle",
    "mission",
    "scenario",
    "output_root",
    "max_wall_seconds",
}


@dataclass(frozen=True)
class RunConfig:
    run_id: str
    world: str
    vehicle: str
    mission: str
    scenario: str
    output_root: Path
    max_wall_seconds: int
    config_sha256: str


def load_run_config(path: str | Path) -> RunConfig:
    source = Path(path)
    try:
        document = json.loads(source.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid run configuration: {source}") from exc
    if not isinstance(document, dict):
        raise ValueError("run configuration must be a JSON object")
    if set(document) != _FIELDS:
        raise ValueError("run configuration has missing or unknown keys")
    try:
        run_uuid = UUID(document["run_id"])
    except (ValueError, AttributeError, TypeError) as exc:
        raise ValueError("run_id must be a valid UUID") from exc
    if any(
        not isinstance(document[field], str) or not document[field]
        for field in ("world", "vehicle", "mission", "scenario", "output_root")
    ):
        raise ValueError("run configuration string fields must be non-empty")
    timeout = document["max_wall_seconds"]
    if isinstance(timeout, bool) or not isinstance(timeout, int) or timeout < 1:
        raise ValueError("max_wall_seconds must be an integer of at least one")
    canonical = json.dumps(document, sort_keys=True, separators=(",", ":")).encode()
    return RunConfig(
        run_id=str(run_uuid),
        world=document["world"],
        vehicle=document["vehicle"],
        mission=document["mission"],
        scenario=document["scenario"],
        output_root=Path(document["output_root"]).resolve(),
        max_wall_seconds=timeout,
        config_sha256=hashlib.sha256(canonical).hexdigest(),
    )
