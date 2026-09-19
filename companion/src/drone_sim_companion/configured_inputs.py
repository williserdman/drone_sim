"""Validate the frozen inputs of the registered search-and-deliver mission."""

from copy import deepcopy
import hashlib
from pathlib import Path

from .qgc_runtime_config import (
    _EXPECTED_COURSE,
    _EXPECTED_SCENARIO,
    _digest,
    _read_regular_file,
    _yaml_document,
)


def validate_search_delivery_sources(raw: dict, directory: Path) -> tuple[Path, Path]:
    # Reuse the calibrated payload/camera contract without changing QGC admission.
    course = deepcopy(_EXPECTED_COURSE)
    for name, (x, y) in {"H": (0, 0), "L": (-8, 6), "F2": (6, 20),
                         "WA": (18, 8), "WM": (18, -8)}.items():
        course["waypoints"][name].update(x=float(x), y=float(y))
    course["attempt"]["duration_seconds"] = 240
    scenario = deepcopy(_EXPECTED_SCENARIO)
    scenario["payloads"] = [{"aruco_id": 3, "color": "yellow", "initial": "WA"}]
    scenario["observer_camera"] = {"width_px": 1280, "height_px": 960}
    scenario["mission"] = {
        "aruco_id": 3, "pickup_zone": "WA", "drop_zone": "F2",
        "search_start": {"x": 16.0, "y": 8.0},
    }
    paths = []
    for label, expected in (("course", course), ("scenario", scenario)):
        path = directory / f"{label}.yaml"
        digest = _digest(raw.get(f"{label}_sha256"), f"{label}_sha256")
        content = _read_regular_file(path, label=f"search delivery {label}").content
        if hashlib.sha256(content).hexdigest() != digest:
            raise ValueError(f"search delivery {label} bytes do not match their SHA-256")
        _yaml_document(content, expected=expected, label=f"search delivery {label}")
        paths.append(path)
    return paths[0], paths[1]
