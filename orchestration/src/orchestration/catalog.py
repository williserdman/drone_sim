"""Operator selection over the complete, approved CI preset catalog."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class Preset:
    id: str
    template: Path
    world: str
    vehicle: str
    mission: str
    scenario: str
    configuration: dict[str, Any]
    manual_arming: bool
    requires_calibration: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id, "label": self.id.replace("-", " ").title(),
            "world": self.world, "vehicle": self.vehicle, "mission": self.mission,
            "scenario": self.scenario, "configuration": self.configuration,
            "manual_arming": self.manual_arming,
            "requires_calibration": self.requires_calibration,
        }


class PresetCatalog:
    def __init__(self, project_directory: Path) -> None:
        root = project_directory.resolve()
        document = json.loads((root / "config/ci-suite.json").read_text())
        if document.get("schema_version") != 1 or not isinstance(document.get("cases"), list):
            raise ValueError("invalid preset catalog")
        self._presets: dict[str, Preset] = {}
        for row in document["cases"]:
            source = (root / row["template"]).resolve()
            if not source.is_relative_to(root):
                raise ValueError("preset template must belong to this checkout")
            configuration = json.loads(source.read_text())
            if configuration.get("runtime_profile") != "phase3":
                raise ValueError("operator presets require the real flight runtime")
            preset = Preset(
                id=row["name"], template=source,
                world=configuration["world"], vehicle=configuration["vehicle"],
                mission=configuration["mission"], scenario=configuration["scenario"],
                configuration=configuration,
                manual_arming=row["acceptance"] == "operator_wait",
                requires_calibration="calibration" in configuration,
            )
            if preset.id in self._presets:
                raise ValueError("duplicate preset identifier")
            self._presets[preset.id] = preset

    def list(self, *, world: str | None = None) -> tuple[Preset, ...]:
        return tuple(row for row in self._presets.values() if world is None or row.world == world)

    def get(self, preset_id: str) -> Preset:
        try:
            return self._presets[preset_id]
        except KeyError as error:
            raise ValueError(f"unknown preset: {preset_id}") from error
