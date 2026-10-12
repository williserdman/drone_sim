import json
import math
from collections.abc import Mapping


def calibration_parameters(config) -> dict[str, float]:
    raw = getattr(config, "calibration_json", None)
    if raw is None:
        return {}

    document = json.loads(raw)
    profile = document["profile"]
    if profile.get("schema_version") == 2:
        baseline = profile["effective_baseline_parameters"]
    else:
        baseline = profile["baseline_parameters"]
    return {**baseline, **document["gains"]}


class CalibrationGate:
    def __init__(self, expected: Mapping[str, float]) -> None:
        self._expected = dict(expected)
        self._observed: dict[str, float] = {}
        self.failure: str | None = None

    @property
    def ready(self) -> bool:
        return self.failure is None and self._observed.keys() == self._expected.keys()

    def observe(self, name: str, value: float) -> None:
        if self.failure is not None or name not in self._expected:
            return
        expected = self._expected[name]
        if not math.isclose(value, expected, rel_tol=1e-5, abs_tol=1e-7):
            self.failure = (
                f"effective required parameter {name} is {value}, expected {expected}"
            )
            return
        self._observed[name] = value

    def snapshot(self) -> dict[str, float]:
        return dict(self._observed)
