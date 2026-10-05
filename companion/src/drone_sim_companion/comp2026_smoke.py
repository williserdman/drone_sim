"""Import the deployed automatic mission without opening devices or transports."""

import importlib
import sys
from collections.abc import Sequence


def main(argv: Sequence[str] | None = None) -> int:
    try:
        for name in (
            "drone.control.drone_control",
            "drone.sensors.camera.camera",
            "drone.sensors.lidar.clearance",
            "drone.sensors.lidar.lidar",
        ):
            importlib.import_module(name)
        functions = importlib.import_module("drone.auto_attempt")._original_mission_functions()
        if not all(callable(getattr(functions, name)) for name in ("fm1", "fm2", "fm3")):
            raise ImportError("FM1/FM2/FM3 must be callable")
    except (ImportError, AttributeError) as error:
        print(f"Comp2026 import smoke failed: {error}", file=sys.stderr)
        return 1
    print("Comp2026 FM1/FM2/FM3 import smoke passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
