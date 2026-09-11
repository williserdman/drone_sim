"""Dependency-injected checks for an explicitly provisioned SITL binary."""

from __future__ import annotations

from dataclasses import dataclass
import time
from collections.abc import Callable
from typing import Any

from ..common_types import GPSCoord


HOME_LAT = 41.501900
HOME_LON = -81.604900
HOME_AMSL = 300
TARGET_ALT = 30
WAYPOINT = GPSCoord(41.501400, -81.604400, 30)

_SESSION_AUTHORITY = object()


class SitlAuthorizationError(RuntimeError):
    """Raised when simulator-only changes lack an owned SITL session."""


class PrearmRelaxationDisabledError(SitlAuthorizationError):
    """Raised because this helper cannot prove a vehicle belongs to its SITL."""


class ScenarioFailure(RuntimeError):
    """Raised when a controller operation does not report success."""


@dataclass(frozen=True)
class SitlSession:
    process: Any
    connection_string: str
    _authority: object

    def stop(self) -> None:
        self.process.stop()


def _load_sitl_module():
    import dronekit_sitl

    return dronekit_sitl


def start_sitl(
    *,
    executable_path: str,
    sitl_module: Any = None,
    home_lat: float = HOME_LAT,
    home_lon: float = HOME_LON,
    home_amsl_m: float = HOME_AMSL,
) -> SitlSession:
    """Launch a caller-provisioned SITL executable without downloading firmware."""
    if not executable_path:
        raise ValueError("executable_path must be explicit and non-empty")
    module = sitl_module or _load_sitl_module()
    process = module.SITL(executable_path)
    process.launch(
        [
            "-I0",
            "--model",
            "quad",
            f"--home={home_lat},{home_lon},{home_amsl_m},0",
        ],
        await_ready=True,
        restart=True,
    )
    return SitlSession(process, process.connection_string(), _SESSION_AUTHORITY)


def relax_prearm_checks(
    vehicle: Any,
    *,
    session: SitlSession,
    sleep: Callable[[float], None] = time.sleep,
) -> None:
    """Reject safety-check relaxation until SITL can bind its vehicle."""
    raise PrearmRelaxationDisabledError(
        "simulator pre-arm relaxation is disabled until vehicle binding is verified"
    )


def _require_controller_success(operation: str, result: Any) -> None:
    if isinstance(result, bool) or result != 0:
        raise ScenarioFailure(f"{operation} returned {result!r}, expected 0")


def run_scenario(
    controller: Any,
    *,
    waypoint: GPSCoord,
    target_altitude_m: float,
    takeoff: Callable[[Any, float], None],
    hold_seconds: float = 5.0,
    sleep: Callable[[float], None] = time.sleep,
) -> bool:
    """Run a small SITL-only scenario against injected controller operations."""
    takeoff(controller.vehicle, target_altitude_m)
    sleep(hold_seconds)
    _require_controller_success("goto_waypoint", controller.goto_waypoint(waypoint))
    _require_controller_success("simple_land", controller.simple_land())
    print("[SITL] Scenario passed all checked controller outcomes")
    return True


def main() -> int:
    print(
        "SITL controller wiring is pending the production controller interface; "
        "use start_sitl() and run_scenario() with explicit dependencies."
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
