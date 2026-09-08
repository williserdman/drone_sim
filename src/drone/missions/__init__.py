"""Lazy public exports for mission callables."""

from importlib import import_module
import sys
from types import ModuleType
from typing import Any


__all__ = ["fm1", "fm2", "fm3"]
_MISSION_NAMES = frozenset(__all__)


def _resolve_mission(name: str) -> Any:
    mission = getattr(import_module(f"{__name__}.{name}"), name)
    globals()[name] = mission
    return mission


def __getattr__(name: str) -> Any:
    if name not in _MISSION_NAMES:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    return _resolve_mission(name)


class _MissionPackage(ModuleType):
    def __getattribute__(self, name: str) -> Any:
        try:
            value = super().__getattribute__(name)
        except AttributeError:
            if name in _MISSION_NAMES:
                return _resolve_mission(name)
            raise
        if name in _MISSION_NAMES and isinstance(value, ModuleType):
            return getattr(value, name)
        return value


sys.modules[__name__].__class__ = _MissionPackage
