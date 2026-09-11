from __future__ import annotations

import ast
import importlib
from pathlib import Path
from pathlib import PurePosixPath
import sys


ROOT = Path(__file__).parents[2]
NESTED_SRC = ROOT / "companion/comp2026/src"


def _module_file(module: str) -> Path | None:
    candidate = NESTED_SRC.joinpath(*module.split(".")).with_suffix(".py")
    return candidate if candidate.is_file() else None


def _local_imports(module: str, source: str) -> set[str]:
    package = module.rpartition(".")[0]
    imports: set[str] = set()
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Import):
            imports.update(alias.name for alias in node.names if alias.name.startswith("drone."))
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                parts = package.split(".")
                base = ".".join(parts[: len(parts) - node.level + 1])
                imported = f"{base}.{node.module}" if node.module else base
                if node.module:
                    imports.add(imported)
                else:
                    imports.update(f"{base}.{alias.name}" for alias in node.names)
            elif node.module and node.module.startswith("drone."):
                imports.add(node.module)
    return imports


def _python_import_closure() -> set[Path]:
    pending = [
        "drone.auto_attempt",
        "drone.control.listener",
        "drone.missions.fm1",
        "drone.missions.fm2",
        "drone.missions.fm3",
    ]
    found: set[Path] = {NESTED_SRC / "gc/prepare_attempt.py"}
    visited: set[str] = set()
    while pending:
        module = pending.pop()
        if module in visited:
            continue
        visited.add(module)
        path = _module_file(module)
        if path is None:
            continue
        found.add(path)
        relative = path.relative_to(NESTED_SRC)
        for parent in relative.parents:
            initializer = NESTED_SRC / parent / "__init__.py"
            if initializer.is_file():
                found.add(initializer)
        pending.extend(_local_imports(module, path.read_text(encoding="utf-8")))
    found.update(
        {
            NESTED_SRC / "drone/control/mission_data/payloads.json",
            NESTED_SRC / "drone/sensors/camera/calibration.json",
            NESTED_SRC / "drone/sensors/camera/mounting.json",
            NESTED_SRC / "gc/custom_missions.json",
        }
    )
    return found


def _context_includes(path: str, rules: list[str]) -> bool:
    included = True
    target = PurePosixPath(path)
    for raw_rule in rules:
        if not raw_rule or raw_rule.startswith("#"):
            continue
        negated = raw_rule.startswith("!")
        pattern = raw_rule.removeprefix("!").removeprefix("/")
        if pattern.endswith("/**"):
            prefix = pattern.removesuffix("/**")
            matches = path == prefix or path.startswith(f"{prefix}/")
        elif pattern.endswith("/"):
            matches = path == pattern.removesuffix("/")
        else:
            matches = target.match(pattern)
        if matches:
            included = negated
    return included


def test_docker_context_admits_the_actual_qgc_and_automatic_import_closure() -> None:
    rules = (ROOT / ".dockerignore").read_text(encoding="utf-8").splitlines()
    missing = []
    for path in sorted(_python_import_closure()):
        relative = path.relative_to(ROOT).as_posix()
        if not _context_includes(relative, rules):
            missing.append(relative)

    assert missing == []


def test_docker_context_excludes_mutable_operator_runtime_state() -> None:
    rules = (ROOT / ".dockerignore").read_text(encoding="utf-8").splitlines()
    runtime_state = (
        "companion/comp2026/src/drone/control/mission_data/waypoints.json",
        "companion/comp2026/src/gc/current-attempt-session.json",
        "companion/comp2026/src/gc/attempt-ledger.json",
    )

    assert [path for path in runtime_state if _context_includes(path, rules)] == []


def test_qgc_listener_import_is_inert(monkeypatch) -> None:
    monkeypatch.syspath_prepend(str(NESTED_SRC))
    sys.modules.pop("gpiozero", None)

    listener = importlib.import_module("drone.control.listener")

    assert callable(listener.start_repl)
    assert "gpiozero" not in sys.modules
