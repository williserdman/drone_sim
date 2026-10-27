#!/usr/bin/env python3
"""Verify the pinned upstream source and the scope of testbench hooks."""
from __future__ import annotations

import ast
import hashlib
import json
from pathlib import Path
import sys


class _HookBoundary(ast.NodeTransformer):
    def __init__(self, allowed_functions: list[str], *, clock_sentinels: bool):
        self.allowed_functions = allowed_functions
        self.clock_sentinels = clock_sentinels
        self.classes: list[str] = []

    def visit_ClassDef(self, node):
        self.classes.append(node.name)
        result = self.generic_visit(node)
        self.classes.pop()
        return result

    def visit_FunctionDef(self, node):
        if '.'.join([*self.classes, node.name]) in self.allowed_functions:
            return None
        return self.generic_visit(node)

    def visit_Name(self, node):
        if self.clock_sentinels and node.id == 'wall_time':
            node.id = 'time'
        return node

    def visit_Compare(self, node):
        if (self.clock_sentinels and isinstance(node.left, ast.Attribute)
                and isinstance(node.left.value, ast.Name) and node.left.value.id == 'self'
                and node.left.attr in {'mission_begin', 'timer_start'}
                and len(node.ops) == 1 and isinstance(node.ops[0], ast.IsNot)
                and len(node.comparators) == 1
                and isinstance(node.comparators[0], ast.Constant)
                and node.comparators[0].value is None):
            return node.left
        return self.generic_visit(node)


def protected_digest(source: bytes, functions: list[str], *, clock_sentinels: bool = False) -> str:
    tree = ast.parse(source)
    tree.body = [node for node in tree.body if not isinstance(node, (ast.Import, ast.ImportFrom))]
    tree = _HookBoundary(functions, clock_sentinels=clock_sentinels).visit(tree)
    # ast.dump changed its empty-field defaults in Python 3.13. Keep the
    # recorded fingerprint identical on supported developer Python versions.
    def canonical(value):
        if isinstance(value, ast.AST):
            return {"node": type(value).__name__, **{
                name: canonical(child) for name, child in ast.iter_fields(value)
                if child is not None and child != []
            }}
        if isinstance(value, list):
            return [canonical(child) for child in value]
        return value

    return hashlib.sha256(json.dumps(canonical(tree), sort_keys=True).encode()).hexdigest()


def audit(root: Path) -> dict:
    provenance = json.loads((root / 'companion/comp2026-source.json').read_text())
    source = root / 'companion/comp2026'
    changed = []
    errors = []
    for name, expected in provenance['files'].items():
        path = source / name
        if not path.is_file():
            errors.append(f'missing upstream file: {name}')
            continue
        data = path.read_bytes()
        if hashlib.sha256(data).hexdigest() == expected:
            continue
        changed.append(name)
        hook = provenance['hooks'].get(name)
        if hook is None:
            errors.append(f'changed outside hook scope: {name}')
        elif protected_digest(data, hook['functions'], clock_sentinels=hook.get('clock_sentinels', False)) != hook['protected_sha256']:
            errors.append(f'changed flight logic outside injected hooks: {name}')
    ignored = {'__pycache__', '.pytest_cache', '.mypy_cache', '.ruff_cache'}
    for path in source.rglob('*'):
        if not path.is_file() or ignored.intersection(path.relative_to(source).parts):
            continue
        name = path.relative_to(source).as_posix()
        if name not in provenance['files'] and name not in provenance['added_files']:
            errors.append(f'unexpected source addition: {name}')
    if errors:
        raise ValueError('\n'.join(errors))
    return {'upstream_revision': provenance['revision'], 'upstream_tree': provenance['tree'],
            'upstream_files': len(provenance['files']), 'changed_upstream_files': changed,
            'protected_source_unchanged': True}


if __name__ == '__main__':
    try:
        print(json.dumps(audit(Path(__file__).resolve().parents[1]), indent=2))
    except (OSError, ValueError) as error:
        print(error, file=sys.stderr)
        raise SystemExit(1)
