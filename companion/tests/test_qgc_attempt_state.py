from __future__ import annotations

from dataclasses import FrozenInstanceError
import os
from pathlib import Path
import socket
import subprocess
import sys

import pytest

from drone_sim_companion.qgc_attempt_state import (
    AttemptStateBinding,
    AttemptStateError,
    _validate_regular_file_at,
    resolve_attempt_state,
)


DIGEST = "a" * 64
ATTEMPT_STATE_ID = f"sha256-{DIGEST}"


def make_state(tmp_path: Path, *, ledger: bytes = b"not validated as json") -> Path:
    state_root = tmp_path / "state"
    state_directory = state_root / ATTEMPT_STATE_ID
    state_directory.mkdir(parents=True)
    (state_directory / "attempt-ledger.json").write_bytes(ledger)
    (state_directory / "attempt-ledger.json.lock").write_bytes(b"")
    return state_root


def resolve(state_root: Path, *, forbidden_paths: tuple[Path, ...] = ()) -> AttemptStateBinding:
    return resolve_attempt_state(
        DIGEST,
        ATTEMPT_STATE_ID,
        forbidden_paths=forbidden_paths,
        test_only_state_root=state_root,
    )


def snapshot_tree(root: Path) -> tuple[tuple[str, int, int, bytes | None], ...]:
    snapshot = []
    for path in sorted((root, *root.rglob("*"))):
        stat_result = path.lstat()
        snapshot.append(
            (
                str(path.relative_to(root)),
                stat_result.st_mode,
                stat_result.st_mtime_ns,
                path.read_bytes() if path.is_file() else None,
            )
        )
    return tuple(snapshot)


def bind_unix_socket(path: Path) -> socket.socket:
    """Bind using a relative name to stay below AF_UNIX's path-length limit."""

    saved_directory_fd = os.open(".", os.O_RDONLY)
    listener = socket.socket(socket.AF_UNIX)
    try:
        os.chdir(path.parent)
        listener.bind(path.name)
    finally:
        os.fchdir(saved_directory_fd)
        os.close(saved_directory_fd)
    return listener


def test_resolves_canonical_existing_binding_without_reading_ledger_contents(
    tmp_path: Path,
) -> None:
    state_root = make_state(tmp_path)

    binding = resolve(state_root)

    assert binding == AttemptStateBinding(
        attempt_state_id=ATTEMPT_STATE_ID,
        state_directory=state_root / ATTEMPT_STATE_ID,
        ledger_path=state_root / ATTEMPT_STATE_ID / "attempt-ledger.json",
        lock_path=state_root / ATTEMPT_STATE_ID / "attempt-ledger.json.lock",
    )
    with pytest.raises(FrozenInstanceError):
        binding.attempt_state_id = "changed"  # type: ignore[misc]


def test_same_profile_resolves_same_binding_across_distinct_run_paths(
    tmp_path: Path,
) -> None:
    state_root = make_state(tmp_path)
    first_run = tmp_path / "runs" / "run-1"
    second_run = tmp_path / "runs" / "run-2"

    first = resolve(state_root, forbidden_paths=(first_run, first_run / "configuration"))
    second = resolve(
        state_root, forbidden_paths=(second_run, second_run / "configuration")
    )

    assert first == second


def test_resolution_does_not_change_bytes_mtimes_or_directory_entries(
    tmp_path: Path,
) -> None:
    state_root = make_state(tmp_path, ledger=b'{"counter": 7}\n')
    lock_path = state_root / ATTEMPT_STATE_ID / "attempt-ledger.json.lock"
    lock_path.write_bytes(b"existing lock bytes")
    before = snapshot_tree(state_root)

    resolve(state_root)

    assert snapshot_tree(state_root) == before


def test_import_is_inert_in_an_empty_working_directory(tmp_path: Path) -> None:
    source_root = Path(__file__).parents[1] / "src"
    before = tuple(tmp_path.iterdir())

    completed = subprocess.run(
        [sys.executable, "-c", "import drone_sim_companion.qgc_attempt_state"],
        cwd=tmp_path,
        env={**os.environ, "PYTHONPATH": str(source_root)},
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert tuple(tmp_path.iterdir()) == before


@pytest.mark.parametrize(
    ("digest", "attempt_state_id"),
    [
        ("A" * 64, f"sha256-{'A' * 64}"),
        ("a" * 63, f"sha256-{'a' * 63}"),
        ("g" * 64, f"sha256-{'g' * 64}"),
        (DIGEST, DIGEST),
        (DIGEST, f"sha256-{DIGEST}/attempt-ledger.json"),
        (DIGEST, f"sha256-{'b' * 64}"),
    ],
)
def test_rejects_bad_digest_or_attempt_state_id(
    tmp_path: Path, digest: str, attempt_state_id: str
) -> None:
    state_root = make_state(tmp_path)

    with pytest.raises(AttemptStateError):
        resolve_attempt_state(
            digest,
            attempt_state_id,
            forbidden_paths=(),
            test_only_state_root=state_root,
        )


def test_rejects_relative_or_filesystem_root_state_root(tmp_path: Path) -> None:
    for bad_root in (Path("relative-state"), Path("/"), Path("//")):
        with pytest.raises(AttemptStateError):
            resolve_attempt_state(
                DIGEST,
                ATTEMPT_STATE_ID,
                forbidden_paths=(),
                test_only_state_root=bad_root,
            )


def test_rejects_nul_state_root_without_leaking_directory_descriptors(
    tmp_path: Path,
) -> None:
    malformed_root = f"{tmp_path}/\0state"
    descriptors_before = len(tuple(Path("/proc/self/fd").iterdir()))

    for _ in range(10):
        with pytest.raises(AttemptStateError):
            resolve_attempt_state(
                DIGEST,
                ATTEMPT_STATE_ID,
                forbidden_paths=(),
                test_only_state_root=malformed_root,
            )

    assert len(tuple(Path("/proc/self/fd").iterdir())) == descriptors_before


def test_rejects_nul_forbidden_path_as_domain_error(tmp_path: Path) -> None:
    state_root = make_state(tmp_path)

    with pytest.raises(AttemptStateError):
        resolve(state_root, forbidden_paths=(f"{tmp_path}/\0output",))


def test_rejects_double_slash_state_root_alias_of_forbidden_path(
    tmp_path: Path,
) -> None:
    state_root = make_state(tmp_path)
    double_slash_alias = f"//{str(state_root).lstrip('/')}"

    with pytest.raises(AttemptStateError):
        resolve_attempt_state(
            DIGEST,
            ATTEMPT_STATE_ID,
            forbidden_paths=(state_root,),
            test_only_state_root=double_slash_alias,
        )


def test_rejects_double_slash_forbidden_path_alias(tmp_path: Path) -> None:
    state_root = make_state(tmp_path)
    double_slash_alias = f"//{str(state_root).lstrip('/')}"

    with pytest.raises(AttemptStateError):
        resolve(state_root, forbidden_paths=(double_slash_alias,))


@pytest.mark.parametrize("missing_part", ["root", "directory", "ledger"])
def test_rejects_missing_required_state(
    tmp_path: Path, missing_part: str
) -> None:
    state_root = tmp_path / "state"
    if missing_part != "root":
        state_root.mkdir()
    if missing_part == "ledger":
        (state_root / ATTEMPT_STATE_ID).mkdir()

    with pytest.raises(AttemptStateError):
        resolve(state_root)


def test_requires_regular_lock_file(tmp_path: Path) -> None:
    state_root = make_state(tmp_path)
    lock_path = state_root / ATTEMPT_STATE_ID / "attempt-ledger.json.lock"
    lock_path.unlink()
    with pytest.raises(AttemptStateError, match="lock"):
        resolve(state_root)

    lock_path.write_bytes(b"")
    assert resolve(state_root).lock_path.name == "attempt-ledger.json.lock"


@pytest.mark.parametrize(
    "target",
    ["root-link", "intermediate-root-component", "directory", "ledger", "lock"],
)
def test_rejects_symlinks_in_state_paths(tmp_path: Path, target: str) -> None:
    real_root = make_state(tmp_path / "real")
    state_root = real_root
    state_directory = state_root / ATTEMPT_STATE_ID

    if target == "root-link":
        alias = tmp_path / "state-alias"
        alias.symlink_to(real_root, target_is_directory=True)
        state_root = alias
    elif target == "intermediate-root-component":
        alias = tmp_path / "parent-alias"
        alias.symlink_to(real_root.parent, target_is_directory=True)
        state_root = alias / real_root.name
    elif target == "directory":
        alternate = tmp_path / "alternate-directory"
        alternate.mkdir()
        (alternate / "attempt-ledger.json").write_bytes(b"ledger")
        state_directory.rename(tmp_path / "saved-directory")
        state_directory.symlink_to(alternate, target_is_directory=True)
    elif target == "ledger":
        ledger = state_directory / "attempt-ledger.json"
        alternate = tmp_path / "alternate-ledger"
        alternate.write_bytes(b"ledger")
        ledger.unlink()
        ledger.symlink_to(alternate)
    else:
        lock_path = state_directory / "attempt-ledger.json.lock"
        lock_path.unlink()
        lock_path.symlink_to(
            state_directory / "attempt-ledger.json"
        )

    with pytest.raises(AttemptStateError):
        resolve(state_root)


@pytest.mark.parametrize("kind", ["directory", "fifo", "socket"])
def test_rejects_non_regular_ledger(tmp_path: Path, kind: str) -> None:
    state_root = make_state(tmp_path)
    ledger_path = state_root / ATTEMPT_STATE_ID / "attempt-ledger.json"
    ledger_path.unlink()

    listener: socket.socket | None = None
    if kind == "directory":
        ledger_path.mkdir()
    elif kind == "fifo":
        os.mkfifo(ledger_path)
    elif kind == "socket":
        listener = bind_unix_socket(ledger_path)

    try:
        with pytest.raises(AttemptStateError):
            resolve(state_root)
    finally:
        if listener is not None:
            listener.close()


def test_rejects_real_device_as_non_regular_file() -> None:
    device_directory_fd = os.open("/dev", os.O_RDONLY | os.O_DIRECTORY)
    try:
        with pytest.raises(AttemptStateError):
            _validate_regular_file_at(
                device_directory_fd,
                "null",
                label="attempt ledger",
                required=True,
            )
    finally:
        os.close(device_directory_fd)


@pytest.mark.parametrize("kind", ["directory", "fifo", "socket"])
def test_rejects_non_regular_present_lock(tmp_path: Path, kind: str) -> None:
    state_root = make_state(tmp_path)
    lock_path = state_root / ATTEMPT_STATE_ID / "attempt-ledger.json.lock"
    lock_path.unlink()
    listener: socket.socket | None = None
    if kind == "directory":
        lock_path.mkdir()
    elif kind == "fifo":
        os.mkfifo(lock_path)
    else:
        listener = bind_unix_socket(lock_path)

    try:
        with pytest.raises(AttemptStateError):
            resolve(state_root)
    finally:
        if listener is not None:
            listener.close()


def test_rejects_extra_or_alternate_filename(tmp_path: Path) -> None:
    state_root = make_state(tmp_path)
    (state_root / ATTEMPT_STATE_ID / "ledger.json").write_bytes(b"alternate")

    with pytest.raises(AttemptStateError):
        resolve(state_root)


@pytest.mark.parametrize(
    "forbidden_selector",
    [
        "root",
        "parent",
        "directory",
        "ledger",
        "lock",
        "inside-state",
    ],
)
def test_rejects_state_and_forbidden_path_overlap(
    tmp_path: Path, forbidden_selector: str
) -> None:
    state_root = make_state(tmp_path)
    state_directory = state_root / ATTEMPT_STATE_ID
    choices = {
        "root": state_root,
        "parent": state_root.parent,
        "directory": state_directory,
        "ledger": state_directory / "attempt-ledger.json",
        "lock": state_directory / "attempt-ledger.json.lock",
        "inside-state": state_directory / "future-configuration",
    }

    with pytest.raises(AttemptStateError):
        resolve(state_root, forbidden_paths=(choices[forbidden_selector],))


def test_rejects_forbidden_symlink_alias_of_state(tmp_path: Path) -> None:
    state_root = make_state(tmp_path)
    alias = tmp_path / "output-alias"
    alias.symlink_to(state_root, target_is_directory=True)

    with pytest.raises(AttemptStateError):
        resolve(state_root, forbidden_paths=(alias,))
