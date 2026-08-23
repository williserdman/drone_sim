import hashlib
import os
from pathlib import Path

import pytest

from artifacts.validation import (
    ValidationResult,
    ValidationStatus,
    validate_regular_file,
    validate_tree,
)


def test_validation_result_is_immutable():
    result = ValidationResult(ValidationStatus.VALID, 3, "a" * 64, "valid regular file")

    with pytest.raises(AttributeError):
        result.detail = "changed"


def test_validate_regular_file_returns_size_and_streamed_sha256(tmp_path):
    path = tmp_path / "logs/artifacts.jsonl"
    path.parent.mkdir()
    path.write_bytes(b"abc")

    result = validate_regular_file(tmp_path, "logs/artifacts.jsonl")

    assert result == ValidationResult(
        ValidationStatus.VALID,
        3,
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
        "valid regular file",
    )


def test_validate_regular_file_reports_missing_path(tmp_path):
    assert validate_regular_file(tmp_path, "missing.bin") == ValidationResult(
        ValidationStatus.MISSING, None, None, "path is missing"
    )


@pytest.mark.parametrize("relative_path", ["../outside.bin", "/tmp/outside.bin"])
def test_validate_regular_file_rejects_path_escape(tmp_path, relative_path):
    assert validate_regular_file(tmp_path, relative_path) == ValidationResult(
        ValidationStatus.INVALID, None, None, "path escapes run directory"
    )


def test_validate_regular_file_rejects_symlink(tmp_path):
    target = tmp_path / "target.bin"
    target.write_bytes(b"target")
    (tmp_path / "link.bin").symlink_to(target)

    assert validate_regular_file(tmp_path, "link.bin") == ValidationResult(
        ValidationStatus.INVALID, None, None, "symlinks are not allowed"
    )


def test_validate_regular_file_rejects_fifo(tmp_path):
    os.mkfifo(tmp_path / "pipe")

    assert validate_regular_file(tmp_path, "pipe") == ValidationResult(
        ValidationStatus.INVALID, None, None, "path is not a regular file"
    )


def test_validate_regular_file_rejects_multiple_hard_links(tmp_path):
    source = tmp_path / "source.bin"
    source.write_bytes(b"same inode")
    os.link(source, tmp_path / "linked.bin")

    assert validate_regular_file(tmp_path, "source.bin") == ValidationResult(
        ValidationStatus.INVALID,
        None,
        None,
        "regular files must have exactly one hard link",
    )


def test_validate_regular_file_rejects_unreadable_file(tmp_path, monkeypatch):
    path = tmp_path / "unreadable.bin"
    path.write_bytes(b"secret")
    real_open = os.open

    def reject_target(candidate, flags):
        if Path(candidate) == path:
            raise PermissionError("unreadable fixture")
        return real_open(candidate, flags)

    monkeypatch.setattr(os, "open", reject_target)

    assert validate_regular_file(tmp_path, "unreadable.bin") == ValidationResult(
        ValidationStatus.INVALID, None, None, "file could not be read"
    )


def test_validate_tree_hashes_sorted_relative_rows_and_sums_file_sizes(tmp_path):
    tree = tmp_path / "rosbag"
    (tree / "nested").mkdir(parents=True)
    (tree / "z.txt").write_bytes(b"z")
    (tree / "nested/a.txt").write_bytes(b"ab")
    a_sha = hashlib.sha256(b"ab").hexdigest()
    z_sha = hashlib.sha256(b"z").hexdigest()
    rows = f"nested/a.txt\0{2}\0{a_sha}\n" f"z.txt\0{1}\0{z_sha}\n"

    result = validate_tree(tmp_path, "rosbag")

    assert result == ValidationResult(
        ValidationStatus.VALID,
        3,
        hashlib.sha256(rows.encode("utf-8")).hexdigest(),
        "valid directory tree",
    )


def test_validate_tree_reports_missing_path(tmp_path):
    assert validate_tree(tmp_path, "rosbag") == ValidationResult(
        ValidationStatus.MISSING, None, None, "path is missing"
    )


def test_validate_tree_rejects_empty_required_tree(tmp_path):
    (tmp_path / "rosbag").mkdir()

    assert validate_tree(tmp_path, "rosbag") == ValidationResult(
        ValidationStatus.INVALID, None, None, "directory tree is empty"
    )


def test_validate_tree_rejects_file_where_directory_is_required(tmp_path):
    (tmp_path / "rosbag").write_bytes(b"not a directory")

    assert validate_tree(tmp_path, "rosbag") == ValidationResult(
        ValidationStatus.INVALID, None, None, "path is not a directory"
    )


def test_validate_tree_rejects_symlink_anywhere_in_tree(tmp_path):
    tree = tmp_path / "rosbag"
    tree.mkdir()
    target = tmp_path / "target.bin"
    target.write_bytes(b"target")
    (tree / "link.bin").symlink_to(target)

    assert validate_tree(tmp_path, "rosbag") == ValidationResult(
        ValidationStatus.INVALID, None, None, "symlinks are not allowed"
    )


def test_validate_tree_rejects_fifo_anywhere_in_tree(tmp_path):
    tree = tmp_path / "rosbag"
    tree.mkdir()
    os.mkfifo(tree / "pipe")

    assert validate_tree(tmp_path, "rosbag") == ValidationResult(
        ValidationStatus.INVALID,
        None,
        None,
        "directory tree contains a non-regular file",
    )


def test_validate_tree_rejects_path_escape(tmp_path):
    assert validate_tree(tmp_path, "../outside") == ValidationResult(
        ValidationStatus.INVALID, None, None, "path escapes run directory"
    )
