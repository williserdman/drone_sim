from __future__ import annotations

import json
import os

import pytest

from drone_sim_scorekeeper.status import write_score_finished


RUN_ID = "11111111-1111-4111-8111-111111111111"


def _run_directory(tmp_path):
    (tmp_path / ".status").mkdir()
    return tmp_path


def test_score_finished_has_exact_cross_uid_no_clobber_document(tmp_path):
    """A mutable or unreadable completion fact could falsely close a run."""
    run = _run_directory(tmp_path)
    document = {"run_id": RUN_ID, "finished": True, "sim_timestamp_ns": 50}

    path = write_score_finished(run, RUN_ID, document)

    assert json.loads(path.read_text()) == document
    assert path.stat().st_mode & 0o777 == 0o644
    assert write_score_finished(run, RUN_ID, document) == path
    with pytest.raises(FileExistsError):
        write_score_finished(
            run,
            RUN_ID,
            {"run_id": RUN_ID, "finished": True, "sim_timestamp_ns": 100},
        )


def test_score_finished_mode_is_explicit_under_restrictive_umask(tmp_path):
    """Container umask must not prevent the host controller from reading status."""
    run = _run_directory(tmp_path)
    previous = os.umask(0o077)
    try:
        path = write_score_finished(
            run,
            RUN_ID,
            {"run_id": RUN_ID, "finished": True, "sim_timestamp_ns": 50},
        )
    finally:
        os.umask(previous)

    assert path.stat().st_mode & 0o777 == 0o644
