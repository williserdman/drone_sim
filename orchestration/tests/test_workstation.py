import os
from pathlib import Path
import subprocess
import sys

import pytest

from orchestration.workstation import workstation_lock


def test_workstation_lock_is_reentrant_only_for_same_owner_and_blocks_other_process(tmp_path):
    lock_path = tmp_path / "workstation.lock"
    with workstation_lock(lock_path):
        with workstation_lock(lock_path, allow_nested=True):
            result = subprocess.run(
                [sys.executable, "-c", "from pathlib import Path\nfrom orchestration.workstation import workstation_lock\n"
                 f"with workstation_lock(Path({str(lock_path)!r})):\n    pass\n"],
                capture_output=True, text=True,
            )
            assert result.returncode != 0
            assert "already owns" in result.stderr
    with workstation_lock(lock_path):
        pass


def test_lock_rejects_symlink_without_touching_target(tmp_path):
    target = tmp_path / "target"
    target.write_text("unchanged")
    link = tmp_path / "lock"
    link.symlink_to(target)
    with pytest.raises(OSError):
        with workstation_lock(link):
            pass
    assert target.read_text() == "unchanged"
