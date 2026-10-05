"""Workstation lock, one build, frozen provenance and independent inspection."""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
import fcntl
import os
from pathlib import Path
import subprocess
import time
from uuid import uuid4

from artifacts import ImageDigest, SourceRevision
from artifacts.acceptance import inspect_phase3_via_container
from orchestration.config import PHASE3_TOPOLOGY
from orchestration.controller import RunController
from .compose import ComposeRuntime


@dataclass(frozen=True)
class FrozenSuiteRuntime:
    source_revisions: tuple[SourceRevision, ...]
    image_digests: tuple[ImageDigest, ...]


class SuiteRuntime:
    def __init__(self, *, project_directory: Path, controller=None, runner=subprocess.run) -> None:
        self.project_directory = project_directory.resolve()
        self.controller = controller or RunController(project_directory=self.project_directory)
        self.runner = runner
        run_id = str(uuid4())
        probe = self.project_directory / "runs" / run_id
        self.compose = ComposeRuntime(project_directory=self.project_directory,
            run_id=run_id, run_directory=probe, config_path=probe / "configuration/run.json",
            topology=PHASE3_TOPOLOGY)

    @contextmanager
    def lock(self):
        path = Path(f"/tmp/drone-sim-suite-{os.getuid()}.lock")
        with path.open("a") as handle:
            try:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise RuntimeError("a drone-sim suite is already running on this workstation") from error
            try:
                yield
            finally:
                fcntl.flock(handle, fcntl.LOCK_UN)

    def _sources(self):
        records = self.controller.source_revisions(time.monotonic() + 30)
        if not records or any(record.dirty for record in records):
            raise RuntimeError("suite requires clean committed source")
        return records

    def build_and_freeze(self) -> FrozenSuiteRuntime:
        sources = self._sources()
        environment = dict(self.compose.environment)
        environment["SIM_COMP2026_REVISION"] = next(row.revision for row in sources if row.name == "comp2026")
        command = ["docker", "compose", "--file", str(self.project_directory / "compose.yaml"),
            "--project-directory", str(self.project_directory), "--profile", "phase3", "build"]
        result = self.runner(command, cwd=self.project_directory, env=environment, timeout=3600, check=False)
        if result.returncode:
            raise RuntimeError("suite runtime image build failed")
        result = self.runner(["docker", "run", "--rm", "drone-sim-companion-runtime:phase3",
            "python3", "-m", "drone_sim_companion.comp2026_smoke"], env=environment, timeout=120, check=False)
        if result.returncode:
            raise RuntimeError("Comp2026 runtime import smoke failed")
        self.compose.bind_source_revisions(sources, 30)
        images = self.compose.image_digests(30)
        if len(images) != 7:
            raise RuntimeError("suite requires seven Phase 3 images")
        frozen = FrozenSuiteRuntime(sources, images)
        self.assert_unchanged(frozen)
        return frozen

    def assert_unchanged(self, frozen: FrozenSuiteRuntime) -> None:
        if self._sources() != frozen.source_revisions:
            raise RuntimeError("suite source changed after build")
        if self.compose.image_digests(30) != frozen.image_digests:
            raise RuntimeError("suite runtime image changed after build")

    def inspect_run(self, case, run_directory: Path, frozen: FrozenSuiteRuntime):
        import json
        if case.acceptance == "operator_wait" and not (run_directory / "logs/docker/operator.jsonl").is_file():
            raise RuntimeError("operator case lacks external operator evidence")
        config = json.loads((run_directory / "configuration/run.json").read_text())
        rules = config["scenario"] if config["scenario"] in {"calibration_v1", "moving_pad_v1", "competition_v1"} else "descent_v1"
        return inspect_phase3_via_container(run_directory,
            rules_path=self.project_directory / f"scorekeeper/rules/{rules}.json",
            expected_source_revisions={row.name: row.revision for row in frozen.source_revisions},
            expected_source_dirty={row.name: row.dirty for row in frozen.source_revisions},
            expected_image_digests={row.name: row.digest for row in frozen.image_digests},
            require_maximum_score=case.acceptance not in {"hover_roll", "autotune_roll"})
