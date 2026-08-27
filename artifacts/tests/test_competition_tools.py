from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from artifacts.acceptance import BundleAcceptanceError


ROOT = Path(__file__).parents[2]
RUN_ID = "00000000-0000-4000-8000-000000000606"
IMAGES = (
    "drone-sim-orchestration-runtime:phase2",
    "drone-sim-artifacts-runtime:phase2",
    "drone-sim-companion-runtime:phase3",
    "drone-sim-ardupilot-runtime:phase3",
    "drone-sim-gazebo-runtime:phase3",
    "drone-sim-electromagnet-runtime:phase3",
    "drone-sim-scorekeeper-runtime:phase3",
)


def _load(name: str):
    path = ROOT / "scripts" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_competition_inspector_uses_current_parent_nested_and_image_provenance(tmp_path):
    module = _load("inspect_competition_run")
    commands = []

    def runner(command, **kwargs):
        commands.append(tuple(command))
        if command[-2:] == ["rev-parse", "HEAD"]:
            output = "a" * 40 if command[2] == str(ROOT) else "b" * 40
        elif command[-3:] == ["status", "--porcelain", "--untracked-files=normal"]:
            output = "?? protected-user-file\n"
        else:
            image_index = IMAGES.index(command[-1])
            output = "sha256:" + f"{image_index + 1:064x}" + "\n"
        return SimpleNamespace(returncode=0, stdout=output, stderr="")

    captured = {}

    def inspector(run_directory, **kwargs):
        captured.update(run_directory=run_directory, **kwargs)
        return SimpleNamespace(to_dict=lambda: {"accepted": True, "run_id": RUN_ID})

    report = module.inspect_competition_run(
        tmp_path, runner=runner, bundle_inspector=inspector
    )

    assert report == {"accepted": True, "run_id": RUN_ID}
    assert captured["expected_source_revisions"] == {
        "drone_sim": "a" * 40,
        "comp2026": "b" * 40,
    }
    assert captured["expected_source_dirty"] == {
        "drone_sim": True,
        "comp2026": True,
    }
    assert tuple(captured["expected_image_digests"]) == IMAGES
    assert captured["require_maximum_score"] is True
    assert len(commands) == 11


def _accepted_documents(run_directory: Path, *, complete=True, achieved=150.0):
    (run_directory / "scoring").mkdir(parents=True)
    manifest = {
        "run_id": RUN_ID,
        "terminal_status": "COMPLETED" if complete else "FAILED",
        "source_revisions": [
            {"name": "drone_sim", "revision": "a" * 40, "dirty": True},
            {"name": "comp2026", "revision": "b" * 40, "dirty": True},
        ],
        "simulation_timing": {
            "start_ns": 0,
            "end_ns": 23_100_000_000,
            "duration_ns": 23_100_000_000,
        },
        "scoring": {
            "achieved_score": achieved,
            "maximum_available_score": 150.0,
            "scoring_checksum": "c" * 64,
        },
    }
    result = {
        "run_id": RUN_ID,
        "ruleset_id": "competition_v1",
        "complete": complete,
        "achieved_score": achieved,
        "maximum_available_score": 150.0,
        "scoring_checksum": "c" * 64,
        "rule_results": [
            {"awarded_points": value}
            for value in (20.0, 30.0, 10.0, 20.0, 15.0, 50.0, 5.0)
        ],
    }
    (run_directory / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    (run_directory / "scoring/result.json").write_text(json.dumps(result), encoding="utf-8")


def _canonical_report(run_directory: Path) -> dict[str, object]:
    return {
        "accepted": True,
        "run_id": RUN_ID,
        "achieved_score": 150.0,
        "maximum_available_score": 150.0,
        "compose_project": "drone-sim-" + RUN_ID.replace("-", ""),
        "manifest_sha256": hashlib.sha256(
            (run_directory / "manifest.json").read_bytes()
        ).hexdigest(),
    }


def test_verification_writer_names_accepted_evidence_and_checkpoints(
    tmp_path, monkeypatch
):
    module = _load("write_competition_verification")
    run_directory = tmp_path / "run"
    output = tmp_path / "verification.md"
    _accepted_documents(run_directory)

    inspected = []

    def inspector(directory):
        inspected.append(directory)
        return _canonical_report(directory)

    monkeypatch.setattr(module, "_canonical_inspection", inspector)

    module.write_competition_verification(run_directory, output)

    note = output.read_text(encoding="utf-8")
    assert inspected == [run_directory.resolve()]
    for expected in (
        RUN_ID,
        "a" * 40,
        "b" * 40,
        "COMPLETED",
        "23.1 s",
        "150/150",
        "80/150",
        "145/150",
        "manifest.json",
        "video/onboard.mp4",
        "video/observer.mp4",
        "rosbag",
    ):
        assert expected in note


@pytest.mark.parametrize(("complete", "achieved"), [(False, 150.0), (True, 145.0)])
def test_verification_writer_refuses_unaccepted_manifest(
    tmp_path, monkeypatch, complete, achieved
):
    module = _load("write_competition_verification")
    run_directory = tmp_path / "run"
    _accepted_documents(run_directory, complete=complete, achieved=achieved)

    monkeypatch.setattr(module, "_canonical_inspection", _canonical_report)

    with pytest.raises(ValueError, match="accepted 150/150"):
        module.write_competition_verification(
            run_directory, tmp_path / "verification.md"
        )


def test_public_verification_writer_rejects_caller_supplied_inspector(tmp_path):
    """A caller-controlled success report must not bypass canonical acceptance."""
    module = _load("write_competition_verification")
    run_directory = tmp_path / "run"
    output = tmp_path / "verification.md"
    _accepted_documents(run_directory)

    with pytest.raises(TypeError, match="unexpected keyword argument 'inspector'"):
        module.write_competition_verification(
            run_directory,
            output,
            inspector=_canonical_report,
        )

    assert not output.exists()


def test_verification_writer_refuses_forged_manifest_and_result_only_bundle(
    tmp_path, monkeypatch
):
    """Two convincing JSON files cannot bypass the canonical bundle inspector."""
    module = _load("write_competition_verification")
    run_directory = tmp_path / "forged-run"
    output = tmp_path / "verification.md"
    _accepted_documents(run_directory)

    monkeypatch.syspath_prepend(str(ROOT / "scripts"))

    with pytest.raises(
        ValueError, match="canonical competition inspection failed"
    ) as raised:
        module.write_competition_verification(run_directory, output)

    assert isinstance(raised.value.__cause__, BundleAcceptanceError)
    assert not output.exists()
