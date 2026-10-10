from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
import hashlib
import importlib
import json
from pathlib import Path
import pkgutil

import pytest
from pymavlink.dialects import v10, v20

from artifacts.qgc_operator import MavlinkIdentity
import orchestration.qgc_actions as qgc_actions_module
from orchestration.qgc_actions import QgcActionMaterial, QgcActionStore


RUN_ID = "11111111-1111-4111-8111-111111111111"
QGC_SOURCE = MavlinkIdentity(255, 190)


def test_actions_bind_exact_bytes_and_monotonic_tokens(tmp_path: Path) -> None:
    store = QgcActionStore(tmp_path / "state", QGC_SOURCE)

    first = store.allocate(RUN_ID)
    second = store.allocate("22222222-2222-4222-8222-222222222222")

    document = json.loads(first.action_bytes)
    assert document["version"] == 1
    assert document["fileType"] == "MavlinkActions"
    assert [action["mavCmd"] for action in document["actions"]] == [31020, 31021]
    assert [action["label"] for action in document["actions"]] == [
        "Start selected mission",
        "Abort and recover",
    ]
    assert all(action["compId"] == 191 for action in document["actions"])
    assert all(action["param1"] == 1 for action in document["actions"])
    assert all(
        action[f"param{index}"] == 0
        for action in document["actions"]
        for index in range(2, 8)
    )
    assert first.descriptor.attempt_token == 1
    assert second.descriptor.attempt_token == 2
    assert first.descriptor.start_request_id != first.descriptor.abort_request_id
    assert hashlib.sha256(first.action_bytes).hexdigest() == first.descriptor.actions_sha256
    assert first.run_id == RUN_ID
    assert first.published_name == f"drone-sim-{RUN_ID}.json"
    assert first.descriptor.actions_path == "configuration/qgc-actions.json"

    with pytest.raises(TypeError, match="published_name"):
        QgcActionMaterial(
            run_id=RUN_ID,
            descriptor=first.descriptor,
            action_bytes=first.action_bytes,
            published_name="custom.json",
        )
    with pytest.raises(ValueError, match="canonical UUID"):
        replace(first, run_id="../custom")


def test_reserved_commands_do_not_collide_with_shipped_or_legacy_registries() -> None:
    reserved = {31020, 31021}
    for family in (v10, v20):
        for module_info in pkgutil.iter_modules(family.__path__):
            if module_info.name.startswith("_"):
                continue
            dialect = importlib.import_module(f"{family.__name__}.{module_info.name}")
            assert reserved.isdisjoint(dialect.enums.get("MAV_CMD", {})), module_info.name

    legacy_path = (
        Path(__file__).parents[2]
        / "companion/comp2026/src/gc/custom_missions.json"
    )
    legacy_commands = {
        action["mavCmd"]
        for action in json.loads(legacy_path.read_text(encoding="utf-8"))["actions"]
    }
    legacy_commands.update(range(31000, 31016))
    assert reserved.isdisjoint(legacy_commands)


def test_allocation_never_reuses_after_failure_or_corruption(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    state = tmp_path / "state"
    store = QgcActionStore(state, QGC_SOURCE)
    run_ids = [f"00000000-0000-4000-8000-{index:012d}" for index in range(1, 9)]
    with ThreadPoolExecutor(max_workers=8) as executor:
        materials = list(executor.map(store.allocate, run_ids))
    assert sorted(item.descriptor.attempt_token for item in materials) == list(range(1, 9))

    real_uuid4 = qgc_actions_module.uuid4
    uuid_calls = 0

    def crash_after_ledger_write():
        nonlocal uuid_calls
        uuid_calls += 1
        if uuid_calls == 1:
            return real_uuid4()
        raise RuntimeError("crash after durable increment")

    monkeypatch.setattr(
        qgc_actions_module,
        "uuid4",
        crash_after_ledger_write,
    )
    with pytest.raises(RuntimeError, match="crash after durable increment"):
        store.allocate("00000000-0000-4000-8000-000000000009")
    monkeypatch.setattr(qgc_actions_module, "uuid4", real_uuid4)
    assert (
        store.allocate("00000000-0000-4000-8000-000000000010")
        .descriptor.attempt_token
        == 10
    )

    (state / "attempt-ledger.json").unlink()
    with pytest.raises(RuntimeError, match="missing"):
        store.allocate("00000000-0000-4000-8000-000000000011")

    corrupt_state = tmp_path / "corrupt"
    corrupt_store = QgcActionStore(corrupt_state, QGC_SOURCE)
    corrupt_store.allocate("00000000-0000-4000-8000-000000000012")
    (corrupt_state / "attempt-ledger.json").write_text("not json", encoding="utf-8")
    with pytest.raises(RuntimeError, match="corrupt"):
        corrupt_store.allocate("00000000-0000-4000-8000-000000000013")

    real_state = tmp_path / "real-state"
    real_state.mkdir()
    symlink_state = tmp_path / "symlink-state"
    symlink_state.symlink_to(real_state, target_is_directory=True)
    with pytest.raises(RuntimeError, match="symlink"):
        QgcActionStore(symlink_state, QGC_SOURCE).allocate(
            "00000000-0000-4000-8000-000000000014"
        )

    exhausted_state = tmp_path / "exhausted"
    exhausted_store = QgcActionStore(exhausted_state, QGC_SOURCE)
    exhausted_store.allocate("00000000-0000-4000-8000-000000000015")
    ledger = exhausted_state / "attempt-ledger.json"
    ledger.write_text(
        json.dumps(
            {"schema_version": 1, "last_allocated_token": 16_777_215},
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    with pytest.raises(RuntimeError, match="exhausted"):
        exhausted_store.allocate("00000000-0000-4000-8000-000000000016")


def test_publish_preserves_existing_qgc_files(tmp_path: Path) -> None:
    material = QgcActionStore(tmp_path / "state", QGC_SOURCE).allocate(RUN_ID)
    save_directory = tmp_path / "MavlinkActions"
    unrelated = save_directory / "custom_missions.json"
    save_directory.mkdir()
    unrelated.write_bytes(b"user-owned\n")

    published = QgcActionStore(tmp_path / "state", QGC_SOURCE).publish(
        material, save_directory
    )

    assert published == save_directory / material.published_name
    assert published.read_bytes() == material.action_bytes
    assert unrelated.read_bytes() == b"user-owned\n"
    assert QgcActionStore(tmp_path / "state", QGC_SOURCE).publish(
        material, save_directory
    ) == published

    conflict = replace(material, action_bytes=b"different\n")
    with pytest.raises(ValueError, match="digest"):
        QgcActionStore(tmp_path / "state", QGC_SOURCE).publish(
            conflict, save_directory
        )

    different_run = QgcActionStore(tmp_path / "state", QGC_SOURCE).allocate(
        "22222222-2222-4222-8222-222222222222"
    )
    forced_conflict = replace(different_run, run_id=RUN_ID)
    with pytest.raises(FileExistsError):
        QgcActionStore(tmp_path / "state", QGC_SOURCE).publish(
            forced_conflict, save_directory
        )
    assert published.read_bytes() == material.action_bytes
    assert unrelated.read_bytes() == b"user-owned\n"
