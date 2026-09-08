import json
import os

import pytest

from drone import timebase
from drone.common_types import GPSCoord
from drone.control import mission_info


class FakeClock:
    def __init__(self, now_value: float):
        self.now_value = now_value

    def now(self) -> float:
        return self.now_value

    def sleep(self, seconds: float) -> None:
        self.now_value += seconds


def _use_store(monkeypatch, tmp_path):
    path = tmp_path / "mission_data" / "waypoints.json"
    monkeypatch.setattr(mission_info, "WAYPOINTS_PATH", path)
    return path


def _write_store(path, entries):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(entries))


def _entry(lat=41.5, long=-81.6, alt=10.0, loaded_at=1_800_000_000.0):
    return {
        "coords": {"lat": lat, "long": long, "alt": alt},
        "loaded_at": loaded_at,
    }


def test_trackers_with_explicit_paths_do_not_cross_read_write_or_clear(
    monkeypatch, tmp_path
):
    first_path = tmp_path / "first" / "waypoints.json"
    second_path = tmp_path / "second" / "waypoints.json"
    monkeypatch.setattr(timebase._wall_time, "time", lambda: 1_800_000_000.0)
    first = mission_info.MissonTracker(waypoint_path=first_path)
    second = mission_info.MissonTracker(waypoint_path=second_path)
    first.initialize_waypoint_store()
    second.initialize_waypoint_store()

    first.set_waypoint("L", GPSCoord(41.5, -81.6, 10.0))
    second.set_waypoint("TARGET", GPSCoord(42.0, -82.0, 20.0))

    assert first.get_waypoint_record("TARGET") is None
    assert second.get_waypoint("L") is None
    assert first.snapshot_for_attempt(
        {"L"}, lambda _name, _coord: True
    ).get_waypoint("L") == GPSCoord(41.5, -81.6, 10.0)
    assert second.snapshot_for_attempt(
        {"TARGET"}, lambda _name, _coord: True
    ).get_waypoint("TARGET") == GPSCoord(42.0, -82.0, 20.0)
    assert first.clear_waypoint("L") is True
    assert second.get_waypoint("TARGET") == GPSCoord(42.0, -82.0, 20.0)


def test_default_path_is_bound_before_global_or_cwd_changes(monkeypatch, tmp_path):
    construction_cwd = tmp_path / "construction"
    later_cwd = tmp_path / "later"
    construction_cwd.mkdir()
    later_cwd.mkdir()
    monkeypatch.chdir(construction_cwd)
    monkeypatch.setattr(
        mission_info, "WAYPOINTS_PATH", "stores/../stores/waypoints.json"
    )
    tracker = mission_info.MissonTracker()
    bound_path = construction_cwd / "stores" / "waypoints.json"

    monkeypatch.chdir(later_cwd)
    monkeypatch.setattr(
        mission_info, "WAYPOINTS_PATH", tmp_path / "redirected" / "waypoints.json"
    )
    monkeypatch.setattr(timebase._wall_time, "time", lambda: 1_800_000_000.0)
    tracker.initialize_waypoint_store()
    tracker.set_waypoint("TARGET", GPSCoord(41.5, -81.6, 10.0))

    assert tracker.waypoint_path == bound_path
    assert tracker.get_waypoint_record("TARGET") == mission_info.WaypointRecord(
        41.5, -81.6, 10.0, 1_800_000_000.0
    )
    assert not (later_cwd / "stores" / "waypoints.json").exists()
    assert not (tmp_path / "redirected" / "waypoints.json").exists()


def test_missing_explicit_store_is_inert_until_initialized(monkeypatch, tmp_path):
    path = tmp_path / "explicit" / "waypoints.json"
    tracker = mission_info.MissonTracker(waypoint_path=path)

    assert tracker.waypoint_path == path
    assert not path.parent.exists()
    with pytest.raises(mission_info.WaypointStoreError, match="does not exist"):
        tracker.set_waypoint("TARGET", GPSCoord(41.5, -81.6, 10.0))
    assert not path.parent.exists()

    monkeypatch.setattr(timebase._wall_time, "time", lambda: 1_800_000_000.0)
    tracker.initialize_waypoint_store()
    tracker.set_waypoint("TARGET", GPSCoord(41.5, -81.6, 10.0))
    assert tracker.get_waypoint("TARGET") == GPSCoord(41.5, -81.6, 10.0)


def test_malformed_explicit_store_bytes_are_preserved(tmp_path):
    path = tmp_path / "explicit" / "waypoints.json"
    path.parent.mkdir()
    path.write_bytes(b"not-json")
    tracker = mission_info.MissonTracker(waypoint_path=path)

    assert tracker.get_waypoint("TARGET") is None
    assert "malformed" in tracker.waypoint_unavailable_reason
    with pytest.raises(mission_info.WaypointStoreError, match="already exists"):
        tracker.initialize_waypoint_store()
    assert path.read_bytes() == b"not-json"


def test_explicit_path_initialization_refuses_a_dangling_symlink(tmp_path):
    path = tmp_path / "explicit" / "waypoints.json"
    path.parent.mkdir()
    path.symlink_to(path.parent / "missing-target.json")
    tracker = mission_info.MissonTracker(waypoint_path=path)

    with pytest.raises(mission_info.WaypointStoreError, match="already exists"):
        tracker.initialize_waypoint_store()

    assert path.is_symlink()
    assert not (path.parent / "missing-target.json").exists()


def test_timer_only_construction_does_not_create_a_waypoint_store(monkeypatch, tmp_path):
    path = _use_store(monkeypatch, tmp_path)
    tracker = mission_info.MissonTracker()

    assert tracker.get_waypoint("TARGET") is None
    assert "does not exist" in tracker.waypoint_unavailable_reason
    assert not path.exists()


def test_zero_byte_store_is_reported_and_preserved(monkeypatch, tmp_path):
    path = _use_store(monkeypatch, tmp_path)
    path.parent.mkdir()
    path.touch()
    tracker = mission_info.MissonTracker()

    assert tracker.get_waypoint("TARGET") is None
    assert "malformed JSON" in tracker.waypoint_unavailable_reason
    assert path.read_bytes() == b""


def test_invalid_text_encoding_is_reported_as_a_malformed_store(monkeypatch, tmp_path):
    path = _use_store(monkeypatch, tmp_path)
    path.parent.mkdir()
    path.write_bytes(b"\xff")
    tracker = mission_info.MissonTracker()

    assert tracker.get_waypoint("TARGET") is None
    assert "malformed" in tracker.waypoint_unavailable_reason
    assert path.read_bytes() == b"\xff"


def test_missing_store_requires_explicit_initialization_before_recording(
    monkeypatch, tmp_path
):
    path = _use_store(monkeypatch, tmp_path)
    tracker = mission_info.MissonTracker()

    tracker.initialize_waypoint_store()
    tracker.set_waypoint("TARGET", GPSCoord(41.5, -81.6, 10.0))

    assert tracker.get_waypoint("TARGET") == GPSCoord(41.5, -81.6, 10.0)
    assert path.exists()


def test_explicit_initialization_refuses_to_replace_an_existing_malformed_store(
    monkeypatch, tmp_path
):
    path = _use_store(monkeypatch, tmp_path)
    _write_store(path, [])

    with pytest.raises(mission_info.WaypointStoreError, match="already exists"):
        mission_info.MissonTracker().initialize_waypoint_store()

    assert path.read_text() == "[]"


def test_explicit_initialization_refuses_a_dangling_symlink(monkeypatch, tmp_path):
    path = _use_store(monkeypatch, tmp_path)
    path.parent.mkdir()
    path.symlink_to(path.parent / "missing-target.json")

    with pytest.raises(mission_info.WaypointStoreError, match="already exists"):
        mission_info.MissonTracker().initialize_waypoint_store()

    assert path.is_symlink()


def test_concurrent_file_creation_wins_over_explicit_initialization(
    monkeypatch, tmp_path
):
    path = _use_store(monkeypatch, tmp_path)
    original_named_temporary_file = mission_info.tempfile.NamedTemporaryFile
    competing_bytes = b'{"operator":"data"}'

    class CreateCompetingFileOnClose:
        def __init__(self, temporary):
            self.temporary = temporary

        def __enter__(self):
            return self.temporary.__enter__()

        def __exit__(self, *args):
            result = self.temporary.__exit__(*args)
            path.write_bytes(competing_bytes)
            return result

    def racing_named_temporary_file(*args, **kwargs):
        return CreateCompetingFileOnClose(
            original_named_temporary_file(*args, **kwargs)
        )

    monkeypatch.setattr(
        mission_info.tempfile, "NamedTemporaryFile", racing_named_temporary_file
    )

    with pytest.raises(mission_info.WaypointStoreError, match="already exists"):
        mission_info.MissonTracker().initialize_waypoint_store()

    assert path.read_bytes() == competing_bytes


def test_waypoint_storage_is_independent_of_working_directory(monkeypatch, tmp_path):
    canonical_path = _use_store(monkeypatch, tmp_path)
    unrelated_cwd = tmp_path / "elsewhere"
    unrelated_cwd.mkdir()
    monkeypatch.chdir(unrelated_cwd)
    monkeypatch.setattr(timebase._wall_time, "time", lambda: 1_800_000_000.0)
    tracker = mission_info.MissonTracker()
    tracker.initialize_waypoint_store()

    tracker.set_waypoint("TARGET", GPSCoord(41.5, -81.6, 10.0))

    assert json.loads(canonical_path.read_text())["TARGET"] == _entry()
    assert not (unrelated_cwd / "mission_data" / "waypoints.json").exists()


@pytest.mark.parametrize(
    ("entry", "reason"),
    [
        ({"coords": {"lat": 41.5, "long": -81.6, "alt": 10.0}}, "schema"),
        (
            {
                "coords": {"lat": 41.5, "long": -81.6, "alt": 10.0, "x": 1},
                "loaded_at": 1_800_000_000.0,
            },
            "schema",
        ),
        (_entry(lat=True), "latitude"),
        (_entry(lat=float("nan")), "latitude"),
        (_entry(lat=10**400), "latitude"),
        (_entry(lat=91.0), "latitude"),
        (_entry(long=-181.0), "longitude"),
        (_entry(alt=float("inf")), "altitude"),
        (_entry(loaded_at=0.0), "loaded_at"),
        (_entry(loaded_at=float("nan")), "loaded_at"),
        (_entry(loaded_at=1_800_000_001.0), "future"),
    ],
)
def test_invalid_waypoint_records_are_unavailable_with_a_reason(
    monkeypatch, tmp_path, entry, reason
):
    path = _use_store(monkeypatch, tmp_path)
    _write_store(path, {"TARGET": entry})
    monkeypatch.setattr(timebase._wall_time, "time", lambda: 1_800_000_000.0)
    tracker = mission_info.MissonTracker()

    assert tracker.get_waypoint("TARGET") is None
    assert reason in tracker.waypoint_unavailable_reason


def test_waypoint_age_uses_persisted_epoch_for_target(monkeypatch, tmp_path):
    path = _use_store(monkeypatch, tmp_path)
    _write_store(path, {"TARGET": _entry(loaded_at=1_799_999_970.0)})
    monkeypatch.setattr(timebase._wall_time, "time", lambda: 1_800_000_000.0)
    tracker = mission_info.MissonTracker()

    assert tracker.get_waypoint("TARGET", max_age_seconds=31.0) == GPSCoord(
        41.5, -81.6, 10.0
    )
    assert tracker.get_waypoint("TARGET", max_age_seconds=29.0) is None
    assert "30.0" in tracker.waypoint_unavailable_reason
    assert tracker.get_waypoint_record("TARGET").loaded_at == 1_799_999_970.0


def test_wall_clock_rollback_makes_waypoint_unavailable(monkeypatch, tmp_path):
    path = _use_store(monkeypatch, tmp_path)
    _write_store(path, {"TARGET": _entry()})
    monkeypatch.setattr(timebase._wall_time, "time", lambda: 1_799_999_999.0)
    tracker = mission_info.MissonTracker()

    assert tracker.get_waypoint("TARGET") is None
    assert "future" in tracker.waypoint_unavailable_reason


def test_set_waypoint_rejects_invalid_coordinates_before_writing(monkeypatch, tmp_path):
    path = _use_store(monkeypatch, tmp_path)
    _write_store(path, {"L": _entry()})
    original = path.read_bytes()

    with pytest.raises(mission_info.WaypointStoreError, match="latitude"):
        mission_info.MissonTracker().set_waypoint(
            "TARGET", GPSCoord(float("nan"), -81.6, 10.0)
        )

    assert path.read_bytes() == original


def test_failed_atomic_replace_preserves_previous_store(monkeypatch, tmp_path):
    path = _use_store(monkeypatch, tmp_path)
    _write_store(path, {"L": _entry()})
    monkeypatch.setattr(timebase._wall_time, "time", lambda: 1_800_000_000.0)
    original = path.read_bytes()
    tracker = mission_info.MissonTracker()

    def fail_replace(source, destination):
        raise OSError("replace failed")

    monkeypatch.setattr(os, "replace", fail_replace)
    with pytest.raises(OSError, match="replace failed"):
        tracker.set_waypoint("TARGET", GPSCoord(41.6, -81.7, 11.0))

    assert path.read_bytes() == original


def test_snapshot_requires_all_names_and_four_hour_freshness(monkeypatch, tmp_path):
    path = _use_store(monkeypatch, tmp_path)
    now = 1_800_000_000.0
    _write_store(path, {"L": _entry(loaded_at=now - 14_401.0)})
    monkeypatch.setattr(timebase._wall_time, "time", lambda: now)
    tracker = mission_info.MissonTracker()

    with pytest.raises(mission_info.WaypointStoreError, match="older than 14400"):
        tracker.snapshot_for_attempt({"L"}, lambda _name, _coord: True)

    _write_store(path, {"L": _entry(loaded_at=now)})
    with pytest.raises(mission_info.WaypointStoreError, match="TARGET.*missing"):
        tracker.snapshot_for_attempt({"L", "TARGET"}, lambda _name, _coord: True)


def test_snapshot_requires_explicit_operating_area_acceptance(monkeypatch, tmp_path):
    path = _use_store(monkeypatch, tmp_path)
    _write_store(path, {"L": _entry()})
    monkeypatch.setattr(timebase._wall_time, "time", lambda: 1_800_000_000.0)

    with pytest.raises(mission_info.WaypointStoreError, match="operating area"):
        mission_info.MissonTracker().snapshot_for_attempt(
            {"L"}, lambda _name, _coord: False
        )


def test_present_optional_waypoint_must_be_valid_for_area(monkeypatch, tmp_path):
    path = _use_store(monkeypatch, tmp_path)
    _write_store(path, {"L": _entry(), "TARGET": _entry(lat=42.0)})
    monkeypatch.setattr(timebase._wall_time, "time", lambda: 1_800_000_000.0)

    with pytest.raises(mission_info.WaypointStoreError, match="TARGET.*operating area"):
        mission_info.MissonTracker().snapshot_for_attempt(
            {"L"}, lambda _name, coord: coord.lat < 42.0, optional_names={"TARGET"}
        )


def test_snapshot_is_unchanged_by_file_and_returned_coordinate_mutation(
    monkeypatch, tmp_path
):
    path = _use_store(monkeypatch, tmp_path)
    _write_store(path, {"L": _entry()})
    monkeypatch.setattr(timebase._wall_time, "time", lambda: 1_800_000_000.0)
    snapshot = mission_info.MissonTracker().snapshot_for_attempt(
        {"L"}, lambda _name, _coord: True
    )

    first = snapshot.get_waypoint("L")
    first.alt = 999.0
    _write_store(path, {"L": _entry(lat=1.0, long=2.0, alt=3.0)})

    assert snapshot.get_waypoint("L") == GPSCoord(41.5, -81.6, 10.0)
    assert snapshot.get_record("L").loaded_at == 1_800_000_000.0
    assert snapshot.as_coords()["L"] == GPSCoord(41.5, -81.6, 10.0)


def test_aux_timer_accepts_zero_as_a_valid_start_time(monkeypatch, tmp_path):
    _use_store(monkeypatch, tmp_path)
    clock = FakeClock(now_value=0.0)

    with timebase.configured(clock):
        tracker = mission_info.MissonTracker()
        tracker.begin_aux_timer()
        clock.sleep(2.5)

        assert tracker.end_aux_timer() == 2.5
