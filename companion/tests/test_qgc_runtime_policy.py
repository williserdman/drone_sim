from __future__ import annotations

from dataclasses import FrozenInstanceError
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import pytest

from drone_sim_companion.qgc_runtime_policy import load_qgc_runtime_policy


def valid_policy() -> dict[str, object]:
    return {
        "schema_version": 1,
        "purpose": "drone-sim-comp2026-fm1-fm2",
        "backend": "drone-sim-ros-confirmed-v1",
        "bindings": {
            "course_sha256": "1" * 64,
            "scenario_sha256": "2" * 64,
            "ardupilot_commit": "2a3dc4b7bf2507120f7378a7b2fde73185e0c325",
        },
        "simulator_launch_origin": {
            "latitude_deg": 52,
            "longitude_deg": 13.25,
            "amsl_m": 48,
            "heading_deg": 359.5,
        },
        "connection": {"wait_ready": True, "heartbeat_timeout_s": 12},
        "operating_site": {
            "waypoint_tolerance_m": 2,
            "home_position_tolerance_m": 3,
            "home_altitude_tolerance_m": 4,
            "minimum_recovery_agl_m": 5,
            "maximum_recovery_agl_m": 30,
            "operations": ["RETURN", "LOCAL_LAND"],
            "evidence_reference": "3" * 64,
            "recovery_corridor_evidence": "4" * 64,
        },
        "recovery": {"timeout_s": 60, "local_land_reserve_s": 10},
        "telemetry_startup": {
            "command_ack_timeout_s": 4,
            "collection_timeout_s": 8,
            "home_request_timeout_s": 5,
            "poll_interval_s": 0.25,
            "minimum_distinct_samples": 3,
            "maximum_interval_error_fraction": 0.2,
        },
        "autopilot_version": {
            "firmware_label": "ArduCopter 4.5.7",
            "flight_sw_version": 0x040507FF,
            "flight_custom_version": "0123456789abcdef",
            "evidence_reference": "5" * 64,
        },
        "clearance_calibration": {
            "beam_direction_body_frd": [0, 0, 1],
            "measured_reference_offset_body_frd_m": [0.1, -0.2, 0.3],
            "lidar_mounting_offset_already_applied": True,
            "max_tilt_rad": 0.6,
            "max_age_seconds": 0.5,
            "max_skew_seconds": 0.1,
            "locally_horizontal_planar_surface": True,
        },
        "release_stability": {
            "hold_seconds": 1,
            "timeout_seconds": 8,
            "max_horizontal_speed_m_s": 0.3,
            "max_vertical_speed_m_s": 0.2,
            "max_roll_rad": 0.4,
            "max_pitch_rad": 0.4,
            "horizontal_position_tolerance_m": 0.5,
            "vertical_position_tolerance_m": 0.5,
            "max_observation_skew_seconds": 0.1,
            "max_observation_gap_seconds": 0.2,
            "poll_interval_seconds": 0.05,
            "waypoint_reissue_interval_seconds": 1.5,
        },
        "fc_home_position_tolerance_m": 3,
        "fc_home_altitude_tolerance_m": 4,
        "idle_poll_s": 0.1,
        "cleanup_timeout_s": 5,
        "payload_delay_wall_timeout_s": 2,
        "enabled_phases": [31000, 31001],
    }


def write_policy(path: Path, policy: object) -> str:
    content = json.dumps(policy, separators=(",", ":")).encode()
    path.write_bytes(content)
    return hashlib.sha256(content).hexdigest()


def load(path: Path, policy: object | None = None):
    digest = write_policy(path, valid_policy() if policy is None else policy)
    return load_qgc_runtime_policy(path, expected_sha256=digest)


def set_value(policy: dict[str, object], dotted_name: str, value: object) -> None:
    parts = dotted_name.split(".")
    target = policy
    for part in parts[:-1]:
        target = target[part]  # type: ignore[assignment,index]
    target[parts[-1]] = value


def test_loads_exact_policy_as_normalized_frozen_values(tmp_path: Path) -> None:
    source = valid_policy()
    policy = load(tmp_path / "policy.json", source)

    assert policy.schema_version == 1
    assert policy.bindings.course_sha256 == "1" * 64
    assert policy.simulator_launch_origin.latitude_deg == 52.0
    assert policy.connection.wait_ready is True
    assert policy.operating_site.operations == ("RETURN", "LOCAL_LAND")
    assert policy.autopilot_version.flight_sw_version == 0x040507FF
    assert policy.autopilot_version.flight_custom_version == "0123456789abcdef"
    assert policy.autopilot_version.flight_custom_version_bytes == bytes.fromhex(
        "0123456789abcdef"
    )
    assert policy.clearance_calibration.beam_direction_body_frd == (0.0, 0.0, 1.0)
    assert policy.enabled_phases == (31000, 31001)
    source["enabled_phases"] = [99]
    assert policy.enabled_phases == (31000, 31001)
    with pytest.raises(FrozenInstanceError):
        policy.idle_poll_s = 9


@pytest.mark.parametrize(
    "section",
    [
        None,
        "bindings",
        "simulator_launch_origin",
        "connection",
        "operating_site",
        "recovery",
        "telemetry_startup",
        "autopilot_version",
        "clearance_calibration",
        "release_stability",
    ],
)
@pytest.mark.parametrize("operation", ["missing", "unknown"])
def test_rejects_missing_or_unknown_fields_at_every_level(
    tmp_path: Path, section: str | None, operation: str
) -> None:
    policy = valid_policy()
    target = policy if section is None else policy[section]
    assert isinstance(target, dict)
    if operation == "missing":
        target.pop(next(iter(target)))
    else:
        target["unexpected"] = 1

    with pytest.raises(ValueError, match="fields"):
        load(tmp_path / "policy.json", policy)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("schema_version", 2),
        ("purpose", "other"),
        ("backend", "other"),
        ("bindings.course_sha256", "A" * 64),
        ("bindings.scenario_sha256", "0" * 63),
        ("bindings.ardupilot_commit", "0" * 40),
        ("autopilot_version.firmware_label", "ArduCopter 4.5.6"),
        ("autopilot_version.flight_sw_version", 0x04050700),
        ("autopilot_version.flight_custom_version", "A" * 16),
        ("autopilot_version.flight_custom_version", "0" * 15),
        ("autopilot_version.evidence_reference", "pending"),
        ("operating_site.evidence_reference", "unknown"),
        ("operating_site.recovery_corridor_evidence", "tbd"),
    ],
)
def test_rejects_unpinned_bindings_versions_and_references(
    tmp_path: Path, field: str, value: object
) -> None:
    policy = valid_policy()
    set_value(policy, field, value)
    with pytest.raises(ValueError):
        load(tmp_path / "policy.json", policy)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("simulator_launch_origin.latitude_deg", -90.1),
        ("simulator_launch_origin.latitude_deg", 90.1),
        ("simulator_launch_origin.longitude_deg", -180.1),
        ("simulator_launch_origin.longitude_deg", 180.1),
        ("simulator_launch_origin.heading_deg", -0.1),
        ("simulator_launch_origin.heading_deg", 360),
        ("connection.heartbeat_timeout_s", 0),
        ("operating_site.minimum_recovery_agl_m", -1),
        ("operating_site.maximum_recovery_agl_m", 5),
        ("recovery.local_land_reserve_s", 60),
        ("telemetry_startup.minimum_distinct_samples", 1),
        ("telemetry_startup.maximum_interval_error_fraction", 1.01),
        ("telemetry_startup.poll_interval_s", 4.01),
        ("release_stability.timeout_seconds", 1),
        ("release_stability.poll_interval_seconds", 0.21),
        ("release_stability.max_roll_rad", math.pi / 2),
        ("release_stability.max_pitch_rad", math.pi / 2),
        ("fc_home_position_tolerance_m", 0),
        ("payload_delay_wall_timeout_s", -1),
    ],
)
def test_rejects_out_of_range_or_inconsistent_values(
    tmp_path: Path, field: str, value: object
) -> None:
    policy = valid_policy()
    set_value(policy, field, value)
    with pytest.raises(ValueError):
        load(tmp_path / "policy.json", policy)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("simulator_launch_origin.amsl_m", True),
        ("simulator_launch_origin.amsl_m", "48"),
        ("simulator_launch_origin.amsl_m", math.nan),
        ("simulator_launch_origin.amsl_m", math.inf),
        ("connection.wait_ready", 1),
        ("telemetry_startup.minimum_distinct_samples", 2.0),
        ("autopilot_version.flight_sw_version", True),
        ("clearance_calibration.lidar_mounting_offset_already_applied", 1),
    ],
)
def test_rejects_bool_nonfinite_and_string_numbers(
    tmp_path: Path, field: str, value: object
) -> None:
    policy = valid_policy()
    set_value(policy, field, value)
    with pytest.raises(ValueError):
        load(tmp_path / "policy.json", policy)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("clearance_calibration.beam_direction_body_frd", [0, 1]),
        ("clearance_calibration.beam_direction_body_frd", [0, 0, 2]),
        ("clearance_calibration.beam_direction_body_frd", [0, 0, -1]),
        ("clearance_calibration.beam_direction_body_frd", [0, 0, math.inf]),
        ("clearance_calibration.measured_reference_offset_body_frd_m", [0, 0]),
        ("clearance_calibration.max_tilt_rad", 0),
        ("clearance_calibration.max_tilt_rad", math.pi / 2),
        ("clearance_calibration.max_age_seconds", 0),
        ("clearance_calibration.max_skew_seconds", 0),
        ("clearance_calibration.locally_horizontal_planar_surface", False),
    ],
)
def test_rejects_invalid_clearance_calibration(
    tmp_path: Path, field: str, value: object
) -> None:
    policy = valid_policy()
    set_value(policy, field, value)
    with pytest.raises(ValueError):
        load(tmp_path / "policy.json", policy)


@pytest.mark.parametrize(
    "phases",
    [
        [31001, 31000],
        [31000],
        [31000, 31001, 31002],
        [31000.0, 31001.0],
        [31000, 31001.0],
        [True, 31001],
    ],
)
def test_rejects_any_other_enabled_phase_sequence(
    tmp_path: Path, phases: list[object]
) -> None:
    policy = valid_policy()
    policy["enabled_phases"] = phases
    with pytest.raises(ValueError):
        load(tmp_path / "policy.json", policy)


def test_requires_lowercase_expected_hash_and_exact_file_bytes(tmp_path: Path) -> None:
    path = tmp_path / "policy.json"
    digest = write_policy(path, valid_policy())
    with pytest.raises(ValueError, match="expected_sha256"):
        load_qgc_runtime_policy(path, expected_sha256=digest.upper())
    with pytest.raises(ValueError, match="SHA-256"):
        load_qgc_runtime_policy(path, expected_sha256="0" * 64)


def test_rejects_non_object_and_invalid_utf8(tmp_path: Path) -> None:
    path = tmp_path / "policy.json"
    for content in (b"[]", b"\xff"):
        path.write_bytes(content)
        with pytest.raises(ValueError):
            load_qgc_runtime_policy(
                path, expected_sha256=hashlib.sha256(content).hexdigest()
            )


def test_rejects_final_and_component_symlinks(tmp_path: Path) -> None:
    real_dir = tmp_path / "real"
    real_dir.mkdir()
    real_file = real_dir / "policy.json"
    digest = write_policy(real_file, valid_policy())
    final_link = tmp_path / "policy-link.json"
    final_link.symlink_to(real_file)
    component_link = tmp_path / "dir-link"
    component_link.symlink_to(real_dir, target_is_directory=True)

    for path in (final_link, component_link / "policy.json"):
        with pytest.raises(ValueError, match="symlink"):
            load_qgc_runtime_policy(path, expected_sha256=digest)


@pytest.mark.parametrize("relative", [False, True])
def test_rejects_parent_components_before_symlink_path_normalization(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, relative: bool
) -> None:
    target_dir = tmp_path / "target" / "child"
    target_dir.mkdir(parents=True)
    link = tmp_path / "link"
    link.symlink_to(target_dir, target_is_directory=True)
    lexical_file = tmp_path / "policy.json"
    digest = write_policy(lexical_file, valid_policy())
    supplied = link / ".." / "policy.json"
    if relative:
        monkeypatch.chdir(tmp_path)
        supplied = Path("link") / ".." / "policy.json"

    with pytest.raises(ValueError, match="parent component"):
        load_qgc_runtime_policy(supplied, expected_sha256=digest)


def test_rejects_non_regular_file(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="regular"):
        load_qgc_runtime_policy(tmp_path, expected_sha256="0" * 64)


def test_import_is_inert_and_has_no_flight_stack_dependencies() -> None:
    code = """
import sys
import drone_sim_companion.qgc_runtime_policy
for name in sys.modules:
    if name == 'drone' or name.startswith(('drone.', 'dronekit', 'rospy', 'rclpy')):
        raise SystemExit(name)
"""
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(Path(__file__).parents[1] / "src")
    result = subprocess.run(
        [sys.executable, "-c", code],
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr or result.stdout
