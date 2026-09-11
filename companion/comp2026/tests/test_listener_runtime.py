import json
from dataclasses import replace
from pathlib import Path

import pytest

from drone import timebase
from drone.control.attempt_setup import DeploymentProfile
from drone.control.flight_state import SourceIdentity
from drone.control.listener import construct_after_full_validation
from drone.control.listener_runtime import (
    AutopilotVersionContract,
    ConnectionConfig,
    HardwareComponentConfig,
    InjectedComponentConfig,
    OperatingSitePolicy,
    PayloadConfig,
    RangeConfig,
    RuntimeConfiguration,
    TelemetryStartupPolicy,
    VisionConfig,
    construct_after_runtime_validation,
    shared_monotonic_ns,
)
from drone.control.mission_supervisor import RecoveryPolicy
from drone.control.mission_supervisor import CommandRejected, FM1, FM2, FM3
from drone.control.stability import ReleaseStabilityConfig
from drone.mock_mission import PrecisionMissionPolicy
from drone.sensors.lidar.clearance import ClearanceCalibration
from test_listener import profile


def deployment(selected_profile):
    return DeploymentProfile(
        profile_id=selected_profile.profile_id,
        source_system=selected_profile.qgc_source.system_id,
        source_component=selected_profile.qgc_source.component_id,
        target_system=selected_profile.companion_target.system_id,
        target_component=selected_profile.companion_target.component_id,
        flight_controller_system=selected_profile.flight_controller.system_id,
        flight_controller_component=selected_profile.flight_controller.component_id,
        mavlink_dialect="ardupilotmega",
        mavlink_wire_protocol="2.0",
        firmware=selected_profile.firmware,
        qgc_version="test-only",
        raw_sha256=selected_profile.raw_sha256,
        raw_bytes=b"test-only",
    )


def write_verified_vision_files(tmp_path):
    calibration_path = tmp_path / "camera-calibration.json"
    calibration_path.write_text(
        json.dumps(
            {
                "camera_matrix": [[100.0, 0.0, 32.0], [0.0, 100.0, 24.0], [0.0, 0.0, 1.0]],
                "dist_coeff": [[0.0, 0.0, 0.0, 0.0, 0.0]],
                "image_width_px": 64,
                "image_height_px": 48,
                "dimensions_verified": True,
            }
        )
    )
    mounting_path = tmp_path / "camera-mounting.json"
    mounting_path.write_text(
        json.dumps(
            {
                "camera_to_body_frd": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                "down_offset_m": 0.01,
                "verified": True,
            }
        )
    )
    return calibration_path, mounting_path


def runtime_configuration(tmp_path):
    waypoint_path = tmp_path / "waypoints.json"
    waypoint_path.write_text("{}")
    calibration_path, mounting_path = write_verified_vision_files(tmp_path)
    clearance = ClearanceCalibration(
        beam_direction_body_frd=(0.0, 0.0, 1.0),
        measured_reference_offset_body_frd_m=(0.0, 0.0, 0.01),
        lidar_mounting_offset_already_applied=True,
        max_tilt_rad=0.2,
        max_age_seconds=0.25,
        max_skew_seconds=0.05,
        locally_horizontal_planar_surface=True,
    )
    precision = PrecisionMissionPolicy(
        clearance_calibration=clearance,
        clock=timebase.monotonic,
        max_exposure_age_s=0.2,
        max_image_attitude_skew_s=0.04,
        max_image_location_skew_s=0.04,
        max_attitude_transport_latency_s=0.01,
        max_location_transport_latency_s=0.01,
        acquisition_timeout_s=5.0,
        frame_timeout_s=0.5,
        observation_period_s=0.1,
        target_hover_height_m=2.0,
        hover_tolerance_m=0.2,
        centered_tolerance_m=0.1,
        correction_gain=0.5,
        cruise_altitude_m=10.0,
        desired_drop_height_m=1.5,
    )
    waypoint_check = lambda _name, _coord: True
    home_check = lambda _home: None
    recovery_check = lambda _operation, _home, _altitude: None
    return RuntimeConfiguration(
        connection=ConnectionConfig(
            endpoint="inert:test",
            source_identity=SourceIdentity(1, 191),
            target_identity=SourceIdentity(1, 1),
            wire_protocol="2.0",
            wait_ready=True,
            heartbeat_timeout_s=5.0,
        ),
        waypoint_path=waypoint_path,
        operating_site=OperatingSitePolicy(
            waypoint_check=waypoint_check,
            mission_home_check=home_check,
            recovery_check=recovery_check,
            evidence_reference="test fixture survey record",
        ),
        recovery_policy=RecoveryPolicy(
            check=recovery_check,
            timeout_s=60.0,
            local_land_reserve_s=10.0,
            clock=timebase.monotonic,
        ),
        telemetry=TelemetryStartupPolicy(
            command_ack_timeout_s=1.0,
            collection_timeout_s=5.0,
            home_request_timeout_s=2.0,
            poll_interval_s=0.05,
            minimum_distinct_samples=3,
            maximum_interval_error_fraction=0.25,
        ),
        autopilot_version=AutopilotVersionContract(
            firmware_label="ArduCopter 4.5.7",
            flight_sw_version=0x040507FF,
            flight_custom_version=b"abcdef0\0",
            evidence_reference="test fixture AUTOPILOT_VERSION mapping",
        ),
        vision=VisionConfig(
            marker_size_mm=100.0,
            calibration_path=calibration_path,
            mounting_path=mounting_path,
            receipt_clock_ns=shared_monotonic_ns,
            max_exposure_age_ns=200_000_000,
        ),
        components=InjectedComponentConfig(
            backend="test-injected-components-v1",
            evidence_reference="test fixture component binding",
        ),
        clearance_calibration=clearance,
        release_stability=ReleaseStabilityConfig(
            hold_seconds=1.0,
            timeout_seconds=5.0,
            max_horizontal_speed_m_s=0.2,
            max_vertical_speed_m_s=0.2,
            max_roll_rad=0.1,
            max_pitch_rad=0.1,
            horizontal_position_tolerance_m=0.3,
            vertical_position_tolerance_m=0.2,
            max_observation_skew_seconds=0.05,
            max_observation_gap_seconds=0.2,
            poll_interval_seconds=0.05,
            waypoint_reissue_interval_seconds=0.5,
        ),
        enabled_phases=frozenset((FM1, FM2, FM3)),
        precision_policy=precision,
        cruise_altitude_m=10.0,
        desired_drop_height_m=1.5,
        fc_home_position_tolerance_m=1.0,
        fc_home_altitude_tolerance_m=0.5,
        attempt_timeout_s=600.0,
        idle_poll_s=0.1,
        cleanup_timeout_s=2.0,
    )


def test_complete_explicit_runtime_configuration_precedes_factory(tmp_path):
    selected_profile = profile()
    selected_deployment = deployment(selected_profile)
    config = runtime_configuration(tmp_path)
    calls = []

    result = construct_after_runtime_validation(
        config,
        deployment_profile=selected_deployment,
        flight_profile=selected_profile,
        component_factory=lambda validated: calls.append(validated) or "built",
    )

    assert result == "built"
    assert calls == [config]
    assert config.precision_policy.clearance_calibration is config.clearance_calibration
    assert config.recovery_policy.check is config.operating_site.recovery_check
    assert config.precision_policy.clock is timebase.monotonic
    assert config.recovery_policy.clock is timebase.monotonic


def test_full_startup_rejects_mavlink1_before_factory(tmp_path):
    from test_listener import prepared_startup_files

    attempt_dir = tmp_path / "attempt"
    attempt_dir.mkdir()
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    files, _prepared = prepared_startup_files(attempt_dir)
    config = runtime_configuration(runtime_dir)
    config = replace(
        config,
        connection=replace(config.connection, wire_protocol="1.0"),
    )
    calls = []

    with pytest.raises(ValueError, match="wire protocol"):
        construct_after_full_validation(
            files,
            config,
            component_factory=lambda artifacts, validated: calls.append(
                (artifacts, validated)
            )
            or "built",
        )

    assert calls == []


def test_full_startup_closes_newly_loaded_artifacts_when_runtime_validation_fails(
    tmp_path, monkeypatch,
):
    import drone.control.listener as listener
    from test_listener import prepared_startup_files

    attempt_dir = tmp_path / "attempt"
    attempt_dir.mkdir()
    files, _prepared = prepared_startup_files(attempt_dir)
    before = len(tuple(Path("/proc/self/fd").iterdir()))
    retained = []
    original_validate = listener._validated_artifacts

    def retain_loaded(source):
        artifacts = original_validate(source)
        retained.append(artifacts)
        return artifacts

    monkeypatch.setattr(listener, "_validated_artifacts", retain_loaded)

    with pytest.raises(TypeError, match="RuntimeConfiguration"):
        construct_after_full_validation(
            files,
            object(),
            component_factory=lambda *_args: pytest.fail("factory must not run"),
        )

    assert len(retained) == 1
    assert retained[0].ledger._ledger_fd == -1
    assert retained[0].ledger._lock_fd == -1
    assert retained[0].ledger._directory_fd == -1
    assert len(tuple(Path("/proc/self/fd").iterdir())) == before


def test_full_startup_preserves_caller_owned_artifacts_on_validation_failure(tmp_path):
    from drone.control.listener import load_listener_artifacts
    from test_listener import prepared_startup_files

    attempt_dir = tmp_path / "attempt"
    attempt_dir.mkdir()
    files, _prepared = prepared_startup_files(attempt_dir)
    artifacts = load_listener_artifacts(files)

    with pytest.raises(TypeError, match="RuntimeConfiguration"):
        construct_after_full_validation(
            artifacts,
            object(),
            component_factory=lambda *_args: pytest.fail("factory must not run"),
        )

    assert artifacts.ledger._ledger_fd >= 0
    assert artifacts.ledger._lock_fd >= 0
    assert artifacts.ledger._directory_fd >= 0
    artifacts.close()


def test_full_startup_closes_newly_loaded_artifacts_when_factory_fails(
    tmp_path, monkeypatch,
):
    import drone.control.listener as listener
    from test_listener import prepared_startup_files

    attempt_dir = tmp_path / "attempt"
    attempt_dir.mkdir()
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    files, _prepared = prepared_startup_files(attempt_dir)
    config = runtime_configuration(runtime_dir)
    config = replace(
        config,
        connection=replace(config.connection, wire_protocol="2.0"),
    )
    retained = []
    original_validate = listener._validated_artifacts

    def retain_loaded(source):
        artifacts = original_validate(source)
        retained.append(artifacts)
        return artifacts

    monkeypatch.setattr(listener, "_validated_artifacts", retain_loaded)

    with pytest.raises(RuntimeError, match="injected factory failure"):
        construct_after_full_validation(
            files,
            config,
            component_factory=lambda *_args: (_ for _ in ()).throw(
                RuntimeError("injected factory failure")
            ),
        )

    assert retained[0].ledger._ledger_fd == -1
    assert retained[0].ledger._lock_fd == -1
    assert retained[0].ledger._directory_fd == -1


def test_identity_or_wire_mismatch_fails_before_factory(tmp_path):
    selected_profile = profile()
    selected_deployment = deployment(selected_profile)
    valid = runtime_configuration(tmp_path)
    invalid_connection = replace(
        valid.connection,
        source_identity=SourceIdentity(1, 192),
    )
    calls = []

    with pytest.raises(ValueError, match="connection source identity"):
        construct_after_runtime_validation(
            replace(valid, connection=invalid_connection),
            deployment_profile=selected_deployment,
            flight_profile=selected_profile,
            component_factory=lambda config: calls.append(config),
        )

    assert calls == []


def test_autopilot_version_contract_binds_raw_observation_to_profile(tmp_path):
    selected_profile = profile()
    config = runtime_configuration(tmp_path)
    calls = []

    with pytest.raises(ValueError, match="firmware label|stable ArduCopter"):
        construct_after_runtime_validation(
            replace(
                config,
                autopilot_version=replace(
                    config.autopilot_version,
                    firmware_label="ArduCopter 4.5.8",
                ),
            ),
            deployment_profile=deployment(selected_profile),
            flight_profile=selected_profile,
            component_factory=lambda value: calls.append(value),
        )

    assert calls == []


def test_autopilot_version_contract_requires_exact_raw_build_evidence():
    with pytest.raises(ValueError, match="eight bytes"):
        AutopilotVersionContract(
            firmware_label="ArduCopter 4.5.7",
            flight_sw_version=0x040507FF,
            flight_custom_version=b"short",
            evidence_reference="bench record",
        )

    with pytest.raises(ValueError, match="stable ArduCopter 4.5.7"):
        AutopilotVersionContract(
            firmware_label="ArduCopter 4.5.7-custom",
            flight_sw_version=0x040507FF,
            flight_custom_version=b"abcdef0\0",
            evidence_reference="bench record",
        )


def test_waypoint_store_is_validated_without_following_a_dangling_final_link(tmp_path):
    config = runtime_configuration(tmp_path)
    missing = tmp_path / "missing-waypoints.json"
    with pytest.raises(Exception, match="waypoint"):
        replace(config, waypoint_path=missing)

    dangling = tmp_path / "dangling-waypoints.json"
    dangling.symlink_to(tmp_path / "absent-target.json")
    with pytest.raises(Exception, match="waypoint"):
        replace(config, waypoint_path=dangling)

    assert not missing.exists()
    assert dangling.is_symlink()


def test_precision_and_release_consumers_must_share_exact_calibration(tmp_path):
    config = runtime_configuration(tmp_path)
    different_clearance = replace(
        config.clearance_calibration,
        max_age_seconds=0.2,
    )

    with pytest.raises(ValueError, match="same clearance calibration"):
        replace(config, clearance_calibration=different_clearance)


def test_recovery_must_use_site_check_and_shared_clock(tmp_path):
    config = runtime_configuration(tmp_path)

    with pytest.raises(ValueError, match="operating-site recovery check"):
        replace(
            config,
            recovery_policy=replace(config.recovery_policy, check=lambda *_args: None),
        )

    with pytest.raises(ValueError, match="shared timebase"):
        replace(
            config,
            recovery_policy=replace(config.recovery_policy, clock=lambda: 1.0),
        )


def test_vision_requires_verified_files_and_shared_nanosecond_clock(tmp_path):
    calibration_path, mounting_path = write_verified_vision_files(tmp_path)
    mounting = json.loads(mounting_path.read_text())
    mounting["verified"] = False
    mounting_path.write_text(json.dumps(mounting))

    with pytest.raises(ValueError, match="mounting.*verified"):
        VisionConfig(
            marker_size_mm=100.0,
            calibration_path=calibration_path,
            mounting_path=mounting_path,
            receipt_clock_ns=shared_monotonic_ns,
            max_exposure_age_ns=200_000_000,
        )

    with pytest.raises(ValueError, match="shared monotonic nanosecond"):
        VisionConfig(
            marker_size_mm=100.0,
            calibration_path=calibration_path,
            mounting_path=write_verified_vision_files(tmp_path)[1],
            receipt_clock_ns=lambda: 1,
            max_exposure_age_ns=200_000_000,
        )


@pytest.mark.parametrize("config_type", [RangeConfig, PayloadConfig])
def test_constructor_configuration_classes_reject_missing_values(config_type):
    # Their required dataclass fields deliberately have no defaults.
    with pytest.raises(TypeError):
        config_type()


def test_range_and_payload_constructor_relationships_are_validated():
    zero_origin_range = RangeConfig(
        raw_min_cm=0.0,
        raw_max_cm=100.0,
        mounting_offset_cm=0.0,
        stale_after_seconds=0.2,
        startup_timeout_seconds=1.0,
        poll_interval_seconds=0.1,
    )
    assert zero_origin_range.raw_min_cm == 0.0

    with pytest.raises(ValueError, match="raw range must be ordered"):
        replace(zero_origin_range, raw_min_cm=101.0)
    with pytest.raises(ValueError, match="pins must be unique"):
        PayloadConfig(
            pins=(17, 17),
            release_hold_seconds=0.5,
            permission_check_interval_seconds=0.1,
            min_pulse_width_seconds=0.001,
            max_pulse_width_seconds=0.002,
        )


def test_hardware_component_configuration_requires_validated_physical_values():
    range_sensor = RangeConfig(1.0, 4000.0, 1.0, 0.25, 2.0, 0.02)
    payload = PayloadConfig((17, 27), 0.5, 0.05, 0.001, 0.002)

    assert HardwareComponentConfig(range_sensor, payload) == HardwareComponentConfig(
        range_sensor=range_sensor,
        payload=payload,
    )
    with pytest.raises(ValueError, match="range_sensor"):
        HardwareComponentConfig(object(), payload)
    with pytest.raises(ValueError, match="payload"):
        HardwareComponentConfig(range_sensor, object())


@pytest.mark.parametrize("field", ["backend", "evidence_reference"])
def test_injected_component_configuration_rejects_placeholder_text(field):
    values = {
        "backend": "test-injected-components-v1",
        "evidence_reference": "test fixture component binding",
    }
    values[field] = "pending"

    with pytest.raises(ValueError, match="unverified"):
        InjectedComponentConfig(**values)


def test_unverified_operating_site_evidence_is_rejected():
    with pytest.raises(ValueError, match="unverified"):
        OperatingSitePolicy(
            waypoint_check=lambda _name, _coord: True,
            mission_home_check=lambda _home: None,
            recovery_check=lambda _operation, _home, _altitude: None,
            evidence_reference="pending",
        )


def test_exposure_age_must_match_precision_policy(tmp_path):
    config = runtime_configuration(tmp_path)

    with pytest.raises(ValueError, match="exposure age"):
        replace(
            config,
            vision=replace(config.vision, max_exposure_age_ns=100_000_000),
        )


def test_fm3_disabled_runtime_does_not_require_vision_or_precision(tmp_path):
    selected_profile = profile()
    config = replace(
        runtime_configuration(tmp_path),
        enabled_phases=frozenset((FM1, FM2)),
        vision=None,
        precision_policy=None,
    )
    calls = []

    assert construct_after_runtime_validation(
        config,
        deployment_profile=deployment(selected_profile),
        flight_profile=selected_profile,
        component_factory=lambda validated: calls.append(validated) or "built",
    ) == "built"
    assert calls == [config]
    with pytest.raises(CommandRejected, match="disabled"):
        config.require_command_enabled(FM3)
    config.require_command_enabled(FM1)
    config.require_command_enabled(FM2)


def test_enabled_fm3_requires_complete_precision_dependencies(tmp_path):
    config = runtime_configuration(tmp_path)

    with pytest.raises(ValueError, match="FM3 requires verified vision"):
        replace(config, vision=None)
    with pytest.raises(ValueError, match="FM3 requires.*precision"):
        replace(config, precision_policy=None)


def test_precision_uses_the_same_mission_altitudes(tmp_path):
    config = runtime_configuration(tmp_path)

    with pytest.raises(ValueError, match="cruise altitude"):
        replace(config, cruise_altitude_m=11.0)
    with pytest.raises(ValueError, match="drop height"):
        replace(config, desired_drop_height_m=2.0)


def test_live_attempt_timeout_cannot_exceed_six_hundred_seconds(tmp_path):
    with pytest.raises(ValueError, match="600 seconds"):
        replace(runtime_configuration(tmp_path), attempt_timeout_s=600.001)
