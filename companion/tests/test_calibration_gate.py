import json
from types import SimpleNamespace

from drone_sim_companion.calibration_gate import CalibrationGate, calibration_parameters


def test_calibration_parameters_uses_v2_effective_baseline_then_gains():
    config = SimpleNamespace(calibration_json=json.dumps({
        "gains": {"ATC_RAT_RLL_P": 0.12, "ATC_RAT_PIT_P": 0.13},
        "profile": {
            "schema_version": 2,
            "baseline_parameters": {
                "ATC_RAT_RLL_P": 0.04,
                "INS_GYRO_FILTER": 20.0,
            },
            "effective_baseline_parameters": {
                "ATC_RAT_RLL_P": 0.05,
                "INS_GYRO_FILTER": 10.0,
                "PLND_ENABLED": 1.0,
            },
        },
    }))

    assert calibration_parameters(config) == {
        "ATC_RAT_RLL_P": 0.12,
        "ATC_RAT_PIT_P": 0.13,
        "INS_GYRO_FILTER": 10.0,
        "PLND_ENABLED": 1.0,
    }


def test_calibration_parameters_falls_back_to_v1_source_baseline():
    config = SimpleNamespace(calibration_json=json.dumps({
        "gains": {"ATC_RAT_RLL_P": 0.12},
        "profile": {
            "baseline_parameters": {
                "ATC_RAT_RLL_P": 0.04,
                "INS_GYRO_FILTER": 20.0,
            },
        },
    }))

    assert calibration_parameters(config) == {
        "ATC_RAT_RLL_P": 0.12,
        "INS_GYRO_FILTER": 20.0,
    }


def test_calibration_parameters_returns_empty_without_frozen_calibration():
    assert calibration_parameters(SimpleNamespace(calibration_json=None)) == {}


def test_gate_requires_every_effective_parameter():
    gate = CalibrationGate({"ATC_RAT_RLL_P": 0.1, "ATC_RATE_FF_ENAB": 1.0})

    gate.observe("ATC_RAT_RLL_P", 0.1)
    assert not gate.ready
    assert gate.failure is None

    gate.observe("ATC_RATE_FF_ENAB", 1.0)
    assert gate.ready
    assert gate.failure is None
    assert gate.snapshot() == {
        "ATC_RAT_RLL_P": 0.1,
        "ATC_RATE_FF_ENAB": 1.0,
    }


def test_gate_accepts_float32_readback_tolerance():
    gate = CalibrationGate({"ATC_ACC_R_MAX": 123456.789})

    gate.observe("ATC_ACC_R_MAX", 123456.79)

    assert gate.ready
    assert gate.snapshot() == {"ATC_ACC_R_MAX": 123456.79}


def test_gate_records_first_mismatch_as_terminal_failure():
    gate = CalibrationGate({"ATC_RAT_RLL_P": 0.041})

    gate.observe("UNRELATED", 9.0)
    gate.observe("ATC_RAT_RLL_P", 0.05)
    gate.observe("ATC_RAT_RLL_P", 0.041)

    assert not gate.ready
    assert gate.failure == (
        "effective required parameter ATC_RAT_RLL_P is 0.05, expected 0.041"
    )
    assert gate.snapshot() == {}


def test_empty_gate_is_ready():
    gate = CalibrationGate({})

    assert gate.ready
    assert gate.failure is None
    assert gate.snapshot() == {}
