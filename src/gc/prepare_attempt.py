#!/usr/bin/env python3
"""Offline CLI for preparing a QGC action file and listener session."""

from drone.control.attempt_setup import (
    AttemptLedger,
    COMMANDS,
    DeploymentProfile,
    FIRMWARE_IDENTIFIER,
    LedgerError,
    QGC_IDENTIFIER,
    PreparationError,
    PreparedAttempt,
    REQUIRED_GATES,
    SCHEMA_VERSION,
    SUPPORTED_MAVLINK_DIALECTS,
    SUPPORTED_MAVLINK_WIRE_PROTOCOLS,
    initialize_ledger,
    load_deployment_profile,
    load_prepared_session,
    main,
    prepare_attempt,
)

__all__ = (
    "AttemptLedger",
    "COMMANDS",
    "DeploymentProfile",
    "FIRMWARE_IDENTIFIER",
    "LedgerError",
    "QGC_IDENTIFIER",
    "PreparationError",
    "PreparedAttempt",
    "REQUIRED_GATES",
    "SCHEMA_VERSION",
    "SUPPORTED_MAVLINK_DIALECTS",
    "SUPPORTED_MAVLINK_WIRE_PROTOCOLS",
    "initialize_ledger",
    "load_deployment_profile",
    "load_prepared_session",
    "prepare_attempt",
)


if __name__ == "__main__":
    raise SystemExit(main())
