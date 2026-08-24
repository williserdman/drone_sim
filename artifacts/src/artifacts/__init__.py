"""Artifact bundle domain types."""

from .manifest import (
    ArtifactRecord,
    ConfigurationRecord,
    ImageDigest,
    RunManifest,
    SourceRevision,
    write_manifest_atomic,
)
from ._adapters.rosbag import (
    FIXED_TOPIC_TYPES,
    FIXED_TOPICS,
    RecorderFinalization,
    RosbagRecorder,
    RosbagTopicDiagnostic,
    RosbagValidationResult,
    RosbagValidator,
)
from ._adapters.docker_logs import (
    DockerLogCapture,
    DockerLogCaptureError,
    DockerLogCaptureResult,
    DockerLogCommandResult,
    DockerLogDiagnostic,
)
from ._adapters.video import (
    VideoDiagnostic,
    VideoFinalization,
    VideoStreamRecorder,
    VideoValidationResult,
    VideoValidator,
)
from .recorder_node import VideoRecorderNode
from .session import (
    ArtifactSession,
    FinalizationConflict,
    FinalizationInput,
    FinalizationResult,
)
from .structured_log import StructuredEvent, write_event
from .validation import (
    ValidationResult,
    ValidationStatus,
    read_regular_file_bytes,
    validate_regular_file,
    validate_tree,
)

__all__ = [
    "ArtifactRecord",
    "ArtifactSession",
    "ConfigurationRecord",
    "DockerLogCapture",
    "DockerLogCaptureError",
    "DockerLogCaptureResult",
    "DockerLogCommandResult",
    "DockerLogDiagnostic",
    "FinalizationConflict",
    "FinalizationInput",
    "FinalizationResult",
    "FIXED_TOPIC_TYPES",
    "FIXED_TOPICS",
    "ImageDigest",
    "RunManifest",
    "RecorderFinalization",
    "RosbagRecorder",
    "RosbagTopicDiagnostic",
    "RosbagValidationResult",
    "RosbagValidator",
    "SourceRevision",
    "StructuredEvent",
    "ValidationResult",
    "ValidationStatus",
    "VideoDiagnostic",
    "VideoFinalization",
    "VideoRecorderNode",
    "VideoStreamRecorder",
    "VideoValidationResult",
    "VideoValidator",
    "read_regular_file_bytes",
    "validate_regular_file",
    "validate_tree",
    "write_event",
    "write_manifest_atomic",
]
