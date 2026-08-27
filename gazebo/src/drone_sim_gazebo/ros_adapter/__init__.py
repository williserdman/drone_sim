"""Pure public-sample validation for the Gazebo ROS adapter."""

from .model import (
    AdapterFault,
    AdapterModel,
    AdapterSummary,
    CameraSequence,
    NativeGroundTruth,
    NativeImage,
    PublicFrame,
    PublicGroundTruth,
)
from .aggregation import AggregationFault, NativeOdometry, PrivateTruthAggregator
from .epoch import FRAME_INTERVAL_NS, OutputEpochGate, PublicEpoch
from .live import LiveAdapter
from .payload import PayloadTracker, PublicPayloadState


__all__ = [
    "AdapterFault",
    "AdapterModel",
    "AdapterSummary",
    "CameraSequence",
    "NativeGroundTruth",
    "NativeImage",
    "PublicFrame",
    "PublicGroundTruth",
    "AggregationFault",
    "NativeOdometry",
    "PrivateTruthAggregator",
    "LiveAdapter",
    "FRAME_INTERVAL_NS",
    "OutputEpochGate",
    "PublicEpoch",
    "PayloadTracker",
    "PublicPayloadState",
]
