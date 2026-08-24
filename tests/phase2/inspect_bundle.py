#!/usr/bin/env python3
"""Read-only semantic inspection for one Phase 2 run bundle.

This program intentionally runs in the stable artifacts runtime image: the
host gate does not install FFmpeg or the ROS bag Python stack.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import subprocess
import time
from typing import Any

from artifacts import ValidationStatus
from artifacts._adapters.rosbag import (
    FIXED_TOPICS,
    FIXED_TOPIC_TYPES,
    RosbagValidator,
    _Rosbag2Backend,
)
from artifacts._adapters.video import VideoValidator


RUN_STATE_NAMES = {
    0: "CREATED",
    1: "STARTING",
    2: "READY",
    3: "RUNNING",
    4: "FINALIZING",
    5: "COMPLETED",
    6: "FAILED",
    7: "ABORTED",
}


def _stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _run(command: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        capture_output=True,
        check=False,
        text=True,
        timeout=60,
    )


def _video_facts(bundle: Path, stream: str) -> dict[str, Any]:
    relative = f"video/{stream}.mp4"
    path = bundle / relative
    if not path.is_file():
        return {
            "relative_path": relative,
            "exists": False,
            "probe_ok": False,
            "decode_ok": False,
            "streams": [],
            "decoded_frame_hashes": [],
            "validator": {"status": "missing", "detail": "path is missing"},
        }

    probe = _run(
        [
            "ffprobe",
            "-v",
            "error",
            "-print_format",
            "json",
            "-show_streams",
            "-count_frames",
            str(path),
        ]
    )
    try:
        streams = json.loads(probe.stdout).get("streams", []) if probe.returncode == 0 else []
    except json.JSONDecodeError:
        streams = []
    framehash = _run(
        [
            "ffmpeg",
            "-nostdin",
            "-hide_banner",
            "-loglevel",
            "error",
            "-xerror",
            "-err_detect",
            "explode",
            "-i",
            str(path),
            "-map",
            "0:v:0",
            "-f",
            "framehash",
            "-hash",
            "sha256",
            "-",
        ]
    )
    hashes: list[str] = []
    for line in framehash.stdout.splitlines():
        if not line or line.startswith("#"):
            continue
        fields = [item.strip() for item in line.split(",")]
        if len(fields) == 6 and len(fields[-1]) == 64:
            hashes.append(fields[-1])
    expected = len(hashes) if hashes else 1
    validated = VideoValidator().validate(
        bundle,
        relative,
        expected_frame_count=expected,
        outcome="ABORTED" if expected != 40 else "COMPLETED",
        deadline=time.monotonic() + 60,
    )
    selected_streams = []
    for value in streams:
        selected_streams.append(
            {
                key: value.get(key)
                for key in (
                    "codec_type",
                    "codec_name",
                    "pix_fmt",
                    "width",
                    "height",
                    "avg_frame_rate",
                    "nb_read_frames",
                )
            }
        )
    return {
        "relative_path": relative,
        "exists": True,
        "probe_ok": probe.returncode == 0,
        "decode_ok": framehash.returncode == 0,
        "streams": selected_streams,
        "decoded_frame_hashes": hashes,
        "validator": {
            "status": validated.status.value,
            "detail": validated.detail,
            "frame_count": validated.frame_count,
        },
    }


def _event_payload(topic: str, message: Any) -> dict[str, Any]:
    base = {
        "sim_timestamp_ns": _stamp_ns(message.sim_timestamp),
        "event_id": int(message.event_id),
    }
    if topic == "/simulation/scenario_events":
        return {**base, "magnet_id": message.magnet_id, "state": message.state}
    return {
        **base,
        "event_type": message.event_type,
        "value": float(message.value),
        "evidence_ref": message.evidence_ref,
    }


def _bag_facts(bundle: Path, run_id: str) -> dict[str, Any]:
    bag = bundle / "rosbag"
    backend = _Rosbag2Backend()
    try:
        metadata = backend.read_metadata(bag)
    except Exception as error:
        return {
            "structurally_readable": False,
            "structural_error": f"metadata: {type(error).__name__}: {error}",
            "storage_id": None,
            "topics": [],
            "semantic": {"status": "invalid", "detail": "metadata unreadable"},
        }

    metadata_types = {item.name: item.message_type for item in metadata.topics}
    counts: dict[str, int] = defaultdict(int)
    stamps: dict[str, list[int]] = defaultdict(list)
    frame_ids: dict[str, list[int]] = defaultdict(list)
    payload_hashes: dict[str, list[str]] = defaultdict(list)
    lifecycle: list[str] = []
    scenario_events: list[dict[str, Any]] = []
    score_events: list[dict[str, Any]] = []
    ready_record_index: int | None = None
    first_clock_record_index: int | None = None
    structural_error: str | None = None
    try:
        for index, record in enumerate(
            backend.read_messages(bag, metadata.storage_id, metadata_types)
        ):
            topic = record.topic
            message = record.message
            counts[topic] += 1
            if topic == "/clock":
                stamp = _stamp_ns(message.clock)
                if first_clock_record_index is None:
                    first_clock_record_index = index
            elif topic.endswith("/image_raw"):
                stamp = _stamp_ns(message.header.stamp)
                payload_hashes[topic].append(hashlib.sha256(bytes(message.data)).hexdigest())
            else:
                stamp = _stamp_ns(message.sim_timestamp)
            stamps[topic].append(stamp)

            if topic.endswith("/frame_metadata"):
                frame_ids[topic].append(int(message.frame_id))
            elif topic == "/simulation/run_state":
                lifecycle.append(RUN_STATE_NAMES.get(int(message.state), f"UNKNOWN:{message.state}"))
            elif topic == "/simulation/artifact_status" and bool(message.ready):
                if ready_record_index is None:
                    ready_record_index = index
            elif topic == "/simulation/scenario_events":
                scenario_events.append(_event_payload(topic, message))
            elif topic == "/simulation/score_events":
                score_events.append(_event_payload(topic, message))
    except Exception as error:
        structural_error = f"messages: {type(error).__name__}: {error}"

    semantic = RosbagValidator(run_id).validate(bundle, "rosbag")
    topics = [
        {
            "name": topic,
            "message_type": metadata_types.get(topic),
            "metadata_count": next(
                (item.message_count for item in metadata.topics if item.name == topic), 0
            ),
            "decoded_count": counts.get(topic, 0),
            "sim_timestamps_ns": stamps.get(topic, []),
        }
        for topic in FIXED_TOPICS
    ]
    return {
        "structurally_readable": structural_error is None,
        "structural_error": structural_error,
        "storage_id": metadata.storage_id,
        "topics": topics,
        "metadata_inventory": [
            {
                "name": item.name,
                "message_type": item.message_type,
                "message_count": item.message_count,
            }
            for item in metadata.topics
        ],
        "fixed_topic_types": dict(FIXED_TOPIC_TYPES),
        "frame_ids": dict(frame_ids),
        "image_payload_hashes": dict(payload_hashes),
        "lifecycle": lifecycle,
        "scenario_events": scenario_events,
        "score_events": score_events,
        "artifact_ready_record_index": ready_record_index,
        "first_clock_record_index": first_clock_record_index,
        "semantic": {
            "status": semantic.status.value,
            "detail": semantic.detail,
            "topics": [
                {
                    "name": item.name,
                    "message_type": item.message_type,
                    "message_count": item.message_count,
                    "first_sim_timestamp_ns": item.first_sim_timestamp_ns,
                    "last_sim_timestamp_ns": item.last_sim_timestamp_ns,
                }
                for item in semantic.topics
            ],
        },
    }


def inspect(bundle: Path) -> dict[str, Any]:
    manifest = json.loads((bundle / "manifest.json").read_text(encoding="utf-8"))
    run_id = manifest["run_id"]
    return {
        "run_id": run_id,
        "videos": {
            stream: _video_facts(bundle, stream) for stream in ("onboard", "observer")
        },
        "bag": _bag_facts(bundle, run_id),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bundle", type=Path)
    arguments = parser.parse_args()
    print(
        json.dumps(
            inspect(arguments.bundle),
            allow_nan=False,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
