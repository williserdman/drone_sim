from __future__ import annotations

from dataclasses import FrozenInstanceError
import math

import pytest

from drone_sim_gazebo.ros_adapter import (
    AdapterFault,
    AdapterModel,
    AdapterSummary,
    CameraSequence,
    NativeGroundTruth,
    NativeImage,
    PublicFrame,
    PublicGroundTruth,
)


RUN_ID = "00000000-0000-4000-8000-000000000404"
RGB_PAYLOAD = bytes(320 * 240 * 3)


def native_image(stamp_ns: int = 50_000_000, **changes: object) -> NativeImage:
    values: dict[str, object] = {
        "sim_timestamp_ns": stamp_ns,
        "width": 320,
        "height": 240,
        "encoding": "rgb8",
        "step": 960,
        "data": RGB_PAYLOAD,
    }
    values.update(changes)
    return NativeImage(**values)  # type: ignore[arg-type]


def native_ground_truth(
    stamp_ns: int = 50_000_000, **changes: object
) -> NativeGroundTruth:
    values: dict[str, object] = {
        "sim_timestamp_ns": stamp_ns,
        "position_xyz": (1.0, 2.0, 3.0),
        "orientation_xyzw": (0.0, 0.0, 0.0, 1.0),
        "linear_velocity_xyz": (4.0, 5.0, 6.0),
        "angular_velocity_xyz": (0.1, 0.2, 0.3),
        "in_contact": False,
    }
    values.update(changes)
    return NativeGroundTruth(**values)  # type: ignore[arg-type]


def accept_pair(adapter: AdapterModel, stamp_ns: int) -> tuple[PublicFrame, PublicFrame]:
    onboard = adapter.accept_frame("onboard", native_image(stamp_ns))
    observer = adapter.accept_frame("observer", native_image(stamp_ns))
    return onboard, observer


def test_camera_sequence_assigns_contiguous_ids_and_preserves_native_stamp():
    adapter = CameraSequence(run_id=RUN_ID, stream="onboard", expected_frames=2)

    first = adapter.accept(native_image(stamp_ns=50_000_000))
    second = adapter.accept(native_image(stamp_ns=100_000_000))

    assert (first.frame_id, second.frame_id) == (0, 1)
    assert first.sim_timestamp_ns == first.header_timestamp_ns == 50_000_000
    assert second.sim_timestamp_ns == second.header_timestamp_ns == 100_000_000
    assert first.data is RGB_PAYLOAD
    assert adapter.complete


@pytest.mark.parametrize(
    "samples",
    [
        [50_000_000, 50_000_000],
        [100_000_000, 50_000_000],
        [50_000_000, 100_000_001],
    ],
    ids=["duplicate", "regression", "off-grid"],
)
def test_camera_sequence_fails_closed_on_duplicate_regression_or_off_grid(samples):
    adapter = CameraSequence(run_id=RUN_ID, stream="onboard", expected_frames=2)
    adapter.accept(native_image(stamp_ns=samples[0]))

    with pytest.raises(AdapterFault):
        adapter.accept(native_image(stamp_ns=samples[1]))


@pytest.mark.parametrize(
    ("changes", "case"),
    [
        ({"width": 319}, "width"),
        ({"height": 239}, "height"),
        ({"encoding": "RGB8"}, "encoding"),
        ({"step": 959}, "stride"),
        ({"data": RGB_PAYLOAD[:-1]}, "payload"),
        ({"data": bytearray(RGB_PAYLOAD)}, "mutable-payload"),
    ],
)
def test_camera_sequence_rejects_noncanonical_image_geometry(changes, case):
    del case
    adapter = CameraSequence(run_id=RUN_ID, stream="onboard", expected_frames=1)

    with pytest.raises(AdapterFault):
        adapter.accept(native_image(**changes))


@pytest.mark.parametrize(
    "stamp_ns",
    [0, -1, True, 50_000_000.0],
    ids=["zero", "negative", "boolean", "float"],
)
def test_camera_sequence_requires_a_positive_integer_first_stamp(stamp_ns):
    adapter = CameraSequence(run_id=RUN_ID, stream="onboard", expected_frames=1)

    with pytest.raises(AdapterFault):
        adapter.accept(native_image(stamp_ns=stamp_ns))


@pytest.mark.parametrize(
    ("run_id", "stream", "expected_frames"),
    [
        ("not-a-uuid", "onboard", 1),
        ("AAAAAAAA-AAAA-4AAA-8AAA-AAAAAAAAAAAA", "onboard", 1),
        (RUN_ID, "forward", 1),
        (RUN_ID, "onboard", 0),
        (RUN_ID, "onboard", True),
        (RUN_ID, "onboard", 1.0),
    ],
    ids=[
        "malformed-run-id",
        "noncanonical-run-id",
        "wrong-stream",
        "zero-count",
        "boolean-count",
        "float-count",
    ],
)
def test_camera_sequence_rejects_invalid_identity_or_frame_count(
    run_id, stream, expected_frames
):
    with pytest.raises(AdapterFault):
        CameraSequence(
            run_id=run_id,
            stream=stream,
            expected_frames=expected_frames,
        )


def test_camera_sequence_rejects_frame_overrun():
    adapter = CameraSequence(run_id=RUN_ID, stream="observer", expected_frames=1)
    adapter.accept(native_image(stamp_ns=50_000_000))

    with pytest.raises(AdapterFault):
        adapter.accept(native_image(stamp_ns=100_000_000))


def test_public_frames_are_frozen_and_contain_only_immutable_payloads():
    adapter = CameraSequence(run_id=RUN_ID, stream="observer", expected_frames=1)

    frame = adapter.accept(native_image())

    assert isinstance(frame, PublicFrame)
    assert type(frame.data) is bytes
    with pytest.raises(FrozenInstanceError):
        frame.frame_id = 9  # type: ignore[misc]


def test_adapter_reports_a_camera_pair_only_after_exact_cross_stream_alignment():
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=1)

    onboard = adapter.accept_frame("onboard", native_image())

    assert not adapter.camera_pair_complete(0, 50_000_000)
    observer = adapter.accept_frame("observer", native_image())
    assert adapter.camera_pair_complete(0, 50_000_000)
    assert not adapter.camera_pair_complete(1, 50_000_000)
    assert not adapter.camera_pair_complete(0, 100_000_000)
    assert (onboard.frame_id, observer.frame_id) == (0, 0)


def test_adapter_rejects_wrong_accept_frame_stream():
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=1)

    with pytest.raises(AdapterFault):
        adapter.accept_frame("forward", native_image())


def test_adapter_rejects_mismatched_pair_timestamps():
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=1)
    adapter.accept_frame("onboard", native_image(stamp_ns=50_000_000))

    with pytest.raises(AdapterFault):
        adapter.accept_frame("observer", native_image(stamp_ns=100_000_000))


def test_adapter_bounds_each_unmatched_stream_to_one_frame():
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=2)
    adapter.accept_frame("onboard", native_image(stamp_ns=50_000_000))

    with pytest.raises(AdapterFault):
        adapter.accept_frame("onboard", native_image(stamp_ns=100_000_000))


def test_adapter_rejects_a_later_frame_while_aligned_pair_awaits_truth():
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=2)
    accept_pair(adapter, 50_000_000)

    with pytest.raises(AdapterFault):
        adapter.accept_frame("onboard", native_image(stamp_ns=100_000_000))


def test_ground_truth_maps_world_enu_values_unchanged_once_pair_is_ready():
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=1)
    accept_pair(adapter, 50_000_000)
    native = native_ground_truth(
        position_xyz=(-3.0, 2.5, 8),
        orientation_xyzw=(0.25, -0.5, 0.75, 1.0),
        linear_velocity_xyz=(-4, 5.5, 6.25),
        angular_velocity_xyz=(0.01, -0.02, 0.03),
        in_contact=True,
    )

    public = adapter.accept_ground_truth(native)

    assert public == PublicGroundTruth(
        run_id=RUN_ID,
        vehicle_id="iris",
        sim_timestamp_ns=50_000_000,
        position_xyz=(-3.0, 2.5, 8),
        orientation_xyzw=(0.25, -0.5, 0.75, 1.0),
        linear_velocity_xyz=(-4, 5.5, 6.25),
        angular_velocity_xyz=(0.01, -0.02, 0.03),
        in_contact=True,
    )
    assert adapter.complete


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("position_xyz", (1.0, 2.0)),
        ("orientation_xyzw", (0.0, 0.0, 1.0)),
        ("linear_velocity_xyz", (1.0, 2.0, 3.0, 4.0)),
        ("angular_velocity_xyz", [1.0, 2.0, 3.0]),
    ],
    ids=["position-length", "quaternion-length", "velocity-length", "mutable-vector"],
)
def test_ground_truth_requires_exact_immutable_vector_shapes(field, value):
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=1)
    accept_pair(adapter, 50_000_000)

    with pytest.raises(AdapterFault):
        adapter.accept_ground_truth(native_ground_truth(**{field: value}))


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("position_xyz", (math.nan, 2.0, 3.0)),
        ("orientation_xyzw", (0.0, math.inf, 0.0, 1.0)),
        ("linear_velocity_xyz", (1.0, 2.0, -math.inf)),
        ("angular_velocity_xyz", (0.1, True, 0.3)),
    ],
    ids=["position", "quaternion", "linear-velocity", "angular-velocity-bool"],
)
def test_ground_truth_rejects_nonfinite_or_nonnumeric_values(field, value):
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=1)
    accept_pair(adapter, 50_000_000)

    with pytest.raises(AdapterFault):
        adapter.accept_ground_truth(native_ground_truth(**{field: value}))


@pytest.mark.parametrize(
    ("stamp_ns", "in_contact"),
    [
        (50_000_000.0, False),
        (True, False),
        (50_000_000, 1),
    ],
    ids=["float-stamp", "boolean-stamp", "nonboolean-contact"],
)
def test_ground_truth_requires_integer_stamp_and_boolean_contact(
    stamp_ns, in_contact
):
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=1)
    accept_pair(adapter, 50_000_000)

    with pytest.raises(AdapterFault):
        adapter.accept_ground_truth(
            native_ground_truth(stamp_ns=stamp_ns, in_contact=in_contact)
        )


def test_ground_truth_rejects_a_contact_sample_from_another_capture_epoch():
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=1)
    accept_pair(adapter, 50_000_000)

    with pytest.raises(AdapterFault):
        adapter.accept_ground_truth(
            native_ground_truth(stamp_ns=100_000_000, in_contact=True)
        )


@pytest.mark.parametrize(
    "second_truth_stamp",
    [50_000_000, 49_999_999],
    ids=["duplicate", "regression"],
)
def test_ground_truth_rejects_duplicate_or_regressing_native_time(
    second_truth_stamp
):
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=2)
    accept_pair(adapter, 50_000_000)
    adapter.accept_ground_truth(native_ground_truth(50_000_000))
    accept_pair(adapter, 100_000_000)

    with pytest.raises(AdapterFault):
        adapter.accept_ground_truth(native_ground_truth(second_truth_stamp))


def test_ground_truth_requires_an_aligned_camera_pair_and_never_forms_pose_stream():
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=1)

    with pytest.raises(AdapterFault):
        adapter.accept_ground_truth(native_ground_truth())


def test_adapter_freeze_requires_exact_expected_aligned_pairs():
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=1)
    accept_pair(adapter, 50_000_000)

    with pytest.raises(AdapterFault):
        adapter.freeze()


def test_adapter_freeze_is_idempotent_and_returns_frozen_exact_summary():
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=2)
    accept_pair(adapter, 50_000_000)
    adapter.accept_ground_truth(native_ground_truth(50_000_000))
    accept_pair(adapter, 100_000_000)
    adapter.accept_ground_truth(native_ground_truth(100_000_000, in_contact=True))

    first = adapter.freeze()
    second = adapter.freeze()

    assert first is second
    assert first == AdapterSummary(
        onboard_frames=2,
        observer_frames=2,
        paired_frames=2,
        ground_truth_samples=2,
        first_sim_timestamp_ns=50_000_000,
        last_sim_timestamp_ns=100_000_000,
    )
    with pytest.raises(FrozenInstanceError):
        first.paired_frames = 0  # type: ignore[misc]


@pytest.mark.parametrize("operation", ["frame", "truth"], ids=["frame", "ground-truth"])
def test_frozen_adapter_rejects_every_later_sample(operation):
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=1)
    accept_pair(adapter, 50_000_000)
    adapter.accept_ground_truth(native_ground_truth())
    adapter.freeze()

    with pytest.raises(AdapterFault):
        if operation == "frame":
            adapter.accept_frame("onboard", native_image(100_000_000))
        else:
            adapter.accept_ground_truth(native_ground_truth(100_000_000))
