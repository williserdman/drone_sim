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
    OutputEpochGate,
    PublicEpoch,
    PublicFrame,
    PublicGroundTruth,
)


RUN_ID = "00000000-0000-4000-8000-000000000404"
RGB_PAYLOAD = bytes(320 * 240 * 3)
INTERVAL_NS = 50_000_000
PUBLIC_EPOCH_NATIVE_NS = 90_000_000_000


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


def native_image_at(width: int, height: int) -> NativeImage:
    return NativeImage(
        sim_timestamp_ns=INTERVAL_NS,
        width=width,
        height=height,
        encoding="rgb8",
        step=width * 3,
        data=bytes(width * height * 3),
    )


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


def test_public_epoch_uses_the_configured_native_target_and_drops_earlier_samples():
    epoch = PublicEpoch(PUBLIC_EPOCH_NATIVE_NS)

    assert epoch.native_epoch_ns == PUBLIC_EPOCH_NATIVE_NS
    assert epoch.rebase(PUBLIC_EPOCH_NATIVE_NS - 1) is None
    assert epoch.rebase(PUBLIC_EPOCH_NATIVE_NS) is None
    assert epoch.rebase(PUBLIC_EPOCH_NATIVE_NS + 1) == 1


def test_public_epoch_maps_exactly_1200_native_frames_to_50_ms_through_60_seconds():
    epoch = PublicEpoch(PUBLIC_EPOCH_NATIVE_NS)

    public_stamps = tuple(
        epoch.rebase(PUBLIC_EPOCH_NATIVE_NS + frame_number * INTERVAL_NS)
        for frame_number in range(1, 1_201)
    )

    assert len(public_stamps) == 1_200
    assert public_stamps[0] == 50_000_000
    assert public_stamps[-1] == 60_000_000_000
    assert all(
        later - earlier == INTERVAL_NS
        for earlier, later in zip(public_stamps, public_stamps[1:])
    )


def test_output_epoch_gate_maps_different_warmup_histories_to_the_same_fixed_epoch():
    early = OutputEpochGate(
        expected_frames=1_200,
        public_epoch_native_ns=PUBLIC_EPOCH_NATIVE_NS,
    )
    late = OutputEpochGate(
        expected_frames=1_200,
        public_epoch_native_ns=PUBLIC_EPOCH_NATIVE_NS,
    )

    assert early.accept_clock(45_650_000_000) is None
    assert late.accept_clock(73_200_000_000) is None
    early.request_activation()
    late.request_activation()

    assert early.accept_clock(PUBLIC_EPOCH_NATIVE_NS) == 0
    assert late.accept_clock(PUBLIC_EPOCH_NATIVE_NS) == 0
    assert early.native_epoch_ns == PUBLIC_EPOCH_NATIVE_NS
    assert late.native_epoch_ns == PUBLIC_EPOCH_NATIVE_NS


def test_output_epoch_gate_fails_closed_when_activation_is_late():
    gate = OutputEpochGate(
        expected_frames=1_200,
        public_epoch_native_ns=PUBLIC_EPOCH_NATIVE_NS,
    )
    gate.accept_clock(PUBLIC_EPOCH_NATIVE_NS)

    with pytest.raises(AdapterFault, match="before the configured native epoch"):
        gate.request_activation()


def test_output_epoch_gate_fails_closed_when_reliable_clock_skips_target():
    gate = OutputEpochGate(
        expected_frames=1_200,
        public_epoch_native_ns=PUBLIC_EPOCH_NATIVE_NS,
    )
    gate.accept_clock(PUBLIC_EPOCH_NATIVE_NS - 1_000_000)
    gate.request_activation()

    with pytest.raises(AdapterFault, match="skipped configured native epoch"):
        gate.accept_clock(PUBLIC_EPOCH_NATIVE_NS + 1_000_000)


def test_output_epoch_gate_drops_epoch_queue_and_rebases_all_later_native_stamps():
    gate = OutputEpochGate(
        expected_frames=1_200,
        public_epoch_native_ns=PUBLIC_EPOCH_NATIVE_NS,
    )
    gate.accept_clock(PUBLIC_EPOCH_NATIVE_NS - INTERVAL_NS)
    gate.request_activation()
    assert gate.accept_camera("onboard", PUBLIC_EPOCH_NATIVE_NS + INTERVAL_NS) == INTERVAL_NS
    assert gate.rebase_sample(PUBLIC_EPOCH_NATIVE_NS + INTERVAL_NS) == INTERVAL_NS
    assert gate.accept_clock(PUBLIC_EPOCH_NATIVE_NS) == 0

    assert gate.accept_clock(PUBLIC_EPOCH_NATIVE_NS - INTERVAL_NS) is None
    assert gate.accept_camera("onboard", PUBLIC_EPOCH_NATIVE_NS) is None
    assert gate.rebase_sample(PUBLIC_EPOCH_NATIVE_NS) is None
    assert gate.accept_clock(PUBLIC_EPOCH_NATIVE_NS + 1_000_000) == 1_000_000


def test_output_epoch_gate_caps_public_clock_at_configured_final_frame():
    gate = OutputEpochGate(
        expected_frames=1_200,
        public_epoch_native_ns=PUBLIC_EPOCH_NATIVE_NS,
    )
    gate.accept_clock(PUBLIC_EPOCH_NATIVE_NS - INTERVAL_NS)
    gate.request_activation()
    assert gate.accept_clock(PUBLIC_EPOCH_NATIVE_NS) == 0

    assert gate.accept_clock(PUBLIC_EPOCH_NATIVE_NS + 60_000_000_000) == 60_000_000_000
    assert gate.accept_clock(PUBLIC_EPOCH_NATIVE_NS + 60_050_000_000) is None


def test_offset_native_run_assigns_frame_zero_at_50_ms_and_frame_1199_at_60_seconds():
    epoch = PublicEpoch(PUBLIC_EPOCH_NATIVE_NS)
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=1_200)
    first_frame = None
    last_frame = None

    for frame_number in range(1, 1_201):
        native_stamp = PUBLIC_EPOCH_NATIVE_NS + frame_number * INTERVAL_NS
        public_stamp = epoch.rebase(native_stamp)
        assert public_stamp is not None
        onboard, _observer = accept_pair(adapter, public_stamp)
        adapter.accept_ground_truth(native_ground_truth(public_stamp))
        if first_frame is None:
            first_frame = onboard
        last_frame = onboard

    assert (first_frame.frame_id, first_frame.sim_timestamp_ns) == (0, 50_000_000)
    assert (last_frame.frame_id, last_frame.sim_timestamp_ns) == (
        1_199,
        60_000_000_000,
    )
    assert adapter.freeze() == AdapterSummary(
        onboard_frames=1_200,
        observer_frames=1_200,
        paired_frames=1_200,
        ground_truth_samples=1_200,
        first_sim_timestamp_ns=50_000_000,
        last_sim_timestamp_ns=60_000_000_000,
    )


def test_camera_sequence_assigns_contiguous_ids_and_preserves_native_stamp():
    adapter = CameraSequence(run_id=RUN_ID, stream="onboard", expected_frames=2)

    first = adapter.accept(native_image(stamp_ns=50_000_000))
    second = adapter.accept(native_image(stamp_ns=100_000_000))

    assert (first.frame_id, second.frame_id) == (0, 1)
    assert first.sim_timestamp_ns == first.header_timestamp_ns == 50_000_000
    assert second.sim_timestamp_ns == second.header_timestamp_ns == 100_000_000
    assert first.data is RGB_PAYLOAD
    assert adapter.complete


@pytest.mark.parametrize("width,height", [(320, 240), (640, 480)])
def test_adapter_uses_resolved_image_geometry(width, height):
    """Freezing image geometry would break one of the two approved profiles."""
    model = AdapterModel(
        run_id=RUN_ID,
        expected_frames=1,
        width_px=width,
        height_px=height,
    )

    frame = model.accept_frame("onboard", native_image_at(width, height))

    assert (frame.width, frame.height, frame.step) == (width, height, width * 3)


def test_adapter_rejects_native_geometry_that_disagrees_with_resolved_config():
    """The adapter must reject rather than rescale a wrong native image."""
    model = AdapterModel(
        run_id=RUN_ID,
        expected_frames=1,
        width_px=640,
        height_px=480,
    )

    with pytest.raises(AdapterFault, match="640x480"):
        model.accept_frame("onboard", native_image_at(320, 240))


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


def test_camera_sequence_latches_timing_fault_and_rejects_valid_replacement():
    adapter = CameraSequence(run_id=RUN_ID, stream="onboard", expected_frames=2)
    adapter.accept(native_image(stamp_ns=50_000_000))

    with pytest.raises(AdapterFault) as rejected:
        adapter.accept(native_image(stamp_ns=50_000_000))
    with pytest.raises(AdapterFault) as replacement:
        adapter.accept(native_image(stamp_ns=100_000_000))

    assert str(replacement.value) == str(rejected.value)
    assert adapter.accepted_frames == 1
    assert not adapter.complete


def test_camera_sequence_latches_malformed_sample_before_advancing_count():
    adapter = CameraSequence(run_id=RUN_ID, stream="observer", expected_frames=1)

    with pytest.raises(AdapterFault) as rejected:
        adapter.accept(native_image(width=319))
    with pytest.raises(AdapterFault) as replacement:
        adapter.accept(native_image())

    assert str(replacement.value) == str(rejected.value)
    assert adapter.accepted_frames == 0
    assert not adapter.complete


def test_camera_sequence_overrun_fault_revokes_complete_state_irreversibly():
    adapter = CameraSequence(run_id=RUN_ID, stream="onboard", expected_frames=1)
    adapter.accept(native_image(stamp_ns=50_000_000))
    assert adapter.complete

    with pytest.raises(AdapterFault) as rejected:
        adapter.accept(native_image(stamp_ns=100_000_000))
    with pytest.raises(AdapterFault) as replacement:
        adapter.accept(native_image(stamp_ns=100_000_000))

    assert str(replacement.value) == str(rejected.value)
    assert adapter.accepted_frames == 1
    assert not adapter.complete


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


def test_adapter_latches_pair_mismatch_without_accepting_valid_replacement():
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=1)
    onboard = adapter.accept_frame("onboard", native_image(stamp_ns=50_000_000))

    with pytest.raises(AdapterFault) as rejected:
        adapter.accept_frame("observer", native_image(stamp_ns=100_000_000))
    with pytest.raises(AdapterFault) as replacement:
        adapter.accept_frame("observer", native_image(stamp_ns=50_000_000))
    with pytest.raises(AdapterFault) as frozen:
        adapter.freeze()

    assert onboard.frame_id == 0
    assert not adapter.complete
    assert not adapter.camera_pair_complete(0, 50_000_000)
    assert str(replacement.value) == str(rejected.value) == str(frozen.value)


def test_adapter_latches_malformed_frame_before_any_public_output():
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=1)

    with pytest.raises(AdapterFault) as rejected:
        adapter.accept_frame("onboard", native_image(data=RGB_PAYLOAD[:-1]))
    with pytest.raises(AdapterFault) as replacement:
        adapter.accept_frame("onboard", native_image())
    with pytest.raises(AdapterFault) as frozen:
        adapter.freeze()

    assert not adapter.complete
    assert str(replacement.value) == str(rejected.value) == str(frozen.value)


@pytest.mark.parametrize("leading_stream", ["onboard", "observer"])
def test_adapter_pairs_exact_frames_when_one_camera_callback_leads_by_one(
    leading_stream,
):
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=2)
    trailing_stream = "observer" if leading_stream == "onboard" else "onboard"

    first = adapter.accept_frame(leading_stream, native_image(50_000_000))
    second = adapter.accept_frame(leading_stream, native_image(100_000_000))
    trailing_first = adapter.accept_frame(trailing_stream, native_image(50_000_000))
    trailing_second = adapter.accept_frame(trailing_stream, native_image(100_000_000))

    assert (first.frame_id, second.frame_id) == (0, 1)
    assert (trailing_first.frame_id, trailing_second.frame_id) == (0, 1)
    assert adapter.camera_pair_complete(0, 50_000_000)
    assert adapter.camera_pair_complete(1, 100_000_000)
    adapter.accept_ground_truth(native_ground_truth(50_000_000))
    adapter.accept_ground_truth(native_ground_truth(100_000_000))
    assert adapter.complete


def test_adapter_rejects_a_two_frame_camera_callback_lead_without_dropping():
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=3)
    adapter.accept_frame("onboard", native_image(stamp_ns=50_000_000))
    adapter.accept_frame("onboard", native_image(stamp_ns=100_000_000))

    with pytest.raises(AdapterFault, match="onboard camera lookahead buffer is full"):
        adapter.accept_frame("onboard", native_image(stamp_ns=150_000_000))


def test_adapter_buffers_one_lookahead_pair_while_prior_pair_awaits_truth():
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=2)
    accept_pair(adapter, 50_000_000)
    accept_pair(adapter, 100_000_000)

    first = adapter.accept_ground_truth(native_ground_truth(50_000_000))
    second = adapter.accept_ground_truth(native_ground_truth(100_000_000))

    assert first.sim_timestamp_ns == 50_000_000
    assert second.sim_timestamp_ns == 100_000_000
    assert adapter.complete


def test_adapter_rejects_a_third_pair_while_two_pairs_await_truth():
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=3)
    accept_pair(adapter, 50_000_000)
    accept_pair(adapter, 100_000_000)

    with pytest.raises(AdapterFault, match="lookahead"):
        adapter.accept_frame("onboard", native_image(stamp_ns=150_000_000))


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


def test_ground_truth_accepts_huge_finite_integers_without_validation_overflow():
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=1)
    accept_pair(adapter, 50_000_000)
    huge = 10**1000

    public = adapter.accept_ground_truth(
        native_ground_truth(
            position_xyz=(huge, -huge, 0),
            linear_velocity_xyz=(0, huge, -huge),
        )
    )

    assert public.position_xyz == (huge, -huge, 0)
    assert public.linear_velocity_xyz == (0, huge, -huge)


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


def test_adapter_latches_invalid_truth_and_rejects_valid_replacement():
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=1)
    accept_pair(adapter, 50_000_000)

    with pytest.raises(AdapterFault) as rejected:
        adapter.accept_ground_truth(
            native_ground_truth(position_xyz=(math.nan, 2.0, 3.0))
        )
    with pytest.raises(AdapterFault) as replacement:
        adapter.accept_ground_truth(native_ground_truth())
    with pytest.raises(AdapterFault) as frozen:
        adapter.freeze()

    assert not adapter.complete
    assert str(replacement.value) == str(rejected.value) == str(frozen.value)


def test_ground_truth_requires_an_aligned_camera_pair_and_never_forms_pose_stream():
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=1)

    with pytest.raises(AdapterFault):
        adapter.accept_ground_truth(native_ground_truth())


def test_adapter_freeze_requires_exact_expected_aligned_pairs():
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=1)
    accept_pair(adapter, 50_000_000)

    with pytest.raises(AdapterFault):
        adapter.freeze()


def test_adapter_overrun_fault_prevents_successful_freeze():
    adapter = AdapterModel(run_id=RUN_ID, expected_frames=1)
    accept_pair(adapter, 50_000_000)
    adapter.accept_ground_truth(native_ground_truth())
    assert adapter.complete

    with pytest.raises(AdapterFault) as rejected:
        adapter.accept_frame("onboard", native_image(100_000_000))
    with pytest.raises(AdapterFault) as frozen:
        adapter.freeze()

    assert not adapter.complete
    assert str(frozen.value) == str(rejected.value)


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
