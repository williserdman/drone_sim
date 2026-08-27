from types import SimpleNamespace

from drone_sim_gazebo.ros_adapter import NativeImage
from drone_sim_gazebo.ros_adapter.live import LiveAdapter


RUN_ID = "11111111-1111-4111-8111-111111111111"
PIXELS = bytes(320 * 240 * 3)


def _image(stamp):
    return NativeImage(stamp, 320, 240, "rgb8", 960, PIXELS)


def _odom(stamp):
    return SimpleNamespace(
        sim_timestamp_ns=stamp,
        position_xyz=(1.0, 2.0, 3.0),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        linear_velocity_xyz=(0.0, 0.0, -1.0),
        angular_velocity_xyz=(0.0, 0.0, 0.0),
    )


def test_live_adapter_uses_the_resolved_competition_image_geometry():
    """The live wrapper must not drop dimensions before native validation."""
    adapter = LiveAdapter(
        run_id=RUN_ID,
        expected_frames=1,
        width_px=640,
        height_px=480,
    )

    frame = adapter.accept_image(
        "onboard",
        NativeImage(
            50_000_000,
            640,
            480,
            "rgb8",
            1920,
            bytes(640 * 480 * 3),
        ),
    )[0]

    assert (frame.width, frame.height, frame.step) == (640, 480, 1920)


def test_live_adapter_publishes_one_truth_after_image_pair_when_truth_arrives_first():
    adapter = LiveAdapter(run_id=RUN_ID, expected_frames=1)
    assert adapter.accept_odometry(_odom(50_000_000)) == ()
    assert adapter.accept_contact(50_000_000, False) == ()

    onboard = adapter.accept_image("onboard", _image(50_000_000))
    observer_and_truth = adapter.accept_image("observer", _image(50_000_000))

    assert len(onboard) == 1
    assert onboard[0].stream == "onboard"
    assert [value.stream for value in observer_and_truth if hasattr(value, "stream")] == ["observer"]
    truths = [value for value in observer_and_truth if hasattr(value, "vehicle_id")]
    assert len(truths) == 1
    assert truths[0].sim_timestamp_ns == 50_000_000
    assert adapter.complete is True
    assert adapter.freeze().ground_truth_samples == 1


def test_live_adapter_publishes_truth_when_native_truth_arrives_after_pair():
    adapter = LiveAdapter(run_id=RUN_ID, expected_frames=1)
    adapter.accept_image("observer", _image(50_000_000))
    adapter.accept_image("onboard", _image(50_000_000))

    assert adapter.accept_contact(50_000_000, True) == ()
    output = adapter.accept_odometry(_odom(50_000_000))

    assert len(output) == 1
    assert output[0].in_contact is True
    assert adapter.complete is True


def test_live_adapter_treats_no_contact_event_before_next_odometry_as_false():
    adapter = LiveAdapter(run_id=RUN_ID, expected_frames=1)
    adapter.accept_image("onboard", _image(50_000_000))
    adapter.accept_image("observer", _image(50_000_000))
    assert adapter.accept_odometry(_odom(50_000_000)) == ()

    output = adapter.accept_odometry(_odom(100_000_000))

    assert len(output) == 1
    assert output[0].sim_timestamp_ns == 50_000_000
    assert output[0].in_contact is False
    assert adapter.complete is True


def test_live_adapter_accepts_next_camera_pair_before_odometry_closes_prior_truth():
    adapter = LiveAdapter(run_id=RUN_ID, expected_frames=2)
    adapter.accept_image("onboard", _image(50_000_000))
    adapter.accept_image("observer", _image(50_000_000))
    assert adapter.accept_odometry(_odom(50_000_000)) == ()

    adapter.accept_image("onboard", _image(100_000_000))
    adapter.accept_image("observer", _image(100_000_000))
    first_truth = adapter.accept_odometry(_odom(100_000_000))
    second_truth = adapter.accept_odometry(_odom(150_000_000))

    assert [value.sim_timestamp_ns for value in first_truth] == [50_000_000]
    assert [value.sim_timestamp_ns for value in second_truth] == [100_000_000]
    assert adapter.complete is True


def test_live_adapter_preserves_truth_when_odometry_leads_pending_camera_callbacks():
    adapter = LiveAdapter(run_id=RUN_ID, expected_frames=2)

    assert adapter.accept_contact(50_000_000, False) == ()
    assert adapter.accept_odometry(_odom(50_000_000)) == ()
    assert adapter.accept_odometry(_odom(100_000_000)) == ()
    assert adapter.accept_contact(51_000_000, True) == ()

    observed = []
    for stamp in (50_000_000, 100_000_000):
        observed.extend(adapter.accept_image("onboard", _image(stamp)))
        observed.extend(adapter.accept_image("observer", _image(stamp)))

    truth_stamps = [
        value.sim_timestamp_ns
        for value in observed
        if hasattr(value, "vehicle_id")
    ]
    truth_contact_states = [
        value.in_contact for value in observed if hasattr(value, "vehicle_id")
    ]
    frame_stamps = [
        value.sim_timestamp_ns for value in observed if hasattr(value, "stream")
    ]
    assert truth_stamps == [50_000_000, 100_000_000]
    assert truth_contact_states == [False, True]
    assert frame_stamps == [50_000_000, 50_000_000, 100_000_000, 100_000_000]
    assert adapter.complete is True


def test_live_adapter_retains_exact_twenty_hz_native_timestamps():
    adapter = LiveAdapter(run_id=RUN_ID, expected_frames=2)
    observed = []
    for stamp in (50_000_000, 100_000_000):
        adapter.accept_contact(stamp, False)
        adapter.accept_odometry(_odom(stamp))
        observed.extend(adapter.accept_image("onboard", _image(stamp)))
        observed.extend(adapter.accept_image("observer", _image(stamp)))

    frame_stamps = [value.sim_timestamp_ns for value in observed if hasattr(value, "stream")]
    assert frame_stamps == [50_000_000, 50_000_000, 100_000_000, 100_000_000]
    assert adapter.freeze().last_sim_timestamp_ns == 100_000_000
