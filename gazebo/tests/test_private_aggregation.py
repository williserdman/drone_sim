from types import SimpleNamespace

import pytest

from drone_sim_gazebo.ros_adapter.aggregation import (
    AggregationFault,
    PrivateTruthAggregator,
)


STAMP = 50_000_000


def _odometry(stamp_ns=STAMP):
    return SimpleNamespace(
        sim_timestamp_ns=stamp_ns,
        position_xyz=(1.0, 2.0, 3.0),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        linear_velocity_xyz=(0.1, 0.2, -0.3),
        angular_velocity_xyz=(0.0, 0.0, 0.4),
    )


def test_exact_odometry_and_contact_stamp_produce_one_ground_truth():
    aggregator = PrivateTruthAggregator()

    assert aggregator.accept_odometry(_odometry()) is None
    truth = aggregator.accept_contact(STAMP, True)

    assert truth.sim_timestamp_ns == STAMP
    assert truth.position_xyz == (1.0, 2.0, 3.0)
    assert truth.linear_velocity_xyz == (0.1, 0.2, -0.3)
    assert truth.in_contact is True
    assert aggregator.take(STAMP) is truth
    assert aggregator.take(STAMP) is None


def test_physics_rate_contact_samples_are_downsampled_at_odometry_stamp():
    aggregator = PrivateTruthAggregator()

    for stamp_ns in range(1_000_000, STAMP, 1_000_000):
        assert aggregator.accept_contact(stamp_ns, True) is None

    truth = aggregator.accept_odometry(_odometry())

    assert truth is not None
    assert truth.sim_timestamp_ns == STAMP
    assert truth.in_contact is True


def test_contact_samples_for_next_epoch_can_arrive_before_completed_truth_is_taken():
    aggregator = PrivateTruthAggregator()
    aggregator.accept_contact(STAMP, True)
    truth = aggregator.accept_odometry(_odometry())

    for stamp_ns in range(STAMP + 1_000_000, STAMP * 2, 1_000_000):
        assert aggregator.accept_contact(stamp_ns, True) is None

    assert aggregator.take(STAMP) is truth
    next_truth = aggregator.accept_odometry(_odometry(STAMP * 2))
    assert next_truth is not None
    assert next_truth.sim_timestamp_ns == STAMP * 2
    assert next_truth.in_contact is True


def test_contact_timeline_advancing_past_odometry_proves_no_contact_at_epoch():
    aggregator = PrivateTruthAggregator()
    aggregator.accept_odometry(_odometry())

    truth = aggregator.accept_contact(STAMP + 1_000_000, True)

    assert truth is not None
    assert truth.sim_timestamp_ns == STAMP
    assert truth.in_contact is False


def test_advancing_odometry_closes_missing_contact_as_false_without_growth():
    aggregator = PrivateTruthAggregator()
    aggregator.accept_odometry(_odometry())

    truth = aggregator.accept_odometry(_odometry(STAMP * 2))

    assert truth.sim_timestamp_ns == STAMP
    assert truth.in_contact is False
    assert aggregator.take(STAMP) is truth


def test_truth_aggregation_rejects_nonadvancing_contact_stamps():
    aggregator = PrivateTruthAggregator()
    aggregator.accept_contact(STAMP, True)

    with pytest.raises(AggregationFault, match="contact timestamps must advance"):
        aggregator.accept_contact(STAMP, False)


def test_truth_lookahead_rejects_nonadvancing_odometry_stamps():
    aggregator = PrivateTruthAggregator()
    aggregator.accept_contact(STAMP, False)
    aggregator.accept_odometry(_odometry())

    with pytest.raises(AggregationFault, match="odometry timestamps must advance"):
        aggregator.accept_odometry(_odometry())


def test_completed_truth_lookahead_holds_observed_five_camera_epochs():
    aggregator = PrivateTruthAggregator()
    for index in range(1, 6):
        aggregator.accept_contact(STAMP * index, False)
        aggregator.accept_odometry(_odometry(STAMP * index))

    for index in range(1, 6):
        assert aggregator.take(STAMP * index).sim_timestamp_ns == STAMP * index


def test_completed_truth_lookahead_rejects_an_eleventh_camera_epoch():
    aggregator = PrivateTruthAggregator()
    for index in range(1, 11):
        aggregator.accept_contact(STAMP * index, False)
        aggregator.accept_odometry(_odometry(STAMP * index))

    with pytest.raises(AggregationFault, match="lookahead is full"):
        aggregator.accept_odometry(_odometry(STAMP * 11))
