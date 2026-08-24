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


def test_advancing_odometry_closes_missing_contact_as_false_without_growth():
    aggregator = PrivateTruthAggregator()
    aggregator.accept_odometry(_odometry())

    truth = aggregator.accept_odometry(_odometry(STAMP * 2))

    assert truth.sim_timestamp_ns == STAMP
    assert truth.in_contact is False
    assert aggregator.take(STAMP) is truth


def test_truth_aggregation_rejects_mismatched_native_stamps():
    aggregator = PrivateTruthAggregator()
    aggregator.accept_odometry(_odometry())

    with pytest.raises(AggregationFault, match="timestamps do not align"):
        aggregator.accept_contact(STAMP * 2, False)


def test_completed_truth_must_be_consumed_before_next_native_sample():
    aggregator = PrivateTruthAggregator()
    aggregator.accept_contact(STAMP, False)
    aggregator.accept_odometry(_odometry())

    with pytest.raises(AggregationFault, match="awaits camera pair"):
        aggregator.accept_contact(STAMP * 2, False)
