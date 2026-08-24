# ROS Adapter Internal Interface

`CameraSequence(run_id, stream, expected_frames)` accepts frozen
`NativeImage` values, validates the fixed camera contract, and returns frozen
`PublicFrame` values. The first positive native timestamp establishes the
stream epoch; each later timestamp must be exactly `50,000,000` ns later.

`AdapterModel(run_id, expected_frames)` owns one sequence for each of the two
fixed streams. `accept_frame(stream, sample)` returns the corresponding public
frame. `camera_pair_complete(frame_id, stamp_ns)` is true only while both
camera samples with that exact ID and native timestamp form the current pair.
`accept_ground_truth(sample)` requires that pair and returns its single frozen
`PublicGroundTruth` with unchanged world-frame ENU values.

Buffering is fail-closed and constant: at most one unmatched frame is held for
each stream and at most two aligned camera pairs await ground truth. The single
lookahead pair is required because Gazebo's no-contact truth for one stamp is
closed by odometry from the next stamp, while independent ROS subscriptions may
deliver that next camera pair first. A third pair raises `AdapterFault`;
ground truth is never buffered as an independent pose stream.

Every `AdapterFault` raised while accepting a native frame or ground-truth
sample irreversibly latches the relevant sequence and adapter. All later
acceptance and freeze attempts raise the same deterministic first-fault
diagnostic. Cross-stream candidates are validated and compared with the
unmatched peer before sequence counts or slots advance, so a rejected mismatch
is not represented as accepted state. Constructor and pair-query validation do
not mutate an existing adapter.

`freeze()` succeeds only after exactly `expected_frames` aligned triples. It is
idempotent, returns the same frozen `AdapterSummary`, and makes every later
sample unacceptable. All returned collections and payloads are immutable
tuples or bytes. This package depends only on the Python standard library.

The live layer adds `PrivateTruthAggregator` and `LiveAdapter`. The aggregator
holds only the current odometry/contact candidate and one completed truth
value. Gazebo emits a contact sample when contact exists but does not emit an
empty sample for every no-contact tick; advancing odometry therefore closes
the preceding candidate as `in_contact=false`. An explicit same-stamp contact
closes it with the native state. The live adapter supports either callback
arrival order but releases ground truth only when the pure model reports the
same current camera pair.

`GazeboAdapterNode` is the only ROS-dependent adapter class. It republishes the
private native clock through the sole public `/clock` publisher, publishes the
fixed camera/metadata/truth contract, and has no camera-ack subscription.
