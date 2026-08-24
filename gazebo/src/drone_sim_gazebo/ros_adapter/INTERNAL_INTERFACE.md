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
each stream and at most one aligned camera pair awaits ground truth. A second
sample that would require queue growth raises `AdapterFault`; ground truth is
never buffered as an independent pose stream.

`freeze()` succeeds only after exactly `expected_frames` aligned triples. It is
idempotent, returns the same frozen `AdapterSummary`, and makes every later
sample unacceptable. All returned collections and payloads are immutable
tuples or bytes. This package depends only on the Python standard library.
