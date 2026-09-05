# Payload timestamp fault

The verification run `28628656-a001-4628-be37-7bb62367aad6` stopped near
192.3 simulated seconds while preparing to take off with the third payload
(internal marker ID 4). The fault came from the tracker for marker ID 2:
`joint truth timestamps must advance by exactly 50000000 ns`.

## Reproduction and solution

The adapter requires every 50 ms joint-state sample, but its reliable ROS
subscription retained only the latest ten messages. A focused test sends valid,
consecutive messages while the adapter executor is temporarily not consuming
callbacks. The old queue evicts samples and reproduces the exact fault.

The adapter subscription and the bridge queues for the three joint-state topics
now retain 1,000 messages. Timestamp validation is unchanged: genuinely missing
or invalid physical samples still fail. The scorer source and Docker image are
unchanged.

The backlog reproduction and existing adapter/payload checks passed (28 tests),
as did the focused runtime/resource checks (22 tests). The image build also passed
its two existing C++ integration tests.

## Limits

The failed run did not record the private joint-state topic, so the exact point
where its sample was lost cannot be reconstructed. Queue eviction is a reproduced
failure mechanism, not direct proof of the historical message's path. The larger
queues remain finite; broader transport hardening is deferred.

## Verification flight

Verification run `259863d9-c558-4102-ae34-fe6c31f5cf94` completed its flight.
It ran from `.worktrees/joint-backlog-verification`, pinning the nested mission
to `7e45d51cc2b9752457db0ac5ae5f005c74d7a176` because the active mission checkout
advanced during the rebuild. The isolated checkout includes the tested queue
changes and the existing 0.50 m/s landing settings.

- Gazebo image: `sha256:0caccd816ef716e4ca0817b4a5965176c3514d23181a9772f52b304146566cb9`.
- Unchanged scorer image: `sha256:b7c0b67b48b980ea1cc393b942cd32ee9fd3dbe22df4bbea0855cfc62746070e`.

An independent read of recorded physical states (not scorer output) confirmed:

- Third payload attached at 181.10 simulated seconds.
- It rose more than one metre at 196.15 seconds and reached a maximum lift of
  10.114 metres, with 457 airborne attached samples.
- It released at 218.95 seconds; the next 50 ms physical sample was detached.
- It settled fully inside the delivery zone by 221.45 seconds.
- Home completion was at 261.20 seconds. Ground truth at that exact timestamp
  confirmed contact, zero speed, and position only 0.0062 metres from home centre.

The independent check returned `physical_mission_verified: true`. Its
[recorded evidence](../runs/259863d9-c558-4102-ae34-fe6c31f5cf94/review_video/physical-verification.json)
contains the attachment, lift, release, settled position, and home-state values.

The timestamp fault did not recur during the flight. Both independent review
videos were closed normally and fully decoded with FFmpeg without errors:

- [Onboard video](../runs/259863d9-c558-4102-ae34-fe6c31f5cf94/review_video/onboard.mp4)
  (264.70 seconds).
- [Observer video](../runs/259863d9-c558-4102-ae34-fe6c31f5cf94/review_video/observer.mp4)
  (264.90 seconds).

Review approximately 3:15–3:42 for the third payload's lift, transport, and drop;
home touchdown is around 4:21. These files cover the flight, not the remaining
idle scoring window.

Scoring is separate: the final manifest reports **150/150**, but the overall run
is **FAILED**, not a validated end-to-end pass. After the completed flight, at
01:34:57 UTC on 2026-09-05, the adapter reported a different stream fault:
`range sequence faulted: downward range timestamps must advance by exactly 50000000 ns`.
The manifest also flags the rosbag's competition payload grid as invalid.
These do not negate the recorded third delivery and home landing, but the full
600-second harness did not finish successfully. The payload-joint fix does not
change the range queues; diagnosis of that later fault and complete-run artifact
validation remain deferred. The scorer source and image remain unchanged.
