# ROS Adapter External Interface

The pure adapter model does not import ROS or Gazebo and does not publish a
transport endpoint itself. The later runtime-facing node maps its validated
values to the fixed public interfaces below.

Each public run has exactly the `onboard` and `observer` camera streams. Every
image is `320x240`, `rgb8`, has step `960`, contains `230400` bytes, and keeps
its positive zero-based public timestamp in both the image header and frame
metadata. Frame IDs are contiguous per stream from zero and capture
timestamps advance by exactly `50,000,000` ns. Camera delivery is reliable at
the ROS boundary; this model never sleeps, drops, retimes, repairs, or creates
a sample.

One public ground-truth value exists only after the same pair ID and native
timestamp have been observed on both cameras and one pose/twist value has that
timestamp. A same-stamp native contact event sets `in_contact=true`; when
odometry advances without such an event, the preceding state is deterministically
`false`. Its vehicle ID is `iris`; position, quaternion, and world-frame ENU
linear and angular velocity pass through unchanged. There is no unrelated
pose-rate public stream.

Any malformed, duplicate, regressing, off-grid, overrun, misaligned, or
post-freeze sample raises `AdapterFault`. Before freeze, the first native
sample-processing fault permanently faults that sequence and the containing
adapter: a later valid sample cannot replace the rejected input, and freeze
cannot report success. Repeated attempts report the same first-fault
diagnostic. Successful completion contains exactly the configured number of
aligned camera/ground-truth pairs.

The production ROS node consumes only private Gazebo-to-ROS bridge topics.
Clock, odometry, contact, and both cameras are one-way inputs; no public topic
is bridged back into Gazebo. Callback-order alignment is bounded to the one
current timestamp, and adapter output freezes before bridge/server shutdown.
Before activation, the node publishes no clock, camera, metadata, ground-truth,
or contact-derived output. The required exact configured native target (Phase 3
default `90.0` seconds) is armed before `RUNNING`; late activation or a
reliable native clock that skips the target faults closed. Native inputs at or
before the target are dropped. A bounded pre-zero queue holds at most two
public epochs (six already-validated frame/truth outputs), faults on overflow,
and drains only after public `/clock=0` is published. The first camera/truth
sample at target + 50 ms maps to public 50 ms. Public clock is capped at
`expected_frames * 50,000,000` ns and closes with adapter completion. Native
Gazebo recording remains unchanged and is never reset.
