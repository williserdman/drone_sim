# Gazebo Server Internal Interface

`ServerSpec` is the subprocess seam. It contains the exact argv, defensively
copied immutable six-key environment, canonical owned paths, seed, and
checksums needed for the single diagnostic preamble. Its four static launch
values are the exact pinned ROS Jazzy setup values proven necessary offline;
partition and resource authority remain run-derived. Construction revalidates canonical UUID identity,
the fixed Phase 3 world/vehicle/configuration, absolute non-symlink local
resources, immutable checksum tuples, and current-run path containment.

`GazeboServer.start()` descriptor-walks or creates only the run's `gazebo`
directory, rejects every unexpected owned-path collision, and deliberately
requires the `state` record path not to exist so Gazebo creates it exactly,
opens the partial log with create-exclusive/no-follow/append flags, and
durably writes one JSON data record before `Popen`. `shell=False` and
`start_new_session=True` make the tuple argv and process group explicit.
Spawn failure closes all parent descriptors but deliberately preserves the
diagnostic partial.

`GazeboServer.stop()` first checks the deadline, then observes leader exit with
Linux `waitid(..., WNOWAIT)` so the unreaped leader PID continues to anchor the
original session and process-group identity. It never calls `poll()`. A live or
zombie leader must still report the original PGID before a signal; a Linux
`/proc` probe accepts only live members with the retained PGID and session ID.
The leader is reaped exactly once, only after that whole group is empty. Every
potentially long group probe is followed by a fresh deadline sample before a
sleep, signal, or other work. No phase starts a new budget, and group emptiness
precedes all artifact work.

Once quiescent, the implementation descriptor-opens Gazebo's new `state`
directory and its exact `state.tlog`, then fsyncs and validates the nonempty
single-link regular file without releasing either identity. After validation,
one final no-follow inventory and descriptor-to-canonical-name check detects a
renamed directory, replacement file, or unexpected sibling before publication.

The startup log descriptor remains open through publication. Before linking,
its device/inode/type must match the named single-link partial. The temporary
partial/final link pair must both match that descriptor with link count two.
The final link is directory-fsynced and the deadline is checked before the
partial unlink commit point. Failure before commit retains the partial (and,
after linking, also the final hard link); failure after commit retains the
durable final diagnostic. The commit deliberately makes no crash-durability
claim for removal of the partial name. Closing the retained parent stream after
that commit is best-effort cleanup and cannot reverse or withhold the immutable
successful summary.

Process construction, monotonic time/sleep, process-group identity/probing,
and group signaling are internal seams accepted by `GazeboServer` for
host-only tests. They do not widen the
external interface or permit caller changes to authoritative argv,
environment, paths, or publication behavior.
