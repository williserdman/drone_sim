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

`GazeboServer.stop()` first checks the deadline, then probes the retained
session/process-group identity. A live leader must still report the original
PGID; after leader exit, a Linux `/proc` probe accepts only live members with
the retained PGID and session ID. Wait timeouts are recomputed from the same
caller deadline; no phase starts a new budget, and group emptiness precedes all
artifact work. Once quiescent, the implementation descriptor-opens Gazebo's
new `state` directory, revalidates retained parent identities, and fsyncs and
identity-validates the single-link nonempty `state.tlog`.

The startup log descriptor remains open through publication. Before linking,
its device/inode/type must match the named single-link partial. The temporary
partial/final link pair must both match that descriptor with link count two.
The final link is directory-fsynced and the deadline is checked before the
partial unlink commit point. Failure before commit retains the partial (and,
after linking, also the final hard link); failure after commit retains the
durable final diagnostic. The commit deliberately makes no crash-durability
claim for removal of the partial name.

Process construction, monotonic time/sleep, process-group identity/probing,
and group signaling are internal seams accepted by `GazeboServer` for
host-only tests. They do not widen the
external interface or permit caller changes to authoritative argv,
environment, paths, or publication behavior.
