# Gazebo Server Internal Interface

`ServerSpec` is the subprocess seam. It contains the exact argv, immutable
two-key environment, canonical owned paths, seed, and checksums needed for the
single diagnostic preamble. Construction revalidates canonical UUID identity,
the fixed Phase 3 world/vehicle/configuration, absolute non-symlink local
resources, immutable checksum tuples, and current-run path containment.

`GazeboServer.start()` descriptor-walks or creates only the run's `gazebo` and
`gazebo/state` directories, rejects every unexpected owned-path collision,
opens the partial log with create-exclusive/no-follow/append flags, and
durably writes one JSON data record before `Popen`. `shell=False` and
`start_new_session=True` make the tuple argv and process group explicit.
Spawn failure closes all parent descriptors but deliberately preserves the
diagnostic partial.

`GazeboServer.stop()` polls before signaling so a reaped PID is never signaled.
The new-session leader cannot be reused between a live poll and signal because
an intervening exit remains unreaped. Wait timeouts are recomputed from the
same caller deadline; no phase starts a new budget. Once the child is reaped,
retained parent descriptors must still identify the named run directories.
The implementation fsyncs and identity-validates the single-link nonempty
`state.tlog`, then fsyncs and publishes the retained partial-log inode without
replacement. State validation or publication failure never creates a success
summary or removes the recoverable partial.

Process construction, monotonic time, and process-group signaling are internal
seams accepted by `GazeboServer` for host-only tests. They do not widen the
external interface or permit caller changes to authoritative argv,
environment, paths, or publication behavior.
