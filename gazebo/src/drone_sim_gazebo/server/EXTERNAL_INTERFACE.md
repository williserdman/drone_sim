# Gazebo Server External Interface

The `server` package has no repository-wide transport endpoint. It exports a
small in-process interface used only by the sibling `runtime` package:
`server_spec(...)` derives one frozen `ServerSpec`, and `GazeboServer` starts
and stops that exact child once. `stop(deadline)` returns a frozen
`NativeArtifactSummary` only after native state and the final raw log are
stable.

The child always starts paused with the exact shell-free `gz sim -s` argv. Its
replacement environment has exactly six keys: pinned-image `PATH`,
`GZ_CONFIG_PATH`, and `LD_LIBRARY_PATH`; deterministic `HOME=/tmp`; the
run-isolated `GZ_PARTITION`; and the resolved local `GZ_SIM_RESOURCE_PATH`.
Offline pinned-image probes require the first three to find and load the
ROS-vendored Gazebo executable, and require `HOME` for native state recording.
No ambient Gazebo, SDF, loader, ROS, locale, or Compose value is inherited.
Child stdout and stderr share one append-only
`gazebo/server.log.partial` descriptor and never use a pipe.

`stop` treats its argument as one absolute monotonic infrastructure deadline.
It rejects an already-expired deadline before inspecting or signaling the
child. It sends process-group `SIGTERM`, reserves the remaining budget across
the graceful and forced phases, escalates at most once to `SIGKILL`, and
requires the retained new session's whole process group to be empty even when
the leader exited first. It never uses wall time as simulation time.

The successful native artifact paths are exactly `gazebo/server.log` and
`gazebo/state/state.tlog` below the current canonical run. The `state`
record-path is absent at spawn so Gazebo creates that exact path instead of a
collision-suffixed sibling. Publication descriptor-binds the startup inode,
creates and validates a same-directory no-clobber hard link, durably syncs the
final name, and checks the deadline immediately before committing by removing
the partial name. A deadline at that edge leaves both diagnostic names; after
the commit, no later clock sample reverses success. The final name is durable;
the implementation does not claim that absence of the partial name survives a
crash. Missing, empty, linked, replaced, or unsafe evidence raises
`ServerProcessError` and preserves a named diagnostic. Repeated successful stop
returns the same immutable summary; the first lifecycle failure remains
authoritative on every later call.
