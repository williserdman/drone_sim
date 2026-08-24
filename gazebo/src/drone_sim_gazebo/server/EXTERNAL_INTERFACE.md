# Gazebo Server External Interface

The `server` package has no repository-wide transport endpoint. It exports a
small in-process interface used only by the sibling `runtime` package:
`server_spec(...)` derives one frozen `ServerSpec`, and `GazeboServer` starts
and stops that exact child once. `stop(deadline)` returns a frozen
`NativeArtifactSummary` only after native state and the final raw log are
stable.

The child always starts paused with the exact shell-free `gz sim -s` argv. Its
environment contains only the run-isolated `GZ_PARTITION` and the resolved
local `GZ_SIM_RESOURCE_PATH`; ambient Gazebo, SDF, loader, ROS, and Compose
values are not inherited. Child stdout and stderr share one append-only
`gazebo/server.log.partial` descriptor and never use a pipe.

`stop` treats its argument as one absolute monotonic infrastructure deadline.
It sends process-group `SIGTERM`, reserves the remaining budget across the
graceful and forced phases, escalates at most once to `SIGKILL`, and reaps the
tracked group leader. It never uses wall time as simulation time.

The successful native artifact paths are exactly `gazebo/server.log` and
`gazebo/state/state.tlog` below the current canonical run. Publication is a
same-directory hard-link/unlink no-clobber commit followed by directory
`fsync`. Missing, empty, linked, replaced, or unsafe evidence raises
`ServerProcessError` and retains the partial diagnostics. Repeated successful
stop returns the same immutable summary; the first lifecycle failure remains
authoritative on every later call.
