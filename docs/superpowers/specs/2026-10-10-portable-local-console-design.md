# Portable local simulation console

2026-10-10. The user approved this written specification. The
[implementation plan](../plans/2026-10-10-portable-local-console.md) was approved
for execution on 2026-10-12. Implementation is in progress; this describes intended behavior, not verified
platform support.

This extends the [QGC console specification](2026-10-08-local-simulation-console-spec.md)
by replacing its Mac-specific deployment assumptions. Its mission authority,
command admission, recovery, timing, scoring, and evidence contracts remain in
effect. Current verification is recorded in [handoff](../../handoff.md).

## Outcome and scope

Developers can edit Comp 2026, build matching images, select one world and mission,
and operate the simulated aircraft through native QGroundControl on macOS,
Linux, or Windows through WSL2. Gazebo and the camera remain browser views of
the local Linux runtime. Preparation and flight start remain separate actions.

The same mission source under `companion/comp2026` is built on every platform.
Simulation adapters stay under `companion/src/drone_sim_companion`. Platform
code belongs in host orchestration and build tooling. This change does not
create another mission implementation or an aircraft deployment procedure.

The scope includes host detection, QGC integration, native ARM64 and AMD64 image
builds, prerequisites, documentation, and platform verification. It excludes a
native Windows Python controller, remote simulation servers, native desktop
Gazebo installations, automatic QGC flight commands, and new mission behavior.

## Initial support matrix

These are acceptance targets. A row becomes verified only after its checks pass
on that environment and the results identify exact versions.

| Developer system | Controller | Local Linux engine | Images | QGC |
| --- | --- | --- | --- | --- |
| Apple Silicon macOS 13 or newer | Native Python | Colima | `linux/arm64` | Native macOS application |
| Intel macOS 13 or newer | Native Python | Colima | `linux/amd64` | Native macOS application |
| Ubuntu 24.04 desktop, ARM64 | Native Python | Docker Engine | `linux/arm64` | Native Linux application |
| Ubuntu 24.04 desktop, x86-64 | Native Python | Docker Engine | `linux/amd64` | Native Linux application |
| Windows 11, x86-64 | Ubuntu 24.04 in WSL2 | Docker Desktop with WSL2 integration | `linux/amd64` | Native Windows application |

Other distributions and Docker providers may work, but are not inferred to be
verified from this matrix. Windows ARM64 and a native Windows controller are
outside the initial matrix. The QGC acceptance baseline stays at 5.0.8; this
work does not silently upgrade the command implementation or action format.

Python 3.12, the locked workspace dependencies, Docker Compose, and the existing
CPU, memory, and disk requirements remain prerequisites. Native Linux and WSL
do not require Colima. Host checks report the selected provider and architecture
instead of suggesting a Mac-only repair command.

## Operator and developer experience

The entry command remains `uv run --locked drone-sim console`, launched from
the checkout. Windows developers run it inside the selected WSL distribution.
The console opens in the host's default browser.

The workflow remains Build matching images, choose a preset, Prepare, select
the displayed action file in QGC, and Start selected mission in QGC. Flight
interaction and Abort and recover remain in QGC. The browser provides status,
views, evidence, Stop simulation, and Run again. Stop remains infrastructure
teardown and does not imply a landing.

Host detection chooses macOS, Linux, or WSL2 once at startup, with WSL2 detected
before generic Linux. A local `.local-runtime/host.json` file supplies optional
QGC executable and settings-file paths when automatic discovery is ambiguous.
`console --host-config PATH` selects an alternative file. The version-1 document
contains `schema_version`, `platform`, and optional `qgc.executable` and
`qgc.settings_file`; `platform` is `auto`, `macos`, `linux`, or `wsl2`.
Unknown fields and incompatible platform overrides fail with a specific error.

Executable and settings paths are interpreted on the machine running QGC.
For WSL they are Windows paths, including spaces and non-ASCII characters.
Settings are read from the selected installation. The action directory is
derived from QGC's actual `savePath`, not an independently configured folder.
The console displays the native path the operator can find in QGC.

Preparation preserves all existing QGC preferences and the selected action file.
The operator still selects each newly published run file. Missing or ambiguous
settings fail preparation with the exact path and remediation. Platform setup
never claims to have completed a native QGC selection or click.

## Module boundaries

`OperatorSession` continues to own one local session. `LocalRuntime` continues
to coordinate prerequisites and image builds. They receive one resolved host
integration object instead of branching on the operating system throughout
their lifecycle code.

The host integration boundary provides QGC discovery, native paths, action-file
publication, QGC launch or focus, browser launch, and a duplex QGC datagram
transport. Mac, Linux, and WSL implementations share the same lifecycle
contract. Configuration parsing and process/filesystem validation happen at
this boundary.

Token allocation and action rendering stay in `QgcActionStore`. Move its host
publication responsibility behind the publisher boundary and migrate every
caller in the same change. Mission descriptors and action bytes retain their
current format. There is no compatibility wrapper left for the old publisher.

`GcsBridge` retains MAVLink parsing, intervention recording, exact identity
observations, finalization checks, and its TCP connection to SITL. It exchanges
datagrams through the selected host transport. The transport preserves bytes
and order; it does not interpret, generate, acknowledge, or retry flight commands.

The existing Mac and Linux transport uses a local UDP socket and the pinned
QGC peer. The WSL transport exchanges datagrams with a Windows helper over
process pipes. Neither path learns an arbitrary peer from the first packet.

## Windows helper

Use a repository-owned PowerShell script and .NET APIs, launched by WSL
interoperability as the logged-in Windows user. This is a child process with
bounded startup and shutdown, not an installed service. It needs no Windows
Python installation and does not request elevation or change execution policy.
If local policy or disabled interoperability prevents launch, report that
prerequisite rather than attempting a bypass.
Prerequisite checks execute a harmless helper version probe. Policy must permit
the repository script; Windows client defaults can block scripts, and UNC paths
can have additional restrictions. Preserve that explicit setup requirement in
the Windows runbook. See [PowerShell execution policies](https://learn.microsoft.com/en-us/powershell/module/microsoft.powershell.core/about/about_execution_policies).

The helper locates the selected Windows QGC installation, reads its settings,
publishes action files, opens QGC and browser URLs, and owns a Windows loopback
UDP socket. Its expected QGC peer remains exactly `127.0.0.1:14550`.

For the official stable 5.0.8 installation, discovery checks
`%ProgramFiles%\QGroundControl\bin\QGroundControl.exe` and
`%APPDATA%\QGroundControl\QGroundControl.ini`. An explicit path can select a
different installation. Read `[General] savePath` and append `MavlinkActions`.
Confirm the selected executable/version and any existing QGC process agree
before focusing it; blindly starting a second instance is not a focus operation.
The effective QGC MAVLink identity must match the existing 255/190 descriptor.
Report a mismatch without rewriting the user's settings.
These defaults follow the pinned [Windows installer](https://github.com/mavlink/qgroundcontrol/blob/v5.0.8/deploy/windows/nullsoft_installer.nsi),
[QGC settings initialization](https://github.com/mavlink/qgroundcontrol/blob/v5.0.8/src/QGCApplication.cc),
[Qt INI locations](https://doc.qt.io/qt-6/qsettings.html#platform-specific-notes),
and [QGC save-path implementation](https://github.com/mavlink/qgroundcontrol/blob/v5.0.8/src/Settings/AppSettings.cc).

Versioned UTF-8 JSON messages carry requests, responses, helper status, and
base64-encoded UDP datagrams over stdin/stdout. Diagnostics go to stderr.
Every message carries its kind and session ID; requests and responses also
carry a request ID. Control requests are limited to the fixed operations above.
No arbitrary shell command is accepted. Limit each serialized message to
128 KiB and each decoded datagram to the UDP payload limit. Bound queues and
operation deadlines; malformed messages, overflow, or a dead pipe close the
transport with a visible error instead of silently dropping or replaying data.

Only `GcsBridge` parses MAVLink. It sends retained bytes once, and applies its
existing finalization guard immediately before each SITL `tcp.sendall`. This
includes commands already buffered in helper, pipe, or reader queues. Once
finalization is requested, discard those pending commands. Helper reconnect
never replays queued commands. A helper or pipe failure marks QGC disconnected
while healthy simulation services continue under their existing policies.
Before another helper can forward traffic, verify the old process has exited,
reacquire ownership, and confirm the same run and publication. Native QGC itself
is not terminated during helper cleanup.

This avoids cross-VM UDP listeners, Windows host-address discovery, and a
dependency on WSL mirrored networking. Windows browser access to the console
and viewer still uses local TCP forwarding and must pass a real reachability
check. Do not expose listeners on all interfaces to compensate for a failure.
Microsoft documents both [Windows executable interoperability](https://learn.microsoft.com/en-us/windows/wsl/filesystems)
and the [NAT/mirrored networking distinction](https://learn.microsoft.com/en-us/windows/wsl/networking).
The helper protocol is a design choice; those documents do not prove this
application's Windows behavior.

## State, publication, and ownership

Keep macOS token state at its existing Application Support path so upgrading
does not reset its counter. On Linux, use the user's XDG state directory, with
`~/.local/state` as the default. WSL uses its Linux state directory. Checkouts,
run evidence, session receipts, and token state in WSL must reside on the Linux
filesystem. Reject unsupported Windows-mounted storage before allocation.

The existing workstation lease still covers build, direct launch, suite,
Prepare, and cleanup. On WSL, those operations also acquire a Windows named
mutex scoped to the Windows user. Acquire the POSIX lease first, then the
Windows lease, nonblocking. Release in reverse order. Nested controller calls
reuse the already-held leases, and all entry points use this same mechanism.

Two WSL distributions have separate Linux token ledgers. Therefore a durable
Windows registration binds one controller installation and ledger identity to
the Windows user. The helper records that binding under Local AppData while
holding the Windows lease. A different distribution or ledger identity fails
with an ownership error. Missing registered state, an abandoned lease, or an
unresolved run requires recovery inspection before a new run. The Windows
record also retains the active run and helper-session identity
before run side effects begin. A released or abandoned mutex does not clear
that record. Only verified run cleanup clears active ownership. The first
release does not migrate a ledger between WSL distributions or reset a lost
ledger.

The binding includes the WSL registration GUID resolved from `WSL_DISTRO_NAME`
under the Windows user's Lxss registry key, a persisted installation UUID,
and ledger initialization UUID. It does not include the checkout path, so
multiple worktrees share the same owner and
monotonic token sequence. An imported clone of a distribution must not pass as
the registered owner merely because its Linux files contain the same UUID.
A partially initialized registration fails closed.
Neither a new worktree nor a Prepare failure resets consumed tokens.
Missing registration is accepted only for pristine first initialization under
the lease. An initialized ledger with missing registration is an error.

The registry GUID lookup is an implementation-dependent WSL integration, not a
documented public API. Probe the expected registration shape and reject missing
or ambiguous identity instead of falling back to the distribution name. Record
the WSL versions tested against it, including export/import and renamed-distro
cases. A changed or unsupported registration requires an explicit supported
recovery path; it must not silently enroll a new token ledger.

Mac and Linux retain the existing exclusive-file, no-follow, and flush
publication guarantees. Every publisher also verifies the final bytes. On
Windows, publish through native filesystem APIs to a local NTFS QGC save
directory. Create and flush a temporary file in that directory with
`FileStream.Flush(true)`, backed by `FlushFileBuffers`, then publish
with `MoveFileExW` and `MOVEFILE_WRITE_THROUGH`, without
`MOVEFILE_REPLACE_EXISTING`. Reopen and hash the destination before success.
Accept an existing file only when it is a regular file with identical bytes.
Reject conflicting files, unsafe reparse-point destinations, and unsupported
storage. Verify the final bytes and return their SHA-256 plus the native path.
Only exact agreement with the bundled action material permits Compose startup.
These operations follow the Windows [flush](https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-flushfilebuffers)
and [file-move](https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-movefileexw)
contracts. Native fault-injection checks must establish this publisher's
behavior; API choice alone is not a successful durability test.

The Windows helper supplies actual Windows durability operations; do not assume
that directory `fsync` through `/mnt/c` has POSIX semantics. A publication failure
retains the run and consumed token and starts no simulator. A crash cannot
automatically resume a mission. Failed or uncertain publication is never
reported as successful. Preserve original action files and QGC settings.

## Images and provenance

The local build selects `linux/arm64` or `linux/amd64` from the local Docker
engine and verifies that execution uses the host's native architecture. Reject
unsupported architectures, accidental emulation, and remote engine contexts
whose published ports cannot represent this workstation.

Parameterize the existing build and check path; keep `scripts/local-build.sh`
as the documented entry point. Replace the host `shasum` dependency with Python
hashing while preserving deterministic path ordering and the covered input set.
Use a single platform model for prerequisites, builds, and build-record checks.
Do not maintain separate copies of the build script for each operating system.

Add and verify the AMD64 viewer package lock and its Dockerfile selection.
Audit the existing AMD64 branches of the core images, including pinned base
manifests and package availability. Retain exact package locks and source
revisions; do not replace them with floating versions to make another
architecture build. Architecture-specific package versions remain explicit.

The build record binds source revision, runtime input digest, local Docker
context, selected platform, and all eight image IDs. Check actual image
architectures, not only the requested build flag. A platform or context change
requires a matching build. Preserve the existing ARM64 record when it already
satisfies the same schema and checks; an invalid record must not be relabeled.

Builds stay explicit and cached. Source edits do not update running containers.
The build lease prevents this project's commands from replacing tags during a
run. Artifact provenance identifies the actual images used by that run.

## Views, lifecycle, and evidence

The console and local runtime ports remain bound to loopback. The camera stays
behind the existing same-origin proxy. Gazebo remains the existing interactive
streamed viewer with navigation and read-only inspection. Its status uses the
actual browser connection, independently of flight readiness.

On WSL, the helper opens the Windows browser only after probing the browser-
visible console endpoint. Verify viewer access separately. Failure to open a
browser displays the URL and does not start or stop a mission. Preserve origin
and request-token checks. Browser closure or stale views cannot revoke onboard
mission authority or block an otherwise eligible native QGC command.

Host integration records the detected platform, selected QGC paths/version,
transport kind, helper version/digest when present, native action publication
path/digest, and ownership lifecycle in run logs included in the manifest.
Do not include unrelated environment variables or credentials. Transport status,
QGC command admission, physical outcome, score, and artifact validity remain
separate observations.

Cleanup closes project-owned run transports and manifest log writers before
inventory, publishes media from the existing Linux named-volume spool, and
completes ordinary finalization. The helper may retain a lease-only control
connection through the existing post-manifest container teardown. The bundled
host log ends at `cleanup_pending`; final teardown and owner-clear results are
recorded in session and native registration receipts outside the manifest.
No inventoried log is appended after its writers quiesce.
Keep the lease until owned-resource cleanup is confirmed. Cleanup preserves QGC
and its settings. An orphaned helper or failed media publication blocks a new
run until the existing recovery path resolves it without deleting evidence.

## Verification and completion

| Area | Required evidence |
| --- | --- |
| Host resolution | Mac, Linux, WSL2, unknown host, ambiguous QGC install, explicit paths with spaces/non-ASCII, and invalid configuration cases |
| Image builds | Eight actual images and matching provenance on native ARM64 and AMD64; wrong platform/context/source/image fails before Prepare |
| QGC publication | Exact bytes, preserved settings/selection, duplicate identical file, conflicting file, interrupted write, readback mismatch, and unsupported filesystem |
| Ownership | Competing worktrees and entry points, two WSL distributions, stale helper, partial registration, missing ledger, and cleanup failure |
| Helper transport | Windows process pipe integration, exact UDP peer, byte-for-byte bidirectional MAVLink, bounded queues, malformed messages, EOF, and no replay on restart |
| Browser views | Actual host browser reaches console and Gazebo; camera and stale/reconnect states remain truthful; no browser flight controls |
| Native flight workflow | Genuine QGC action-file selection and Start/Abort, configured takeoff/landing, native takeover, and fresh identity on Run Again |
| Evidence and teardown | Retained QGC transport evidence, independent touchdown/disarm, separate score/artifact results, stopped owned resources, and preserved QGC state |

Use focused tests for host adapters and preserved interfaces, the existing
browser regression, and the supported Linux regression suite. Native Windows
tests must execute the actual helper APIs and pipe transport; mocks cannot
establish Windows publication or process behavior. Run WSL integration checks
under both default NAT and mirrored networking on recorded versions.

For each support-matrix row, perform a real local preparation, native QGC
Start and recovery, viewer/camera inspection, Stop, and Run Again. Exercise the
Comp2026 path as well as configured diagnostics; a configured mission does not
test FM1/FM2/FM3 composition. Capture genuine QGC evidence before final artifact
inventory, following the existing [inspection procedure](../../runbook.md#inspect-qgc-control-evidence).

The previous QGC implementation's open native acceptance gates remain open
until evidence satisfies them. Portability work must not convert unit tests,
historical flights, or a successful build into current flight acceptance.
Record unsupported or unavailable test environments explicitly. Do not mark
the complete platform matrix verified when only the Mac environment was run.

Implementation updates README, orchestration and affected image-module guides,
architecture, runbook, handoff, and LEDGER in the same change. The source map and
Comp2026 development loop remain the entry point for mission changes. Record
verified versions, setup steps, command output, and remaining gaps without
duplicating mission behavior in platform-specific documentation.

## Alternatives and tradeoffs

A native Windows controller would require replacing POSIX lease and protocol
file guarantees throughout orchestration. WSL preserves those interfaces and
limits Windows code to native QGC integration.

Direct WSL-to-Windows UDP would require distinct binding, peer-address, and
firewall behavior under NAT and mirrored networking. The pipe-connected helper
adds a small process/protocol boundary but keeps QGC traffic on Windows loopback.

A streamed Linux QGC would reduce host integration, but the agreed workflow
uses the developer's native ground-control application. The helper preserves
that workflow without changing the mission command path.
