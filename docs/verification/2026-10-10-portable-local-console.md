# Portable local console verification — 2026-10-10

This record separates software coverage from native workstation evidence. A row
is native-verified only for the checks listed in that row. Historical flights do
not close current portable-console gaps.

## Evidence matrix

| Target | Recorded environment | Source and build evidence | Native result | Outstanding gaps |
| --- | --- | --- | --- | --- |
| macOS 26.6.2, Apple M-series ARM64 | Darwin 25.6.0; 10 host CPUs; 32 GiB RAM; Colima 0.10.3 using Apple virtualization; Docker client 29.6.1 and server 29.5.2; Compose 5.2.0; QGC 5.0.8 universal ARM64/x86-64 | Eight-image `linux/arm64` build and check passed at `19b499c96e60a75456443354c0018b207d712c8e`: [build log](../../.local-runtime/verification/2026-10-10-portable-local-console/arm64-build-19b499c.log), [check log](../../.local-runtime/verification/2026-10-10-portable-local-console/arm64-check-19b499c.log), and [build record](../../.local-runtime/verification/2026-10-10-portable-local-console/build-19b499c.json). The later matching build and idle console restart are recorded separately in the [handoff receipt](../../.local-runtime/verification/2026-10-10-portable-local-console/final-handoff.json). | Configured diagnostic `95bc1701-a129-4d3f-9e88-f741a213d4e2`: genuine QGC launch and bidirectional transport, fresh camera, real noVNC RFB connection, `PREPARED` after 15 minutes 20 seconds, and supported Stop/verified cleanup. See the [Prepare response](../../.local-runtime/verification/2026-10-10-portable-local-console/mac-prepare-19b499c.json), [live session](../../.local-runtime/verification/2026-10-10-portable-local-console/mac-session-live-19b499c.json), [prepared session](../../.local-runtime/verification/2026-10-10-portable-local-console/mac-prepared-19b499c.json), [final session](../../.local-runtime/verification/2026-10-10-portable-local-console/mac-session-final-19b499c.json), [cleanup inventory](../../.local-runtime/verification/2026-10-10-portable-local-console/mac-cleanup-resources-19b499c.log), and [manifest](../../.local-runtime/verification/2026-10-10-portable-local-console/mac-manifest-19b499c.json). No aircraft command was sent. | Native action selection and click, Start/Abort, touchdown/disarm, takeover, stale-action rejection, Comp2026 flight, scoring, artifact acceptance, and native tlog remain pending. |
| macOS 13+, Intel x86-64 | No machine available. | Source tests cover the host adapter; no native build. | Not run. | Native eight-image AMD64 build/check, QGC, Prepare, flight, and cleanup. |
| Ubuntu 24.04 desktop, ARM64 | No machine available. | Source tests cover native Linux discovery. | Not run. | Architecture-matched QGC metadata/install, native build/check, Prepare, flight, and cleanup. |
| Ubuntu 24.04 desktop, x86-64 | No machine available. | AMD64 package candidates were resolved from signed indexes, downloaded, and checksum-verified; this did not execute AMD64 code. | Not run. | Native package scripts, compilation, eight-image build/check, QGC, Prepare, flight, and cleanup. |
| Windows 11 x86-64 with Ubuntu 24.04 WSL2 | No machine available. Supplemental macOS PowerShell 7.6.6 checks are not Windows results. | Python/helper unit and protocol coverage passed in the Linux suite. | Not run. | Windows PowerShell 5.1, local NTFS/registry faults, WSL ext4/UNC identity, competing owners, fixed-peer UDP, helper crash/reconnect, default NAT, mirrored networking, native QGC, and full lifecycle. |

Windows ARM64, emulation, and remote Docker engines are outside the initial
support matrix.

## Supported software gates

The final portability corrections were committed as `81c538c`. The Linux
harness reported base revision `19fa037` plus the tracked corrections because
the gate ran immediately before that commit. The [final fix report](../../.local-runtime/verification/2026-10-10-portable-local-console/final-fix-report.md)
retains the commands and output; the [scoped review](../../.local-runtime/verification/2026-10-10-portable-local-console/final-rereview.md)
confirms both findings were resolved and no new issue was found.

```text
scripts/test-linux.sh orchestration/tests artifacts/tests companion/tests \
  electromagnet/tests gazebo/tests ardupilot_sitl/tests scorekeeper/tests tests -q
2986 passed, 42 skipped in 62.77s

uv run --locked mypy
Success: no issues found in 13 source files

```

The browser and imported mission source were unchanged by the final fixes.
Their preceding successful gates remain:

```text
uv run --with playwright python tests/browser/check_console_views.py
PASS: native-QGC guidance, Prepare/Stop/Run Again requests, desktop reconnect,
and disconnected view indicators
```

The imported Comp2026 tests ran in a separate process to avoid its module-name
collision with the top-level companion tests:

```text
PYTHONPATH=companion/comp2026/src:companion/comp2026/tests \
  uv run --locked pytest \
  companion/comp2026/tests/test_drone_control_connection.py \
  companion/comp2026/tests/test_mission_supervisor.py \
  companion/comp2026/tests/test_auto_attempt.py \
  companion/comp2026/tests/test_qgc_flight_path.py \
  companion/comp2026/tests/test_flight_safety.py \
  companion/comp2026/tests/test_listener_live.py \
  companion/comp2026/tests/test_mission_phase_results.py \
  companion/comp2026/tests/test_precision_mission.py \
  companion/comp2026/tests/test_remaining_development_tools.py \
  companion/comp2026/tests/test_flight_state.py -q
684 passed in 4.54s
```

The 42 Linux skips are explicit capability gates for dependencies unavailable
inside the nested test container. Native results above are recorded separately.

## ARM64 build and prepared-session evidence

The final handoff uses the same build and check commands below. Its exact
source revision, eight image identities, and idle console restart are recorded
in the [handoff receipt](../../.local-runtime/verification/2026-10-10-portable-local-console/final-handoff.json),
[build output](../../.local-runtime/verification/2026-10-10-portable-local-console/arm64-build-final.log),
[check output](../../.local-runtime/verification/2026-10-10-portable-local-console/arm64-check-final.log),
and [build record](../../.local-runtime/verification/2026-10-10-portable-local-console/build-final.json).
The historical run below remains evidence for `19b499c`; a later image build
does not upgrade its flight or artifact outcome.

The exact native build commands and results were:

```text
./scripts/local-build.sh --build
exit 0; compiled the native Gazebo plugins; 5/5 plugin CTests passed; ok:true

./scripts/local-build.sh --check
exit 0; ok:true; source, Colima context, linux/arm64 platform, and all eight
image IDs matched the published build record
```

The [build log](../../.local-runtime/verification/2026-10-10-portable-local-console/arm64-build-19b499c.log),
[check log](../../.local-runtime/verification/2026-10-10-portable-local-console/arm64-check-19b499c.log),
and [schema-1 build record](../../.local-runtime/verification/2026-10-10-portable-local-console/build-19b499c.json)
retain the complete output. The matching `19b499c` build compiled the native
Gazebo plugins and passed all five plugin CTests.
The build record identified Colima and `linux/arm64` with source tree SHA-256
`729aeeb66b64d7dad85efb5d07d8e3a4a2a25aedf5b57e0a81e261162ab564c1`.

| Image | Image ID at `19b499c` |
| --- | --- |
| `drone-sim-orchestration-runtime:phase2` | `sha256:98d697a1490d15f9c45660f2c1f0c95783d7123a6357e677a16db901c1e10357` |
| `drone-sim-artifacts-runtime:phase2` | `sha256:122dc1b6cfc0ec1fa2e3b3ffb02db72cb9eb04ce8dbb45f8d87408901a1562d8` |
| `drone-sim-companion-runtime:phase3` | `sha256:4c002826627ddb4d96811b60a7cd858cf8c6c2533c5fd8f8df3b771c53ace0a7` |
| `drone-sim-ardupilot-runtime:phase3` | `sha256:e6c1931a31ac9d7a0432f33c3bce1ee4ba3e84fb990c19acf0271af8d47ca635` |
| `drone-sim-gazebo-runtime:phase3` | `sha256:8caae3c5561fafcba791d986065bb039b8e26ffa518c92a69b4cf70d3ac6ec0e` |
| `drone-sim-electromagnet-runtime:phase3` | `sha256:493a59df21a4317f0e7f584128c0fdd47113b35d1e32fc62195b49d3299caa84` |
| `drone-sim-scorekeeper-runtime:phase3` | `sha256:b6dba6eefe826c040e2b9a3893a72205c1fbddbca0070307dcd45e292d276b8a` |
| `drone-sim-gazebo-viewer:local` | `sha256:43649a5015358ea51ff12be9669ce4e95e5b1527c39763af6b81d2c32052af16` |

Prepare allocated attempt token 3 and published
`drone-sim-95bc1701-a129-4d3f-9e88-f741a213d4e2.json` with SHA-256
`c293014104cf64e8477ad77b488b9da9510ac68f83cf6f04cc6b60f7af38e787`.
The exact native console lifecycle commands were:

```text
uv run --locked drone-sim console --no-browser
Simulation console: http://127.0.0.1:8765

uv run --locked drone-sim prepare --preset configured-descent \
  --preparation-wall-seconds 1800
exit 0; state PREPARING; request c2d3c129-1fa0-47b6-b32e-9dbc25175b96

uv run --locked drone-sim stop 95bc1701-a129-4d3f-9e88-f741a213d4e2
exit 0; cancellation accepted while the response still reflected PREPARED;
the first poll less than one second later recorded ABORTED, prepared_cancelled,
cleanup complete, and no remaining resources
```

The retained session evidence is the [console log](../../.local-runtime/verification/2026-10-10-portable-local-console/mac-console-19b499c.log),
[Prepare response](../../.local-runtime/verification/2026-10-10-portable-local-console/mac-prepare-19b499c.json),
[live session](../../.local-runtime/verification/2026-10-10-portable-local-console/mac-session-live-19b499c.json),
[prepared session](../../.local-runtime/verification/2026-10-10-portable-local-console/mac-prepared-19b499c.json),
[Stop response](../../.local-runtime/verification/2026-10-10-portable-local-console/mac-stop-19b499c.json),
and [final session](../../.local-runtime/verification/2026-10-10-portable-local-console/mac-session-final-19b499c.json).
QGC 5.0.8 opened as a real native process and exchanged traffic with the
dedicated SITL GCS channel. The session reported QGC fresh, `STABILIZE`,
disarmed, landed, and zero relative altitude. A headless Chrome session opened
the console's noVNC viewer; the UI reported `Live` and the backend recorded
`available:true, connected:true`. The camera reported fresh 320×240 onboard
frames. Loopback ports 5762, 6080, and 8081 were reachable; no all-interface
listener was introduced.

Cold Prepare intentionally waited for the configured 90-second private native
epoch at a 0.1 target real-time factor. Companion prearm became healthy at
23.15 native seconds; the last readiness reason remained the native warmup floor
until `PREPARED`; the observed cold Prepare took 15 minutes 20 seconds. This
delay is relevant to local iteration time before an operator can select Start.

Supported Stop ended the prepared run as `ABORTED: prepared_cancelled` with
cleanup complete. Closed-media publication was verified, all exact-project
containers, the network, and the labeled volume were removed, and native QGC
remained running. Because no mission started, `artifacts-final.json` records
`complete:false`, missing videos, and an invalid zero-clock rosbag. Successful
media ownership cleanup does not make those flight artifacts valid.

The [artifact-final receipt](../../.local-runtime/verification/2026-10-10-portable-local-console/mac-artifacts-final-19b499c.json),
[closed-media receipt](../../.local-runtime/verification/2026-10-10-portable-local-console/mac-media-published-19b499c.json),
[empty resource inventory](../../.local-runtime/verification/2026-10-10-portable-local-console/mac-cleanup-resources-19b499c.log),
[manifest](../../.local-runtime/verification/2026-10-10-portable-local-console/mac-manifest-19b499c.json),
and [final host observation](../../.local-runtime/verification/2026-10-10-portable-local-console/mac-host-integration-19b499c.jsonl)
retain these separate outcomes.

The immutable manifest records `ABORTED`, reason `prepared_cancelled`, score
0/100, and those three incomplete paths. Its final pre-inventory host observation
was captured after quiescence: the SITL bridge was closed, the QGC UDP transport
was still open, the vehicle remained disarmed in STABILIZE at zero relative
altitude, and no fresh QGC traffic was claimed at that boundary. This is separate
from the earlier live session snapshot that recorded fresh bidirectional QGC
traffic. The manifest marks the source dirty only because the required untracked
`.DS_Store` remained preserved; tracked source matched `19b499c`.

No tool with native macOS desktop-control capability was available. The run did
not select the published action, click a QGC action, arm, move, or send a flight
command. It therefore establishes host integration and prepared-session behavior,
not flight acceptance.

## Retained-run cleanup and corrections

Before changing shared image tags, the existing run
`79c1661c-ee24-441b-84e4-ffea8b937552` was re-read. Its original owner was gone,
its session was still preparing, and all eight containers, network, and labeled
media volume remained. Supported Stop produced finalization and media-publication
receipts, retained the incomplete evidence honestly, and removed only that run's
resources. The old bundle was not relabeled as a valid flight.

Two lifecycle defects appeared during this verification and were fixed only
after focused reproductions:

- `745bba5` preserves session `FINALIZING` while orphan cleanup runs instead of
  letting stale controller `READY` mask it. The first Stop started cleanup; a
  second Stop was not required and only observed completion.
- `19b499c` preserves a terminal session after verified cleanup on console
  restart instead of resurrecting stale controller state as `ORPHANED`. A
  terminal receipt without verified cleanup remains blocked.

## Release boundary

`git diff --name-only 9247ca9..HEAD -- companion/comp2026 config` was empty at
the verified source, so the portability work did not change imported mission or
mission configuration bytes. The production console still uses QGC source
255/190 and run-scoped action publication. The existing selected action remained
`esp32_servo_action.json`; its SHA-256 stayed
`a1bbaeaf695dc69fa2e3d364591d55a776f1ccacbde3fc52321f6af0022b0e88`.
Launching QGC updated its normal INI runtime bytes, but did not change the
`flyViewActionsFile` selection. Console, Gazebo desktop, camera, and SITL GCS
ports remain bound to `127.0.0.1`. Host evidence uses a fixed allowlist rather
than an environment dump. The shared Workstation owns lease/publication paths;
no legacy production publisher or lease caller was reintroduced.

Additional retained evidence includes the [Linux gate](../../.local-runtime/verification/2026-10-10-portable-local-console/linux-final.log),
[mypy output](../../.local-runtime/verification/2026-10-10-portable-local-console/mypy-final.log),
[browser gate](../../.local-runtime/verification/2026-10-10-portable-local-console/browser.log),
[separate Comp2026 gate](../../.local-runtime/verification/2026-10-10-portable-local-console/comp2026.log),
[real noVNC check](../../.local-runtime/verification/2026-10-10-portable-local-console/mac-novnc-playwright-19b499c.log),
and the [implementation decision inventory](../../.local-runtime/verification/2026-10-10-portable-local-console/implementation-decisions.md).
Run bundles remain under `runs/<run_id>/`; this verification did not append to
any previously finalized bundle.
