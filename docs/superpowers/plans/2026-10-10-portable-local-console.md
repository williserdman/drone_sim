# Portable local console implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run the same local simulation workflow on macOS, Ubuntu, and Windows through WSL2, with all aircraft interaction in native QGroundControl.

**Architecture:** Keep the POSIX controller, Linux simulation containers, and existing mission authority. Resolve one host integration at startup and inject it into the session, runtime, and workstation lease. A Windows PowerShell/.NET child handles native QGC integration over bounded process pipes; the existing bridge remains the only MAVLink parser.

**Tech Stack:** Python 3.12, locked uv workspace, pymavlink, Docker Compose, Colima on macOS, Docker Engine on Ubuntu, Docker Desktop with WSL2 integration, PowerShell/.NET, QGC 5.0.8, existing Gazebo/noVNC and camera views.

**Spec:** [Approved portable console design](../specs/2026-10-10-portable-local-console-design.md).

**Status:** Approved for implementation by the user on 2026-10-12. Work is in progress. Native platform acceptance remains separate from software implementation.

## Global Constraints

- Acceptance targets: Apple Silicon and Intel macOS 13 or newer; Ubuntu 24.04 desktop on ARM64 and x86-64; Windows 11 x86-64 with Ubuntu 24.04 in WSL2. Windows ARM64 is outside the initial matrix.
- The QGC acceptance baseline stays at 5.0.8. Retain Python 3.12, locked dependencies, Docker Compose, and current resource requirements: minimum 4 CPUs/8 GiB memory, recommended 6 CPUs/12 GiB, and the existing 8 GiB disk check.
- Use native `linux/arm64` or `linux/amd64` images from a local engine. Do not add emulation, remote runtime support, floating package versions, or a native Windows Python controller.
- Prepare and flight start remain separate. Start, Abort, takeover, and aircraft commands remain in QGC. Stop simulation remains infrastructure teardown, not a landing command.
- Keep mission changes in `companion/comp2026`, including the active FM3 in `src/drone/mock_mission.py`. Keep simulation adapters in `companion/src/drone_sim_companion`; this work changes neither mission behavior nor aircraft deployment.
- Preserve mission descriptors, QGC source identity 255/190, action format, monotonic token range through 16,777,215, command admission, recovery, timing, scoring, and immutable evidence contracts.
- Preserve the macOS token directory `~/Library/Application Support/drone-sim/qgc` and its existing counter. Linux/WSL use XDG state, defaulting to `~/.local/state`; WSL checkout, run, session, and token paths must stay on its Linux filesystem.
- No QGC preference edits, automatic action selection, execution-policy bypass, elevation, installed Windows service, arbitrary helper commands, or QGC termination during cleanup.
- Limit serialized helper messages to 128 KiB of UTF-8 bytes and decoded UDP payloads to 65,507 bytes. Pin the production Windows UDP peer to `127.0.0.1:14550`.
- Keep loopback bindings, same-origin camera proxy, request-token/Origin protections, named-volume media retention, explicit builds, and exact source/image provenance. Do not relabel old build records.
- Update `docs/LEDGER.md` during each task. Preserve imported source history, the original checkout, existing QGC settings/action files, failed-run evidence, and open native acceptance gates.

## Review Focus

1. Windows paths containing spaces/non-ASCII, conflicting running QGC installations, or missing settings must produce an exact native-path error without changing preferences. Tests: Tasks 1, 3, and 8.
2. A copied WSL ledger, partial registration, abandoned mutex, or two worktrees must not create a second owner or reuse tokens. Tests: Tasks 3 and 5.
3. Bytes buffered before Stop must not reach SITL after finalization; helper restart must not replay them. Tests: Tasks 4 and 6.
4. An ambient `DOCKER_HOST` or `DOCKER_DEFAULT_PLATFORM` must not defeat local/native build checks even when the context label and old record look valid. Tests: Task 7.
5. Browser reachability checks must run after HTTP starts serving; unavailable interop or blocked scripts must yield a usable console with remediation, not a startup deadlock. Tests: Tasks 4 and 8.

---

## File map and execution order

Paths below are relative to the console worktree. Task file lists distinguish new files from existing ones; a test named in this plan is an assertion to implement, not an already passing check.

| Boundary | Files and responsibility |
| --- | --- |
| Host configuration | New `orchestration/src/orchestration/host_config.py`: strict config, host facts, native image platform, WSL storage checks |
| Host contract | New `orchestration/src/orchestration/host_integration.py`: typed host results/protocols and one adapter resolver |
| POSIX QGC | New `orchestration/src/orchestration/_adapters/posix_host.py` and `qgc_publication.py`: discovery, launch, publication, local UDP |
| Windows native operations | New `orchestration/src/orchestration/windows/HostFilesystem.psm1` and `HostLease.psm1`: QGC paths/files and Windows ownership |
| Windows process boundary | New `orchestration/src/orchestration/windows/host.ps1`, `windows_helper.py`, and `_adapters/wsl_host.py`: fixed protocol, process lifecycle, adapter |
| Shared ownership | Existing `workstation.py`; new `controller_identity.py`: nested lease owner and WSL installation/ledger identities |
| Runtime composition | Existing `gcs_bridge.py`, `local_runtime.py`, `operator_session.py`, `controller.py`, `console.py`, `cli.py`, and `_adapters/suite.py` |
| Build/provenance | New `orchestration/src/orchestration/local_build.py`; existing `scripts/local-build.sh`, image Dockerfiles/locks, new AMD64 viewer lock |
| Verification | Focused Python tests, new `tests/windows/test_host_helper.ps1`, new `tests/integration/test_wsl_host.py`, existing browser and Linux harnesses |
| Human guidance | README, orchestration/image guides, architecture, runbook, handoff, and LEDGER |

Work in the existing isolated feature worktree. Tasks 1–6 establish the contracts before final composition; Task 7 uses the reviewed host and lease interfaces; Task 8 assembles the console; Task 9 records real acceptance. Independent read-only audits and bounded test work can be delegated with separate file ownership. Do not have concurrent workers edit the same lifecycle module or mutate shared Docker tags.

Run commands from the worktree root. Each task's GREEN gate reruns its RED command and `git diff --check`. Keep Linux-only persistence tests in the supported Linux harness. Stage only that task's files and its LEDGER entry before the named commit.

### Task 1: Parse host configuration and resolve platform facts

**Files:**

- Create: `orchestration/src/orchestration/host_config.py`, `orchestration/tests/test_host_config.py`.
- Modify: `pyproject.toml` (configured mypy coverage), `orchestration/README.md`, `docs/LEDGER.md`.

**Interfaces:**

- Produces immutable `HostConfig(platform: Literal["auto", "macos", "linux", "wsl2"], qgc_executable: str | None, qgc_settings_file: str | None)`.
- Produces immutable `HostFacts(kind: Literal["macos", "linux", "wsl2"], machine: Literal["arm64", "amd64"], image_platform: Literal["linux/arm64", "linux/amd64"], provider: str)`.
- Produces `load_host_config(project_directory: Path, config_path: Path | None = None) -> HostConfig`, `detect_host(config: HostConfig) -> HostFacts`, and `validate_state_paths(facts: HostFacts, paths: Sequence[Path]) -> None`; invalid external inputs raise `HostConfigurationError` with the path/field and remediation.

- [x] **Write RED tests** for default/explicit config, absent optional default versus absent explicitly selected file, unknown keys at both levels, wrong scalar types, incompatible overrides, WSL-before-Linux detection, unsupported CPU/host, and resolved paths on Windows-mounted storage. Preserve native QGC strings:

  ```python
  # test_windows_paths_remain_native_strings
  assert config.qgc_executable == r"C:\Program Files\Équipe\QGroundControl.exe"
  assert config.qgc_settings_file == r"C:\Users\Zoë\AppData\Roaming\QGroundControl\QGroundControl.ini"
  # test_wsl_is_detected_before_linux
  assert facts.kind == "wsl2"
  assert facts.image_platform == "linux/amd64"
  # test_wsl_rejects_windows_storage_before_allocation
  assert allocations == []
  assert token_writes == []
  ```

- [x] **Run RED:** `uv run --locked pytest orchestration/tests/test_host_config.py -q`. Require failures for the missing new behavior; resolve unrelated collection failures first.
- [x] **Implement the three functions** in `host_config.py`. Parse schema version 1, use `.local-runtime/host.json` by default, normalize architecture aliases once, and reject an incompatible explicit platform. QGC path strings remain opaque to POSIX path handling. For WSL storage, resolve symlinks and inspect the containing mount; reject drvfs/9p Windows mounts and unsupported filesystems for every supplied state path, including nearest existing parents.
- [x] **Run GREEN and typecheck:** rerun RED and `uv run --locked mypy`. Add the new typed modules to the configured gate as they are introduced in subsequent tasks.
- [x] **Document configuration** with one version-1 example and platform-specific path semantics; record current implementation scope in LEDGER. Commit: `Add strict host configuration and platform resolution`.

### Task 2: Extract host publication and preserve native Mac/Linux behavior

**Files:**

- Create: `orchestration/src/orchestration/host_integration.py`, `orchestration/src/orchestration/qgc_publication.py`, `orchestration/src/orchestration/_adapters/posix_host.py`, `orchestration/tests/test_posix_host.py`, `orchestration/tests/test_qgc_publication.py`, `orchestration/tests/test_local_runtime.py`.
- Modify: `orchestration/src/orchestration/qgc_actions.py`, `orchestration/src/orchestration/local_runtime.py`, `orchestration/src/orchestration/operator_session.py`, `orchestration/tests/test_qgc_actions.py`, `orchestration/tests/test_local_cleanup.py`, `orchestration/tests/test_operator_session.py`, `orchestration/README.md`, `docs/LEDGER.md`.

**Interfaces:**

- Consumes Task 1's `HostConfig`/`HostFacts` and existing `QgcActionMaterial`/`MavlinkIdentity`.
- Produces immutable `QgcInstallation(executable: str, settings_file: str, save_path: str, version: str, source: MavlinkIdentity)`, `PublicationReceipt(native_path: str, filename: str, sha256: str)`, `HostIssue(code: str, message: str)`, and `LaunchResult(opened: bool, detail: str)` in `host_integration.py`.
- Defines `DatagramTransport.send(payload: bytes) -> None`, `receive(timeout_s: float) -> bytes | None`, and `close() -> None`. A timeout returns `None`; a failed transport raises `HostTransportError`. Receipt order is FIFO and close is idempotent.
- Defines immutable `ControllerIdentity(installation_id: UUID, ledger_id: UUID)` and `HostLease.register_controller(identity: ControllerIdentity, *, pristine: bool) -> None`, `HostLease.begin_run(run_id: str) -> None`, `verify_resume(run_id: str, publication: PublicationReceipt) -> None`, and `finish_run(run_id: str, *, verified_cleanup: bool) -> None`. Windows persists these transitions; POSIX retains its existing local session/run evidence.
- Defines `HostIntegration.facts: HostFacts`, `state_directory: Path`, `preflight(paths: Sequence[Path]) -> tuple[HostIssue, ...]`, `inspect_qgc() -> QgcInstallation`, `acquire_native_lease() -> ContextManager[HostLease]`, `publish_actions(material: QgcActionMaterial, installation: QgcInstallation, lease: HostLease) -> PublicationReceipt`, `open_qgc(installation: QgcInstallation) -> LaunchResult`, `open_browser(url: str) -> LaunchResult`, and `open_datagrams(lease: HostLease, run_id: str, publication: PublicationReceipt) -> DatagramTransport`.
- Produces `resolve_host(project_directory: Path, config_path: Path | None = None) -> HostIntegration`, initially selecting `PosixHost` for Mac/Linux and reporting WSL integration unavailable until Task 8 enables the adapter. Produces `publish_posix(material: QgcActionMaterial, directory: Path) -> PublicationReceipt`.

- [x] **Write RED tests** for exact publication bytes/readback hash, identical duplicates, conflicting/symlink files, interrupted writes, readback mismatch, Mac's unchanged counter path, Linux XDG paths, missing/ambiguous QGC settings, and existing-process focus. Assert:

  ```python
  # test_publication_preserves_qgc_preferences_and_selection
  assert settings_file.read_bytes() == before
  assert published.read_bytes() == material.action_bytes
  assert receipt.sha256 == hashlib.sha256(published.read_bytes()).hexdigest()
  # test_mac_upgrade_preserves_counter
  assert next_token == previous_token + 1
  # test_posix_udp_ignores_unpinned_peer
  assert received == [pinned_payload]
  ```

- [x] **Run RED:** `uv run --locked pytest orchestration/tests/test_posix_host.py orchestration/tests/test_qgc_publication.py orchestration/tests/test_qgc_actions.py orchestration/tests/test_local_runtime.py orchestration/tests/test_local_cleanup.py orchestration/tests/test_operator_session.py -q`.
- [x] **Implement `PosixHost` and `publish_posix`.** Retain exclusive/no-follow file operations and file/directory flushes; add final byte readback. Discover QGC from selected native installation/settings, derive `MavlinkActions` from `[General] savePath`, validate effective 255/190 identity and 5.0.8 metadata, and preserve settings. Ambiguity or unverifiable installation yields a prerequisite error. Use argument arrays for process launch. Mac keeps its current state path; Linux uses `$XDG_STATE_HOME/drone-sim/qgc` or its default.
- [x] **Migrate publication callers now.** Add keyword `host: HostIntegration | None = None` to `LocalRuntime` and `OperatorSession`, resolving once at the composition root and reusing it. Remove `QgcActionStore.publish` and `LocalRuntime.qgc_actions_directory`, including their old constructor-only settings override; migrate tests to injected host config. Keep token allocation/rendering untouched. Add the local UDP implementation with an injectable test peer port; production uses 14550 and never learns a peer.
- [x] **Run GREEN**, including real loopback UDP tests and fault-injection publication tests in the Linux harness where required. Confirm `rg -n 'qgc_actions_directory|qgc_action_store\.publish' orchestration` has no production callers.
- [x] **Update orchestration responsibilities and LEDGER.** Commit: `Separate native QGC integration from action allocation`.

### Task 3: Implement native Windows QGC files and durable ownership

**Files:**

- Create: `orchestration/src/orchestration/windows/HostFilesystem.psm1`, `orchestration/src/orchestration/windows/HostLease.psm1`, `tests/windows/test_host_helper.ps1`.
- Modify: `orchestration/pyproject.toml` (package data), `orchestration/README.md`, `docs/runbook.md`, `docs/LEDGER.md`.

**Interfaces:**

- Consumes the host contract's installation/publication/identity semantics; native functions return objects with those exact JSON property names.
- Produces `Get-QgcInstallation -Executable <string> -SettingsFile <string>`, `Publish-QgcActions -Directory <string> -Filename <string> -Bytes <byte[]> -Sha256 <string>`, and `Open-Qgc -Installation <object>` in `HostFilesystem.psm1`.
- Produces `Enter-ControllerLease`, `Register-Controller -InstallationId <Guid> -LedgerId <Guid> -DistroName <string>`, `Set-ActiveRun -RunId <string> -SessionId <string>`, `Confirm-RunRecovery -RunId <string> -Publication <object>`, `Complete-ActiveRun -RunId <string> -VerifiedCleanup <bool>`, and `Exit-ControllerLease` in `HostLease.psm1`.

- [x] **Write native RED tests** using a temporary local NTFS directory and child PowerShell processes; do not require an extra Pester installation. Pin the relevant assertions:

  ```powershell
  # Test-UnicodePublicationAndUnchangedPreferences
  Assert-Equal $receipt.sha256 $expectedHash
  Assert-Equal ([IO.File]::ReadAllBytes($settings) -join ',') $originalSettings
  # Test-AbandonedMutexRetainsActiveRun
  Assert-Equal $registration.active_run_id $runId
  # Test-ClonedDistroCannotReuseLedger
  Assert-Equal $result.code 'controller_identity_mismatch'
  ```

  Add duplicate/conflict, reparse parent/target, non-NTFS/network destination, interruption before/after rename, failed flush/readback, competing Windows mutex, missing registration with existing ledger, partial enrollment, and running QGC executable/version mismatch cases. Inject fault points around real native APIs; also exercise successful real calls.
- [ ] **Run native RED:** `powershell.exe -NoLogo -NoProfile -NonInteractive -File tests/windows/test_host_helper.ps1`. Run on Windows with policy already permitting this repository script; report unavailable hardware or policy as an unexecuted gate, never pass via mocks or bypass flags.
- [x] **Implement native discovery/publication.** Use the spec's exact Windows QGC paths, read actual savePath and identity, and validate running executable/version before focus. Check local NTFS and reparse components through native handles, create a same-directory exclusive temporary file, call `FileStream.Flush(true)`, publish with `MoveFileExW(MOVEFILE_WRITE_THROUGH)` without replacement, then reopen/hash the destination. Identical existing regular files succeed; uncertain writes fail and retain evidence. Do not change the selected action or any INI bytes.
- [x] **Implement Windows ownership.** Use a named mutex scoped to the Windows user SID and durable versioned registration under `%LOCALAPPDATA%\drone-sim`. Resolve the distribution GUID by uniquely matching `WSL_DISTRO_NAME` in the expected HKCU Lxss registration shape. Store GUID, installation UUID, ledger UUID, and active run/helper identity. Use flushed native writes for registration updates. Acquire/release the .NET mutex on the same helper event-loop thread. Missing/ambiguous identity, partial registration, or abandoned ownership requires recovery, not enrollment/reset.
- [ ] **Run native GREEN** and retain raw output with Windows, PowerShell, filesystem, and WSL versions. Verify modules ship as package data. Document execution policy/UNC restrictions and default native paths without changing policy. Commit: `Add Windows QGC publication and controller ownership`.

Native Windows RED/GREEN execution remains pending because no Windows host is available. Source, packaging, supplementary checks, and independent review are complete.

### Task 4: Add bounded Windows helper IPC and the WSL adapter

**Files:**

- Create: `orchestration/src/orchestration/windows/host.ps1`, `orchestration/src/orchestration/windows_helper.py`, `orchestration/src/orchestration/_adapters/wsl_host.py`, `orchestration/tests/test_windows_helper.py`, `orchestration/tests/test_wsl_host.py`, `tests/integration/test_wsl_host.py`.
- Modify: `orchestration/src/orchestration/host_integration.py`, `tests/windows/test_host_helper.ps1`, `orchestration/pyproject.toml`, `orchestration/README.md`, `docs/LEDGER.md`.

**Interfaces:**

- Consumes Tasks 1–3. Produces `WslHost` implementing the complete `HostIntegration` protocol.
- Produces `WindowsHelper.start(*, session_id: str) -> WindowsHelper`, `request(operation: str, payload: Mapping[str, object], *, timeout_s: float = 10.0) -> Mapping[str, object]`, `send(payload: bytes) -> None`, `receive(timeout_s: float) -> bytes | None`, and `close() -> None`. Validate the fixed operation allowlist before writing requests; this is not a shell execution API.
- Wire envelope: `version: 1`, `kind: request|response|status|datagram`, `session_id`; requests/responses also contain `request_id`. Requests contain `operation` and `payload`, responses a typed success or error result, datagrams `data_base64`. Unknown versions/kinds/fields or mismatched IDs fail the connection.
- Fixed operations: `probe`, `inspect_qgc`, `acquire_lease`, `register_controller`, `begin_run`, `verify_resume`, `finish_run`, `publish_actions`, `open_qgc`, `open_browser`, `open_transport`, `close_transport`, `release_lease`, `shutdown`.

- [ ] **Write RED protocol/process tests** for UTF-8 byte limits, invalid/base64-oversized frames, unknown/mismatched IDs, duplicate responses, partial lines, EOF, stderr isolation, queue saturation, deadline expiry, and close twice. Use 256 datagrams per direction, 10-second startup/request deadlines, and 5-second shutdown grace; overflow disconnects rather than dropping. Assert:

  ```python
  # test_stderr_cannot_be_parsed_as_control_output
  assert responses == [expected_response]
  # test_restart_does_not_replay_buffered_datagrams
  assert replacement.received == []
  # test_disabled_interop_or_policy_is_actionable
  assert issue.code in {"wsl_interop_unavailable", "helper_policy_blocked"}
  assert flight_commands == []
  ```

- [ ] **Run RED:** `uv run --locked pytest orchestration/tests/test_windows_helper.py orchestration/tests/test_wsl_host.py -q`; on WSL also run `uv run --locked pytest tests/integration/test_wsl_host.py -q`. Native tests explicitly skip with a capability reason elsewhere.
- [ ] **Implement helper and Python client.** Use UTF-8 JSON lines and base64 bytes, cap lines before full allocation, and keep diagnostics on stderr. The helper owns only Windows loopback UDP with an ephemeral local port and the exact pinned peer. The Python pipe reader parses envelopes into bounded queues, never MAVLink. Control replies have a separate bounded queue so a datagram backlog cannot starve shutdown; cap outstanding requests at 32. Reject packets from any other peer. Validate fixed operations and argument schemas at both boundaries, use native API calls/argument arrays, and report helper version plus source digest.
- [ ] **Implement `WslHost` and its harmless preflight probe.** Launch the packaged script through `powershell.exe` interoperability without execution-policy overrides. Convert the script location for the Windows process; preserve QGC native paths as strings. A blocked probe becomes a prerequisite result. Exercise the adapter directly here; Task 8 enables it in the production resolver after shared leases are integrated. Use short-lived probe/launch processes outside run ownership and the lease-owning helper for run publication/traffic. Before reconnecting, prove the old helper exited, reacquire the Windows mutex under the existing POSIX lease, and verify the same registration/run/publication; otherwise remain disconnected and block a new run.
- [ ] **Run GREEN and native byte round trips.** Exercise actual process pipes with Windows loopback UDP, arbitrary binary/MAVLink frames, exact peer filtering, EOF, and real process termination. Run under WSL default NAT and mirrored networking and record both environments. Mocks remain unit coverage only.
- [ ] **Update protocol/lifecycle documentation and LEDGER.** Commit: `Connect WSL orchestration to a bounded Windows host helper`.

### Task 5: Share leases and controller identity across every entry point

**Files:**

- Create: `orchestration/src/orchestration/controller_identity.py`, `orchestration/tests/test_controller_identity.py`, `orchestration/src/orchestration/local_build.py`, `orchestration/tests/test_local_build.py`.
- Modify: `scripts/local-build.sh`, `orchestration/src/orchestration/qgc_actions.py`, `orchestration/tests/test_qgc_actions.py`, `orchestration/src/orchestration/workstation.py`, `orchestration/src/orchestration/controller.py`, `orchestration/src/orchestration/local_runtime.py`, `orchestration/src/orchestration/operator_session.py`, `orchestration/src/orchestration/_adapters/suite.py`, `orchestration/src/orchestration/cli.py`, `orchestration/tests/test_workstation.py`, `orchestration/tests/test_controller.py`, `orchestration/tests/test_operator_session.py`, `orchestration/tests/test_suite_runtime.py`, `orchestration/tests/test_suite.py`, `orchestration/README.md`, `docs/LEDGER.md`.

**Interfaces:**

- Consumes `HostIntegration.acquire_native_lease()` and Task 2's `HostLease` methods.
- Consumes Task 2's `ControllerIdentity` and `HostLease.register_controller(identity, pristine=...)`. Produces `load_or_enroll_identity(state_directory: Path, lease: HostLease) -> ControllerIdentity`; enrollment persists local UUIDs and registers them through the held Windows lease.
- Produces `QgcActionStore.initialize() -> None`, reusing the existing locked initialization/validation without consuming a token; this lets a pristine WSL enrollment establish a complete ledger before any run.
- Produces `Workstation(host: HostIntegration, *, lock_path: Path | None = None)`, `hold(*, allow_nested: bool = False) -> ContextManager[HostLease]`, and `owned() -> HostLease` in `workstation.py`.
- Produces `runtime_input_digest(project_directory: Path) -> str`, `check_build(project_directory: Path, host: HostIntegration) -> dict[str, object]`, `build_images(project_directory: Path, host: HostIntegration, workstation: Workstation) -> None`, and `main(argv: Sequence[str] | None = None) -> int` in `local_build.py`, initially preserving the existing ARM64 build behavior. Task 7 extends the platform gate.
- Adds keyword `workstation: Workstation | None = None` to `RunController`, `LocalRuntime`, `OperatorSession`, and `SuiteRuntime`. The composition root supplies the same instance; standalone entry points construct one from the same default host config.

- [ ] **Write RED concurrency/ordering tests** with competing processes, nested same-thread calls, and injected helper loss. Cover Prepare/build/direct start/suite/cleanup, lease acquisition and reverse release order, failure to acquire Windows lease, partial enrollment, missing ledger, copied UUIDs with a different distro GUID, and persistent active ownership after mutex release. Also prove that initial Build can enroll, exit, and reacquire without a partial ledger or a consumed token. Before replacing the build script, capture its digest for fixture trees with byte-ordered Unicode paths, untracked inputs, deletions, and changed contents; require the Python extraction to match exactly. Assert:

  ```python
  # test_run_identity_is_durable_before_allocation_side_effects
  assert events.index("windows_begin_run") < events.index("allocate_token")
  assert events.index("windows_begin_run") < events.index("allocate_directory")
  assert events.index("windows_begin_run") < events.index("compose_up")
  # test_cleanup_failure_does_not_clear_owner
  assert registration.active_run_id == run_id
  assert next_prepare.started is False
  # test_worktrees_share_monotonic_ledger
  assert second_token == first_token + 1
  ```

- [ ] **Run RED:** `uv run --locked pytest orchestration/tests/test_controller_identity.py orchestration/tests/test_qgc_actions.py orchestration/tests/test_local_build.py orchestration/tests/test_workstation.py orchestration/tests/test_controller.py orchestration/tests/test_operator_session.py orchestration/tests/test_suite_runtime.py orchestration/tests/test_suite.py -q`. Use Linux harness for existing `/proc/self/fd` persistence cases.
- [ ] **Implement shared ownership.** Preserve current POSIX lock path, no-follow/flock behavior, nonblocking exclusion, and nested thread ownership. Acquire POSIX then Windows; release in reverse. Resolve storage checks before identity/token allocation. Add WSL UUID metadata alongside its ledger; do not reinterpret the existing constant Mac initialization marker or reset counters. For WSL, `Workstation.hold` calls `load_or_enroll_identity` after acquiring both leases. In pristine enrollment, initialize the zero-token ledger and UUID metadata and register them before returning; a crash at an intermediate step remains an explicit recovery error. Pristine enrollment alone may create identities and registration; any partial or missing previously initialized state fails closed with recovery details.
- [ ] **Migrate every caller and remove the old lease API.** Replace `workstation_lock`/`exclusive_run` with explicit `Workstation.hold` scopes, reusing the supplied instance in nested controller calls. Keep headless Linux suite operation independent of QGC installation. At the controller's run-UUID generation boundary, call `lease.begin_run(run_id)` before configuration/action allocation; failures retain the active identity for recovery even if directory allocation never completes. Clear it only after exact run cleanup is verified. Do not treat an environment variable as proof of a held lease.
- [ ] **Move the standalone build under the same lease.** Extract the existing build/check implementation to `local_build.py` without changing its eight-image order, record/check schema, ARM64 behavior, or digest algorithm (specified in Task 7). Make `scripts/local-build.sh` resolve the repo and exec `uv run --locked python -m orchestration.local_build`; call `build_images` directly from `LocalRuntime.build`. Delete the duplicate shell lock and `DRONE_SIM_WORKSTATION_LOCK_HELD` bypass in this same commit. Test real process contention between the standalone builder and all run entry points.
- [ ] **Run GREEN plus real Windows contention.** On two WSL distributions, prove that a copied ledger cannot enroll against the registered owner; verify export/import and renamed-distro lookup behavior with recorded WSL versions. Exercise stale helper and failed media publication; neither permits a new run. Preserve the existing seven-image cloud-suite lifecycle and provenance.
- [ ] **Document ownership/recovery and LEDGER.** Commit: `Unify workstation ownership across local entry points`.

### Task 6: Keep one MAVLink owner across both transports

**Files:**

- Modify: `orchestration/src/orchestration/gcs_bridge.py`, `orchestration/src/orchestration/operator_session.py`, `orchestration/tests/test_gcs_bridge.py`, `orchestration/tests/test_operator_session.py`, `orchestration/README.md`, `docs/LEDGER.md`.

**Interfaces:**

- Consumes `DatagramTransport` and `HostIntegration.open_datagrams(lease, run_id, publication)`.
- Changes bridge constructor to `GcsBridge(run_directory: Path, *, transport: DatagramTransport, tcp_port: int = 5762)`; preserves existing bridge start/stop/status interfaces. Remove its direct UDP ownership and migrate custom-port tests to the POSIX transport fixture.
- Produces the existing intervention/vehicle observation files unchanged, plus visible transport failure status. No new MAVLink parser, command constructor, ACK/retry path, or learned peer.

- [ ] **Write RED byte/race tests** using real loopback TCP and both direct-UDP and queued transport fixtures. Preserve existing MAVLink frame/source tests; add buffered COMMAND_LONG and other command frames, fragmented TCP, interleaved observations, foreign sender, helper EOF during flight, and reconnect after verified same-run recovery. Assert:

  ```python
  # test_finalize_discards_command_already_buffered_in_transport
  assert tcp_writes_after_finalize == []
  # test_single_owner_forwards_exact_frame_once
  assert sitl_received == original_frame
  assert intervention_records == [expected_record]
  # test_transport_failure_does_not_stop_healthy_simulation
  assert status.qgc_connected is False
  assert compose_stop_calls == []
  ```

- [ ] **Run RED:** `uv run --locked pytest orchestration/tests/test_gcs_bridge.py orchestration/tests/test_operator_session.py -q`.
- [ ] **Implement transport injection and finalization guard.** Keep parsing, exact FC/QGC identity, intervention persistence, and TCP connection ownership in the bridge worker. Recheck the existing finalization signal immediately before each `tcp.sendall`, including queued frames. Drop pending frames at finalization or disconnect; no replay after transport replacement. Open the run transport after successful action publication in the configuration-ready callback, before Compose startup; move bridge construction out of the earlier allocation callback. Close only project-owned transport resources. Wire adapter errors into existing connection observations without converting them into aircraft commands or a score verdict.
- [ ] **Run GREEN**, the actual WSL pipe/UDP round trip from Task 4, and configured mypy. Retain evidence of no command writes after finalization under controlled queue barriers, not a sleep-based race test.
- [ ] **Update bridge ownership documentation and LEDGER.** Commit: `Preserve MAVLink authority across portable host transports`.

### Task 7: Build and verify eight native ARM64 or AMD64 images

**Files:**

- Create: `gazebo/local-viewer/viewer-packages.amd64.lock`.
- Modify: `orchestration/src/orchestration/local_build.py`, `orchestration/tests/test_local_build.py`, `scripts/local-build.sh`, `orchestration/src/orchestration/local_runtime.py`, `gazebo/local-viewer/Dockerfile`, `tests/integration/test_local_deployment.py`, `gazebo/tests/test_image_contract.py`, `tests/integration/test_phase2_runtime_contract.py`, `tests/integration/test_phase3_runtime_contract.py`, `tests/contracts/test_mission_suite_workflow.py`, `orchestration/README.md`, `gazebo/README.md`, `docs/LEDGER.md`.
- Audit before changing, and edit only when native build evidence requires it: `ardupilot_sitl/Dockerfile`, `ardupilot_sitl/build-packages.arm64.lock`, `artifacts/Dockerfile`, `artifacts/ffmpeg-packages.lock`, `artifacts/ffmpeg-packages.arm64.lock`, `gazebo/Dockerfile`, `gazebo/harmonic-packages.lock`, `gazebo/harmonic-packages.arm64.lock`, their module READMEs, and the shared ROS base manifests consumed by companion/electromagnet/orchestration/scorekeeper.

**Interfaces:**

- Consumes Task 1's canonical host/platform facts and Task 5's shared `Workstation`.
- Extends Task 5's `check_build(project_directory, host)` and `build_images(project_directory, host, workstation)` to both native image platforms; preserves `runtime_input_digest` and `main` signatures.
- Preserves shell CLI `scripts/local-build.sh --build|--check`, exit behavior, schema-1 build record fields `source_revision`, `source_tree_sha256`, `docker_context`, `target_platform`, and `images`; each ordered image row retains `image`, `image_id`, sorted `repo_digests`, and `os_architecture`. Preserve the existing check-result keys and diagnostics.

- [ ] **Write RED tests** for native ARM/AMD aliases, wrong actual image architecture, remote effective endpoint despite a local context label, mismatching `DOCKER_DEFAULT_PLATFORM`, source mutation mid-build, stale companion revision, missing/replaced images, and eight-image ordering. Retain Task 5's deterministic hash fixtures and compare against the captured legacy digests. Assert:

  ```python
  # test_python_fingerprint_matches_existing_schema
  assert runtime_input_digest(fixture_tree) == legacy_digest
  # test_environment_cannot_hide_remote_daemon
  assert result["ok"] is False
  assert build_calls == []
  # test_matching_arm_record_is_checked_without_relabeling
  assert record.read_bytes() == before
  ```

- [ ] **Run RED:** `uv run --locked pytest orchestration/tests/test_local_build.py tests/integration/test_local_deployment.py gazebo/tests/test_image_contract.py tests/integration/test_phase2_runtime_contract.py tests/integration/test_phase3_runtime_contract.py tests/contracts/test_mission_suite_workflow.py -q`.
- [ ] **Extend the shared Python builder.** Keep Task 5's thin shell entry point and in-process `LocalRuntime.build` call under the shared lease. Check the effective Docker endpoint, including `DOCKER_HOST`/`DOCKER_CONTEXT`/TLS environment and inspected context, not only its display name. Validate daemon OS/architecture against the host and selected provider; explicit incompatible platform overrides fail before build or Prepare.
- [ ] **Preserve hashing and provenance exactly.** Cover `.dockerignore`, `compose.yaml`, `compose.local.yaml`, `config`, `pyproject.toml`, `uv.lock`, `scripts/local-build.sh`, `ardupilot_sitl`, `artifacts`, `companion`, `electromagnet`, `gazebo`, `orchestration`, `ros_ws`, and `scorekeeper`. Use Git's tracked/untracked-excluding-ignored input list sorted as `LC_ALL=C` bytes; feed each path plus NUL followed by inner SHA-256 hex plus newline, or `deleted\n`, into the outer SHA-256. Retain pre/post revision/digest checks and companion revision label checks. Inspect all eight actual images before writing the record. Never rewrite a record merely to make a check pass.
- [ ] **Add the AMD64 viewer lock and selector.** Resolve exact packages against the pinned AMD64 base, review the complete lock, and use Docker's `TARGETARCH` selection. Audit every core AMD64 branch and base manifest through actual builds. Keep unattended/cloud suite topology and its seven-image provenance unchanged; do not make its workflow depend on a native QGC desktop.
- [ ] **Run GREEN and native build gates:** run `scripts/local-build.sh --build` then `scripts/local-build.sh --check` on a native ARM64 workstation and native AMD64 workstation. Require `ok: true`, all eight native image identities, matching context/source, and retained logs. The Linux test container has no Docker socket and cannot substitute for these gates. Commit source/lock fixes before the final matching rebuild.
- [ ] **Update build/platform guidance and LEDGER.** Commit: `Build local simulation images for native ARM64 and AMD64`.

### Task 8: Assemble the portable console, lifecycle evidence, and developer guide

**Files:**

- Modify: `orchestration/src/orchestration/host_integration.py`, `orchestration/src/orchestration/console.py`, `orchestration/src/orchestration/cli.py`, `orchestration/src/orchestration/local_runtime.py`, `orchestration/src/orchestration/operator_session.py`, `orchestration/src/orchestration/controller.py`, `orchestration/src/orchestration/console_static/index.html`, `orchestration/src/orchestration/console_static/console.js`, `orchestration/tests/test_cli.py`, `orchestration/tests/test_console_http.py`, `orchestration/tests/test_local_runtime.py`, `orchestration/tests/test_operator_session.py`, `orchestration/tests/test_controller.py`, `tests/browser/check_console_views.py`, `README.md`, `orchestration/README.md`, `docs/architecture.md`, `docs/runbook.md`, `docs/handoff.md`, `docs/LEDGER.md`.

**Interfaces:**

- Consumes the reviewed host, workstation, transport, and builder interfaces from Tasks 1–7.
- Extends `serve_console(project_directory: Path, *, port: int = 8765, open_browser: bool = True, host_config: Path | None = None) -> None` and CLI `console --host-config PATH`. Construct one host and workstation, then inject both through the existing runtime/session/controller composition.
- Retains the current HTTP endpoints/status contract; extend prerequisite and publication display data with native path and platform details. No browser flight endpoints or automatic QGC action selection.

- [ ] **Write RED lifecycle/UI tests** for all three host kinds, explicit config, Windows native-path display, missing QGC/settings, policy/interop errors, current provider/resource diagnostics, failed publication before Compose, failed helper/media cleanup, and no QGC process termination. Assert:

  ```python
  # test_windows_browser_probe_runs_after_http_is_serving
  assert events.index("http_ready") < events.index("windows_url_probe")
  assert launch_result.opened is True
  # test_browser_open_failure_preserves_console_and_url
  assert status.console_url == expected_url
  assert server_is_serving is True
  assert mission_starts == []
  # test_native_publication_mismatch_never_starts_compose
  assert compose_up_calls == []
  assert allocated_run.exists()
  assert next_token > consumed_token
  # test_all_manifest_logs_are_closed_before_inventory
  assert writes_after_inventory == []
  ```

- [ ] **Run RED:** `uv run --locked pytest orchestration/tests/test_cli.py orchestration/tests/test_console_http.py orchestration/tests/test_local_runtime.py orchestration/tests/test_operator_session.py orchestration/tests/test_controller.py -q`; then `uv run --with playwright python tests/browser/check_console_views.py`.
- [ ] **Enable `WslHost` in the resolver and wire console startup/views.** Start HTTP serving before scheduling native browser launch. WSL `open_browser` probes the Windows-visible loopback console URL with a bounded deadline before opening it; report URL/remediation on failure. Probe viewer reachability separately and retain actual noVNC connection state. Keep camera freshness, flight readiness, and viewer connection independent. Do not fall back to all-interface listeners, gate QGC admission on browser heartbeat, or alter the native click sequence.
- [ ] **Wire preparation and teardown.** Verify native action bytes/hash against the bundled material before Compose. Keep the lease through owned-resource cleanup, media publication, and existing finalization ordering. Capture genuine QGC telemetry through the existing pre-inventory artifact-session seam. Quiesce transport/helper logs before final inventory; persist final ownership/cleanup results before closing the helper. Keep unresolved active ownership and retained run evidence on any uncertain cleanup; orphan recovery must verify old process exit and exact run resources before allowing a new run.
- [ ] **Record host evidence.** Write a manifest-included host log with platform/provider, QGC paths/version, transport kind, helper version/digest, publication path/digest, and ownership transitions. Include only these allowlisted facts, not environment dumps or credentials. Keep command admission, connection, physical outcome, score, and artifact results distinct.
- [ ] **Run GREEN and browser regression.** Exercise Build/Prepare/Stop/Run Again, exact action filename/path, disabled-prerequisite explanations, viewer reconnect, and camera staleness. Assert Start/Abort flight buttons remain absent, legacy direct browser/CLI flight handlers remain rejected for descriptor runs, and native QGC settings/selection stay byte-for-byte unchanged.
- [ ] **Update the user-facing guides.** Add exact setup/launch/rebuild steps for each target platform, WSL Linux filesystem placement, Docker Desktop integration, explicit script-policy prerequisite, Windows QGC paths, and supported recovery. Label tested versus pending rows. Preserve the Comp2026 source map and development loop; link rather than duplicate mission logic. Update architecture/handoff and LEDGER. Commit: `Integrate portable host setup into the simulation console`.

### Task 9: Verify assembled behavior and publish platform evidence

**Files:**

- Create: `docs/verification/2026-10-10-portable-local-console.md`.
- Modify: `README.md`, `docs/runbook.md`, `docs/handoff.md`, `docs/LEDGER.md`, this plan's completion boxes. Fix code only with a reproducing test and the relevant task's review/checks.

**Interfaces:**

- Consumes the complete operator workflow and existing evidence inspectors, including `scripts/inspect_qgc_control_run.py` and `scripts/inspect_competition_run.py`.
- Produces a per-platform evidence table containing OS/CPU, Docker/provider/WSL/PowerShell/QGC versions, source revision, build-check path, native run IDs, commands/results, and outstanding gaps. Link retained evidence; do not append new data to already finalized run bundles.

- [ ] **Run complete supported software regressions:** `scripts/test-linux.sh orchestration/tests artifacts/tests companion/tests electromagnet/tests gazebo/tests ardupilot_sitl/tests scorekeeper/tests tests -q`; `uv run --locked mypy`; and `uv run --with playwright python tests/browser/check_console_views.py`. Preserve explicit capability skips and resolve new failures before flight.
- [ ] **Run imported Comp2026 tests in their own process:** `PYTHONPATH=companion/comp2026/src:companion/comp2026/tests uv run --locked pytest companion/comp2026/tests/test_drone_control_connection.py companion/comp2026/tests/test_mission_supervisor.py companion/comp2026/tests/test_auto_attempt.py companion/comp2026/tests/test_qgc_flight_path.py companion/comp2026/tests/test_flight_safety.py companion/comp2026/tests/test_listener_live.py companion/comp2026/tests/test_mission_phase_results.py companion/comp2026/tests/test_precision_mission.py companion/comp2026/tests/test_remaining_development_tools.py companion/comp2026/tests/test_flight_state.py -q`. Keep its module namespace separate from the top-level companion tests.
- [ ] **Run native Windows/WSL gates** from Tasks 3–5 on recorded versions, including NAT and mirrored networking, real filesystem faults, competing owners, export/import/rename identity checks, fixed-peer traffic, helper crash/reconnect, and policy/interop failure. If a machine is unavailable, keep that gate pending and record the limitation; do not infer it from Mac or mocked results.
- [ ] **Build/check the final source on native ARM64 and AMD64** with the supported script. Record the exact revision and all eight images. Source commits after this check require a new matching build before another run; preserve earlier records as historical evidence.
- [ ] **Exercise every available support-matrix row with genuine QGC.** Prepare a configured diagnostic, manually select the displayed file, Start in QGC, observe Gazebo/camera, verify physical touchdown/disarm independently, Stop, and Run Again with fresh identity. Verify stale action rejection, Abort/recovery, native takeover, browser closure, and blocked cleanup behavior. Record native QGC evidence before artifact inventory; do not impersonate a QGC click from scripts.
- [ ] **Exercise Comp2026 as well as diagnostics** on each row before claiming that row verified. Use the existing runbook inspectors with exact source/image expectations. Retain independent command admission, physical outcome, score, and artifact validity results. Carry forward the earlier QGC plan's still-open native landing/recovery/precision-loss/takeover gates; a portable console or configured flight does not close those automatically.
- [ ] **Review and record the release boundary.** Confirm no unintended mission changes, preference edits, remaining legacy publisher/lease callers, provenance weakening, exposed listeners, or unrelated environment logging. Update the evidence table and current support claims to the results actually obtained. Commit: `Record portable console verification and remaining platform gaps`. Do not push, merge, or declare untested rows complete.

## Plan self-review

Coverage: platform/configuration and paths map to Tasks 1–4; state/publication/ownership to Tasks 2–5; byte forwarding/finalization to Tasks 4/6; images/provenance to Task 7; console/views/lifecycle/evidence to Task 8; real native acceptance and documentation to Task 9. All five review-focus cases have named assertions in their owning task.

The shared types are defined once in Tasks 1/2 and consumed by later tasks. Publication and lease migrations remove their old APIs in the owning task. Existing mission, artifact, and cloud-suite interfaces remain the authority for their behavior. Native API choices, unit results, image builds, and actual QGC flights are separate proof gates. Unavailable platforms remain explicit unfinished acceptance work.
