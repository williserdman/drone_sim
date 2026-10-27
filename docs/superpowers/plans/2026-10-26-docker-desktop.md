# Docker Desktop migration implementation plan

**Goal:** Run the existing local simulation on Docker Desktop on macOS, removing the Colima/Lima dependency from the current developer workflow.

**Architecture:** Keep the eight Compose services, native QGC, host Python console, native image architecture, loopback ports, workstation ownership, and evidence lifecycle. Select Docker Desktop for Mac and WSL2, and Docker Engine for native Linux. Validate the effective local Docker endpoint and engine rather than requiring a particular context name.

**Spec:** User's Docker Desktop migration request and `docs/LEDGER.md#docker-desktop-migration-2026-10-26`.

**Constraints:** Preserve original checkout, imported mission code, QGC settings/actions, tokens and run evidence. Finish cleanup against the old engine before switching. Do not accept installation terms for the user. Do not issue aircraft commands. Do not claim unavailable platform or flight acceptance.

## Task 1: Provider behavior and regression coverage

- [x] In `orchestration/tests/test_host_config.py`, require Mac ARM64 and AMD64 detection to select `docker-desktop`.
- [x] In `orchestration/tests/test_local_build.py`, cover Docker Desktop via `desktop-linux`, the default socket and a renamed local context. Reject a non-Desktop engine when Desktop is selected, including a stale local Colima endpoint; preserve remote/TLS/platform/provenance rejection and environment precedence. Keep Linux Docker and WSL2 coverage.
- [x] Observe the relevant failures, then update `host_config.py` and `local_build.py`. Verify the effective engine's Docker Desktop identity, independent of context name. Migrate affected fixtures and remove obsolete Colima UI naming from `console_static/console.js`.
- [x] Update `orchestration/README.md`. Run affected tests and configured mypy; run the supported Linux suite once Docker Desktop is available.

## Task 2: Machine migration and operator documentation

- [x] Restore the stopped old engine only for exact-run cleanup; preserve receipts, then stop the old console and Colima normally.
- [x] Install Docker Desktop from the official signed application, with package checksum verification. Leave any first-run agreement to the user. Select its context after cleanup.
- [x] Update README, runbook, architecture and handoff with Desktop setup, resource settings, shared-folder and context guidance, migration/rebuild requirements, and the fact that Mac Desktop still manages a Linux VM. Preserve dated historical Colima evidence.
- [x] Independently review the change. Commit reviewed source and documentation before the final matching image build/check.
- [ ] Build all eight native images, verify source/image provenance, run relevant browser checks, and check the live console. If first-run setup blocks native verification, record the exact missing step and leave source work complete and reviewable.

## Review focus

- An arbitrary context name must not be mistaken for an engine identity.
- Environment overrides must not silently select a remote or old engine.
- Incomplete cleanup belongs to its original engine, not an empty new engine.
- Old image records must not be relabeled as Desktop builds.
- Native Linux and Windows/WSL2 must keep their existing platform behavior.

Native handoff results are recorded in `.local-runtime/verification/2026-10-26-docker-desktop/final-handoff.json` so the verified source commit does not change while recording its build result.
