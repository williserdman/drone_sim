# Code simplification and hardening design

## Objective

Reduce the amount of code and the number of independent implementations a
maintainer must understand, while making failure behavior more predictable. The
cleanup covers all seven runtime modules. It must preserve flight behavior,
simulation timing, QoS, scoring, artifact formats, operator commands, and the
ordering of startup and finalization unless separate evidence proves a bug.

The work is complete only after every module has been audited, every
high-confidence simplification or hardening change has been implemented, and
the remaining candidates have a concrete reason to stay.

## Chosen approach

Use incremental consolidation. Make one behavior-preserving change at a time,
run focused tests, then continue. This keeps failures attributable and each
change reversible.

A broad rewrite was rejected because this system has long-running integration
paths and existing runtime faults. Reworking the whole package layout at once
would make regressions difficult to distinguish. A dead-code-only pass was also
rejected because it would leave duplicated safety-critical protocol code.

## Design rules

- Preserve observable runtime behavior unless a separately diagnosed bug has an
  approved fix.
- Keep fail-closed validation even when deleting it would shorten the code.
- Extract a module only when it gives callers a smaller interface, owns distinct
  invariants, and can be tested through that interface.
- Delete duplicate implementations and pass-through wrappers before adding new
  abstraction.
- Keep host-only concerns out of runtime containers.
- Preserve generated code, vendored code, asset provenance, licenses, run
  evidence, the nested mission checkout, and unrelated working-tree changes.
- Update every affected module README and shared guide in the same change.

## First change: durable protocol files

`artifacts.runtime_protocol` and `orchestration.status_store` independently
implement descriptor-relative JSON reads and atomic writes. The implementations
already reject symlinks, oversized documents, duplicate keys, non-finite JSON,
and files that change while being read. Maintaining two copies risks subtle
drift in these security and durability checks.

Add one deep module under `artifacts`, which orchestration already depends on.
It will own:

- canonical JSON encoding;
- strict JSON-object decoding;
- descriptor-relative reads with no symlink traversal;
- bounded reads and before/opened/after identity checks;
- atomic temporary-file writes, file and directory syncing, and cleanup;
- three explicit persistence policies: identical retry allowed, first writer
  wins, and atomic replacement.

The interface will accept a deadline callback for bounded host reads and a file
mode for the host/runtime ownership difference. It will raise one low-level
protocol I/O error. Each caller will translate that error and retain its own
domain validation, public exception type, naming rules, and policy decisions.

This should remove about 150 lines of duplicated sensitive code without merging
the host status model with the runtime protocol model.

## Module audit sequence

After the protocol change, inspect the largest implementations in risk order:

1. Orchestration lifecycle, terminal-cause selection, and finalization.
2. Artifact recording, subprocess management, and validation.
3. Companion runtime composition and nested-mission adapter.
4. Gazebo process supervision and public-data adapter.
5. Scorekeeper competition evidence state machine.
6. Electromagnet payload coordination.
7. ArduPilot process wrapper.

File size alone does not justify extraction. A candidate proceeds only if its
new interface hides meaningful policy and production callers can use the same
interface that tests exercise. Likely candidates include subprocess lifetime,
deadline accounting, terminal-cause selection, and time-grid validation. The
audit will decide the exact changes from current code and tests.

## Waste removal

For each module, search for unused functions, classes, configuration, legacy
branches, repeated constants, repeated validators, and test-only hooks in
production code. Confirm each candidate against imports, entry points, Compose,
Dockerfiles, runtime configuration, plugins, and the nested mission integration.
Delete it only when those sources and focused tests show that no supported path
depends on it.

Do not edit generated interfaces or vendored sources to make repository metrics
look better. Do not remove apparently defensive code until its invariant is
understood and retained elsewhere.

## Failure-path hardening

Every changed module must:

- close descriptors and subprocesses on every exit path;
- keep cleanup within the original deadline rather than starting a new budget;
- preserve the first actionable failure and retain later diagnostics;
- reject malformed, stale, conflicting, or cross-run input;
- define retries as idempotent or invalid;
- avoid publishing or writing after its quiescence point.

Where relevant, tests will cover interrupted writes, oversized input, symlink or
name replacement, concurrent writers, timeout, and partial shutdown.

## Verification

Each slice must pass its focused tests before the next slice starts. Final
verification will run:

- all seven host module test suites;
- ROS schema and repository integration-contract tests;
- base, Phase 2, and GPU Compose resolution;
- documentation link and run-template checks;
- repository-owned static checks;
- a final diff and checksum review for unrelated files and the nested mission
  repository.

Existing failures will be reproduced and reported separately. Source-only tests
do not prove a flight. If a cleanup changes runtime behavior, stop and treat it
as a separate diagnosed change. Do not claim live verification without matching
image builds and an end-to-end mission run.

## Documentation lifetime

This design is the temporary authority for the cleanup. The implementation plan
will reference it. After the cleanup is complete, fold lasting ownership and
operator facts into the module READMEs and central guides, then remove this
design and the implementation plan from the active tree. Git history retains
both.
