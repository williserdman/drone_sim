# Documentation cleanup and contribution rules

## Decision

Keep one human entry point: `README.md`. Retain three supporting guides and add
a short root `AGENTS.md` that humans can also read as contribution instructions.
Delete duplicated interface documents and obsolete plans/reports after preserving
their current contents. This replaces the recursive three-document-per-module
convention; it does not redesign runtime communication.

The user approved one entry point, deletion of obsolete plans and agent reports,
and an `AGENTS.md` requiring relevant specs/docs/tests to stay aligned with changes.
This written specification is the review checkpoint before implementation.

## Retained documentation

| File | Responsibility |
| --- | --- |
| `README.md` | Start here, first checks, and links to the guides and contribution rules |
| `AGENTS.md` | How to make changes and assess documentation/test impact |
| `docs/architecture.md` | Ownership, communication, durable behavioral requirements, and code/test links |
| `docs/runbook.md` | Setup, builds, operation, testing, diagnosis, and any needed asset-maintenance procedure |
| `docs/handoff.md` | Dated evidence, current limitations, and prioritized unfinished work |

Keep issue and verification records containing distinct evidence. They remain
dated records, not competing current specifications. Preserve all licenses,
provenance manifests, schemas, configuration used by runtime, and source/tests.

The architecture guide incorporates the communication contract. Do not add a
separate communication guide or replacement interface stubs in every module.
Exact fields belong in ROS `.msg`/`.srv` definitions; deployed recording QoS
belongs in `artifacts/recording-qos.yaml`; endpoint settings belong beside their
publishers/subscribers. Guides link to these rather than copying exhaustive
inventories. Tests express intended requirements, not permission to bless a bug.

## Contribution rules

Keep `AGENTS.md` short and task-oriented, with conditional pointers:

1. Start at README; read architecture before changing ownership, communication,
   timing, or physical/scoring guarantees. Read the runbook before changing
   setup, builds, operator commands, or debugging procedures.
2. Before changing an interface, inspect its definition, producer, consumers,
   and relevant tests. Preserve the distinction between physical truth, mission
   intent, score, and artifact validity.
3. A behavior-changing task is complete only when the affected documentation
   and tests agree with the intended behavior. Update them in the same change,
   or state why no documentation update is needed in the handoff.
4. Investigate disagreements among specs, tests, and implementation. Do not
   weaken a requirement or test merely to match an unexplained failure.
5. Update existing current guides; keep exact values in their executable source
   when practical. Use Git history for superseded plans instead of adding layers
   of obsolete instructions to the working tree.
6. Record verification commands and outcomes accurately. Distinguish skipped
   checks, known failures, and historical evidence from a current verified pass.
7. Preserve unrelated changes, nested repositories, run evidence, and provenance.
   Keep implementation minimal; scope deletions explicitly and preserve local-only
   material before removal.

These are contribution instructions, not a mechanical guarantee of documentation
freshness. Existing tests enforce concrete contracts; semantic review remains
necessary. No documentation generator, topic registry, or new CI framework is
introduced. Existing global user instructions continue to apply.

## Deletion scope

Use an explicit inventory from the main checkout, not recursive wildcards across
worktrees or nested repositories. The reviewed candidates are:

- 28 tracked `EXTERNAL_INTERFACE.md` / `INTERNAL_INTERFACE.md` files at the root,
  in the seven top-level modules, and in Gazebo subpackages/provenance.
- Eight tracked root/module `PLAN.md` files.
- The stale `config/recording-qos.yaml` duplicate; retain the deployed file under
  `artifacts/` unchanged.
- The untracked root `SYSTEM_DIAGRAM.md` and historical
  `docs/IMPLEMENTATION_ROADMAP.md`.
- The 11 pre-existing historical files under `docs/superpowers/` and obsolete
  report/brief material under `.superpowers/` (13 tracked, 41 local ignored files
  in the earlier audit). Re-inventory before deletion; counts may change.

This active design and its forthcoming execution plan are not obsolete while
the cleanup is in progress. Once complete, retain the approved record in Git
history and remove those temporary planning files from the final working tree.

Before deleting the Gazebo provenance interface pair, preserve its unique asset
audit/refresh procedure in the runbook next to links to the manifest, original
assets, and license. Do not remove the provenance data or license files.

Preserve issue notes, `docs/verification/`, and the existing technical-debt ledger.
When retained historical notes cite a deleted plan, replace the obsolete live
path with its exact historical Git revision/path; preserve their recorded facts.
Update the handoff so the old ledger is clearly historical, not the active queue.

## Recovery and concurrent work

At review, the parent is on `main` at `5e0c49d`, with uncommitted documentation
and ArduPilot parameter/test changes. Several deletion candidates have never-
committed edits. `SYSTEM_DIAGRAM.md` and ignored reports cannot be recovered from
Git HEAD. Do not claim Git history protects them without first preserving them.

Before removal, archive the exact deletion candidates outside the repository in
a private, persistent local directory; verify the file inventory and checksums,
and report the recovery path. Include modified, untracked, and ignored candidates
without publishing their contents or committing potentially sensitive reports.
Use Git history for tracked baseline versions. Recheck for concurrent changes
before applying deletion patches. Preserve any unexpected new material.

Do not alter `companion/comp2026`, `.worktrees`, `runs`, flight parameters, scorer
behavior, runtime Python/C++ code, images, or host configuration. Refresh current
guides against the actual checkout; do not overwrite concurrent networking work.

## Necessary test repair

`tests/integration/test_phase2_runtime_contract.py` currently reads the stale QoS
file and already fails because it expects raw image topics in the bag. Repair
that test to inspect the deployed recorder configuration and metadata-only bag
inventory, while retaining its reliable live video-transport assertions. Do not
delete the test or change runtime recording behavior to satisfy obsolete checks.

## Acceptance

- README is the single entry point, and links to the three guides and AGENTS.
- AGENTS contains the conditional reading and completion rules above, without
  duplicating the system specification or promising automatic enforcement.
- Unique communication, timing, failure/finalization, and provenance semantics
  from deleted documents are preserved in the appropriate retained guide.
- All approved obsolete candidates are removed; no replacement stub hierarchy
  or in-tree archive is created. Recovery inventory/checksums are verified.
- Retained links and historical references resolve to current files or explicit
  recoverable Git objects. The architecture QoS link points to the deployed file.
- The repaired recording-contract test and relevant artifact/schema tests pass.
  Existing unrelated failures, including the roll-gain expectation mismatch,
  remain explicitly reported rather than being silently fixed in this task.
- CLI help, current template resolution, Compose rendering, and `git diff --check`
  are checked. No new flight/build is required for this behavior-neutral cleanup.
- Final diff review proves runtime behavior, licenses/provenance, nested source,
  run evidence, and unrelated work were preserved.
