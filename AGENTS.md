# Contributing: agents and humans

[README.md](README.md) is the project's single human entry point. This file
defines how to change the project; it is not another system introduction.

## Before changing a module

1. Read that module's README: [orchestration](orchestration/README.md),
   [artifacts](artifacts/README.md), [companion](companion/README.md),
   [ArduPilot SITL](ardupilot_sitl/README.md), [Gazebo](gazebo/README.md),
   [electromagnet](electromagnet/README.md), or [scorekeeper](scorekeeper/README.md).
2. For changes to ownership, communication, timing, or physical/scoring
   guarantees, read [architecture](docs/architecture.md) and inspect the actual
   interface definition, producer, consumers, and relevant tests.
3. For setup, builds, operator commands, or diagnostic procedures, read
   [the runbook](docs/runbook.md). Consult [handoff](docs/handoff.md) for dated
   verification and known gaps, not as proof of today's behavior.

## Keep specifications and documentation current

- Define intended behavior and success/failure conditions before implementation.
  Keep a mission's local contract beside its entry point; document shared
  behavioral requirements in architecture and module-specific constraints in
  the module README.
- In the **same change**, update every affected module README when its
  responsibilities, interfaces, entry points, testing instructions, or important
  constraints change. Cross-module work requires checking each affected module.
- Update architecture for changed shared boundaries/guarantees and the runbook
  for changed operator procedures. Update handoff when verified status or known
  limitations change, retaining its date and evidence scope.
- Link to executable schemas, configuration, rules, and endpoint code for exact
  fields/settings. Maintain one description of each concept rather than copying
  configuration tables across documents.
- Investigate disagreements among requirements, tests, and implementation.
  Do not weaken a test or rewrite a specification merely to match a bug.
- Update current guides instead of layering new internal/external/plan documents
  into each folder. Once a plan is superseded, preserve it in Git history rather
  than leaving it in the active documentation path.

## Completion gate

A behavior-changing task is not complete until intended behavior, relevant tests,
and affected documentation agree. Before handing off:

1. Run focused checks for the changed behavior and verify edited links/commands
   in proportion to risk. Distinguish passed, failed, skipped, and unrun checks.
2. Identify affected module READMEs and shared guides. Report which were updated,
   or explain why no documentation update was needed; avoid cosmetic edits made
   only to satisfy this gate.
3. Report physical mission outcome, score, and artifact validity separately when
   evaluating a flight. Historical evidence is not a current end-to-end pass.

## Scope and preservation

Keep patches small and reversible. Preserve unrelated work, the independent
`companion/comp2026` repository, run evidence, and licenses/provenance. Check Git
status before editing or cleanup; explicitly back up local-only deletion targets.
Source edits do not rebuild images. Rebuild and record matching provenance when
testing changed runtime code; do not alter evidence to fit a new checkout.

These instructions require a documentation-impact check; they do not automatically
prove that prose is correct. Use tests for executable contracts and judgment for
whether the documentation accurately explains the intended system.
