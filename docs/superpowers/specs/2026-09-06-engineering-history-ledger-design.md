# Engineering history ledger design

Date: 2026-09-06

## Purpose

Create a one-time, public-safe history of the engineering work performed in the
`drone_sim` repository. The history must preserve every meaningful technical
event while remaining useful to future maintainers and outside evaluators.

The ledger will record requirements, design decisions, implementation changes,
failed hypotheses, experiments, bugs, fixes, verification, reversals, and known
debt. It will omit routine command output, repeated monitoring, and other work
that did not change understanding or project state.

## Scope

The history begins with creation of the clean `drone_sim` repository on
2026-08-22 and ends with the last relevant Codex transcript on 2026-09-05.

Work performed in `vast_drone`, `companion/comp2026`, and the transferred
Comp2026 repository is excluded. A cross-repository fact may appear only when it
directly explains a decision or result inside `drone_sim`. Such entries must
identify the external system as context and must not turn into a history of that
system.

## Public document structure

The public history will use three files:

- `docs/engineering-history/README.md` provides the project arc, scope, reading
  paths, and an interview-oriented index of the strongest engineering stories.
- `docs/engineering-history/chronology.md` contains the semantically exhaustive
  event ledger in chronological order.
- `docs/engineering-history/case-studies.md` explains selected technical
  problems in depth, including wrong turns, evidence, the final solution, and
  measured results.

The root `README.md` will link to the history. Existing architecture, runbook,
handoff, verification, and technical-debt documents remain authoritative for
current behavior. The history will link to them instead of copying current
operating instructions.

## Entry model

Each chronological event has a stable identifier such as `EH-0042` and records:

1. Date and title.
2. Problem, requirement, or constraint.
3. Observation and investigation.
4. Attempted approaches, including meaningful failures.
5. Decision, fix, or resulting understanding.
6. Verification and measured outcome.
7. Later correction or remaining debt.
8. Public evidence through commit hashes, repository paths, tests, or run IDs.

Claims use one of four evidence states: verified, inferred, disputed, or
superseded. A later correction must point back to the earlier entry rather than
silently rewriting the history.

## Evidence processing

The source corpus contains 208 project-related Codex sessions, about 1.1 GB.
Seven read-only passes have already divided that corpus by date and extracted
candidate events with local transcript coordinates.

Production follows this sequence:

1. Normalize the seven reports into candidate events.
2. Remove duplicates caused by long-running sessions crossing date boundaries.
3. Exclude events owned by out-of-scope repositories.
4. Check claims against `drone_sim` Git history, current files, and existing
   verification documents.
5. Assign stable event IDs and evidence states.
6. Write the public chronology and derive the overview and case studies from it.
7. Audit the final documents for omissions, contradictions, broken links, and
   private information.

A local `.ledger-private/evidence.jsonl` file will map event IDs to transcript
file names, timestamps, and ordinals during the audit. The repository will
ignore this directory. Public files must not contain absolute home paths,
session identifiers, account details, provider balances, credentials, or secret
values.

## Case-study selection

Case studies should have a concrete constraint, an evidence-backed diagnosis,
at least one non-obvious technical decision, and a result that can be explained
outside drone simulation. Expected subjects include:

- exact simulation time and public epoch design;
- bounded buffering and causal timestamp joins;
- ROS 2 QoS, callback ordering, and executor isolation;
- lockstep startup and distributed readiness;
- durable no-clobber artifact publication;
- strict evidence, scoring, and provenance boundaries;
- float precision in geographic command transport;
- physical flight tuning guided by logged evidence;
- performance diagnosis across CPU scheduling, rendering, and data movement.

The final set may merge related topics when one narrative explains them better.
It must not inflate minor fixes into case studies.

## Failure handling

- Conflicting transcript claims stay visible and receive `disputed` or
  `superseded` status until repository evidence resolves them.
- Claims without public corroboration may remain only when technically important;
  they must be labeled `inferred` and avoid confidential details.
- Missing commits or deleted paths are cited through stable commit hashes when
  possible. The ledger will not restore superseded source or documentation.
- If public safety is uncertain, omit the sensitive detail while preserving the
  engineering lesson.
- The ledger will distinguish physical mission outcome, score, and artifact
  validity. One must never stand in for another.

## Verification

Focused verification will check:

- every in-scope transcript date range is represented or explicitly empty;
- every normalized event appears once in the chronology;
- case-study claims trace to chronology entries;
- cited commits and repository paths resolve when available;
- Markdown links resolve;
- no absolute home path, transcript session ID, provider account detail, or
  secret-like value appears in tracked files;
- current-behavior claims agree with the architecture, runbook, handoff, tests,
  and implementation;
- `git diff --check` passes.

No simulator build or flight is required because this task changes historical
documentation only. Existing test and run results will be reported as historical
evidence, not rerun or promoted into current verification.

## Documentation impact

The root README will gain one link. No module responsibility, runtime interface,
operator procedure, timing rule, or scoring guarantee changes, so module READMEs,
architecture, and runbook need no behavioral edits. The handoff should change
only if the finished history materially changes its documentation map.
