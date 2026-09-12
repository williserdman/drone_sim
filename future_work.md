# QGC integration handoff

Updated: 2026-09-12

Continue on branch `integration/qgc-full-mission` in the existing
`qgc-full-integration` worktree. Do not merge to `main` until the remaining
implementation, review, and flight-acceptance gates pass.

## Source of truth

- Local issue/spec: primary checkout `.git/qgc-integration-issue.md`
- Local implementation plan: primary checkout `.git/qgc-integration-plan.md`
- Execution ledger and task reports: `.superpowers/sdd/qgc-integration-plan/`
- Main/QGC reconciliation report: primary checkout
  `.superpowers/merge-qgc-main-report.md`

These local planning artifacts intentionally were not published. Read them
instead of reconstructing requirements from this handoff.

## Completed and reviewed

- `75f8525` reconciles the guarded QGC foundation with current `main` while
  retaining both automatic and QGC-selected modes.
- Task 1 (`01e1a78`) adds the strict full FM1/FM2/FM3 policy and deferred
  precision-camera readiness. Review: approved.
- Task 2 (`d1d9540`, `b75721f`) adds confirmed payload 2/3/4 actuation and
  closes the post-dispatch uncertainty race. Review: approved.
- Task 3 (`7ad6c3e`, `392122c`) connects the real onboard camera and fixes
  camera-worker teardown ordering. Review: approved.
- Task 4 (`6e9521e`, `9ea60ef`) implements the exact eleven-event competition
  lifecycle and post-release physical-evidence intervals. Review: approved.

Exact test evidence and review findings are in the ledger and per-task reports.

## Interrupted Task 5 checkpoint

Task 5 was interrupted after implementation work but before its required
self-review, documentation pass, final verification, report, and independent
task review. Treat the checkpoint as unapproved work.

Current work includes:

- an operator-selected absolute attempt-state host root carried through QGC
  template/resolved configuration;
- a bind of only that root's attempt-specific `sha256-...` directory to the
  fixed container attempt-state path;
- a repository full-simulation policy and `scripts/prepare_qgc_run.py`;
- the Copter 4.5.7 roll-acceleration cap of `180000` cdeg/s², required precision
  landing parameters, and a dedicated MAVLink2 `serial1` TCP endpoint on 5762;
- loopback-only publication of TCP 5762 in the QGC Compose overlay.

The last reported focused results were 298 passing
config/wrapper/parser/parameter tests and 113 passing controller tests. They
were run before final documentation and integration verification and are not a
Task 5 completion claim.

Resume Task 5 from `task-5-brief.md`. First inspect the checkpoint diff and
write or recover the missing implementer report. Verify that external QGC can
connect to `tcp://127.0.0.1:5762`, while companion remains on private serial0
TCP 5760 and non-QGC runs do not publish 5762. Finish the README/module/runbook/
handoff updates, run every Task 5 command, self-review, commit any correction,
then dispatch a fresh read-only Task 5 reviewer. Resolve all Critical and
Important findings before Task 6.

## Task 6 and acceptance

After Task 5 approval, build images from the final source and record matching
revision/digest provenance. Run the full 600-second competition simulation.
Previous full simulations have taken roughly 80 minutes of wall time, so
monitor rather than assuming a stall.

Completion requires separate evidence for:

- payload 2 release;
- payload 3 pickup, lift, and release;
- payload 4 pickup, lift, and release;
- original-home landing and disarm;
- score `150/150`;
- terminal `COMPLETED` state and valid semantic artifact bundle;
- directly accessible onboard and overview recordings.

A protocol injector may automate the flight through the same listener, but it
does not prove the QGroundControl desktop application. Record any desktop
connection/action-loading/acknowledgement check separately and label limitations
accurately. Do not modify historical evidence to fit the new run.

After Task 6, run a fresh whole-branch review and verification gate. Keep the
branch separate until the user explicitly authorizes merging to `main`.

## Suggested skills

- `superpowers:using-superpowers`
- `superpowers:subagent-driven-development`
- `superpowers:test-driven-development`
- `superpowers:verification-before-completion`
- `superpowers:requesting-code-review`
- `superpowers:finishing-a-development-branch`
- `diagnosing-bugs` or `superpowers:systematic-debugging` if the live run fails
