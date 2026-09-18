# Work handoff

Updated: 2026-09-18

## Current state

`main` is at `deac918` and does not contain the QGC integration. Keep it that
way until the user explicitly approves a merge.

The pushed branch `integration/qgc-full-mission` is at `1133716`. Its root
`future_work.md` is the source of truth for the remaining QGC work. Read that
file before changing code. The feature branch includes reviewed Tasks 1 through
4 and an explicitly incomplete Task 5 checkpoint.

The primary checkout is not safe for switching to `main`: it contains an
independent, untracked `companion/comp2026` checkout that would collide with
the monorepo files. Use the existing feature worktree for QGC work. Do not move,
delete, or absorb that independent checkout.

## Next work

1. Open the `integration/qgc-full-mission` worktree and read `future_work.md`.
2. Finish Task 5 tests, documentation, implementation report, and independent
   task review. Treat commit `1133716` as a checkpoint, not approved work.
3. Build images from the reviewed revision. Record source and image provenance.
4. Run the full 600-second simulation and verify physical delivery, score,
   artifact validity, and both recordings separately.
5. Run a whole-branch review. Merge only after explicit user approval.

Previous full simulations took about 80 minutes of wall time. Monitor the run
instead of treating elapsed time alone as a stall. Acceptance still requires
all payloads, original-home landing and disarm, `150/150`, terminal
`COMPLETED`, valid artifacts, and accessible onboard and overview recordings.

## References

- Feature handoff: `future_work.md` on `integration/qgc-full-mission`
- Local spec: primary checkout `.git/qgc-integration-issue.md`
- Local plan: primary checkout `.git/qgc-integration-plan.md`
- Task ledger and reports: feature worktree
  `.superpowers/sdd/qgc-integration-plan/`
- Current operator and architecture docs: `README.md`, `docs/runbook.md`,
  `docs/architecture.md`, and `docs/handoff.md`

The spec, plan, and execution reports intentionally remain local. Do not
recreate or publish them without a user request.

## Suggested skills

- `superpowers:using-superpowers`
- `superpowers:subagent-driven-development`
- `superpowers:test-driven-development`
- `superpowers:verification-before-completion`
- `superpowers:requesting-code-review`
- `superpowers:finishing-a-development-branch`
- `diagnosing-bugs` if the full simulation fails
