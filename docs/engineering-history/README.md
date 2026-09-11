# Engineering history

This is a one-time reconstruction of the engineering work performed in
`drone_sim` between 2026-08-22 and 2026-09-05. It follows the project as the
team defined ownership boundaries, built a runnable simulation, made its
evidence trustworthy, integrated flight behavior, and then chased correctness
and performance faults exposed by full runs.

This history is explanatory, not authoritative. For current behavior, read the
[architecture](../architecture.md), [runbook](../runbook.md),
[handoff](../handoff.md), module READMEs, code, configuration, and tests. When a
historical account disagrees with one of those sources, the current source wins.

## How to read it

- Maintainers should start with the current documents above, then use the
  [chronology](chronology.md) to recover why a boundary, invariant, or awkward
  implementation detail exists. Stable `EH-` identifiers make individual
  events easy to cite in an issue or review.
- Interviewers and other outside evaluators should start with the
  [case studies](case-studies.md). Each one states the constraint, failed
  approaches, technical decision, and measured result, then links back to the
  detailed ledger.
- Readers auditing a specific claim can follow its chronology entry to commits,
  repository paths, tests, or recorded-run evidence.

## Strong technical stories

The case studies collect the problems that travel well outside drone
simulation:

- [Publishing validated artifacts without race windows](case-studies.md#publishing-validated-artifacts-without-race-windows).
  Descriptor-relative reads, anonymous temporary inodes, and atomic hard links
  closed mutation and overwrite races while retaining partial evidence.
- [Keeping simulation time exact and the public epoch stable](case-studies.md#keeping-simulation-time-exact-and-the-public-epoch-stable).
  Integer nanoseconds, private warmup, and a rebased public epoch made the same
  timing contract hold under normal and constrained execution.
- [Joining asynchronous sensor streams with bounded memory](case-studies.md#joining-asynchronous-sensor-streams-with-bounded-memory).
  Timestamp-keyed bounded buffers kept work off hot callbacks without hiding
  missing samples or allowing memory to grow indefinitely.
- [Preserving lockstep actuator order under host contention](case-studies.md#preserving-lockstep-actuator-order-under-host-contention).
  Sequence-aware buffering and bounded recovery preserved actuator frames when
  a slow host delayed callbacks.
- [Separating transport, mission, and evidence readiness](case-studies.md#separating-transport-mission-and-evidence-readiness).
  Independent gates stopped a connected process, a ready aircraft, and ready
  recorders from being mistaken for the same state.
- [Serializing payload commands against timestamp-coherent state](case-studies.md#serializing-payload-commands-against-timestamp-coherent-state).
  One command authority and exact state joins prevented overlapping payload
  operations from corrupting mission state.
- [Scoring physical truth independently of mission claims](case-studies.md#scoring-physical-truth-independently-of-mission-claims).
  A passive scorer used Gazebo truth and an explicit phase machine, so mission
  self-reports could not award points.
- [Moving blocking control work off callback paths](case-studies.md#moving-blocking-control-work-off-callback-paths).
  Serialized workers kept service calls and flight commands from starving ROS
  subscriptions.
- [Treating QoS and queue depth as correctness](case-studies.md#treating-qos-and-queue-depth-as-correctness).
  Delivery acknowledgements, larger evidence queues, and separate callback
  groups fixed startup and timestamp loss that unit logic alone could not see.
- [Profiling past aggregate CPU and reducing scheduler pressure](case-studies.md#profiling-past-aggregate-cpu-and-reducing-scheduler-pressure).
  Targeted reductions cut callback and logging load, while full-run profiles
  showed that scheduler and rendering costs still limited real-time factor.

The [full chronology](chronology.md) is broader. It records 115 meaningful
events, including requirements, designs, implementation changes, wrong turns,
review findings, experiments, verification, reversals, and remaining debt.
Routine command output and repeated monitoring are omitted because they did not
change the project's state or understanding.

## Evidence states

Every chronology entry carries one of four evidence states:

- **Verified** means a commit, test, inspection, or recorded run supports the
  claim.
- **Inferred** marks a useful conclusion without direct public proof.
- **Disputed** preserves evidence that remained in conflict.
- **Superseded** records an approach or interpretation that later evidence
  replaced.

A verified historical result proves only what that evidence covered at the
time. It is not a fresh test of the current checkout. The ledger also keeps
physical mission outcome, score, and artifact validity separate. Success in one
does not imply success in the others.

## Scope and limits

The reconstruction uses project conversations, Git history, current files, and
the repository's retained verification records. Conversations can contain
abandoned hypotheses and incomplete observations, so public claims were checked
against repository evidence where possible. Contradictions remain visible as
disputed or superseded entries instead of being cleaned up after the fact.

The scope is `drone_sim` only. Work owned by `vast_drone` and the independent
`companion/comp2026` repository is excluded. A narrow external fact appears only
when it is needed to explain a decision or accepted result in this repository.
No provider account details, local transcript identifiers, credentials, or
private machine paths appear here.

Some cited paths existed only at the referenced commit and were later removed
during documentation consolidation. Use the commit as the stable evidence in
those cases. Generated run bundles remain local unless the repository cites a
sanitized retained artifact.
