# Engineering History Ledger Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce a one-time, public-safe, semantically exhaustive engineering history of `drone_sim` from 2026-08-22 through 2026-09-05.

**Architecture:** Normalize the completed date-sliced transcript archaeology into one chronological source of truth, corroborate it against Git and current documentation, then derive a concise evaluator index and technical case studies. Keep local transcript coordinates in an ignored evidence map and expose only public repository evidence in tracked files.

**Tech Stack:** Markdown, Git, Bash, `rg`, `jq`, existing Codex JSONL sessions

**Spec:** `docs/superpowers/specs/2026-09-06-engineering-history-ledger-design.md`

## Global Constraints

- Include work performed in `drone_sim` from 2026-08-22 through 2026-09-05.
- Exclude work owned by `vast_drone`, `companion/comp2026`, and the transferred Comp2026 repository.
- Include cross-repository facts only when they directly explain a `drone_sim` decision or result.
- Preserve every meaningful requirement, decision, failure, correction, verification result, and unresolved debt.
- Exclude routine tool output, repeated monitoring, and mechanical command chatter.
- Keep tracked files free of absolute home paths, transcript session IDs, account details, provider balances, credentials, and secret values.
- Distinguish physical mission outcome, score, and artifact validity.
- Do not change runtime code or current behavioral contracts.
- Preserve unrelated working-tree changes.

---

### Task 1: Build the private evidence inventory

**Files:**
- Modify: `.gitignore`
- Create, ignored: `.ledger-private/evidence.jsonl`
- Create, ignored: `.ledger-private/reports/`

**Interfaces:**
- Consumes: Codex session JSONL under `/home/willis/.codex/sessions`, seven completed date-sliced archaeology reports, parent Git history
- Produces: one local JSON object per candidate event with `candidate_id`, `date`, `repository`, `summary`, `session_file`, `timestamp`, `ordinal`, `commit_refs`, `path_refs`, `evidence_state`, and `duplicate_of`

- [ ] **Step 1: Add the private directory to Git ignore rules**

Add this exact root-relative rule to `.gitignore`:

```gitignore
/.ledger-private/
```

- [ ] **Step 2: Preserve the seven extraction reports locally**

Write each completed report into `.ledger-private/reports/` using its date-range task name. Do not copy report text into tracked files.

- [ ] **Step 3: Normalize candidate events**

Create `.ledger-private/evidence.jsonl`. Use one JSON object per event with the exact fields named in the Interfaces block. Assign temporary IDs `C-0001`, `C-0002`, and so on in chronological order.

- [ ] **Step 4: Apply scope and duplicate rules**

Mark events from excluded repositories with their repository name and omit them from the public set. Mark overlap from long-running sessions through `duplicate_of`. Do not delete contradictory claims; give them separate candidates with `disputed` or `superseded` evidence state.

- [ ] **Step 5: Verify the private inventory**

Run:

```bash
test -s .ledger-private/evidence.jsonl
jq -e -s 'all(.[]; has("candidate_id") and has("date") and has("repository") and has("summary") and has("session_file") and has("timestamp") and has("ordinal") and has("commit_refs") and has("path_refs") and has("evidence_state") and has("duplicate_of"))' .ledger-private/evidence.jsonl
git check-ignore .ledger-private/evidence.jsonl
```

Expected: the file is nonempty, `jq` prints `true`, and Git reports the evidence path as ignored.

- [ ] **Step 6: Commit the ignore rule**

```bash
git add .gitignore
git commit -m "Ignore private ledger evidence"
```

### Task 2: Write the exhaustive chronology

**Files:**
- Create: `docs/engineering-history/chronology.md`

**Interfaces:**
- Consumes: non-duplicate, in-scope candidates from `.ledger-private/evidence.jsonl`; `git log`; existing verification and handoff documents
- Produces: stable public entries `EH-0001` onward, in chronological order, each with evidence state and public evidence

- [ ] **Step 1: Establish chronology sections**

Use date headings and group tightly related iterations beneath a shared problem heading. Begin with the clean repository architecture on 2026-08-22 and end on 2026-09-05.

- [ ] **Step 2: Convert every public candidate into a ledger entry**

Each entry must contain a stable `EH-####` ID, date, title, problem or constraint, meaningful investigation and failed attempts, decision or result, verification, later correction or debt, and public evidence. Combine candidates only when they describe one continuous investigation and no meaningful reversal disappears.

- [ ] **Step 3: Corroborate public references**

For every cited commit, run `git cat-file -e <commit>^{commit}`. For every current path, run `test -e <path>`. Cite deleted or renamed paths through the commit that contained them rather than claiming they remain current.

- [ ] **Step 4: Check semantic coverage**

Compare the public event IDs against every non-duplicate, in-scope candidate. Add an ignored cross-reference field `ledger_id` to each included JSONL object. No included candidate may lack a ledger ID.

- [ ] **Step 5: Run focused chronology checks**

```bash
rg -n '^### EH-[0-9]{4} ' docs/engineering-history/chronology.md
rg -n 'verified|inferred|disputed|superseded' docs/engineering-history/chronology.md
git diff --check -- docs/engineering-history/chronology.md
```

Expected: stable entries are present, evidence states appear, and the Markdown diff has no whitespace errors.

- [ ] **Step 6: Commit the chronology**

```bash
git add docs/engineering-history/chronology.md
git commit -m "Document drone simulation engineering chronology"
```

### Task 3: Write case studies and the evaluator index

**Files:**
- Create: `docs/engineering-history/case-studies.md`
- Create: `docs/engineering-history/README.md`
- Modify: `README.md`

**Interfaces:**
- Consumes: stable `EH-####` entries from `chronology.md`
- Produces: technical narratives linked to chronology IDs, plus a short reading guide for maintainers and evaluators

- [ ] **Step 1: Select evidence-rich case studies**

Choose only topics with a concrete constraint, a non-obvious diagnosis or representation, a meaningful wrong turn, and a measured or otherwise verified result. Cover time, buffering or causal joins, concurrency or QoS, filesystem durability, evidence integrity, control or numeric precision, and performance when the chronology supports them.

- [ ] **Step 2: Write each technical narrative**

Use the same shape for every case study: problem, why the obvious approach failed, evidence, final design, result, and reusable lesson. Link each claim to relevant `EH-####` entries, commits, and current or historical files. Keep historical claims in past tense.

- [ ] **Step 3: Write the history entry point**

Explain the project arc in plain language. Provide separate reading paths for maintainers and interview evaluators. Include a compact index naming the problem, technical idea, result, and case-study anchor.

- [ ] **Step 4: Link from the root README**

Add one link in the existing “How to read this repository” section. Do not duplicate the case-study index or current run instructions.

- [ ] **Step 5: Verify internal links and stable IDs**

Check that every case-study `EH-####` reference exists in `chronology.md`, every case-study anchor exists, and all new relative Markdown links resolve.

- [ ] **Step 6: Commit the public reading layer**

```bash
git add README.md docs/engineering-history/README.md docs/engineering-history/case-studies.md
git commit -m "Add engineering history case studies"
```

### Task 4: Audit completeness, public safety, and documentation impact

**Files:**
- Modify if required: `docs/engineering-history/README.md`
- Modify if required: `docs/engineering-history/chronology.md`
- Modify if required: `docs/engineering-history/case-studies.md`
- Delete after completion: `docs/superpowers/specs/2026-09-06-engineering-history-ledger-design.md`
- Delete after completion: `docs/superpowers/plans/2026-09-06-engineering-history-ledger.md`

**Interfaces:**
- Consumes: all public history files and the ignored evidence cross-reference
- Produces: a public-safe, internally consistent history with design and plan retained in Git history only

- [ ] **Step 1: Run the completeness audit**

Confirm every in-scope date has at least one entry or a recorded reason for no entry. Confirm every included private candidate maps to exactly one public ledger ID. Review excluded candidates to ensure no `drone_sim` implementation event was dropped with cross-repository work.

- [ ] **Step 2: Run the public-safety scan**

```bash
! rg -n '/home/|\.codex/sessions|rollout-[0-9]|session[_ -]?id|VAST.*balance|API[_ -]?KEY|BEGIN .*PRIVATE KEY' README.md docs/engineering-history
```

Expected: no matches.

- [ ] **Step 3: Validate commit and file references**

Extract commit-like hexadecimal references and verify each intended Git commit. Check current path links and historical links separately so deleted files are not presented as current.

- [ ] **Step 4: Validate Markdown links and entry references**

Run a local link checker over `README.md` and `docs/engineering-history/*.md`. Confirm every case-study entry reference exists once in the chronology.

- [ ] **Step 5: Check current-document agreement**

Compare current-behavior summaries with `docs/architecture.md`, `docs/runbook.md`, `docs/handoff.md`, affected tests, and implementation. Correct historical prose rather than changing current contracts.

- [ ] **Step 6: Remove superseded planning documents**

Delete the design and plan files after all checks pass. Their earlier commits preserve them in Git history, as required by the repository documentation policy.

- [ ] **Step 7: Run final focused verification**

```bash
git diff --check
git status --short
```

Report documentation impact explicitly. No simulator build, unit suite, image rebuild, or flight is required because runtime behavior does not change.

- [ ] **Step 8: Commit the audit corrections and plan cleanup**

```bash
git add README.md .gitignore docs/engineering-history docs/superpowers/specs/2026-09-06-engineering-history-ledger-design.md docs/superpowers/plans/2026-09-06-engineering-history-ledger.md
git commit -m "Finalize public engineering history"
```
