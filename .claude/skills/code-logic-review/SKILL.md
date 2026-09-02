---
name: code-logic-review
description: Review just-written or just-edited code for logic correctness against the module's plan, requirement, and history — not style. Checks the implementation actually does what was planned, algorithms/loops are optimal, edge cases are handled, and data flow matches the interfaces doc. Invoke manually after coding, separately from /code-standards-review. Auto-fixes mismatches and re-reviews until logic matches, up to 3 iterations.
---

# Code Logic Review

Style is handled by `/code-standards-review`. This skill only checks: **does the code do the right thing, correctly and efficiently.**

**Where this fits in the flow:** run this SECOND, after `/code-standards-review` has settled the style/structure (a style refactor can move code, so review logic on the final shape). Both are the gate before a task moves to `done` in the task lifecycle protocol.

## Ground truth sources (read these first, in order)

1. `agent/description/project_overview.md` — overall system goals.
2. `agent/description/interfaces.md` — expected data contracts between modules (message formats, function signatures, units, coordinate frames).
3. The active task file in `agent/tasks/YYYY-MM-DD_<slug>.md` for this work — Task/Input/Expected output/Plan sections. **There are many task files; pick the active one by its `status` field (not `done`).** If several are open, or none clearly matches the changed files, stop and ask the user which task this review is for — do not guess.
4. `agent/plan/<module>_plan.md` for the module being reviewed.
5. Most recent relevant file(s) in `agent/history/` — prior decisions that constrain current behavior (e.g. "we chose Open3D pose-graph SLAM, not ICP-only").

If any of these are missing or the task file doesn't exist, say so and ask the user for the requirement instead of guessing intent.

## Checklist

### 1. Plan conformance
- Every step in the task file's Plan section is actually implemented — no silently skipped or partially-done steps.
- The code doesn't implement something the plan/requirement didn't ask for (scope drift).

### 2. Interface conformance
- Function signatures, message formats, units (mm vs m, deg vs rad), and coordinate frames match `agent/description/interfaces.md`.
- Inputs/outputs match what the task file's "Input"/"Expected output" describe.

### 3. Logic correctness
- Trace the actual control flow against the intended behavior described in the plan — not just "does it compile/run."
- Boundary/edge cases relevant to the domain are handled: empty point cloud, zero/negative disparity, out-of-range sensor reads, first-frame-has-no-previous-frame, division by zero, empty ZMQ message, malformed UART packet.
- State that should reset between runs/frames actually resets (no stale state leaking across iterations).
- Off-by-one errors in indexing/loops, especially around frame buffers, grid cells, or point-cloud arrays.

### 4. Algorithmic efficiency — prioritize this
- Flag any loop that does **O(n²)** or worse work when a known O(n log n) or O(n) approach exists for that problem (e.g. nested-loop nearest-neighbor search instead of a KD-tree/voxel grid; linear search instead of a hash/set lookup; repeated recomputation of a value that's loop-invariant and should be hoisted out).
- Flag redundant recomputation inside a loop: recreating objects, reopening files/connections, recompiling regex, reallocating buffers, or recomputing the same transform/matrix every iteration when it only needs to be computed once.
- Flag unnecessary full-data copies inside a hot loop (e.g. copying a whole point cloud or image per iteration when an in-place or windowed operation would do).
- Prefer vectorized/library operations over manual Python loops where the codebase already depends on something that provides them (numpy/Open3D) — a manual per-point Python loop over a point cloud is a red flag when Open3D/numpy has a batch operation for it.
- For C++: flag avoidable heap allocation inside a loop (`new`/`push_back` without `reserve`, string concatenation in a loop) where it's on a hot path (per-frame, per-point).
- Don't over-optimize: a loop over a handful of config values or a one-time setup path doesn't need algorithmic scrutiny. Focus on per-frame / per-point / per-message hot paths.

### 5. Consistency with prior decisions
- Doesn't contradict a decision already recorded in `agent/history/` (e.g. don't reintroduce stereo/DA-V2 disparity code after `astra_slam_pipeline` history recorded it was deleted in favor of Astra Pro + Open3D SLAM).

## Process

1. Read the ground-truth sources listed above for the module/task in scope.
2. Identify the files changed in this task (`git diff --name-only` or the files just edited).
3. Read each changed file fully.
4. Walk the checklist; for each violation record file:line, what's wrong, and what the plan/requirement/history actually specifies.
5. Fix mismatches directly (Edit tool) **only when the code is what's wrong** — keep fixes scoped to correcting the logic/algorithm flagged, don't restyle unrelated code (that's `/code-standards-review`'s job). If the code looks correct and the *plan/requirement* looks stale or wrong, do NOT edit code to match it — stop and flag to the user (see closing note).
6. Re-check the fixed logic against the same checklist.
7. Repeat steps 4–6 until a pass finds zero violations, or up to 3 fix iterations. If still mismatched after 3 iterations, stop and report to the user — this usually means the plan/requirement itself is ambiguous or needs a decision, not that the agent can keep guessing.
8. Report a summary: what was checked against which ground-truth source, mismatches found, mismatches fixed, and any algorithmic optimization applied (old approach → new approach, and why).
9. Append one line to the active task file's Execution log recording the outcome, e.g. `[HH:MM] logic-review: N mismatches fixed, K flagged for user | info: <key finding or optimization>` — this ties the review into the task lifecycle before the task moves to `done`.

If the plan/requirement itself appears wrong or outdated (not the code), stop and flag this to the user rather than "fixing" code to match a stale plan.
