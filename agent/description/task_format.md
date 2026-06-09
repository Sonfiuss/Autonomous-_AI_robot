# Task File Format

## Location
`agent/tasks/YYYY-MM-DD_<slug>.md`
slug = short-kebab-case name, e.g. `pose-controller-tuning`

## Status values
`planning` → `approved` → `executing` → `testing` → `done` | `blocked`

## Full template

```markdown
---
id: YYYY-MM-DD_<slug>
status: planning
module: motivation | stereo-camera | simulation | communication
started: YYYY-MM-DD
---

## Task
One-line description.

## Input
What user provided: files, constraints, example data, requirements.

## Expected output
Concrete definition of done — what the user will test to confirm success.

## Plan
- [ ] 1. Step one
- [ ] 2. Step two
- [ ] 3. Step three

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->
<!-- Keep under 15 lines — merge/compress older entries when it grows -->

## Test result
<!-- Filled when user confirms. Pass / Fail / Partial + brief note -->
```

## Rules
- Create the file at task intake (when user describes the task)
- Wait for user approval of Plan before changing status to `executing`
- Append one Execution log line after EACH plan step completes
- Log risks immediately when encountered: `| risk: <what and why>`
- Log key discovered facts: `| info: <fact that would surprise a future agent>`
- At status `testing`: tell user exactly what command/action to run to verify
- At status `done`: write Test result, then update the relevant `agent/plan/<module>_plan.md`
- Keep execution log lines SHORT — this is token budget for future context
