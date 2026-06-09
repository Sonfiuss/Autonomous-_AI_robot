# Robot Project — Master Context

## MANDATORY: Session start protocol
At the start of EVERY session, read these files before any other action:
1. `agent/description/project_overview.md`
2. `agent/description/interfaces.md`
3. Most recent file in `agent/history/` (use `ls -t agent/history/ | head -1`)
4. `agent/plan/<module>_plan.md` for the module you are about to work on

Do NOT skip this. The context loader hook also injects a summary automatically.

## Project root
`/home/nvidia/Documents`

## Modules
| Folder | Language | Purpose |
|--------|----------|---------|
| `motivation/` | C++ | Omni-wheel motion control via ESP32 UART |
| `stereo-camera/` | C++ | Stereo depth capture + 3D point-cloud builder |
| `simulation/` | Python/Flask | Browser path planner + ZMQ bridge to Jetson |
| `communication/` | TBD | Voice / LLM robot control |

## Task lifecycle protocol (FOLLOW THIS FOR EVERY NEW TASK)

### 1 — Intake
When user describes a task: create `agent/tasks/YYYY-MM-DD_<slug>.md` (see `agent/description/task_format.md`).
Fill: Task, Module, Input, Expected output. Set status `planning`.

### 2 — Plan
Write numbered Plan steps in the task file. Present to user. **Wait for approval before executing.**
On approval: set status `approved` → `executing`.

### 3 — Execute
After EACH plan step completes, append ONE line to Execution log:
`[HH:MM] step N: <summary> | risk: <if any> | info: <key discovered fact>`

### 4 — Test
When all steps done: set status `testing`.
Tell user the exact command or action to run to confirm.

### 5 — Done
User confirms → set status `done`. Write Test result.
Update `agent/plan/<module>_plan.md` to reflect completed work.

### Active task file location
`agent/tasks/` — inject automatically via context_loader if status is not `done`.

## Session end protocol
When a work session ends (either when asked or when stopping), write a summary to:
`agent/history/YYYY-MM-DD_<module>.md`

Include: what was implemented, decisions made, pending tasks, interface changes.

## Agent folder layout
```
agent/
  description/   ← module architecture docs (read at session start)
  plan/          ← active task lists per module (read + update during work)
  history/       ← session logs sorted by date (read latest at session start)
  scripts/       ← auto-run hooks (do not edit manually)
```
