#!/usr/bin/env bash
# UserPromptSubmit hook — injects project context on first prompt of each session.
# Uses PPID (Claude Code process PID) as stable session ID.

AGENT_DIR="/home/nvidia/Documents/agent"
SESSION_MARKER="/tmp/claude_robot_session_${PPID}"

# Only inject on first prompt of this session
if [ -f "$SESSION_MARKER" ]; then
    exit 0
fi
touch "$SESSION_MARKER"

# ── Collect context ────────────────────────────────────────────────────────────
CONTEXT="=== ROBOT PROJECT AUTO-CONTEXT (session start) ===\n\n"

if [ -f "$AGENT_DIR/description/project_overview.md" ]; then
    CONTEXT+="$(cat "$AGENT_DIR/description/project_overview.md")\n\n"
fi

if [ -f "$AGENT_DIR/description/interfaces.md" ]; then
    CONTEXT+="$(cat "$AGENT_DIR/description/interfaces.md")\n\n"
fi

LATEST_HISTORY=$(ls -t "$AGENT_DIR/history/"*.md 2>/dev/null | head -1)
if [ -n "$LATEST_HISTORY" ]; then
    CONTEXT+="=== LAST SESSION LOG: $(basename "$LATEST_HISTORY") ===\n"
    CONTEXT+="$(cat "$LATEST_HISTORY")\n\n"
fi

# ── Active tasks (not done) ───────────────────────────────────────────────────
ACTIVE_TASKS=$(grep -rl "status: planning\|status: approved\|status: executing\|status: testing\|status: blocked" \
    "$AGENT_DIR/tasks/" 2>/dev/null | sort -r)
if [ -n "$ACTIVE_TASKS" ]; then
    CONTEXT+="=== ACTIVE TASKS ===\n"
    for task_file in $ACTIVE_TASKS; do
        CONTEXT+="--- $(basename "$task_file") ---\n"
        CONTEXT+="$(cat "$task_file")\n\n"
    done
fi

CONTEXT+="=== END AUTO-CONTEXT ===\n"
CONTEXT+="Task format: agent/description/task_format.md\n"
CONTEXT+="Plan files: agent/plan/motivation_plan.md, stereo_plan.md, simulation_plan.md\n"

# ── Output JSON for Claude Code injection ────────────────────────────────────
python3 -c "
import json, sys
text = sys.stdin.read()
print(json.dumps({'additionalContext': text}))
" <<< "$(printf '%b' "$CONTEXT")"
