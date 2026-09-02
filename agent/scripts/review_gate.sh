#!/usr/bin/env bash
# review_gate.sh — Stop hook.
# If source code changed this turn and hasn't been reviewed yet, block the stop
# once and ask the agent to run the two review skills. The "reviewed" marker
# makes this fire at most once per user turn, so the review's own edits do NOT
# retrigger it (no infinite loop). Part of the auto-review gate.

set -u

STATE_DIR="${TMPDIR:-/tmp}/claude_review_state"
mkdir -p "$STATE_DIR"

payload="$(cat)"
sid="$(printf '%s' "$payload" \
  | grep -oE '"session_id"[[:space:]]*:[[:space:]]*"[^"]*"' \
  | head -1 | sed -E 's/.*"([^"]*)"$/\1/')"
[ -n "$sid" ] || sid="default"

CHANGED="$STATE_DIR/${sid}.changed"
REVIEWED="$STATE_DIR/${sid}.reviewed"

# Nothing edited, or already reviewed this turn -> let the stop proceed.
if [ ! -f "$CHANGED" ] || [ -f "$REVIEWED" ]; then
  exit 0
fi

# Mark reviewed first so the follow-up edits from the review can't re-trigger.
touch "$REVIEWED"

REASON="Source code was edited this turn but not yet reviewed. Before stopping, run the /code-standards-review skill, then the /code-logic-review skill, on the files changed this turn. Apply their auto-fix loops and log the outcome to the active task file, then finish."

# Emit the block decision as JSON (kept on one line, no embedded quotes/newlines).
printf '{"decision":"block","reason":"%s"}\n' "$REASON"
exit 0
