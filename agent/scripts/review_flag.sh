#!/usr/bin/env bash
# review_flag.sh — PostToolUse hook (matcher: Write|Edit).
# If a source file was written/edited this turn, set the "code changed" marker
# so the Stop gate knows a review is owed. Part of the auto-review gate.

set -u

STATE_DIR="${TMPDIR:-/tmp}/claude_review_state"
mkdir -p "$STATE_DIR"

payload="$(cat)"
sid="$(printf '%s' "$payload" \
  | grep -oE '"session_id"[[:space:]]*:[[:space:]]*"[^"]*"' \
  | head -1 | sed -E 's/.*"([^"]*)"$/\1/')"
[ -n "$sid" ] || sid="default"

# Extract the edited file path from tool_input.file_path.
file="$(printf '%s' "$payload" \
  | grep -oE '"file_path"[[:space:]]*:[[:space:]]*"[^"]*"' \
  | head -1 | sed -E 's/.*"([^"]*)"$/\1/')"

# Only source files trigger a review (matches the two review skills' scope).
case "$file" in
  *.cpp|*.hpp|*.h|*.cc|*.py) touch "$STATE_DIR/${sid}.changed" ;;
esac

exit 0
