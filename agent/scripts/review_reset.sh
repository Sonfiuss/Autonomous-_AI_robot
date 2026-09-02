#!/usr/bin/env bash
# review_reset.sh — UserPromptSubmit hook.
# Clears this session's review markers so each new user turn starts fresh.
# Part of the auto-review gate (see review_flag.sh / review_gate.sh).

set -u

STATE_DIR="${TMPDIR:-/tmp}/claude_review_state"
mkdir -p "$STATE_DIR"

# Read the hook payload from stdin and pull out session_id (no jq available).
payload="$(cat)"
sid="$(printf '%s' "$payload" \
  | grep -oE '"session_id"[[:space:]]*:[[:space:]]*"[^"]*"' \
  | head -1 | sed -E 's/.*"([^"]*)"$/\1/')"
[ -n "$sid" ] || sid="default"

# Fresh turn: drop both the "code changed" and "already reviewed" markers.
rm -f "$STATE_DIR/${sid}.changed" "$STATE_DIR/${sid}.reviewed"

exit 0
