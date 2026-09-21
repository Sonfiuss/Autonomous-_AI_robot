---
id: 2026-09-12_llm-grounding
status: testing
module: CM (project/src/CM)
started: 2026-09-12
---

## Task
LLM-based target grounding on the 2D room map: user types a natural-language location ("near my bed"),
the LLM fills slots (target object -> side -> spot) and asks clarifying questions until exactly one
precomputed reachable point is matched; the UI highlights the matched candidate points on the map.
Prompt is a JSON file the user can edit.

## Module
`project/src/CM/` (Python). Reuses `simulation/room/room_generator.py` scenes (JSON v2.0).

## Input
- Scene JSON v2.0 from `simulation/room/` (objects with `free_sides`, `near`, polygons; robot hull).
- Roadmap `agent/description/robot_process_llm.md` L2 (LLM = words only) / L3 (code = geometry).
- LLM provider: OpenAI SDK installed (`openai` 1.109); Anthropic optional (`pip install anthropic`).
  No API key set in this shell yet. Both wired behind a config flag.
- User example: "Near my bed" -> "which side of the bed?" -> "left" -> "top right corner of the bed" -> point.

## Expected output
- `python project/src/CM/app.py` -> http://localhost:5001 shows the room + chat panel.
  Typing a location produces either a question (candidate spots highlighted dim) or a final goal
  (one bright point + heading). Reset restarts the dialog.
- `python project/src/CM/candidates.py --seed 7` prints all candidate spots for a scene.
- `python project/src/CM/test_grounding.py` runs the funnel + validator offline with a fake LLM.

## Plan
- [x] 1. Task file + `agent/plan/cm_plan.md`.
- [x] 2. `candidates.py`: per object x free side -> spots corner_a / center / corner_b, offset by hull
        radius + clearance, theta facing object, collision-filtered; compass + landmark description.
- [x] 3. `prompts/grounding.json` (system rules, ask policy, output schema, few-shot).
- [x] 4. `llm_client.py` (openai | anthropic | fake), `dialog.py` (session, prompt build, validator,
        auto-fill single options).
- [x] 5. `app.py` Flask (:5001) + `static/index.html` (renderer + chat + highlight).
- [x] 6. `test_grounding.py` offline scenario (bed example) with fake LLM.
- [x] 7. README, reviews, plan + history update.

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->
[00:20] step 1: task file + agent/plan/cm_plan.md created | info: user chose project/src/CM as module dir
[00:35] step 2: candidates.py (spots corner_a/center/corner_b per free side, hull collision filter, compass+landmark text, door spot) | risk: two spots at the same physical corner can share a description (e.g. back:corner_a and right:corner_b) -> LLM may pick either, both valid | info: seed 7 -> 31 spots; reuses room_generator geometry via sys.path
[00:45] step 3: prompts/grounding.json (system, 9 rules, JSON schema, 4 few-shot) | info: screen words top/right mapped to north/east in the prompt
[00:50] step 4: llm_client.py (openai json_schema response_format; anthropic beta.messages + output_config + fallbacks=default; fake) + dialog.py (validator + 1 retry, wildcard ids, autofill, code decides ask/goal/none) | risk: anthropic/openai paths not exercised live, no API key on this PC
[01:00] step 5: app.py :5001 + static/index.html (chat, pink = possible spots, green hull = goal, 'all spots' toggle) | info: scenes shared with simulation/room/scenes
[01:05] step 6: test_grounding.py ALL OK (200 seeds collision-free + facing check, bed funnel 3 turns, autofill, validator retry, none) | info: fallback after 2 invalid LLM replies now highlights nothing when no target is known
[01:08] step 7: README written; Flask smoke test with CM_PROVIDER=fake: new/say/reset/prompt all 200
[01:20] standards-review: 6 violations fixed (1e-9/200/"grounding"/default question/retry hint -> constants, unused param, __import__ hack), 0 deferred | info: index.html JS not covered by checklist
[01:25] logic-review: 3 fixes (openai content None -> LLMError not TypeError; retry hint folded into last user turn to keep role alternation for Anthropic; door target with 0 spots no longer KeyErrors), 0 flagged | info: door spot is a small scope addition beyond the plan (near-lists reference door_1), kept
[01:26] step 7: task -> testing; awaiting user run of app.py with a real API key
[01:40] user change: API keys moved to project/src/CM/.env (python-dotenv, loaded in config.py; .env git-ignored, .env.example kept) | info: shell vars still override .env
[01:50] user change: provider gemini added (OpenAI-compatible endpoint via installed openai SDK, GEMINI_API_KEY in .env, default model gemini-2.5-flash) and made the default | risk: Gemini compat endpoint accepts response_format json_schema but schema keywords like type arrays ["string","null"] may be ignored -> validator still guards | info: no google SDK needed
[02:05] fix: Gemini reply truncated mid-JSON -> thinking tokens ate max_tokens; now thinking_budget=0 via extra_body.google (nested form), GEMINI_MAX_TOKENS 8192, finish_reason=length raises a clear LLMError | info: live 3-turn bed dialog on seed 7 with gemini-2.5-flash: ask(6) -> ask(2) -> goal(1) OK
## Test result
