"""L2 grounding dialog: slot filling (target -> side -> spot) over the candidate list.

The LLM only reads words and picks candidate ids. Code owns the state machine:
  * validates every LLM reply against the scene (ids exist, side is free, spot exists),
  * auto-fills a slot that has exactly one option, so the user is never asked a trivial question,
  * downgrades a premature "goal" to a question and upgrades a complete slot set to a goal.
"""
import json
import logging

import config
from candidates import build_candidates, scene_summary
from llm_client import LLMError

logger = logging.getLogger(__name__)

SLOT_ORDER = ("target", "side", "spot")
EMPTY_SLOTS = {"target": None, "side": None, "spot": None}
DEFAULT_TARGET_QUESTION = "Which object do you mean?"
RETRY_HINT = "Your previous JSON was invalid: {error}. Answer again using only ids from the MAP."


def load_prompt(path=config.PROMPT_FILE):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


class GroundingSession:
    """One dialog over one scene. Call say(text) per user message."""

    def __init__(self, scene, llm, prompt=None):
        self.scene = scene
        self.llm = llm
        self.prompt = prompt or load_prompt()
        self.spots = build_candidates(scene)
        self.by_id = {s["id"]: s for s in self.spots}
        self.objects = {o["id"]: o for o in scene["objects"]}
        self.targets = set(self.objects) | {d["id"] for d in scene["doors"]}
        self.slots = dict(EMPTY_SLOTS)
        self.history = []                  # [{"role": "user"|"assistant", "content": str}]
        self.summary = scene_summary(scene, self.spots)
        self.goal = None                   # last resolved spot (L3 -> L4 handover to MV)

    # ------------------------------------------------------------ public
    def reset(self):
        self.slots = dict(EMPTY_SLOTS)
        self.history = []
        self.goal = None

    def resolved_goal(self):
        """The spot the dialog last resolved, or None while the target is still ambiguous."""
        return self.goal

    def say(self, text):
        """Process one user utterance -> response dict for the UI."""
        self.history.append({"role": "user", "content": text})
        reply, error = None, None
        for attempt in range(config.MAX_VALIDATION_RETRIES + 1):
            try:
                raw = self.llm.complete(self._system_prompt(), self._messages(error), self.prompt["output_schema"])
            except LLMError as exc:
                logger.error("LLM failure: %s", exc)
                return self._response("error", str(exc), [])
            reply, error = self._validate(raw)
            if error is None:
                break
            logger.warning("LLM reply rejected (attempt %d): %s", attempt + 1, error)
        if error is not None:
            reply = self._fallback_reply()
        self.history.append({"role": "assistant", "content": json.dumps(reply)})
        return self._finalize(reply)

    def state(self):
        return {"slots": self.slots, "history": self.history, "candidates": self.spots}

    # ------------------------------------------------------------ prompt
    def _system_prompt(self):
        p = self.prompt
        parts = [p["system"], "RULES:"] + [f"- {r}" for r in p["rules"]]
        parts.append("OUTPUT: one JSON object matching this schema:\n" + json.dumps(p["output_schema"]))
        if p.get("examples"):
            parts.append("EXAMPLES (ids refer to a different room):")
            for ex in p["examples"]:
                parts.append(f"user: {ex['user']}\nassistant: {json.dumps(ex['assistant'])}")
        parts.append("MAP:\n" + self.summary)
        return "\n\n".join(parts)

    def _messages(self, error):
        msgs = [dict(m) for m in self.history]
        if error:
            msgs[-1]["content"] += "\n\n" + RETRY_HINT.format(error=error)
        return msgs

    # ------------------------------------------------------------ validation
    def _validate(self, raw):
        """Return (clean_reply, None) or (None, error_text)."""
        if not isinstance(raw, dict):
            return None, "not a JSON object"
        rtype = raw.get("type")
        if rtype not in ("ask", "goal", "none"):
            return None, f"type must be ask|goal|none, got {rtype!r}"
        slots = dict(EMPTY_SLOTS)
        slots.update({k: v for k, v in (raw.get("slots") or {}).items() if k in SLOT_ORDER})
        if slots["target"] is not None and slots["target"] not in self.targets:
            return None, f"unknown target {slots['target']!r}"
        if slots["side"] is not None:
            if slots["target"] is None:
                return None, "side given without target"
            if slots["side"] not in self._sides_of(slots["target"]):
                return None, f"side {slots['side']!r} is not a free side of {slots['target']}"
        if slots["spot"] is not None:
            if slots["side"] is None:
                return None, "spot given without side"
            if f"{slots['target']}:{slots['side']}:{slots['spot']}" not in self.by_id:
                return None, f"spot {slots['spot']!r} does not exist on {slots['target']}:{slots['side']}"
        matches = []
        for mid in raw.get("matches") or []:
            expanded = self._expand(mid)
            if not expanded:
                return None, f"unknown candidate id {mid!r}"
            matches.extend(m for m in expanded if m not in matches)
        return {"type": rtype, "question": str(raw.get("question") or ""), "slots": slots,
                "matches": matches}, None

    def _expand(self, mid):
        """Accept exact ids and wildcards 'obj_1:*' / 'obj_1:left:*'."""
        if mid in self.by_id:
            return [mid]
        if mid.endswith(":*"):
            prefix = mid[:-1]
            return [s for s in self.by_id if s.startswith(prefix)]
        return []

    def _sides_of(self, target):
        if target in self.objects:
            return self.objects[target]["free_sides"]
        return ["inside"] if f"{target}:inside:center" in self.by_id else []

    def _fallback_reply(self):
        """LLM kept sending invalid ids: keep the slots we trust and ask the code question."""
        slots = dict(self.slots)
        matches = self._matches_for(slots) if slots["target"] is not None else []
        return {"type": "ask", "question": "", "slots": slots, "matches": matches}

    # ------------------------------------------------------------ state machine
    def _finalize(self, reply):
        if reply["type"] == "none":
            self.slots = dict(EMPTY_SLOTS)
            return self._response("none", reply["question"], [])
        slots = reply["slots"]
        if slots["target"] is None:
            self.slots = dict(EMPTY_SLOTS)
            return self._response("ask", reply["question"] or DEFAULT_TARGET_QUESTION, reply["matches"])
        slots = self._autofill(slots)
        self.slots = slots
        matches = self._matches_for(slots)
        if len(matches) == 1:
            self.slots["side"], self.slots["spot"] = matches[0].split(":")[1:3]
            return self._response("goal", reply["question"], matches)
        if not matches:
            self.slots = dict(EMPTY_SLOTS)
            return self._response("none", f"{self._class_of(slots['target'])} has no reachable spot.", [])
        question = reply["question"] if reply["type"] == "ask" else ""
        return self._response("ask", question or self._code_question(slots, matches), matches)

    def _autofill(self, slots):
        slots = dict(slots)
        if slots["side"] is None:
            sides = self._sides_of(slots["target"])
            if len(sides) == 1:
                slots["side"] = sides[0]
        if slots["side"] is not None and slots["spot"] is None:
            spots = [s for s in self.spots if s["target_id"] == slots["target"] and s["side"] == slots["side"]]
            if len(spots) == 1:
                slots["spot"] = spots[0]["spot"]
        return slots

    def _matches_for(self, slots):
        out = []
        for s in self.spots:
            if slots["target"] is not None and s["target_id"] != slots["target"]:
                continue
            if slots["side"] is not None and s["side"] != slots["side"]:
                continue
            if slots["spot"] is not None and s["spot"] != slots["spot"]:
                continue
            out.append(s["id"])
        return out

    def _class_of(self, target):
        return self.objects[target]["class"] if target in self.objects else "door"

    def _code_question(self, slots, matches):
        """Deterministic question when the LLM did not provide one."""
        cls = self._class_of(slots["target"])
        if slots["side"] is None:
            sides = sorted({self.by_id[m]["side_compass"] for m in matches})
            return f"Which side of the {cls}: {', '.join(sides)}?"
        opts = [self.by_id[m]["description"].split(" of the ")[0] for m in matches]
        return f"Where along the {self.by_id[matches[0]]['side_compass']} side of the {cls}: {', '.join(opts)}?"

    def _response(self, rtype, text, matches):
        self.goal = self.by_id[matches[0]] if rtype == "goal" and matches else None
        return {
            "type": rtype,
            "text": text,
            "slots": dict(self.slots),
            "matches": [self.by_id[m] for m in matches],
            "resolved": rtype == "goal",
        }
