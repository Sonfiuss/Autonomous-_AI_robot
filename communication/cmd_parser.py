"""Text command -> validated relative motion steps (the "primitive" branch of L2 in
agent/description/robot_process_llm.md: a direct motion skips the map, L3 and L4).

Two parsers produce the same raw reply, {"steps": [{"action", "value", "unit"}], "question"}, and ONE
validator turns it into MotionSteps:
  * LLM through CM's llm_client (gemini | openai | anthropic): free phrasing, Vietnamese or English;
  * rules: regexes over the diacritic-folded text. Offline, no key, a fixed phrase set.
The LLM only transcribes. Every number the user did not write (the default turn angle, a U-turn) is
filled in here, and a number the LLM returns that the user never wrote is refused: a hallucinated
distance must never become motion.
"""
import json
import logging
import math
import re
import unicodedata
from dataclasses import dataclass

import drive_config as cfg

logger = logging.getLogger(__name__)

ACTION_FORWARD = "forward"
ACTION_BACKWARD = "backward"
ACTION_TURN_LEFT = "turn_left"
ACTION_TURN_RIGHT = "turn_right"
ACTION_TURN_AROUND = "turn_around"
ACTIONS = (ACTION_FORWARD, ACTION_BACKWARD, ACTION_TURN_LEFT, ACTION_TURN_RIGHT, ACTION_TURN_AROUND)
LINEAR_ACTIONS = (ACTION_FORWARD, ACTION_BACKWARD)
PROVIDER_AUTO = "auto"
PROVIDER_RULE = "rule"
PROVIDER_INJECTED = "injected"   # a client handed in by the caller (tests)
LLM_PROVIDERS = ("gemini", "openai", "anthropic")
RETRY_HINT = "Your previous JSON was invalid: {error}. Answer again, copying only numbers the user wrote."
QUESTION_FALLBACK = "Bạn muốn robot di chuyển thế nào? Ví dụ: 'đi thẳng 30 cm, rẽ trái, đi tiếp 60 cm'."
QUESTION_INVALID = "Lệnh bị từ chối ({error}). Hãy nói lại lệnh, ghi rõ số và đơn vị."
QUESTION_NO_MATCH = "Không nhận ra lệnh di chuyển nào. Ví dụ: 'đi thẳng 30 cm, rẽ trái, đi tiếp 60 cm'."
QUESTION_LEFTOVER = "Chưa hiểu phần '{part}'. Hãy ghi rõ khoảng cách (cm/m) hoặc góc (độ)."

# Numbers a user may say as a single word, diacritic-folded (see fold()). They only WIDEN the set
# the validator accepts; compounds ("ba muoi", "mot met ruoi") must be typed as digits. "sau" (6) is
# left out on purpose: "sau do" (after that) is in nearly every command and would always allow a 6.
NUMBER_WORDS = {
    "mot": 1, "hai": 2, "ba": 3, "bon": 4, "nam": 5, "bay": 7, "tam": 8, "chin": 9, "muoi": 10,
    "nua": 0.5,
    "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6, "seven": 7, "eight": 8,
    "nine": 9, "ten": 10, "half": 0.5,
}

# ---- rule parser vocabulary (diacritic-folded, lower case)
_NUM = r"(\d+(?:[.,]\d+)?)"
_LINEAR_UNIT = r"(mm|cm|centimet(?:er|re)?s?|phan|met(?:er|re)?s?|m)\b"
_ANGLE_UNIT = r"(?:do\b|deg(?:rees?)?\b|°)"
_SIDE = r"(trai|phai|left|right)\b"
_TURN_VERB = r"\b(?:re|queo|quay|xoay|turn)"
_TOWARDS = r"(?:(?:sang|ve|qua|to the|to)\s+)?(?:(?:phia|ben)\s+)?"
_LINEAR_VERBS = "lui|back|backward|backwards|reverse|di|tien|chay|forward|go|move|drive"
_LINEAR_FILLERS = ("lui|lai|thang|tiep|len|toi|ve|phia|truoc|sau|khoang|chung|straight|ahead|"
                   "forward|back|backward|backwards|on|about|for")
_BACKWARD_WORDS = frozenset(("lui", "back", "backward", "backwards", "reverse", "sau"))
_RIGHT_WORDS = frozenset(("phai", "right"))

_TURN_AROUND_RE = re.compile(r"\b(?:quay dau|quay lai|u-turn|u turn|turn around|turn back)\b")
_TURN_ANGLE_FIRST_RE = re.compile(_TURN_VERB + r"\s+" + _NUM + r"\s*" + _ANGLE_UNIT + r"\s*" + _TOWARDS + _SIDE)
_TURN_RE = re.compile(_TURN_VERB + r"\s+" + _TOWARDS + _SIDE + r"(?:\s*" + _NUM + r"\s*" + _ANGLE_UNIT + r")?")
_LINEAR_RE = re.compile(r"\b(" + _LINEAR_VERBS + r")\b((?:\s+(?:" + _LINEAR_FILLERS + r")\b)*)\s*"
                        + _NUM + r"\s*" + _LINEAR_UNIT)
# Left over after every match is removed, any of these means a move the rules did not understand.
_LEFTOVER_RE = re.compile(r"\d|\b(?:di|tien|lui|re|queo|quay|xoay|chay|forward|backward|back|turn|"
                          r"left|right|trai|phai|go|move|drive|reverse)\b")
_TOKEN_RE = re.compile(r"\d+(?:[.,]\d+)?|[a-z]+")
_LEFTOVER_STRIP = " .,;"         # separators trimmed off an unmatched fragment before quoting it
# Which regex a rule match came from.
_KIND_AROUND = "around"
_KIND_ANGLE_FIRST = "angle_first"
_KIND_TURN = "turn"
_KIND_LINEAR = "linear"


class ParserSetupError(RuntimeError):
    """An explicitly requested LLM provider cannot be used (SDK missing, no API key)."""


@dataclass(frozen=True)
class MotionStep:
    """One relative move. amount: metres for forward/backward, degrees for turns; always > 0."""
    action: str
    amount: float
    stated: bool          # False when the code filled the amount in (default turn, U-turn)

    def describe(self):
        if self.action in LINEAR_ACTIONS:
            return f"{self.action} {self.amount * cfg.CM_PER_M:.1f} cm"
        return f"{self.action} {self.amount:.1f} deg" + ("" if self.stated else " (default)")

    def to_dict(self):
        return {"action": self.action, "amount": self.amount, "stated": self.stated}


@dataclass(frozen=True)
class ParseResult:
    """steps is empty exactly when question is set: then nothing may move."""
    steps: tuple = ()
    question: str = ""
    provider: str = ""    # parser that produced it: gemini | openai | anthropic | rule | injected
    raw: object = None    # the parser's raw reply, kept for the run log


def fold(text):
    """Lower case without Vietnamese diacritics, 'đ' -> 'd': 'Rẽ trái' -> 're trai'."""
    text = unicodedata.normalize("NFC", text).lower().replace("đ", "d")
    return "".join(c for c in unicodedata.normalize("NFD", text) if not unicodedata.combining(c))


def numbers_in(text):
    """Every number the user wrote, in the order written: digits ('0,5' reads 0.5) or a single
    number word."""
    found = []
    for token in _TOKEN_RE.findall(fold(text)):
        if token[0].isdigit():
            found.append(float(token.replace(",", ".")))
        elif token in NUMBER_WORDS:
            found.append(float(NUMBER_WORDS[token]))
    return found


def _next_match(value, written, start):
    """Index just past the first number at or after `start` equal to value, or None."""
    for index in range(start, len(written)):
        if abs(value - written[index]) <= cfg.NUMBER_MATCH_TOL:
            return index + 1
    return None


def _validate_step(item, written):
    """One raw step -> (MotionStep, None) or (None, error)."""
    if not isinstance(item, dict):
        return None, "not an object"
    action, value, unit = item.get("action"), item.get("value"), item.get("unit")
    if action not in ACTIONS:
        return None, f"unknown action {action!r}"
    if action == ACTION_TURN_AROUND:
        return MotionStep(action, cfg.TURN_AROUND_DEG, False), None
    if value is None:
        if action in LINEAR_ACTIONS:
            return None, f"{action} without a distance"
        return MotionStep(action, cfg.DEFAULT_TURN_DEG, False), None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return None, f"value {value!r} is not a number"
    if not any(abs(value - w) <= cfg.NUMBER_MATCH_TOL for w in written):
        return None, f"value {value:g} does not appear in the command"
    if action in LINEAR_ACTIONS:
        if unit not in cfg.LINEAR_UNITS_M:
            return None, f"unit {unit!r} is not a distance unit (mm | cm | m)"
        metres = value * cfg.LINEAR_UNITS_M[unit]
        if not cfg.MIN_LINEAR_M <= metres <= cfg.MAX_LINEAR_M:
            return None, f"{metres:g} m is outside {cfg.MIN_LINEAR_M:g}..{cfg.MAX_LINEAR_M:g} m"
        return MotionStep(action, metres, True), None
    if unit not in cfg.ANGLE_UNITS:
        return None, f"unit {unit!r} is not an angle unit (deg)"
    if not cfg.MIN_TURN_DEG <= value <= cfg.MAX_TURN_DEG:
        return None, f"{value:g} deg is outside {cfg.MIN_TURN_DEG:g}..{cfg.MAX_TURN_DEG:g} deg"
    return MotionStep(action, float(value), True), None


def validate_steps(raw, user_text, ordered=True):
    """Raw parser reply -> (steps, None) or (None, error). user_text holds every user message of the
    exchange, so a number given in an answer to a question counts as written.
    ordered: the stated values must also come in the order the user wrote them, which catches an LLM
    that swaps two real numbers ("30 cm ... 60 do" read as 60 cm and 30 deg). Only meaningful for a
    single message: an answer to a question legitimately supplies a number for an earlier step."""
    if not isinstance(raw, dict) or not isinstance(raw.get("steps"), list):
        return None, "reply must be an object with a 'steps' list"
    items = raw["steps"]
    if len(items) > cfg.MAX_STEPS:
        return None, f"{len(items)} steps, at most {cfg.MAX_STEPS}"
    written = numbers_in(user_text)
    steps, cursor = [], 0
    for index, item in enumerate(items, 1):
        step, error = _validate_step(item, written)
        if error is not None:
            return None, f"step {index}: {error}"
        if ordered and step.stated:
            cursor = _next_match(item["value"], written, cursor)
            if cursor is None:
                return None, f"step {index}: values are not in the order the command gives them"
        steps.append(step)
    return tuple(steps), None


def _linear_unit(word):
    if word.startswith("centimet") or word == "phan":
        return "cm"
    if word.startswith("met"):
        return "m"
    return word                                  # mm | cm | m


def _rule_step(kind, match):
    """A regex match -> the raw step dict the LLM would have produced."""
    if kind == _KIND_AROUND:
        return {"action": ACTION_TURN_AROUND, "value": None, "unit": None}
    if kind == _KIND_LINEAR:
        words = {match.group(1)} | set(match.group(2).split())
        action = ACTION_BACKWARD if words & _BACKWARD_WORDS else ACTION_FORWARD
        return {"action": action, "value": float(match.group(3).replace(",", ".")),
                "unit": _linear_unit(match.group(4))}
    if kind == _KIND_ANGLE_FIRST:
        number, side = match.group(1), match.group(2)
    else:
        side, number = match.group(1), match.group(2)
    action = ACTION_TURN_RIGHT if side in _RIGHT_WORDS else ACTION_TURN_LEFT
    if number is None:
        return {"action": action, "value": None, "unit": None}
    return {"action": action, "value": float(number.replace(",", ".")), "unit": cfg.ANGLE_UNIT}


def rule_parse(text):
    """Offline parser. Returns the raw reply shape; 'question' says what was not understood."""
    folded = fold(text)
    found = []
    for kind, pattern in ((_KIND_AROUND, _TURN_AROUND_RE), (_KIND_ANGLE_FIRST, _TURN_ANGLE_FIRST_RE),
                          (_KIND_TURN, _TURN_RE), (_KIND_LINEAR, _LINEAR_RE)):
        found += [(m.start(), -(m.end() - m.start()), kind, m) for m in pattern.finditer(folded)]
    found.sort(key=lambda f: (f[0], f[1]))       # by position; the longest wins a shared start

    steps, spans, covered_to = [], [], 0
    for start, _, kind, match in found:
        if start < covered_to:
            continue                             # overlaps a match already taken
        steps.append(_rule_step(kind, match))
        spans.append((start, match.end()))
        covered_to = match.end()

    leftover, cursor = [], 0
    for start, end in spans:
        leftover.append(folded[cursor:start])
        cursor = end
    leftover.append(folded[cursor:])
    for part in leftover:
        if _LEFTOVER_RE.search(part):
            return {"steps": [], "question": QUESTION_LEFTOVER.format(part=part.strip(_LEFTOVER_STRIP))}
    if not steps:
        return {"steps": [], "question": QUESTION_NO_MATCH}
    return {"steps": steps, "question": ""}


def load_prompt(path=cfg.PROMPT_FILE):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


class CommandParser:
    """Text -> ParseResult. provider: auto (CM's configured LLM, falling back to the rules when it
    cannot be reached), rule, or one of LLM_PROVIDERS (errors surface, never a silent fallback)."""

    def __init__(self, provider=PROVIDER_AUTO, client=None, prompt=None):
        self.prompt = prompt or load_prompt()
        self._fallback = provider == PROVIDER_AUTO
        self.history = []                        # LLM turns [{"role", "content"}]
        self.user_texts = []                     # every user message of the current exchange
        self._llm_error = Exception              # narrowed by _make_client to what the client raises
        self.provider, self._client = self._make_client(provider, client)

    def reset(self):
        self.history = []
        self.user_texts = []

    def parse(self, text):
        self.user_texts.append(text)
        if self._client is None:
            result = self._parse_rules(text)
        else:
            try:
                result = self._parse_llm(text)
            except self._llm_error as exc:
                if not self._fallback:
                    raise
                logger.warning("LLM %s failed (%s): falling back to the rule parser", self.provider, exc)
                # The whole exchange goes, not only the LLM's side of it: numbers from messages the
                # LLM will never see again must not count as written (nor switch the order check off).
                self.reset()
                result = self._parse_rules(text)
        if result.steps:
            self.reset()                         # the next message starts a new command
        return result

    # ------------------------------------------------------------ setup
    def _make_client(self, provider, client):
        if client is not None:
            self._llm_error = RuntimeError       # llm_client.LLMError is one; so is a test double's
            return PROVIDER_INJECTED, client
        if provider == PROVIDER_RULE:
            return PROVIDER_RULE, None
        if provider != PROVIDER_AUTO and provider not in LLM_PROVIDERS:
            raise ValueError(f"unknown provider {provider!r} (auto | rule | {' | '.join(LLM_PROVIDERS)})")
        cfg.add_cm_to_path()
        try:
            import llm_client                    # CM's; pulls in CM config (.env with the API keys)
            self._llm_error = llm_client.LLMError
            if provider == PROVIDER_AUTO:
                return llm_client.config.PROVIDER, llm_client.make_client()
            return provider, llm_client.make_client(provider)
        except Exception as exc:                 # missing SDK or dotenv, missing API key
            if provider != PROVIDER_AUTO:
                raise ParserSetupError(f"LLM provider {provider!r} unusable: {exc}") from exc
            logger.warning("no LLM available (%s): using the rule parser", exc)
            return PROVIDER_RULE, None

    # ------------------------------------------------------------ parsers
    def _parse_rules(self, text):
        raw = rule_parse(text)
        if raw["question"]:
            return ParseResult(question=raw["question"], provider=PROVIDER_RULE, raw=raw)
        steps, error = validate_steps(raw, text)
        if error is not None:
            return ParseResult(question=QUESTION_INVALID.format(error=error), provider=PROVIDER_RULE, raw=raw)
        return ParseResult(steps=steps, provider=PROVIDER_RULE, raw=raw)

    def _parse_llm(self, text):
        self.history.append({"role": "user", "content": text})
        exchange = "\n".join(self.user_texts)
        raw, steps, error = None, (), None
        for attempt in range(cfg.MAX_VALIDATION_RETRIES + 1):
            raw = self._client.complete(self._system_prompt(), self._messages(error),
                                        self.prompt["output_schema"])
            steps, error = validate_steps(raw, exchange, ordered=len(self.user_texts) == 1)
            if error is None:
                break
            logger.warning("LLM reply rejected (attempt %d): %s", attempt + 1, error)
        self.history.append({"role": "assistant", "content": json.dumps(raw, ensure_ascii=False)})
        if error is not None:
            return ParseResult(question=QUESTION_INVALID.format(error=error), provider=self.provider, raw=raw)
        if not steps:
            question = str(raw.get("question") or "").strip() or QUESTION_FALLBACK
            return ParseResult(question=question, provider=self.provider, raw=raw)
        return ParseResult(steps=steps, provider=self.provider, raw=raw)

    def _system_prompt(self):
        p = self.prompt
        parts = [p["system"], "RULES:"] + [f"- {r}" for r in p["rules"]]
        parts.append("OUTPUT: one JSON object matching this schema:\n" + json.dumps(p["output_schema"]))
        parts.append("EXAMPLES:")
        for ex in p.get("examples", []):
            parts.append(f"user: {ex['user']}\nassistant: {json.dumps(ex['assistant'], ensure_ascii=False)}")
        return "\n\n".join(parts)

    def _messages(self, error):
        messages = [dict(m) for m in self.history]
        if error:
            messages[-1]["content"] += "\n\n" + RETRY_HINT.format(error=error)
        return messages
