"""Text command -> validated relative motion steps, or a place to go to (the "primitive" and "goal"
branches of L2 in agent/description/robot_process_llm.md). A place is only copied here, in the user's own
words: the real room's map resolves and plans it (communication/goto.py, realroom/).

Two parsers produce the same raw reply, {"steps": [{"action", "value", "unit"}], "question", "goal"}, and
ONE validator turns it into MotionSteps:
  * LLM through CM's llm_client (gemini | openai | anthropic): free phrasing, Vietnamese or English;
  * rules: regexes over the diacritic-folded text - offline, no key; turns in any word order, carried-on
    steps, purpose clauses. Asked, they keep the command pending and put the answer where the question
    pointed (CommandParser._parse_rules).
The LLM only transcribes. Every number the user did not write (the default turn angle, a U-turn) is
filled in here, and a number the LLM returns that the user never wrote is refused: a hallucinated
distance must never become motion. A turn with no side is decided here (half / full turn) or asked.
"""
import json
import logging
import math
import os
import re
import time
import unicodedata
from dataclasses import dataclass

import drive_config as cfg

logger = logging.getLogger(__name__)

ACTION_FORWARD = "forward"
ACTION_BACKWARD = "backward"
ACTION_TURN_LEFT = "turn_left"
ACTION_TURN_RIGHT = "turn_right"
ACTION_TURN_AROUND = "turn_around"
ACTION_TURN = "turn"             # raw only: a turn with no side; the validator decides it or asks
ACTIONS = (ACTION_FORWARD, ACTION_BACKWARD, ACTION_TURN_LEFT, ACTION_TURN_RIGHT, ACTION_TURN_AROUND)
RAW_ACTIONS = ACTIONS + (ACTION_TURN,)
TURN_ACTIONS = (ACTION_TURN_LEFT, ACTION_TURN_RIGHT, ACTION_TURN_AROUND, ACTION_TURN)
TURN_SIGN = {ACTION_TURN_LEFT: 1.0, ACTION_TURN_RIGHT: -1.0, ACTION_TURN_AROUND: 1.0}   # CCW +; a U-turn goes left
LINEAR_SIGN = {ACTION_FORWARD: 1.0, ACTION_BACKWARD: -1.0}
FULL_TURN_DEG = 360.0
INTENT_HEADING_TOL_DEG = 1.0     # "back to the start" is met when the heading is this close to it ...
INTENT_POSITION_TOL_M = 0.01     # ... and, for a position, the centre this close
INTENT_HEADING, INTENT_POSITION = "heading", "position"
LINEAR_ACTIONS = (ACTION_FORWARD, ACTION_BACKWARD)
PROVIDER_AUTO = "auto"
PROVIDER_RULE = "rule"
PROVIDER_INJECTED = "injected"   # a client handed in by the caller (tests)
LLM_PROVIDERS = ("gemini", "openai", "anthropic")
PROVIDER_GEMINI = "gemini"
NO_LLM = "no LLM available: {error}"
BUSY_MARKS = ("429", "503")      # HTTP 429 (out of quota, rate limit) / 503 (overloaded) in an LLM error
RETRY_HINT = "Your previous JSON was invalid: {error}. Answer again, copying only numbers the user wrote."
ASKED_NOTE = "(The robot then asked the user: {question})"
QUESTION_FALLBACK = "Bạn muốn robot di chuyển thế nào? Ví dụ: 'đi thẳng 30 cm, rẽ trái, đi tiếp 60 cm'."
QUESTION_INVALID = "Lệnh bị từ chối ({error}). Hãy nói lại lệnh, ghi rõ số và đơn vị."
QUESTION_NO_MATCH = "Không nhận ra lệnh di chuyển nào. Ví dụ: 'đi thẳng 30 cm, rẽ trái, đi tiếp 60 cm'."
QUESTION_LEFTOVER = "Chưa hiểu phần '{part}'. Hãy ghi rõ khoảng cách (cm/m) hoặc góc (độ)."
QUESTION_SIDE = "Quay {angle:g} độ sang trái hay sang phải?"
QUESTION_TURN = "Quay bao nhiêu độ, sang trái hay sang phải? Ví dụ: 'quay trái 90 độ'."
QUESTION_DISTANCE = "'{part}' bao xa (cm hoặc m)?"
GOAL_FREE_WORDS = frozenset(("the", "a", "an"))   # a copied place may gain an article, nothing else
QUESTION_GOAL_AND_STEPS = ("Mỗi lệnh chỉ đi tới một chỗ, hoặc chỉ gồm các bước đi / quay. Hãy nói riêng: trước "
                           "'{goal}', rồi các bước còn lại.")
UNDERSTOOD_PREFIX = "\n  Đã hiểu: "
PENDING_REPLACE, PENDING_AFTER = "replace", "after"   # where an answer goes in the command it answers
# How a raw step reads back to the user in a question (value None: the code's default).
SAY_ACTION = {ACTION_FORWARD: "đi thẳng", ACTION_BACKWARD: "lùi", ACTION_TURN_LEFT: "quay trái",
              ACTION_TURN_RIGHT: "quay phải", ACTION_TURN_AROUND: "quay đầu", ACTION_TURN: "quay (chưa rõ phía)"}
SAY_UNIT = {"mm": "mm", "cm": "cm", "m": "m", cfg.ANGLE_UNIT: "độ", cfg.REV_UNIT: "vòng"}
QUESTION_RETURN = ("Mình chưa tự tìm đường về '{part}' được. Hãy nói rõ các bước, ví dụ: 'quay trái 180 độ, "
                   "đi thẳng 90 cm'.")
PROVIDER_MAP_SCAN = "map scan"   # a map request with no other move: the scan below, read by no parser
# "lập bản đồ" with nothing else: turn in place left 90, right 180, left 90 - the scan that built map
# 20260929_184936 (the camera looks to either side; the robot ends facing as it began).
MAP_SCAN = ((ACTION_TURN_LEFT, 90.0), (ACTION_TURN_RIGHT, 180.0), (ACTION_TURN_LEFT, 90.0))

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
# A distance may be a single number word ("nua met", "mot met", "half a meter" is not: 'a' is no number):
# only with its unit right after, so "nam" (5) or "ba" (3) are never read as numbers anywhere else.
_DIST_NUM = r"(\d+(?:[.,]\d+)?|mot|hai|ba|bon|nam|nua|one|two|three|four|five|half)"
_LINEAR_UNIT = r"(mm|cm|centimet(?:er|re)?s?|phan|met(?:er|re)?s?|m)\b"
_LINEAR_VERBS = "lui|back|backward|backwards|reverse|di|tien|chay|forward|go|move|drive"
_LINEAR_FILLERS = ("lui|lai|thang|tiep|len|toi|ve|phia|truoc|sau|khoang|chung|straight|ahead|"
                   "forward|back|backward|backwards|up|on|about|for")
_BACKWARD_WORDS = frozenset(("lui", "back", "backward", "backwards", "reverse", "sau"))
_LINEAR_RE = re.compile(r"\b(" + _LINEAR_VERBS + r")\b((?:\s+(?:" + _LINEAR_FILLERS + r")\b)*)\s*"
                        + _DIST_NUM + r"\s*" + _LINEAR_UNIT)
# A move verb with no distance after it ("di thang roi re trai"): asked, and the answer goes right after it.
_LINEAR_BARE_RE = re.compile(r"\b(?:" + _LINEAR_VERBS + r")\b(?:\s+(?:" + _LINEAR_FILLERS + r")\b)*")
_CONNECTOR = r"(?:roi|sau\s+do|them|tiep|then|and)"
# "di thang 30 cm roi 40 cm nua": a connector and a distance right after a linear step carry it on.
_LINEAR_CONTINUE_RE = re.compile(r"\b" + _CONNECTOR + r"\s+(?:(?:them|tiep)\s+)?" + _DIST_NUM + r"\s*" + _LINEAR_UNIT)

# A turn phrase is a turn verb followed by any of these parts, in any order: "quay trai 90 do", "quay 90 do
# sang trai", "rotating 180 degree by the left", "xoay tron mot vong nguoc chieu kim dong ho", "quay lai 180
# do". Order matters where two parts could start alike: rev before angle ("2 vong" is two turns, not 2 deg).
_TURN_VERB = r"(?:re|queo|quay|xoay|turn(?:s|ing)?|rotat(?:e|es|ing|ion)|spin(?:s|ning)?|pivot(?:s|ing)?|u[- ]?turn)"
_COUNT = r"(?:\d+(?:[.,]\d+)?|mot|hai|ba|nua|one|two|three|half|a|an)"
_PART_REV, _PART_ANGLE, _PART_CLOCK, _PART_SIDE, _PART_AROUND, _PART_FILLER = (
    "rev", "angle", "clock", "side", "around", "filler")
_TURN_PARTS = {
    _PART_REV: (r"(?:(?:" + _COUNT + r"\s+)?(?:(?:full|half)\s+)?(?:vong(?:\s+tron)?|circles?|revolutions?)"
            r"|(?:" + _COUNT + r"\s+(?:(?:full|half)\s+)?|(?:full|half)\s+)turns?)"),
    _PART_ANGLE: r"\d+(?:[.,]\d+)?(?:\s*(?:do|deg(?:rees?)?)\b|\s*°)?(?!\s*(?:mm|cm|centimet|phan|met|m\b))",
    _PART_CLOCK: (r"(?:(?:theo\s+)?(?:(?:nguoc|cung|thuan)\s+)?chieu\s+kim\s+dong\s+ho"
              r"|(?:counter|anti)[- ]?clockwise|clockwise)"),
    _PART_SIDE: (r"(?:(?:sang|ve|qua|theo|to|by|towards?|on)\s+)?(?:the\s+)?(?:(?:phia|ben|huong)\s+)?"
             r"(?:trai|phai|left|right)(?:\s+(?:side|hand))?"),
    _PART_AROUND: r"(?:nguoc\s+lai|dau|lai|around|back)",
    _PART_FILLER: r"(?:tai\s+cho|in\s+place|on\s+the\s+spot|tron|mot\s+goc|goc|khoang|about|a|an)",
}
_PART_END = r"(?![a-z0-9])"
_TURN_PART_RE = re.compile("|".join(f"(?P<{name}>{pattern}){_PART_END}" for name, pattern in _TURN_PARTS.items()))
_TURN_RE = re.compile(r"\b(?P<verb>" + _TURN_VERB + r")" + _PART_END + r"(?P<parts>(?:\s+(?:"
                      + "|".join(f"(?:{pattern}){_PART_END}" for pattern in _TURN_PARTS.values()) + r"))*)")
# "xoay mot vong roi nua vong nguoc chieu kim dong ho": after a turn, a connector followed by an amount carries
# the turn on without repeating the verb. Taken only right after a turn step, and only with an amount.
_TURN_CONTINUE_RE = re.compile(r"\b" + _CONNECTOR + r"\s+(?P<parts>(?:\s*(?:"
                               + "|".join(f"(?:{pattern}){_PART_END}" for pattern in _TURN_PARTS.values()) + r"))+)")
_COUNT_NUMBER_RE = re.compile(r"\d+(?:[.,]\d+)?|[a-z]+")
_COUNT_RE = re.compile(_COUNT)
_DIGIT_RE = re.compile(r"\d")
_LEFT_CLOCK_RE = re.compile(r"nguoc|counter|anti")
_RIGHT_SIDE_RE = re.compile(r"\b(?:phai|right)\b")
_UTURN_PREFIX = "u"
# "de tro ve vi tri ban dau", "to get back to the base angle": why the user asked for the steps before it,
# not a step. Without the purpose word in front, "quay lai vi tri ban dau" is a request to go back there, which
# relative steps cannot do by themselves: asked (QUESTION_RETURN), never read as a U-turn.
_BACK_TO_START = (r"(?:(?:get|go|come|turn)\s+)?"
                  r"(?:tro\s+ve|quay\s+ve|quay\s+lai|tro\s+lai|ve\s+lai|ve|back(?:\s+to)?|return(?:\s+to)?)\s+"
                  r"(?:the\s+)?(?:(?:vi\s+tri|huong|goc|cho|diem|trang\s+thai)\s+)?"
                  r"(?:ban\s+dau|cu|xuat\s+phat|dau\s+tien|start(?:ing)?|base|original|initial|beginning)"
                  r"(?:\s+(?:position|angle|angel|heading|point|pose|place|direction))?\b")
_PURPOSE_RE = re.compile(r"\b(?:de|to|in\s+order\s+to|so\s+as\s+to)\s+" + _BACK_TO_START)
_RETURN_RE = re.compile(r"\b" + _BACK_TO_START)
# A purpose naming a place asks for the position back as well; otherwise ("ve huong ban dau", "base angle",
# a bare "ban dau" after in-place turns) the heading alone.
_POSITION_WORDS_RE = re.compile(r"\b(?:vi\s+tri|cho|diem|xuat\s+phat|position|point|place|pose|start(?:ing)?)\b")
# "di toi cai ghe", "den canh ban", "go next to the suitcase": a place in the room to go to. A move verb and
# a place word, then the target's words up to a connector; "den" (to go to) needs no verb before it, "toi" does
# (folded, it is also "toi", I). Linear moves need a number and a direction word is no place ("di toi truoc
# 30 cm"), so neither ever reads as a goal.
_GOAL_RE = re.compile(r"\b(?:(?:di\s+chuyen|di|chay|lai|tien|go|move|drive|head|navigate|come)\s+|(?=den\s)"
                      r"|(?=toi\s+(?:gan|canh|ben\s+canh|cho|sat)\s))"
                      r"(?:(?:toi|den|lai|ra)\s+(?:(?:gan|canh|ben\s+canh|cho|sat)\s+)?|(?:gan|canh|ben\s+canh|sat)\s+"
                      r"|(?:over\s+|up\s+)?to\s+(?:the\s+)?|towards?\s+(?:the\s+)?|next\s+to\s+(?:the\s+)?"
                      r"|near\s+(?:the\s+)?|beside\s+(?:the\s+)?)"
                      r"(?P<target>[a-z]+(?:\s+[a-z]+){0,5})")
_GOAL_END_RE = re.compile(r"\s+(?:roi|sau|then|and|xong|de|to|nhe|nha|di|giup|please)\b.*$")
_NOT_A_PLACE = frozenset(_LINEAR_FILLERS.split("|")) | {"phia", "huong", "ben", "trai", "phai", "left", "right"}
_SIDE_LEADS, _SIDE_NAMES = ("ben", "phia"), ("trai", "phai", "truoc", "sau")   # "ben trai cai ban": a side of it
_SIDED_PLACE_MIN_WORDS = 3                                                    # the side's two words + the object
# "lap ban do", "quet phong", "scan the room": build the room's map from this run (split_map_request). Neither
# parser knows these words; they are taken out before the rest of the command is parsed.
_MAP_RE = re.compile(r"\b(?:(?:lap|dung|ve|tao)\s+(?:lai\s+|moi\s+)?ban\s+do"
                     r"|quet\s+(?:lai\s+)?(?:phong|xung\s+quanh|ban\s+do)"
                     r"|(?:build|make)\s+(?:a\s+|the\s+)?(?:new\s+)?map|(?:map|scan)\s+the\s+room)\b")
_MAP_CONNECTOR = r"(?:roi|va|sau\s+do|xong|then|and)"
_MAP_LEAD_RE = re.compile(r"^(?:[\s,.;!]|" + _MAP_CONNECTOR + r"\b)*")      # left in front of the rest
_MAP_TRAIL_RE = re.compile(r"(?:[\s,.;!]|\b" + _MAP_CONNECTOR + r")*$")     # left after it
# The rest of a map request made of these words only is no move: the request is the scan alone.
_MAP_FILLER_WORDS = frozenset(("roi", "va", "sau", "do", "xong", "then", "and", "hay", "robot", "xe", "di", "nhe",
                               "nha", "giup", "toi", "minh", "cho", "lai", "moi", "mot", "cai", "ngay", "please", "a",
                               "the", "new", "go", "now"))
# Left over after every match is removed, any of these means a move the rules did not understand.
_LEFTOVER_RE = re.compile(r"\d|\b(?:di|tien|lui|re|queo|quay|xoay|chay|forward|backward|back|turn|rotat\w*|spin\w*|"
                          r"pivot\w*|vong|left|right|trai|phai|go|move|drive|reverse|"
                          r"met(?:er|re)?s?|centimet\w*|cm|mm|deg\w*)\b")
_TOKEN_RE = re.compile(r"\d+(?:[.,]\d+)?|[a-z]+")
_LEFTOVER_STRIP = " .,;"         # separators trimmed off an unmatched fragment before quoting it
# Which regex a rule match came from.
_KIND_PURPOSE = "purpose"
_KIND_RETURN = "return"
_KIND_TURN = "turn"
_KIND_CONTINUE = "continue"
_KIND_LINEAR_CONTINUE = "linear_continue"
_KIND_LINEAR = "linear"


class ParserSetupError(RuntimeError):
    """An explicitly requested LLM provider cannot be used (SDK missing, no API key)."""


class Question(str):
    """A validation outcome only the user can settle (which side?): asked as it is, and the LLM is not
    re-asked for it - a retry cannot invent what the user did not say. step: index of the raw step it is
    about, when there is one."""

    def __new__(cls, text, step=None):
        question = super().__new__(cls, text)
        question.step = step
        return question


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
    """At most one of steps, question, goal is set. steps: relative moves to drive. question: ask it,
    nothing moves. goal: the user's own words for a place in the room to go to ("canh cai ghe") - the
    real room's map resolves and plans it (realroom), this parser never does."""
    steps: tuple = ()
    question: str = ""
    provider: str = ""    # parser that produced it: gemini | openai | anthropic | rule | injected
    raw: object = None    # the parser's raw reply, kept for the run log
    goal: str = ""


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


def _stated_number(value, written):
    """None when value is a number the user wrote, else the error."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return f"value {value!r} is not a number"
    if not any(abs(value - w) <= cfg.NUMBER_MATCH_TOL for w in written):
        return f"value {value:g} does not appear in the command"
    return None


def _turn_degrees(action, value, unit):
    """(degrees, stated) of a raw turn whose value, if any, was checked already; degrees None when the
    turn names no angle and no default applies (a sideless turn: which way, how far?)."""
    if value is None:
        if unit == cfg.REV_UNIT:
            return cfg.DEFAULT_REVS * cfg.ANGLE_UNITS_DEG[cfg.REV_UNIT], False
        if action == ACTION_TURN_AROUND:
            return cfg.TURN_AROUND_DEG, False
        return (None if action == ACTION_TURN else cfg.DEFAULT_TURN_DEG), False
    return value * cfg.ANGLE_UNITS_DEG[unit], True


def _validate_step(item, written):
    """One raw step -> (MotionStep, None) or (None, error); the error is a Question when only the user
    can settle it."""
    if not isinstance(item, dict):
        return None, "not an object"
    action, value, unit = item.get("action"), item.get("value"), item.get("unit")
    if action not in RAW_ACTIONS:
        return None, f"unknown action {action!r}"
    if action in LINEAR_ACTIONS:
        if value is None:
            return None, f"{action} without a distance"
        error = _stated_number(value, written)
        if error is not None:
            return None, error
        if unit not in cfg.LINEAR_UNITS_M:
            return None, f"unit {unit!r} is not a distance unit (mm | cm | m)"
        metres = value * cfg.LINEAR_UNITS_M[unit]
        if not cfg.MIN_LINEAR_M <= metres <= cfg.MAX_LINEAR_M:
            return None, f"{metres:g} m is outside {cfg.MIN_LINEAR_M:g}..{cfg.MAX_LINEAR_M:g} m"
        return MotionStep(action, metres, True), None
    if value is not None:
        error = _stated_number(value, written)
        if error is not None:
            return None, error
    if (value is not None or unit is not None) and unit not in cfg.ANGLE_UNITS_DEG:
        return None, f"unit {unit!r} is not an angle unit (deg | rev)"
    degrees, stated = _turn_degrees(action, value, unit)
    if degrees is None:
        return None, Question(QUESTION_TURN)
    if not cfg.MIN_TURN_DEG <= degrees <= cfg.MAX_TURN_DEG:
        return None, f"{degrees:g} deg is outside {cfg.MIN_TURN_DEG:g}..{cfg.MAX_TURN_DEG:g} deg"
    if action == ACTION_TURN:
        if not any(abs(degrees - a) <= cfg.NUMBER_MATCH_TOL for a in cfg.SIDELESS_TURN_DEG):
            return None, Question(QUESTION_SIDE.format(angle=degrees))
        action = ACTION_TURN_LEFT                # a half or full turn: either side ends the same way
    return MotionStep(action, float(degrees), stated), None


def goal_error(raw, user_text):
    """None when the reply's goal (if any) uses only words the user wrote - the LLM copies a place, never
    names one - else the error. Articles are free: 'the kitchen' for 'go to kitchen'."""
    goal = str(raw.get("goal") or "").strip() if isinstance(raw, dict) else ""
    if not goal:
        return None
    if raw.get("steps"):
        return "a goal and steps in one reply: a place is gone to on its own"
    invented = set(_TOKEN_RE.findall(fold(goal))) - set(_TOKEN_RE.findall(fold(user_text))) - GOAL_FREE_WORDS
    return f"goal words {sorted(invented)} do not appear in the command" if invented else None


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
        if isinstance(error, Question):
            return None, Question(error, step=index - 1)
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


def _number(token):
    """A count token ('2', '0,5', 'mot', 'half') -> float, None for 'a' / 'an' (the code's default)."""
    if token[0].isdigit():
        return float(token.replace(",", "."))
    value = NUMBER_WORDS.get(token)
    return None if value is None else float(value)


def _has_amount(parts):
    """True when a turn phrase's parts include an angle or a number of turns."""
    return any(part.lastgroup in (_PART_ANGLE, _PART_REV) for part in _TURN_PART_RE.finditer(parts))


def _turn_step(match):
    """A turn phrase (or its continuation) -> the raw step the LLM would have produced, or None when its
    parts contradict each other (two sides, two angles): the user has to say which."""
    verb = match.groupdict().get("verb")
    around = verb is not None and verb.startswith(_UTURN_PREFIX)
    sides, amounts = set(), []
    for part in _TURN_PART_RE.finditer(match.group("parts")):
        kind, text = part.lastgroup, part.group(0)
        if kind == _PART_AROUND:
            around = True
        elif kind == _PART_CLOCK:
            sides.add(ACTION_TURN_LEFT if _LEFT_CLOCK_RE.search(text) else ACTION_TURN_RIGHT)
        elif kind == _PART_SIDE:
            sides.add(ACTION_TURN_RIGHT if _RIGHT_SIDE_RE.search(text) else ACTION_TURN_LEFT)
        elif kind == _PART_ANGLE:
            amounts.append((_number(_COUNT_NUMBER_RE.match(text).group(0)), cfg.ANGLE_UNIT))
        elif kind == _PART_REV:
            first = _COUNT_NUMBER_RE.match(text).group(0)
            amounts.append((_number(first) if _COUNT_RE.fullmatch(first) else None, cfg.REV_UNIT))
    if len(sides) > 1 or len(amounts) > 1:
        return None
    value, unit = amounts[0] if amounts else (None, None)
    if around:
        action = ACTION_TURN_AROUND              # a side on a U-turn changes nothing: it ends facing back
    else:
        action = sides.pop() if sides else ACTION_TURN
    return {"action": action, "value": value, "unit": unit}


def _rule_step(kind, match):
    """A regex match -> the raw step the LLM would have produced (None: a turn phrase that contradicts
    itself)."""
    if kind in (_KIND_TURN, _KIND_CONTINUE):
        return _turn_step(match)
    if kind == _KIND_LINEAR_CONTINUE:              # the direction comes from the step before (_continued)
        return {"action": ACTION_FORWARD, "value": _number(match.group(1)), "unit": _linear_unit(match.group(2))}
    words = {match.group(1)} | set(match.group(2).split())
    action = ACTION_BACKWARD if words & _BACKWARD_WORDS else ACTION_FORWARD
    return {"action": action, "value": _number(match.group(3)), "unit": _linear_unit(match.group(4))}


def _source(text, folded=None):
    """The text rule_parse's spans index into with the user's own characters: fold() keeps one character
    per character of the NFC text (it only drops combining marks and maps 'đ'), so the NFC text itself,
    diacritics and all - the folded text in the rare case a character folds into two."""
    original = unicodedata.normalize("NFC", text)
    folded = fold(text) if folded is None else folded
    return original if len(original) == len(folded) else folded


def _quote(text, folded, start, end):
    """The user's own words for folded[start:end]."""
    return _source(text, folded)[start:end].strip(_LEFTOVER_STRIP)


def merge_answer(pending, answer):
    """The pending command with the user's answer put where its question pointed: in place of the part
    not understood (PENDING_REPLACE) or right after the turn that lacks a side (PENDING_AFTER)."""
    text = _source(pending["text"])
    cut = pending["start"] if pending["mode"] == PENDING_REPLACE else pending["end"]
    return f"{text[:cut]} {answer} {text[pending['end']:]}"


def say_step(step):
    """A raw step in the user's words: 'quay trái 180 độ', 'đi thẳng 90 cm', 'quay đầu'."""
    words = SAY_ACTION.get(step.get("action"), str(step.get("action")))
    if step.get("value") is None:
        return words + (f" 1 {SAY_UNIT[cfg.REV_UNIT]}" if step.get("unit") == cfg.REV_UNIT else "")
    return f"{words} {step['value']:g} {SAY_UNIT.get(step.get('unit'), step.get('unit'))}"


def with_understood(question, understood):
    """The question, followed by the steps already understood, so the user answers only what is missing."""
    if not understood:
        return question
    return question + UNDERSTOOD_PREFIX + "; ".join(f"{k}) {say_step(s)}" for k, s in enumerate(understood, 1))


def _continued(step, previous):
    """A continuation keeps going the way the step before it did: a distance in its direction ("... roi 40 cm
    nua"), a turn with no side of its own ("... roi 45 do nua") to its side - a U-turn's being left."""
    if step["action"] in LINEAR_ACTIONS:
        return dict(step, action=previous["action"])
    if step["action"] != ACTION_TURN or previous["action"] == ACTION_TURN:
        return step
    side = ACTION_TURN_LEFT if previous["action"] == ACTION_TURN_AROUND else previous["action"]
    return dict(step, action=side)


def _ask(question, understood, start=None, end=None, mode=PENDING_REPLACE):
    """rule_parse's reply when it has to ask: the steps it did understand, and where the answer goes -
    in place of (start, end), or right after it; total: the steps the command should come to (those plus
    the one not read)."""
    reply = {"steps": [], "question": question, "understood": understood}
    if start is not None:
        reply["pending"] = {"start": start, "end": end, "mode": mode, "total": len(understood) + 1}
    return reply


def _sided_place(words):
    """True for 'ben trai cai ban': an object's side, then the object - a place, though it starts with a
    direction word. 'phia truoc' alone (then a number or nothing) is a direction."""
    return (len(words) >= _SIDED_PLACE_MIN_WORDS and words[0] in _SIDE_LEADS and words[1] in _SIDE_NAMES)


def _find_goal(text, folded):
    """(start, end, the user's words for the target) of a go-to-a-place phrase, or None."""
    for match in _GOAL_RE.finditer(folded):
        target = _GOAL_END_RE.sub("", match.group("target"))
        if target.split()[0] in _NOT_A_PLACE and not _sided_place(target.split()):
            continue
        end = match.start("target") + len(target)
        return match.start(), end, _quote(text, folded, match.start("target"), end)
    return None


def split_map_request(text):
    """(True, the rest of the command) when text asks for the room's map ("lập bản đồ", "quét phòng", ...): every
    such phrase and the connectors left at either end taken out, and "" when nothing but filler words remain (the
    request is then the scan alone, MAP_SCAN). (False, text) otherwise."""
    folded = fold(text)
    spans = [(m.start(), m.end()) for m in _MAP_RE.finditer(folded)]
    if not spans:
        return False, text
    source = _source(text, folded)
    for start, end in spans:                     # blanked, not cut: the spans keep their places
        source = source[:start] + " " * (end - start) + source[end:]
        folded = folded[:start] + " " * (end - start) + folded[end:]
    lead = _MAP_LEAD_RE.match(folded).end()
    trail = max(_MAP_TRAIL_RE.search(folded).start(), lead)
    rest = " ".join(source[lead:trail].split())
    logger.debug("map request at %s in %r, the rest %r", spans, text, rest)
    if all(word in _MAP_FILLER_WORDS for word in _TOKEN_RE.findall(fold(rest))):
        return True, ""
    return True, rest


def map_scan_steps():
    """MAP_SCAN as MotionSteps - amounts the code chose, like a default turn."""
    return tuple(MotionStep(action, amount, stated=False) for action, amount in MAP_SCAN)


def rule_parse(text):
    """Offline parser. Returns the raw reply shape - every step with its "span" in the text; on a question
    also "understood" (the steps it did read) and "pending" (where an answer goes: PENDING_*); for a place
    to go to, "goal" (its words) and no steps."""
    folded = fold(text)
    goal = _find_goal(text, folded)
    if goal is not None:
        start, end, words = goal
        source = _source(text, folded)
        rest = rule_parse(source[:start] + " " * (end - start) + source[end:])   # the command without the place
        if rest["steps"] or rest.get("goal") or rest["question"] != QUESTION_NO_MATCH:
            # Any other move with it - a step, a second place, or one the rules could not read ("... roi di
            # thang") - would be dropped by going to the place alone: asked to be said apart instead.
            return _ask(QUESTION_GOAL_AND_STEPS.format(goal=words), [])
        return {"steps": [], "question": "", "goal": words}
    found = []
    for kind, pattern in ((_KIND_PURPOSE, _PURPOSE_RE), (_KIND_RETURN, _RETURN_RE), (_KIND_TURN, _TURN_RE),
                          (_KIND_CONTINUE, _TURN_CONTINUE_RE), (_KIND_LINEAR, _LINEAR_RE),
                          (_KIND_LINEAR_CONTINUE, _LINEAR_CONTINUE_RE)):
        found += [(m.start(), -(m.end() - m.start()), kind, m) for m in pattern.finditer(folded)]
    found.sort(key=lambda f: (f[0], f[1]))       # by position; the longest wins a shared start

    steps, spans, covered_to = [], [], 0
    for start, _, kind, match in found:
        if start < covered_to:
            continue                             # overlaps a match already taken
        if kind == _KIND_CONTINUE and not (steps and steps[-1]["action"] in TURN_ACTIONS
                                           and _has_amount(match.group("parts"))):
            continue                             # a connector that does not carry a turn on
        if kind == _KIND_LINEAR_CONTINUE and not (steps and steps[-1]["action"] in LINEAR_ACTIONS):
            continue                             # a distance with nothing to carry on: left over, asked
        covered_to = match.end()
        if kind == _KIND_RETURN:
            return _ask(QUESTION_RETURN.format(part=_quote(text, folded, start, match.end())), steps, start, match.end())
        if kind == _KIND_PURPOSE:
            spans.append((start, match.end()))   # understood, and not a step
            continue
        step = _rule_step(kind, match)
        if step is None:
            return _ask(QUESTION_LEFTOVER.format(part=_quote(text, folded, start, match.end())), steps, start,
                        match.end())
        if kind in (_KIND_CONTINUE, _KIND_LINEAR_CONTINUE):
            step = _continued(step, steps[-1])
        steps.append(dict(step, span=[start, match.end()]))
        spans.append((start, match.end()))

    gaps, cursor = [], 0
    for start, end in spans:
        gaps.append((cursor, start))
        cursor = end
    gaps.append((cursor, len(folded)))
    for start, end in gaps:
        if not _LEFTOVER_RE.search(folded[start:end]):
            continue
        bare = _LINEAR_BARE_RE.search(folded, start, end)
        if bare is not None and not _DIGIT_RE.search(folded[start:end]):
            return _ask(QUESTION_DISTANCE.format(part=_quote(text, folded, bare.start(), bare.end())), steps,
                        bare.start(), bare.end(), PENDING_AFTER)
        return _ask(QUESTION_LEFTOVER.format(part=_quote(text, folded, start, end)), steps, start, end)
    if not steps:
        return _ask(QUESTION_NO_MATCH, [])
    return {"steps": steps, "question": ""}


def track_steps(steps):
    """(heading_deg, x_m, y_m) after each MotionStep, from where the command starts (0, 0, 0): the plan's own
    dead reckoning, forward along the heading, turns CCW positive."""
    heading, x, y, track = 0.0, 0.0, 0.0, []
    for step in steps:
        if step.action in LINEAR_SIGN:
            distance = LINEAR_SIGN[step.action] * step.amount
            x += distance * math.cos(math.radians(heading))
            y += distance * math.sin(math.radians(heading))
        else:
            heading += TURN_SIGN[step.action] * step.amount
        track.append((heading, x, y))
    return track


def intents(text):
    """(kind, words) of every purpose clause the user gave ("de tro ve vi tri ban dau"): INTENT_POSITION when
    it names a place, else INTENT_HEADING; words are the user's own."""
    folded = fold(text)
    return [(INTENT_POSITION if _POSITION_WORDS_RE.search(m.group(0)) else INTENT_HEADING,
             _quote(text, folded, m.start(), m.end())) for m in _PURPOSE_RE.finditer(folded)]


def check_intents(steps, text):
    """(ok, step_number, words) per purpose clause in text: the first step after which the plan is back at
    the start (heading within INTENT_HEADING_TOL_DEG, and the centre within INTENT_POSITION_TOL_M for a
    position), step_number None when none is. Only reports: the steps stay what the user said."""
    track = track_steps(steps)
    report = []
    for kind, words in intents(text):
        match = None
        for number, (heading, x, y) in enumerate(track, 1):
            off = abs((heading + FULL_TURN_DEG / 2) % FULL_TURN_DEG - FULL_TURN_DEG / 2)
            if off <= INTENT_HEADING_TOL_DEG and (kind == INTENT_HEADING or math.hypot(x, y) <= INTENT_POSITION_TOL_M):
                match = number
                break
        report.append((match is not None, match, words))
    return report


def load_prompt(path=cfg.PROMPT_FILE):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


class QuotaFallbackClient:
    """An LLM client that asks `primary` and, once primary is out of quota or overloaded (a BUSY_MARKS error),
    `secondary` instead - for the next cfg.QUOTA_COOLDOWN_S, across every parser of this process (a chat makes one per
    command). Any other error surfaces as it is. model: the model that answers now."""

    _out_of_quota_until = {}                     # model -> time.time() it is asked again

    def __init__(self, primary, secondary, error_type):
        self.primary, self.secondary, self._error_type = primary, secondary, error_type

    def _primary_resting(self):
        return time.time() < self._out_of_quota_until.get(self.primary.model, 0.0)

    @property
    def model(self):
        return self.secondary.model if self._primary_resting() else self.primary.model

    def complete(self, system, messages, schema):
        if not self._primary_resting():
            try:
                return self.primary.complete(system, messages, schema)
            except self._error_type as exc:
                if not any(mark in str(exc) for mark in BUSY_MARKS):
                    raise
                self._out_of_quota_until[self.primary.model] = time.time() + cfg.QUOTA_COOLDOWN_S
                logger.warning("LLM %s out of quota / overloaded (%s): %s answers for %.0f s", self.primary.model,
                               exc, self.secondary.model, cfg.QUOTA_COOLDOWN_S)
        return self.secondary.complete(system, messages, schema)


class CommandParser:
    """Text -> ParseResult. provider: auto (CM's configured LLM, falling back to the rules when it
    cannot be reached), rule, or one of LLM_PROVIDERS (errors surface, never a silent fallback)."""

    def __init__(self, provider=PROVIDER_AUTO, client=None, prompt=None):
        self.prompt = prompt or load_prompt()
        self._fallback = provider == PROVIDER_AUTO
        self.history = []                        # LLM turns [{"role", "content"}]
        self.user_texts = []                     # every user message of the current exchange
        self._llm_error = Exception              # narrowed by _make_client to what the client raises
        self._pending = None                     # rules: the command a question waits on ("pending" + its text)
        self._rules_only = False                 # the LLM failed during this exchange: the rules finish it
        self.llm_failure = ""                    # why the rules read the last command in the LLM's place ("" = no)
        self.provider, self._client = self._make_client(provider, client)

    @property
    def model(self):
        """The LLM model that answers now (e.g. gemini-2.5-flash, its fallback once out of quota); "" for the rules."""
        return getattr(self._client, "model", "")

    @property
    def llm(self):
        """The LLM client this parser talks to, None when it has only the rules - for the map's grounding."""
        return self._client

    def reset(self):
        self.history = []
        self.user_texts = []
        self._pending = None
        self._rules_only = False

    def parse(self, text):
        self.user_texts.append(text)
        if self._client is None or self._rules_only:
            result = self._parse_rules(text)
        else:
            try:
                result = self._parse_llm(text)
            except self._llm_error as exc:
                if not self._fallback:
                    raise
                logger.warning("LLM %s failed (%s): falling back to the rule parser", self.provider, exc)
                self.llm_failure = str(exc)
                # The rules read every message of this exchange again, in order, so an answer still lands
                # in the command it answers; they also finish the exchange - the LLM's half of it is gone.
                texts = list(self.user_texts)
                self.reset()
                self._rules_only = True
                for message in texts:
                    result = self._parse_rules(message)
                    if result.steps or result.goal:
                        break                    # read complete: later messages cannot add to it
        if result.steps or result.goal:
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
            name = llm_client.config.PROVIDER if provider == PROVIDER_AUTO else provider
            made = llm_client.make_client(name)
            fallback = os.environ.get(cfg.GEMINI_FALLBACK_ENV, cfg.GEMINI_FALLBACK_MODEL)   # .env loaded by now
            if name == PROVIDER_GEMINI and fallback and fallback != made.model:
                made.client = made.client.with_options(max_retries=cfg.GEMINI_PRIMARY_RETRIES)
                second = llm_client.GeminiClient(model=fallback)
                second.extra_body = cfg.GEMINI_FALLBACK_EXTRA_BODY
                made = QuotaFallbackClient(made, second, llm_client.LLMError)
            return name, made
        except Exception as exc:                 # missing SDK or dotenv, missing API key
            if provider != PROVIDER_AUTO:
                raise ParserSetupError(f"LLM provider {provider!r} unusable: {exc}") from exc
            logger.warning("no LLM available (%s): using the rule parser", exc)
            self.llm_failure = NO_LLM.format(error=exc)      # the chat says the rules read every command
            return PROVIDER_RULE, None

    # ------------------------------------------------------------ parsers
    def _parse_rules(self, text):
        """The rules, with context. After a question the answer is tried, in order: as a whole new command
        when it has at least as many steps as the pending one (the user said it all again); put where the
        question pointed (merge_answer); for a side question, in place of the whole turn (an answer with its
        own verb: 'quay trai 90 do'). When none reads, the merged command stays pending, so the next answer
        builds on this one. No reading with fewer steps than the pending command is taken: an answer must
        never drop a step the user asked for."""
        pending, self._pending = self._pending, None
        if pending is None:
            result, self._pending = self._rule_result(text)
            return result
        alone, _ = self._rule_result(text)
        if len(alone.steps) >= pending["total"]:
            return alone
        merged, merged_pending = self._rule_result(merge_answer(pending, text))
        if len(merged.steps) >= pending["total"]:
            return merged
        if pending["mode"] == PENDING_AFTER:
            replaced, _ = self._rule_result(merge_answer(dict(pending, mode=PENDING_REPLACE), text))
            if len(replaced.steps) >= pending["total"]:
                return replaced
        self._pending = merged_pending or pending
        return merged

    def _rule_result(self, text):
        """(ParseResult, pending) of the rules on text; pending says where an answer to its question goes
        (None: nowhere in particular - the answer is read on its own)."""
        raw = rule_parse(text)
        if raw.get("goal"):
            return ParseResult(goal=raw["goal"], provider=PROVIDER_RULE, raw=raw), None
        if raw["question"]:
            spot = raw.get("pending")
            question = with_understood(raw["question"], raw.get("understood", []))
            return (ParseResult(question=question, provider=PROVIDER_RULE, raw=raw),
                    None if spot is None else dict(spot, text=text))
        steps, error = validate_steps(raw, text)
        if isinstance(error, Question):
            pending, understood = None, []
            if error.step is not None:
                start, end = raw["steps"][error.step]["span"]
                pending = {"text": text, "start": start, "end": end, "mode": PENDING_AFTER,
                           "total": len(raw["steps"])}
                understood = raw["steps"][:error.step]
            return ParseResult(question=with_understood(error, understood), provider=PROVIDER_RULE, raw=raw), pending
        if error is not None:
            return ParseResult(question=QUESTION_INVALID.format(error=error), provider=PROVIDER_RULE, raw=raw), None
        return ParseResult(steps=steps, provider=PROVIDER_RULE, raw=raw), None

    def _parse_llm(self, text):
        self.history.append({"role": "user", "content": text})
        exchange = "\n".join(self.user_texts)
        raw, steps, error = None, (), None
        for attempt in range(cfg.MAX_VALIDATION_RETRIES + 1):
            raw = self._client.complete(self._system_prompt(), self._messages(error),
                                        self.prompt["output_schema"])
            steps, error = validate_steps(raw, exchange, ordered=len(self.user_texts) == 1)
            error = error or goal_error(raw, exchange)
            if error is None or isinstance(error, Question):
                break
            logger.warning("LLM reply rejected (attempt %d): %s", attempt + 1, error)
        content = json.dumps(raw, ensure_ascii=False)
        if isinstance(error, Question):
            # The LLM reads its own reply back next turn: note what was asked, so the answer lands in context
            # (in the same message - two assistant turns in a row are refused by some endpoints).
            content += "\n" + ASKED_NOTE.format(question=error)
        self.history.append({"role": "assistant", "content": content})
        if isinstance(error, Question):
            return ParseResult(question=error, provider=self.provider, raw=raw)
        if error is not None:
            return ParseResult(question=QUESTION_INVALID.format(error=error), provider=self.provider, raw=raw)
        goal = str(raw.get("goal") or "").strip()
        if goal:
            return ParseResult(goal=goal, provider=self.provider, raw=raw)
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
