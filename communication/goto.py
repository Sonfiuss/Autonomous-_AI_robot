"""A place to go to -> the real room's map -> the legs to drive: the "goal" branch of L2
(agent/description/robot_process_llm.md; task 2026-09-28_natural-command step 11).

  ground(scene, words, llm, ask)  which spots beside which object, best first. With an LLM: CM's
                                  GroundingSession on realroom_prompt() - it asks only which object (in the
                                  user's language); once the object is known the robot picks: the side and the
                                  spot the user named, else the middle of a side, nearest the robot first
                                  (order_spots). Without one, or when it fails: the rules - an object class
                                  named in the words (CLASS_WORDS) and its color, a question when the map holds
                                  several, the spot nearest the robot (the map's sides are the object's own
                                  front / left / ..., not the speaker's, so a side word is only honoured when it
                                  names one of those).
  route(scene, spot)              realroom's planner on the map's known grid -> RoomPlan.
  route_first(scene, spots)       the first of the spots a route reaches -> (spot, RoomPlan, [(spot, why not)]).
The LLM never sees coordinates and never picks one: it only chooses a spot id the map offers.
"""
import logging
import math

import drive_config as cfg
from cmd_parser import fold

cfg.add_cm_to_path()
cfg.add_realroom_to_path()
import map_store  # noqa: E402  (realroom module, path set just above)
import planner  # noqa: E402
from candidates import build_candidates  # noqa: E402  (CM module)
from dialog import GroundingSession, load_prompt  # noqa: E402

UNKNOWN_CLASS = "unknown"             # vision/scene_export: an object the detector could not name
# Words a user may name an object by (diacritic-folded) -> the detector's class (vision/detector: COCO).
CLASS_WORDS = {
    "ghe sofa": "couch", "sofa": "couch", "couch": "couch", "ghe": "chair", "chair": "chair",
    "vali": "suitcase", "va li": "suitcase", "suitcase": "suitcase", "ban": "dining table", "table": "dining table",
    "giuong": "bed", "bed": "bed", "tivi": "tv", "ti vi": "tv", "tv": "tv", "may tinh": "laptop", "laptop": "laptop",
    "chai": "bottle", "bottle": "bottle", "coc": "cup", "ly": "cup", "cup": "cup", "chau cay": "potted plant",
    "cay": "potted plant", "plant": "potted plant", "tu lanh": "refrigerator", "fridge": "refrigerator",
    "refrigerator": "refrigerator", "nguoi": "person", "person": "person", "balo": "backpack", "backpack": "backpack",
}
SIDE_WORDS = {"truoc": "front", "front": "front", "trai": "left", "left": "left", "sau": "back", "back": "back",
              "phai": "right", "right": "right"}
# A color the user names right after the object ("ghe xanh") or after "mau" ("cai ghe mau xanh"), folded, or an
# English color anywhere -> the map's color names (vision/object_color.COLOR_NAMES). "xanh" alone is green or blue.
COLOR_WORDS = {
    "xanh la cay": ("green",), "xanh la": ("green",), "xanh luc": ("green",), "xanh duong": ("blue",),
    "xanh nuoc bien": ("blue",), "xanh da troi": ("blue",), "xanh": ("green", "blue"), "do": ("red",),
    "den": ("black",), "trang": ("white",), "xam": ("gray",), "ghi": ("gray",), "vang": ("yellow",),
    "cam": ("orange",), "tim": ("purple",), "hong": ("pink",), "nau": ("brown",),
}
ENGLISH_COLORS = ("black", "white", "gray", "grey", "red", "orange", "yellow", "green", "blue", "purple", "pink",
                  "brown")
COLOR_LEAD = "mau"                    # "(cai ghe) mau xanh": a Vietnamese color follows this word
COLOR_MAX_WORDS = 3                   # the longest COLOR_WORDS phrase
# The chat's Vietnamese for the map's classes, colors and an object's own sides (describe()).
VI_CLASS = {"couch": "ghế sofa", "chair": "ghế", "suitcase": "vali", "dining table": "bàn", "bed": "giường",
            "tv": "tivi", "laptop": "máy tính", "bottle": "chai", "cup": "cốc", "potted plant": "chậu cây",
            "refrigerator": "tủ lạnh", "person": "người", "backpack": "balo", "toilet": "bồn cầu",
            "bench": "ghế dài", UNKNOWN_CLASS: "vật"}
VI_COLOR = {"black": "đen", "white": "trắng", "gray": "xám", "red": "đỏ", "orange": "cam", "yellow": "vàng",
            "green": "xanh lá", "blue": "xanh dương", "purple": "tím", "pink": "hồng", "brown": "nâu"}
VI_SIDE = {"front": "trước", "back": "sau", "left": "trái", "right": "phải"}
QUESTION_NO_COLOR = "Bản đồ không có {cls} màu {color}; có: {options}."
NOTE_RULES = "(LLM không chọn được chỗ: {why} - bộ luật chọn)"
BUSY_WORDS = {"429": "hết quota", "503": "quá tải"}   # an HTTP status in the LLM's error -> the chat's words
ERROR_CHARS, ELLIPSIS = 80, "…"                      # a longer LLM error is cut to this in the chat
SPOT_WORDS = "cạnh {side} của {what}"                # describe()
OBJECT_WORDS = "{cls}{color} ({id})"                 # object_name(); color with its leading space, or ""
ENGLISH_ALIASES = {"grey": "gray"}
GROUND_GOAL, GROUND_ASK, GROUND_NONE, GROUND_ERROR = "goal", "ask", "none", "error"   # CM dialog reply types
QUESTION_NOTHING = "Bản đồ thật không có '{words}'. Trên bản đồ có: {classes}."
QUESTION_WHICH = "Bản đồ có {n} {cls}: {options}. Cái nào (gõ số)?"
QUESTION_NO_SPOT = "Không có chỗ nào cạnh {cls} mà robot đứng được."
REASON_CANCELLED = "đã huỷ"
REASON_NO_CHOICE = "không chọn được vật"
NO_NAMED_CLASS = "(chỉ vật chưa rõ loại)"

# realroom's grounding: CM's prompt (shared with the simulation, unchanged on disk) with these rules in place of
# the ones that ask about sides and corners - the robot picks them - and questions in the user's language.
PROMPT_RULE_SWAPS = {
    "If the target has several free sides": (
        "If the target has several free sides and the user did not say which, do NOT ask: answer type 'ask' with "
        "the target filled, side null, question empty - the robot goes to the nearest side it can reach."),
    "If the chosen side has several spots": (
        "Never ask where along a side (corner or middle): leave spot null unless the user named a corner - the "
        "robot goes to the middle."),
    "Answer with ONE JSON object only": (
        "Answer with ONE JSON object only, no prose outside the JSON. 'question' is one short sentence in the "
        "user's language: Vietnamese whenever the user writes any Vietnamese."),
}
PROMPT_EXAMPLES = [   # realroom's own: Vietnamese, and no side / corner questions
    {"user": "đi tới cạnh cái giường",
     "assistant": {"type": "ask", "question": "", "slots": {"target": "obj_1", "side": None, "spot": None},
                   "matches": ["obj_1:*"]}},
    {"user": "đến bên phải cái bàn",
     "assistant": {"type": "ask", "question": "", "slots": {"target": "obj_4", "side": "right", "spot": None},
                   "matches": ["obj_4:right:*"]}},
    {"user": "tới chỗ cái ghế",
     "assistant": {"type": "ask", "question": "Có 2 ghế: ghế đỏ gần cửa sổ (obj_2) hay ghế xanh gần giường (obj_5)?",
                   "slots": {"target": None, "side": None, "spot": None}, "matches": ["obj_2:*", "obj_5:*"]}},
    {"user": "đi tới cái đàn piano",
     "assistant": {"type": "none", "question": "Bản đồ không có đàn piano. Có: giường, bàn, ghế.",
                   "slots": {"target": None, "side": None, "spot": None}, "matches": []}},
]
CENTER_SPOT = "center"                # candidates.py: the middle of a side
MAX_ROUTE_TRIES = 6                   # spots route_first plans before giving up (each is one MV + grid check)

logger = logging.getLogger(__name__)


def _class_phrase(words):
    """(detector class, the folded words after its phrase) the words name (longest phrase first), or (None, "")."""
    folded = f" {fold(words)} "
    for phrase in sorted(CLASS_WORDS, key=len, reverse=True):
        at = folded.find(f" {phrase} ")
        if at >= 0:
            return CLASS_WORDS[phrase], folded[at + len(phrase) + 2:].split()
    return None, []


def _named_class(words):
    """The detector class the words name, or None."""
    return _class_phrase(words)[0]


def _color_at(folded, k):
    """The map colors a Vietnamese color phrase starting at folded[k] names, or None."""
    after = " ".join(folded[k:k + COLOR_MAX_WORDS]) + " "
    for phrase in sorted(COLOR_WORDS, key=len, reverse=True):
        if after.startswith(f"{phrase} "):
            return COLOR_WORDS[phrase]
    return None


def _named_colors(words):
    """The map color names the words ask for ("ghe xanh", "mau xanh la", "green"), or None when they name none."""
    folded = fold(words).split()
    after_class = _class_phrase(words)[1]
    if after_class:
        start = 1 if after_class[0] == COLOR_LEAD else 0
        colors = _color_at(after_class, start)
        if colors is not None:
            return colors
    for k, word in enumerate(folded):
        if word == COLOR_LEAD:
            colors = _color_at(folded, k + 1)
            if colors is not None:
                return colors
        if word in ENGLISH_COLORS:
            return (ENGLISH_ALIASES.get(word, word),)
    return None


def object_name(obj):
    """An object in the chat's words: 'ghế xanh lá (obj_16)'."""
    color = VI_COLOR.get(obj.get("color"))
    return OBJECT_WORDS.format(cls=VI_CLASS.get(obj["class"], obj["class"]), color=f" {color}" if color else "",
                               id=obj["id"])


def describe(scene, spot):
    """A spot in the chat's words: 'cạnh sau của ghế xanh lá (obj_16)'."""
    target = next((o for o in scene["objects"] if o["id"] == spot["target_id"]), None)
    what = object_name(target) if target is not None else spot["target_id"]
    return SPOT_WORDS.format(side=VI_SIDE.get(spot["side"], spot["side"]), what=what)


def _nearest(spots, pose):
    """The spot closest to pose (x, y, theta)."""
    return min(spots, key=lambda s: math.hypot(s["x"] - pose[0], s["y"] - pose[1]))


def ground_by_rules(scene, words, ask):
    """(spot, None) for the words, or (None, why not). ask(question) -> the user's answer ('' = gave up)."""
    cls = _named_class(words)
    objects = [o for o in scene["objects"] if o["class"] == cls]
    if not objects:
        classes = sorted({o["class"] for o in scene["objects"] if o["class"] != UNKNOWN_CLASS}) or [NO_NAMED_CLASS]
        return None, QUESTION_NOTHING.format(words=words, classes=", ".join(classes))
    colors = _named_colors(words)
    logger.debug("rules: %r names %s, colors %s: %d on the map", words, cls, colors, len(objects))
    if colors is not None:
        colored = [o for o in objects if o.get("color") in colors]
        if not colored:
            return None, QUESTION_NO_COLOR.format(cls=cls, color=" / ".join(VI_COLOR[c] for c in colors),
                                                  options=", ".join(object_name(o) for o in objects))
        objects = colored
    target = objects[0]
    if len(objects) > 1:
        options = ", ".join(f"{k}) {o['id']} gần {', '.join(o['near'][:1]) or '-'}" for k, o in enumerate(objects, 1))
        answer = ask(QUESTION_WHICH.format(n=len(objects), cls=cls, options=options)).strip()
        if not answer.isdigit() or not 1 <= int(answer) <= len(objects):
            return None, REASON_NO_CHOICE
        target = objects[int(answer) - 1]
    spots = [s for s in build_candidates(scene) if s["target_id"] == target["id"]]
    if not spots:
        return None, QUESTION_NO_SPOT.format(cls=cls)
    folded = fold(words).split()
    named = {SIDE_WORDS[w] for w in folded if w in SIDE_WORDS}
    sided = [s for s in spots if s["side"] in named] or spots
    return _nearest(sided, map_store.robot_pose(scene)), None


def realroom_prompt():
    """CM's grounding prompt with realroom's rules (PROMPT_RULE_SWAPS) and examples (PROMPT_EXAMPLES)."""
    prompt = load_prompt()
    rules = list(prompt["rules"])
    for start, rule in PROMPT_RULE_SWAPS.items():
        at = [k for k, r in enumerate(rules) if r.startswith(start)]
        if at:
            rules[at[0]] = rule
        else:                               # CM's prompt changed: the rule still reaches the LLM
            logger.warning("CM grounding prompt has no rule starting %r: realroom's is appended", start)
            rules.append(rule)
    return dict(prompt, rules=rules, examples=PROMPT_EXAMPLES)


def order_spots(spots, pose):
    """The spots best first: the middle of a side before its corners, then nearest pose (x, y, theta)."""
    return sorted(spots, key=lambda s: (s["spot"] != CENTER_SPOT, math.hypot(s["x"] - pose[0], s["y"] - pose[1])))


def with_alternatives(scene, first):
    """[first] + the target's other spots (order_spots): where to go when no route reaches first."""
    others = [s for s in build_candidates(scene) if s["target_id"] == first["target_id"] and s["id"] != first["id"]]
    return [first] + order_spots(others, map_store.robot_pose(scene))


def ground(scene, words, llm, ask, say=None):
    """(spots best first, None) or (None, why not): CM's grounding dialogue with the LLM, the rules without one or
    when the LLM fails (then say(note), when given, tells the user). ask(question) -> the user's answer
    ('' = gave up). The LLM is asked only which object; once it is known the robot picks the side and spot."""
    if llm is not None:
        session = GroundingSession(scene, llm, realroom_prompt())
        reply = session.say(words)
        while reply["type"] == GROUND_ASK:
            if session.slots["target"] is not None:     # the object is known: side / spot are the robot's pick
                logger.debug("grounding: %s known, %d spots left - the robot picks", session.slots["target"],
                             len(reply["matches"]))
                return order_spots(reply["matches"], map_store.robot_pose(scene)), None
            answer = ask(reply["text"]).strip()
            if not answer:
                return None, REASON_CANCELLED
            reply = session.say(answer)
        if reply["type"] == GROUND_GOAL:
            return with_alternatives(scene, session.resolved_goal()), None
        if reply["type"] == GROUND_NONE:
            return None, reply["text"]
        logger.warning("grounding LLM failed (%s): the rules pick the spot", reply["text"])
        if say is not None:
            say(NOTE_RULES.format(why=short_error(reply["text"])))
    spot, why = ground_by_rules(scene, words, ask)
    return (None, why) if spot is None else (with_alternatives(scene, spot), None)


def route_first(scene, spots, limit=MAX_ROUTE_TRIES):
    """(spot, RoomPlan, [(spot, why not) of those tried before]) for the first of spots a route reaches, or
    (None, None, the tries) when none of the first `limit` does."""
    tried, grid = [], map_store.load_grid(scene)   # one read of the grid for every try
    for spot in spots[:limit]:
        found = route(scene, spot, grid)
        if found.ok:
            return spot, found, tried
        tried.append((spot, found.reason))
    return None, None, tried


def short_error(text):
    """An LLM error for the chat: out of quota / overloaded in a few words, anything else cut short."""
    for status, said in BUSY_WORDS.items():
        if status in text:
            return said
    return text if len(text) <= ERROR_CHARS else text[:ERROR_CHARS] + ELLIPSIS


def route(scene, spot, grid=None):
    """realroom.planner's RoomPlan from the robot's pose in scene to spot (grid: scene's known grid, read when
    not given)."""
    grid = map_store.load_grid(scene) if grid is None else grid
    return planner.plan(scene, grid, (spot["x"], spot["y"], spot["theta"]))
