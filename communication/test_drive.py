"""Offline tests for the drive demo: rule parser, validator, LLM exchange (scripted fake), motion plan.

Run:  python3 communication/test_drive.py
No API key, camera or robot needed. The motion-plan tests need libmc.so (project/tools/build_mc.sh)
and are skipped without it.
"""
import json
import math
import os
import re

import drive_config as cfg
from cmd_parser import ASKED_NOTE, CommandParser, Question, check_intents, fold, validate_steps
from motion_plan import PlanError, build_plan

cfg.add_cm_to_path()
from llm_client import FakeClient, LLMError  # noqa: E402  (CM module, path set just above)

EXAMPLE = "đi thẳng 30 cm. sau đó rẽ trái, đi tiếp 60 cm"
TOL = 1e-9
PULSE_REL_TOL = 1e-3             # MC's summed pulses vs the closed form; a stale libmc is 27 % off
PLAN_LINE_RE = re.compile(r"^\s*(ROTATE|FORWARD|MOVE|STOP)\b")   # what MissionRunner's reader accepts
CORPUS_FILE = os.path.join(cfg.COMM_DIR, "natural_commands.json")
AMOUNT_TOL = 1e-6


def load_corpus():
    """natural_commands.json's cases (shared with test_live_parser.py)."""
    with open(CORPUS_FILE, encoding="utf-8") as f:
        return json.load(f)["cases"]


def run_case(parser, case):
    """The case's text (or dialogue turns) through parser: (ParseResult of the last message, all text)."""
    turns = case.get("turns") or [case["text"]]
    result = None
    for text in turns:
        result = parser.parse(text)
        if result.steps:
            break
    parser.reset()
    return result, " / ".join(turns)


def case_error(case, result):
    """None when the parser's result is what the case expects, else what went wrong."""
    got = [(s.action, round(s.amount, 6)) for s in result.steps]
    if case.get("asks"):
        return None if not result.steps and result.question else f"moved {got}, expected a question"
    if "goal" in case:
        ok = not result.steps and fold(case["goal"]) in fold(result.goal)
        return None if ok else f"got {got or result.question or result.goal!r}, want the goal {case['goal']!r}"
    want = [(a, round(v, 6)) for a, v in case["steps"]]
    return None if got == want else f"got {got or result.question!r}, want {want}"


def _summary(steps):
    return [(s.action, round(s.amount, 6), s.stated) for s in steps]


def _reply(*steps, question=""):
    return {"steps": [{"action": a, "value": v, "unit": u} for a, v, u in steps], "question": question}


class FailingClient:
    def complete(self, system, messages, schema):
        raise LLMError("network down")


def test_rule_parser_example():
    result = CommandParser("rule").parse(EXAMPLE)
    assert result.provider == "rule" and not result.question, result
    assert _summary(result.steps) == [("forward", 0.3, True), ("turn_left", 90.0, False),
                                      ("forward", 0.6, True)], _summary(result.steps)


def test_rule_parser_variants():
    cases = {
        "lùi 0,5 mét rồi quay phải 45 độ": [("backward", 0.5, True), ("turn_right", 45.0, True)],
        "quay 30 độ sang trái rồi tiến lên 1 m": [("turn_left", 30.0, True), ("forward", 1.0, True)],
        "quay đầu, đi 20cm": [("turn_around", 180.0, False), ("forward", 0.2, True)],
        "di thang 30 cm roi re phai": [("forward", 0.3, True), ("turn_right", 90.0, False)],
        "go forward 50 cm then turn right and move back 10 cm":
            [("forward", 0.5, True), ("turn_right", 90.0, False), ("backward", 0.1, True)],
    }
    parser = CommandParser("rule")
    for text, expected in cases.items():
        result = parser.parse(text)
        assert _summary(result.steps) == expected, (text, _summary(result.steps), result.question)


def test_rule_parser_refuses():
    """Anything the rules cannot account for fully must stop the robot, not drop a step."""
    parser = CommandParser("rule")
    for text in ("đi thẳng rồi rẽ phải",        # a distance is missing
                 "rẽ trái 60 cm",               # a number the rules cannot place
                 "go to the kitchen",           # not a relative move
                 "đi thẳng 5 m",                # over MAX_LINEAR_M
                 "quay trái 400 độ"):           # over MAX_TURN_DEG
        result = parser.parse(text)
        assert not result.steps and result.question, (text, result)


def test_validator_limits():
    bad = [
        _reply(("forward", 0.5, "cm")),                    # under MIN_LINEAR_M
        _reply(("forward", 30, "deg")),                    # angle unit on a distance
        _reply(("turn_left", 30, "cm")),                   # distance unit on a turn
        _reply(("jump", 30, "cm")),                        # unknown action
        _reply(("forward", True, "cm")),                   # a bool is not a number
        _reply(("forward", None, None)),                   # a distance must be stated
        _reply(*[("turn_left", None, None)] * (cfg.MAX_STEPS + 1)),
        {"question": "no steps key"},
    ]
    for raw in bad:
        steps, error = validate_steps(raw, "30 0.5 cm deg")
        assert steps is None and error, raw
    steps, error = validate_steps(_reply(("turn_around", None, None), ("turn_right", None, None)), "")
    assert error is None and _summary(steps) == [("turn_around", 180.0, False), ("turn_right", 90.0, False)]


def test_invented_number_is_refused():
    """The user said 30; an LLM that answers 40 twice gets a question, never a step."""
    client = FakeClient([_reply(("forward", 40, "cm")), _reply(("forward", 40, "cm"))])
    result = CommandParser(client=client).parse("đi thẳng 30 cm")
    assert not result.steps and result.question, result
    assert len(client.calls) == cfg.MAX_VALIDATION_RETRIES + 1
    assert "does not appear" in client.calls[-1][1][-1]["content"], "retry must carry the validator error"


def test_swapped_numbers_are_refused():
    """Both numbers are real, but in the wrong steps: the order check must catch it."""
    swapped = _reply(("forward", 60, "cm"), ("turn_left", 30, "deg"))
    client = FakeClient([swapped, swapped])
    result = CommandParser(client=client).parse("đi thẳng 30 cm rồi quay trái 60 độ")
    assert not result.steps and result.question, result
    assert "order" in client.calls[-1][1][-1]["content"], "retry must say the order is wrong"


def test_answer_may_fill_an_earlier_step():
    """Across messages the order check is off: '30 cm' answers the first step, after the 45."""
    client = FakeClient([_reply(question="Bao xa?"), _reply(("forward", 30, "cm"), ("turn_left", 45, "deg"))])
    parser = CommandParser(client=client)
    assert parser.parse("đi thẳng rồi quay trái 45 độ").question == "Bao xa?"
    result = parser.parse("30 cm")
    assert _summary(result.steps) == [("forward", 0.3, True), ("turn_left", 45.0, True)], result


def test_retry_recovers():
    client = FakeClient([_reply(("forward", 40, "cm")), _reply(("forward", 30, "cm"))])
    result = CommandParser(client=client).parse("đi thẳng 30 cm")
    assert _summary(result.steps) == [("forward", 0.3, True)], result


def test_question_then_answer():
    """A number given in the answer counts as written; the LLM sees the whole exchange."""
    client = FakeClient([_reply(question="Bao xa?"), _reply(("forward", 60, "cm"), ("turn_right", None, None))])
    parser = CommandParser(client=client)
    first = parser.parse("đi thẳng rồi rẽ phải")
    assert not first.steps and first.question == "Bao xa?", first
    second = parser.parse("60 cm")
    assert _summary(second.steps) == [("forward", 0.6, True), ("turn_right", 90.0, False)], second
    roles = [m["role"] for m in client.calls[-1][1]]
    assert roles == ["user", "assistant", "user"], roles
    assert parser.history == [] and parser.user_texts == [], "a finished command must reset the exchange"


def test_number_words():
    client = FakeClient([_reply(("forward", 1, "m"), ("backward", 0.5, "m"))])
    result = CommandParser(client=client).parse("đi thẳng một mét rồi lùi nửa mét")
    assert _summary(result.steps) == [("forward", 1.0, True), ("backward", 0.5, True)], result


class FlakyClient:
    """Fails on the first call (network down), then answers from the script."""

    def __init__(self, replies):
        self.replies = list(replies)
        self.calls = 0

    def complete(self, system, messages, schema):
        self.calls += 1
        if self.calls == 1:
            raise LLMError("network down")
        return self.replies.pop(0)


def test_fallback_finishes_the_exchange_with_rules():
    """After a fallback the rules finish the exchange: the LLM is not asked again mid-command (its half of the
    history is gone, so a number from before could not be checked against what it saw), the answer still
    lands in the command it answers, and the next command goes to the LLM again."""
    client = FlakyClient([_reply(("forward", 30, "cm"))])
    parser = CommandParser("auto", client=client)
    first = parser.parse("đi thẳng rồi rẽ phải 30 độ")     # LLM down -> rules -> "how far?"
    assert first.provider == "rule" and first.question and client.calls == 1, first
    second = parser.parse("40 cm")
    assert second.provider == "rule" and client.calls == 1, second
    assert [(s.action, s.amount) for s in second.steps] == [("forward", 0.4), ("turn_right", 30.0)], second
    third = parser.parse("đi thẳng 30 cm")                  # a new command: the LLM again
    assert third.provider == "injected" and client.calls == 2, third


def test_llm_failure_fallback():
    result = CommandParser("auto", client=FailingClient()).parse(EXAMPLE)
    assert result.provider == "rule" and len(result.steps) == 3, result
    try:
        CommandParser("gemini", client=FailingClient()).parse(EXAMPLE)
    except LLMError:
        pass
    else:
        raise AssertionError("an explicitly chosen LLM must not fall back silently")


def test_motion_plan_example():
    plan = build_plan(CommandParser("rule").parse(EXAMPLE).steps)
    assert [leg.wire for leg in plan.legs] == ["F 0.300 0.150", "T 90.000 0.800", "F 0.600 0.150", "S"]
    forward, turn = plan.legs[0].wheels, plan.legs[1].wheels
    # W1 is the front wheel at 0 deg, its roll is perpendicular to +x: it stays still driving forward.
    assert forward[0].steps == 0 and forward[1].steps == -forward[2].steps != 0, forward
    assert len({w.steps for w in turn}) == 1 and turn[0].steps > 0, turn
    assert plan.legs[2].wheels[1].steps == 2 * forward[1].steps, "60 cm must be twice 30 cm"
    assert plan.duration_s > 0 and plan.legs[-1].duration_s == 0


def test_plan_matches_the_chassis_header():
    """run.json's wheel_radius_m is read from constants.h, the pulses come from libmc: a libmc not
    rebuilt after the header changed (the 2026-09-27 wheel-radius fix) would record the wrong r."""
    import mc_client                              # importable once build_plan has put CM on the path
    plan = build_plan(CommandParser("rule").parse("đi 30 cm").steps)
    chassis = mc_client.chassis()
    assert plan.wheel_radius_m == chassis["wheel_radius_m"]
    # Driving +x, the back wheels at 120 / 240 deg roll sin(60 deg) of the distance.
    expected = math.sin(math.radians(60.0)) * 0.3 / chassis["wheel_radius_m"] * chassis["steps_per_rev"] / (2 * math.pi)
    assert abs(abs(plan.legs[0].wheels[1].steps) - expected) <= PULSE_REL_TOL * expected, (plan.legs[0].wheels[1].steps, expected)
    assert plan.to_dict()["wheel_radius_m"] == chassis["wheel_radius_m"]


def test_turn_chunks_keep_direction():
    """SEQ wraps into (-180, 180]: an unchunked 180 right would be driven as a left turn."""
    plan = build_plan(CommandParser("rule").parse("quay phải 180 độ, quay trái 360 độ, lùi 10 cm").steps)
    turns = [leg.value for leg in plan.legs if leg.kind == "ROTATE"]
    assert all(abs(t) < math.pi for t in turns), turns
    assert abs(sum(t for t in turns if t < 0) + math.pi) < TOL
    assert abs(sum(t for t in turns if t > 0) - 2 * math.pi) < TOL
    assert plan.legs[-2].wire == "F -0.100 0.150", plan.legs[-2].wire


def test_plan_file_is_readable_by_robot_link():
    plan = build_plan(CommandParser("rule").parse(EXAMPLE).steps)
    lines = plan.plan_file_text("FORWARD 9 is only a comment here").splitlines()
    primitives = [line for line in lines if PLAN_LINE_RE.match(line)]
    assert primitives == [leg.plan_line() for leg in plan.legs], primitives


def test_natural_corpus_rules():
    """Every case the rules are meant to read (not "rules": false) comes out as natural_commands.json says."""
    failures = []
    for case in load_corpus():
        if case.get("rules", True):
            error = case_error(case, run_case(CommandParser("rule"), case)[0])
            if error:
                failures.append(f"{case.get('text') or case['turns']}: {error}")
    assert not failures, "\n".join(failures)


def test_intents():
    """A purpose clause is checked against the steps: met after the step the corpus names, or not at all."""
    for case in load_corpus():
        if "intent" not in case:
            continue
        result, text = run_case(CommandParser("rule"), case)
        report = check_intents(result.steps, text)
        assert len(report) == 1, (text, report)
        ok, number, _ = report[0]
        assert (number if ok else None) == case["intent"], (text, report)


def test_turn_validator():
    """turn_around takes a stated angle; a sideless half / full turn goes left, any other angle is asked
    (a Question naming its step); revolutions become degrees in code, a bare 'full circle' one turn."""
    raw = {"steps": [{"action": "turn_around", "value": 90, "unit": "deg"},
                     {"action": "turn", "value": 1, "unit": "rev"},
                     {"action": "turn", "value": None, "unit": "rev"},
                     {"action": "turn_left", "value": 0.5, "unit": "rev"}], "question": ""}
    steps, error = validate_steps(raw, "quay lai 90 do, 1 vong, tron vong, nua vong", ordered=False)
    assert error is None, error
    assert [(s.action, s.amount, s.stated) for s in steps] == [
        ("turn_around", 90.0, True), ("turn_left", 360.0, True), ("turn_left", 360.0, False),
        ("turn_left", 180.0, True)], steps
    raw = {"steps": [{"action": "forward", "value": 30, "unit": "cm"}, {"action": "turn", "value": 90, "unit": "deg"}],
           "question": ""}
    steps, error = validate_steps(raw, "di 30 cm roi quay 90 do")
    assert steps is None and isinstance(error, Question) and error.step == 1, error
    _, error = validate_steps({"steps": [{"action": "turn", "value": None, "unit": None}], "question": ""}, "quay")
    assert isinstance(error, Question), error


def test_llm_question_is_not_retried():
    """An LLM reply that needs the user (which side?) is asked at once, not re-asked of the LLM, and the
    question rides along in the history so the answer completes the same command."""
    client = FakeClient([
        {"steps": [{"action": "turn", "value": 90, "unit": "deg"}, {"action": "forward", "value": 30, "unit": "cm"}],
         "question": ""},
        {"steps": [{"action": "turn_left", "value": 90, "unit": "deg"}, {"action": "forward", "value": 30, "unit": "cm"}],
         "question": ""}])
    parser = CommandParser(client=client)
    first = parser.parse("quay 90 do roi di thang 30 cm")
    assert not first.steps and first.question and len(client.calls) == 1, (first, len(client.calls))
    second = parser.parse("trai")
    assert [(s.action, s.amount) for s in second.steps] == [("turn_left", 90.0), ("forward", 0.3)], second
    asked = ASKED_NOTE.format(question=first.question)
    _, messages = client.calls[1]
    assert any(asked in m["content"] for m in messages if m["role"] == "assistant"), messages


def test_llm_goal_is_copied_not_invented():
    """A place the user named comes back as a goal; a goal with words the user never said is refused."""
    parser = CommandParser(client=FakeClient([{"steps": [], "question": "", "goal": "cái ghế"}]))
    result = parser.parse("đi tới cái ghế nhé")
    assert result.goal == "cái ghế" and not result.steps and not result.question, result
    invented = {"steps": [], "question": "", "goal": "cái bàn"}
    parser = CommandParser(client=FakeClient([invented, invented]))
    result = parser.parse("đi tới cái ghế nhé")
    assert not result.goal and "do not appear" in result.question, result


def test_goal_goes_alone():
    """A place with anything else - a second place, a step, a move the rules cannot read - is asked to be said
    apart (going to the place alone would drop the rest); after a goal the parser starts a new command."""
    parser = CommandParser("rule")
    for text in ("đi tới cái ghế rồi đi tới cái vali", "đi tới cái ghế rồi đi thẳng", "đi tới cái ghế rồi quay trái 90 độ"):
        result = parser.parse(text)
        assert not result.goal and not result.steps and result.question, (text, result)
        parser.reset()
    result = parser.parse("đi tới cái ghế nhé")
    assert result.goal == "cái ghế" and parser.user_texts == [], (result, parser.user_texts)


class AskThenFailClient:
    """Asks a question on the first call, then the network goes down."""

    def __init__(self):
        self.calls = 0

    def complete(self, system, messages, schema):
        self.calls += 1
        if self.calls == 1:
            return {"steps": [], "question": "Bao xa?", "goal": ""}
        raise LLMError("network down")


def test_fallback_replay_stops_at_a_complete_command():
    """The LLM asked about a command the rules read whole; when it then fails, the rules' reading of that
    first message stands - the answer is not parsed over it as a new command."""
    parser = CommandParser("auto", client=AskThenFailClient())
    assert parser.parse("đi thẳng 30 cm rồi rẽ trái").question
    result = parser.parse("ừ")
    assert [(s.action, s.amount) for s in result.steps] == [("forward", 0.3), ("turn_left", 90.0)], result


def test_map_request():
    """'lập bản đồ' and its kin are split off with the connectors around them; filler alone means the scan."""
    from cmd_parser import MAP_SCAN, map_scan_steps, split_map_request
    cases = {"lập bản đồ": (True, ""), "Hãy lập bản đồ nhé": (True, ""), "quét xung quanh": (True, ""),
             "scan the room": (True, ""), "lập lại bản đồ mới": (True, ""),
             "quay trái 360 độ rồi lập bản đồ": (True, "quay trái 360 độ"),
             "quay trái 90 độ, lập bản đồ": (True, "quay trái 90 độ"),       # 'độ' is no connector
             "đi 50 cm sau đó lập bản đồ": (True, "đi 50 cm"),
             "lập bản đồ rồi đi tới cái ghế": (True, "đi tới cái ghế"),
             "build a map then go forward 30 cm": (True, "go forward 30 cm"),
             "đi tới cái vali": (False, "đi tới cái vali"), "đi thẳng 30 cm": (False, "đi thẳng 30 cm")}
    for text, expected in cases.items():
        assert split_map_request(text) == expected, (text, split_map_request(text))
    steps = map_scan_steps()
    assert [(s.action, s.amount) for s in steps] == list(MAP_SCAN) and not any(s.stated for s in steps), steps
    parser = CommandParser("rule")
    _, rest = split_map_request("quay trái 360 độ rồi lập bản đồ")
    assert [(s.action, s.amount) for s in parser.parse(rest).steps] == [("turn_left", 360.0)]


def _two_chair_scene():
    """A 3 x 2 m room, the robot at its left, two chairs and a suitcase (realroom scene shape)."""
    def box(oid, cls, x, y):
        return {"id": oid, "class": cls, "confidence": 0.9, "color": "unknown", "center": {"x": x, "y": y}, "yaw": 0.0,
                "polygon": [{"x": x - 0.2, "y": y - 0.2}, {"x": x + 0.2, "y": y - 0.2}, {"x": x + 0.2, "y": y + 0.2},
                            {"x": x - 0.2, "y": y + 0.2}], "free_sides": ["front", "back", "left", "right"], "near": []}
    return {"version": "2.0", "room": {"width": 3.0, "length": 2.0}, "walls": [], "doors": [],
            "robot": {"x": 0.4, "y": 1.0, "theta": 0.0, "radius": 0.225},
            "objects": [box("obj_1", "chair", 1.5, 0.5), box("obj_2", "chair", 2.4, 1.5), box("obj_3", "suitcase", 1.5, 1.6)]}


def test_rule_grounding():
    """Without an LLM: the class a word names; with two, the user's number picks one; the spot nearest the
    robot; a class the map lacks is said, with what it has."""
    import goto
    scene = _two_chair_scene()
    spot, why = goto.ground_by_rules(scene, "cái vali", lambda q: "")
    assert why is None and spot["target_id"] == "obj_3", (spot, why)
    asked = []
    spot, why = goto.ground_by_rules(scene, "cái ghế", lambda q: asked.append(q) or "2")
    assert why is None and spot["target_id"] == "obj_2" and asked and "2 chair" in asked[0], (spot, why, asked)
    nearest = min(math.hypot(s["x"] - 0.4, s["y"] - 1.0) for s in goto.build_candidates(scene) if s["target_id"] == "obj_2")
    assert abs(math.hypot(spot["x"] - 0.4, spot["y"] - 1.0) - nearest) < 1e-9, spot
    spot, why = goto.ground_by_rules(scene, "the kitchen", lambda q: "")
    assert spot is None and "chair" in why and "suitcase" in why, why


if __name__ == "__main__":
    test_rule_parser_example()
    test_rule_parser_variants()
    test_rule_parser_refuses()
    print("rule parser ok")
    test_validator_limits()
    test_invented_number_is_refused()
    test_swapped_numbers_are_refused()
    test_answer_may_fill_an_earlier_step()
    test_retry_recovers()
    test_question_then_answer()
    test_number_words()
    test_llm_failure_fallback()
    test_fallback_finishes_the_exchange_with_rules()
    test_turn_validator()
    test_llm_question_is_not_retried()
    test_llm_goal_is_copied_not_invented()
    test_goal_goes_alone()
    test_fallback_replay_stops_at_a_complete_command()
    print("validator + LLM exchange ok")
    test_rule_grounding()
    print("grounding (rules) ok")
    test_natural_corpus_rules()
    test_intents()
    print("natural commands (rules) ok")
    test_map_request()
    print("map request ok")
    try:
        test_motion_plan_example()
        test_plan_matches_the_chassis_header()
        test_turn_chunks_keep_direction()
        test_plan_file_is_readable_by_robot_link()
        print("motion plan ok")
    except PlanError as exc:
        print(f"motion plan SKIPPED: {exc}")
    print("ALL OK")
