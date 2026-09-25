"""Offline tests for the drive demo: rule parser, validator, LLM exchange (scripted fake), motion plan.

Run:  python3 communication/test_drive.py
No API key, camera or robot needed. The motion-plan tests need libmc.so (project/tools/build_mc.sh)
and are skipped without it.
"""
import math
import re

import drive_config as cfg
from cmd_parser import CommandParser, validate_steps
from motion_plan import PlanError, build_plan

cfg.add_cm_to_path()
from llm_client import FakeClient, LLMError  # noqa: E402  (CM module, path set just above)

EXAMPLE = "đi thẳng 30 cm. sau đó rẽ trái, đi tiếp 60 cm"
TOL = 1e-9
PLAN_LINE_RE = re.compile(r"^\s*(ROTATE|FORWARD|MOVE|STOP)\b")   # what MissionRunner's reader accepts


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


def test_fallback_resets_the_exchange():
    """After a fallback the LLM starts from nothing, so numbers from before must not count."""
    stale = _reply(("forward", 30, "cm"))
    parser = CommandParser("auto", client=FlakyClient([stale, stale]))   # asked, then re-asked once
    first = parser.parse("đi thẳng rồi rẽ phải 30 độ")     # LLM down -> rules -> a question
    assert first.provider == "rule" and first.question and parser.user_texts == [], first
    second = parser.parse("lùi 20 cm")                       # the LLM invents the old 30
    assert not second.steps and "does not appear" in str(second.question) + str(second.raw), second


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
    test_fallback_resets_the_exchange()
    print("validator + LLM exchange ok")
    try:
        test_motion_plan_example()
        test_turn_chunks_keep_direction()
        test_plan_file_is_readable_by_robot_link()
        print("motion plan ok")
    except PlanError as exc:
        print(f"motion plan SKIPPED: {exc}")
    print("ALL OK")
