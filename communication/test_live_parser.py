"""The natural-command corpus through the configured LLM - a live check, not part of test_drive.py.

Run:  python3 communication/test_live_parser.py [--provider gemini] [--min-accuracy 0.95] [--pace-s 12.5]
                                                [--first N] [--last M]
Calls the LLM once per message of every case (about forty calls on the key in project/src/CM/.env),
--pace-s apart: Gemini 2.5 Flash's free tier takes 5 requests a minute and 20 a day (per model and project,
2026-09-29), so the whole corpus needs two days on it - --first / --last run cases N..M, leaving the day's
remaining calls for the robot (demo_drive's parser and CM grounding share the key).
The provider is named, so an LLM error fails its case instead of falling back to the rules (auto would
hide it). Prints each case, the latency, and the share right; exits 1 below --min-accuracy. A case counts as
right when the validated steps are exactly the corpus's, or the robot asks where the corpus says it must.
The validator's own guarantee - no number the user did not write ever moves the robot - is checked on every
case too: a step whose value is not in the user's words fails the run whatever the accuracy.
"""
import argparse
import logging
import sys
import time

import drive_config as cfg
from cmd_parser import CommandParser, numbers_in
from test_drive import case_error, load_corpus, run_case

DEFAULT_MIN_ACCURACY = 0.95
DEFAULT_PROVIDER = "gemini"
DEFAULT_PACE_S = 12.5                  # 5 requests a minute, with a little slack
EXIT_OK, EXIT_BELOW = 0, 1
FIRST_CASE = 1                         # cases are numbered from 1, as printed


def _parse_args():
    parser = argparse.ArgumentParser(description="Natural-command corpus through the live LLM.")
    parser.add_argument("--provider", default=DEFAULT_PROVIDER, help="an LLM provider (no rule fallback)")
    parser.add_argument("--min-accuracy", type=float, default=DEFAULT_MIN_ACCURACY)
    parser.add_argument("--pace-s", type=float, default=DEFAULT_PACE_S, help="least time between two LLM calls")
    parser.add_argument("--first", type=int, default=FIRST_CASE, help="start at this case (1-based)")
    parser.add_argument("--last", type=int, default=None, help="stop after this case (default: the last one)")
    return parser.parse_args()


def _invented(result, text):
    """Values of an accepted reply that are not numbers the user wrote - the validator must keep this empty."""
    if not result.steps:
        return []
    written = numbers_in(text)
    return [item["value"] for item in result.raw.get("steps", []) if item.get("value") is not None
            and not any(abs(item["value"] - w) <= cfg.NUMBER_MATCH_TOL for w in written)]


class PacedClient:
    """The parser's LLM client, its calls spread at least pace_s apart."""

    def __init__(self, client, pace_s):
        self.client, self.pace_s, self.model, self._last = client, pace_s, getattr(client, "model", ""), None

    def complete(self, system, messages, schema):
        if self._last is not None:
            time.sleep(max(self._last + self.pace_s - time.monotonic(), 0.0))
        self._last = time.monotonic()
        return self.client.complete(system, messages, schema)


def _run(parser, case):
    """run_case, with an LLM error made the case's result instead of ending the run."""
    try:
        return run_case(parser, case)
    except RuntimeError as exc:                  # llm_client.LLMError
        parser.reset()
        return None, f"{case.get('text') or ' / '.join(case['turns'])}  [LLM error: {exc}]"


def main():
    args = _parse_args()
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s")
    corpus = load_corpus()
    in_range = args.first >= FIRST_CASE and (args.last is None or args.last >= args.first)  # no slicing from the end
    cases = corpus[args.first - FIRST_CASE:args.last] if in_range else []
    if not cases:
        print(f"no case in {args.first}..{args.last or len(corpus)}: the corpus has cases {FIRST_CASE}..{len(corpus)}")
        return EXIT_BELOW
    parser = CommandParser(args.provider)
    parser._client = PacedClient(parser._client, args.pace_s)
    print(f"provider {parser.provider} {parser.model}, calls {args.pace_s:g} s apart")
    right, latencies, invented = 0, [], []
    for number, case in enumerate(cases, args.first):
        started = time.monotonic()
        result, text = _run(parser, case)
        latencies.append(time.monotonic() - started)
        if result is None:
            print(f"{number:2d} FAIL {text[:150]}")
            continue
        error = case_error(case, result)
        right += error is None
        invented += _invented(result, text)
        shown = [f"{s.action} {s.amount:g}" for s in result.steps] or [result.question]
        print(f"{number:2d} {'OK  ' if error is None else 'FAIL'} {result.provider:<7} {latencies[-1]:5.1f} s  {text[:70]}")
        if error is not None:
            print(f"        {error}")
        elif not case.get("asks"):
            print(f"        {shown}")
    accuracy = right / len(cases)
    print(f"\n{right} / {len(cases)} right ({accuracy:.0%}), median {sorted(latencies)[len(latencies) // 2]:.1f} s "
          f"per case (paced), invented values {invented or 'none'}")
    return EXIT_OK if accuracy >= args.min_accuracy and not invented else EXIT_BELOW


if __name__ == "__main__":
    sys.exit(main())
