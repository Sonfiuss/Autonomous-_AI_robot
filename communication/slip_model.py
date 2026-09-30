"""Slip ratio k = real / commanded motion, per floor and kind of motion, learned from measured legs
(task 2026-09-27_explore-map Pha 1' D - the short version: one floor, no floor recognition yet).

A leg executor asks k before a leg (command = target / k) and hands every measured leg back:
  kinds    forward / backward (FORWARD, by the sign of the metres), left / right (ROTATE, CCW = left)
  k        sum(measured) / sum(commanded) over the last WINDOW samples it learned (long legs weigh more);
           DEFAULT_K until it has MIN_SAMPLES
  learned  a sample the odometry measured well (not weak), the firmware ran in full (not cut short: the
           2026-09-28 backward legs, a firmware fault rather than the floor), at least LEARN_MIN long, and
           not stuck (ratio >= STUCK_RATIO: something in the way, not slip)
Every sample is kept in SLIP_FILE, learned or not, with the reason when not.
"""
import json
import logging
import math
import os
import statistics
import time

import drive_config as cfg
from motion_plan import ROTATE

SLIP_FILE = os.path.join(cfg.COMM_DIR, "slip_profiles.json")
DEFAULT_FLOOR = "san_01"             # one profile until floors are recognised (plan step 4a)
FORWARD, BACKWARD, LEFT, RIGHT = "forward", "backward", "left", "right"
KINDS = (FORWARD, BACKWARD, LEFT, RIGHT)
# Before a kind has MIN_SAMPLES: tape 2026-09-27 on the grey tile, forward 0.91 / backward 0.95 of the
# rolled distance, and forward 0.93 again 2026-09-28 by VO at the tape scale. Turns are unmeasured since
# the wheel-radius fix, so 1.
DEFAULT_K = {FORWARD: 0.93, BACKWARD: 0.93, LEFT: 1.0, RIGHT: 1.0}
WINDOW = 10
MIN_SAMPLES = 3
LEARN_MIN = {FORWARD: 0.10, BACKWARD: 0.10, LEFT: math.radians(10.0), RIGHT: math.radians(10.0)}
STUCK_RATIO = 0.5
WHY_CUT, WHY_WEAK, WHY_SHORT, WHY_STUCK = "cut short", "weak odometry", "too short", "stuck"
RATIO_DIGITS = 4
TIME_DIGITS = 3                      # wall-clock seconds of a sample (ms)
JSON_INDENT = 1                      # the file grows by a sample per leg: keep it compact but diffable

logger = logging.getLogger(__name__)


def kind_of(primitive, value):
    """The slip kind of a FORWARD (m) or ROTATE (rad) command of this sign."""
    if primitive == ROTATE:
        return LEFT if value >= 0.0 else RIGHT
    return FORWARD if value >= 0.0 else BACKWARD


class SlipModel:
    """k per kind for one floor, persisted in `path`."""

    def __init__(self, path=SLIP_FILE, floor=DEFAULT_FLOOR):
        self.path, self.floor = path, floor
        self.data = {}
        if os.path.exists(path):
            with open(path, encoding="utf-8") as f:
                self.data = json.load(f)
        self.samples = self.data.setdefault(floor, {})
        for kind in KINDS:
            self.samples.setdefault(kind, [])

    def learned(self, kind):
        """The last WINDOW learned samples of a kind."""
        return [s for s in self.samples[kind] if s["learned"]][-WINDOW:]

    def k(self, kind):
        """real / commanded for a kind: its learned samples, DEFAULT_K below MIN_SAMPLES."""
        window = self.learned(kind)
        if len(window) < MIN_SAMPLES:
            return DEFAULT_K[kind]
        return sum(abs(s["measured"]) for s in window) / sum(abs(s["commanded"]) for s in window)

    def add(self, kind, commanded, measured, cut_short=False, weak=False, **detail):
        """Records one measured leg (commanded and measured along its own direction, signed); learns it
        unless there is a reason not to. Saves the file. Returns the sample."""
        ratio = measured / commanded if commanded else math.nan
        why = self._why_not(kind, commanded, ratio, cut_short, weak)
        sample = dict({"t": round(time.time(), TIME_DIGITS), "commanded": commanded, "measured": measured,
                       "ratio": round(ratio, RATIO_DIGITS) if math.isfinite(ratio) else None,
                       "learned": not why, "why": why}, **detail)
        self.samples[kind].append(sample)
        self.save()
        logger.info("slip %s: %+.4f -> %+.4f (%s), k now %.3f", kind, commanded, measured, why or "learned", self.k(kind))
        return sample

    @staticmethod
    def _why_not(kind, commanded, ratio, cut_short, weak):
        """Why a sample is not learned (WHY_*), or "" when it is."""
        if cut_short:
            return WHY_CUT
        if weak:
            return WHY_WEAK
        if abs(commanded) < LEARN_MIN[kind]:
            return WHY_SHORT
        if not ratio >= STUCK_RATIO:         # a NaN ratio (nothing commanded) counts as stuck too
            return WHY_STUCK
        return ""

    def save(self):
        """Writes the file atomically: a crash mid-write leaves the previous one."""
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.data, f, ensure_ascii=False, indent=JSON_INDENT)
        os.replace(tmp, self.path)

    def table(self):
        """Lines: per kind, k, how many samples it rests on, their spread, and the last sample."""
        lines = [f"hệ số trượt sàn {self.floor} (k = thực / lệnh; mẫu học {WINDOW} gần nhất, "
                 f"< {MIN_SAMPLES} mẫu = mặc định):"]
        for kind in KINDS:
            window = self.learned(kind)
            spread = statistics.pstdev([s["ratio"] for s in window]) if window else math.nan
            last = self.samples[kind][-1] if self.samples[kind] else None
            last_text = "-" if last is None else f"{last['ratio']} {'học' if last['learned'] else last['why']}"
            lines.append(f"  {kind:<8} k {self.k(kind):.3f}  trượt {1.0 - self.k(kind):+.1%}  n {len(window)}  "
                         f"độ lệch {spread:.3f}  mẫu cuối {last_text}")
        return lines
