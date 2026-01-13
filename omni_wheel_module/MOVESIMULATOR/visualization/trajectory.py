"""Track robot positions for plotting."""

from __future__ import annotations

from collections import deque
from typing import Deque, List, Tuple


class TrajectoryTracker:
    def __init__(self, max_length: int = 1000) -> None:
        self.max_length = max_length
        self.positions: Deque[Tuple[float, float]] = deque(maxlen=max_length)

    def add_position(self, x: float, y: float) -> None:
        self.positions.append((x, y))

    def get_trajectory(self) -> Tuple[List[float], List[float]]:
        xs, ys = zip(*self.positions) if self.positions else ([], [])
        return list(xs), list(ys)

    def clear(self) -> None:
        self.positions.clear()
