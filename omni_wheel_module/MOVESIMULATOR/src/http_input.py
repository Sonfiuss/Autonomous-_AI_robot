"""Thread-safe input state for HTTP-driven controls."""

from __future__ import annotations

import threading
from typing import Dict, List

from config import settings


class HttpInputState:
    def __init__(self) -> None:
        self.key_states: Dict[str, bool] = {}
        self.override_omegas: List[float] | None = None
        self._lock = threading.Lock()

    def set_key_state(self, key: str, pressed: bool) -> None:
        key = (key or "").lower()
        with self._lock:
            self.key_states[key] = pressed

    def get_wheel_commands(self) -> List[dict]:
        with self._lock:
            if self.override_omegas is not None:
                return []  # signals override active
            states = dict(self.key_states)

        commands = [
            {"accelerate": False, "reverse": False},
            {"accelerate": False, "reverse": False},
            {"accelerate": False, "reverse": False},
        ]
        for key, wheel_index in settings.KEY_MAPPING.items():
            if states.get(key, False):
                if key in ("w", "a", "d"):
                    commands[wheel_index]["accelerate"] = True
                else:
                    commands[wheel_index]["reverse"] = True
        return commands

    def reset(self) -> None:
        with self._lock:
            self.key_states.clear()
            self.override_omegas = None

    def set_override_omegas(self, omegas: List[float]) -> None:
        with self._lock:
            self.override_omegas = list(omegas)

    def get_override_omegas(self) -> List[float] | None:
        with self._lock:
            if self.override_omegas is None:
                return None
            return list(self.override_omegas)
