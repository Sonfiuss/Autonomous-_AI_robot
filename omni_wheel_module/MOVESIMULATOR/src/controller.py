"""Keyboard controller using matplotlib events."""

from __future__ import annotations

from typing import Dict, List

from config import settings


class KeyboardController:
    def __init__(self) -> None:
        self.key_states: Dict[str, bool] = {}

    def on_key_press(self, event) -> None:
        key = (event.key or "").lower()
        if key == "escape":
            import matplotlib.pyplot as plt

            plt.close(event.canvas.figure)
            return
        if key:
            self.key_states[key] = True

    def on_key_release(self, event) -> None:
        key = (event.key or "").lower()
        if key:
            self.key_states[key] = False

    def get_wheel_commands(self) -> List[dict]:
        commands = [
            {"accelerate": False, "reverse": False},
            {"accelerate": False, "reverse": False},
            {"accelerate": False, "reverse": False},
        ]
        for key, wheel_index in settings.KEY_MAPPING.items():
            if self.key_states.get(key, False):
                if key in ("w", "a", "d"):
                    commands[wheel_index]["accelerate"] = True
                else:
                    commands[wheel_index]["reverse"] = True
        return commands

    def reset(self) -> None:
        self.key_states.clear()
