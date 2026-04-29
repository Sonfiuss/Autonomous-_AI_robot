"""
Control - Motion Controller

Chuyển đổi quyết định từ AI (planning) thành lệnh điều khiển cụ thể
gửi đến hardware module.
"""


class MotionController:
    """Chuyển đổi lệnh AI thành lệnh hardware."""

    def __init__(self, hardware_api=None):
        """
        Args:
            hardware_api: Instance của hardware API (serial_interface)
        """
        self.hardware_api = hardware_api
        self.is_active = False

    def execute_command(self, command: dict):
        """
        Thực thi lệnh điều khiển.

        Args:
            command: {"action": str, "params": dict} từ DecisionEngine
        """
        action = command.get("action")
        params = command.get("params", {})

        if action == "move":
            self._move(params)
        elif action == "arm_move":
            self._arm_move(params)
        elif action == "stop":
            self._stop()
        else:
            raise ValueError(f"Unknown action: {action}")

    def _move(self, params: dict):
        """Gửi lệnh di chuyển đến drive controller."""
        # TODO: Gửi qua hardware_api
        vx = params.get("vx", 0)
        vy = params.get("vy", 0)
        omega = params.get("omega", 0)
        print(f"Moving: vx={vx}, vy={vy}, omega={omega}")

    def _arm_move(self, params: dict):
        """Gửi lệnh cánh tay đến arm controller."""
        # TODO: Gửi qua hardware_api
        raise NotImplementedError

    def _stop(self):
        """Dừng tất cả chuyển động."""
        print("Emergency stop!")
