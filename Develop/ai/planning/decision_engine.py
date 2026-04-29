"""
Planning - Decision Engine

Engine ra quyết định điều khiển robot dựa trên thông tin từ perception và planning.
"""


class DecisionEngine:
    """Ra quyết định hành động cho robot."""

    def __init__(self):
        self.current_state = "IDLE"
        self.objectives = []

    def update_state(self, perception_data: dict):
        """
        Cập nhật trạng thái dựa trên dữ liệu perception.

        Args:
            perception_data: Dict chứa detections, depth_map, sensor_data
        """
        # TODO: Implement state machine hoặc behavior tree
        raise NotImplementedError

    def decide(self) -> dict:
        """
        Ra quyết định hành động tiếp theo.

        Returns:
            dict: Lệnh điều khiển {"action": str, "params": dict}
                  Ví dụ: {"action": "move", "params": {"vx": 0.5, "vy": 0, "omega": 0}}
        """
        raise NotImplementedError

    def set_objective(self, objective: str):
        """Đặt mục tiêu cho robot (e.g., 'navigate_to', 'follow_person', 'avoid_obstacle')."""
        self.objectives.append(objective)
