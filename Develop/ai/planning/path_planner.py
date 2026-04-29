"""
Planning - Path Planner

Thuật toán lập kế hoạch đường đi cho robot.
"""


class PathPlanner:
    """Lập kế hoạch đường đi dựa trên bản đồ và thông tin perception."""

    def __init__(self, map_data=None):
        self.map_data = map_data

    def set_map(self, occupancy_grid):
        """
        Cập nhật bản đồ môi trường.

        Args:
            occupancy_grid: 2D numpy array (0 = free, 1 = obstacle)
        """
        self.map_data = occupancy_grid

    def plan(self, start: tuple, goal: tuple) -> list:
        """
        Tìm đường đi từ start đến goal.

        Args:
            start: (x, y) vị trí xuất phát
            goal: (x, y) vị trí đích

        Returns:
            list: Danh sách waypoints [(x1, y1), (x2, y2), ...]
        """
        # TODO: Implement A*, RRT, hoặc thuật toán phù hợp
        raise NotImplementedError

    def is_path_clear(self, path: list) -> bool:
        """Kiểm tra đường đi có bị chặn không."""
        raise NotImplementedError
