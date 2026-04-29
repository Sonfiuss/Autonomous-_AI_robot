"""
Perception - Depth Estimation

Ước lượng khoảng cách sử dụng Depth Anything V2.
Tham chiếu: Vision/Depth-Anything-V2-main/
"""


class DepthEstimator:
    """Ước lượng depth map từ ảnh RGB."""

    def __init__(self, model_type: str = "vits", device: str = "cuda"):
        """
        Args:
            model_type: Loại model ("vits", "vitb", "vitl")
            device: "cuda" hoặc "cpu"
        """
        self.model_type = model_type
        self.device = device
        self.model = None

    def load_model(self):
        """Load Depth Anything V2 model."""
        # TODO: Import từ Vision/Depth-Anything-V2-main/depth_anything_v2/
        raise NotImplementedError

    def estimate(self, frame):
        """
        Ước lượng depth map từ frame RGB.

        Args:
            frame: numpy array (BGR image)

        Returns:
            numpy array: Depth map
        """
        raise NotImplementedError

    def get_distance_at_point(self, depth_map, x: int, y: int) -> float:
        """Lấy khoảng cách tại điểm (x, y) trên depth map."""
        raise NotImplementedError
