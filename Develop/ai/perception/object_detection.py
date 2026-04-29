"""
Perception - Object Detection

Phát hiện và nhận diện vật thể từ camera feed.
"""


class ObjectDetector:
    """Detector vật thể sử dụng model AI."""

    def __init__(self, model_path: str = None):
        self.model_path = model_path
        self.model = None

    def load_model(self):
        """Load model nhận diện."""
        # TODO: Tích hợp YOLO, Detectron2, hoặc model custom
        raise NotImplementedError

    def detect(self, frame):
        """
        Phát hiện vật thể trong frame.

        Args:
            frame: numpy array (BGR image từ OpenCV)

        Returns:
            list: Danh sách các detection [{"class", "confidence", "bbox"}]
        """
        # TODO: Implement detection pipeline
        raise NotImplementedError
