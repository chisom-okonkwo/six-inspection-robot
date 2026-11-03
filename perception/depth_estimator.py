from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_DEPTH_MODEL = PROJECT_ROOT / "models" / "yolo26n-depth.pt"


class DepthEstimator:
    """Optional monocular depth estimator."""

    def __init__(self, model_path=DEFAULT_DEPTH_MODEL, imgsz=384):
        self.model_path = Path(model_path)
        self.imgsz = imgsz
        self.model = None
        self.available = False

        try:
            print(f"[DEPTH] Loading model: {self.model_path}")
            self.model = YOLO(str(self.model_path))
            self.available = True
            print("[DEPTH] Model ready")
        except Exception as error:
            print("\n[DEPTH] Depth model unavailable.")
            print("[DEPTH] Continuing without monocular depth.")
            print("[DEPTH] Reason:", error, "\n")

    def predict(self, frame):
        """Return a float32 HxW depth map in meters, or None."""
        if not self.available:
            return None

        results = self.model.predict(
            source=frame,
            imgsz=self.imgsz,
            verbose=False,
        )
        result = results[0]

        if not hasattr(result, "depth") or result.depth is None:
            raise RuntimeError(
                "Depth model returned no depth map. "
                "Check your Ultralytics version/model."
            )

        depth_map = result.depth.data.cpu().numpy().astype(np.float32)
        frame_h, frame_w = frame.shape[:2]

        if depth_map.shape != (frame_h, frame_w):
            depth_map = cv2.resize(
                depth_map,
                (frame_w, frame_h),
                interpolation=cv2.INTER_LINEAR,
            )

        return depth_map
