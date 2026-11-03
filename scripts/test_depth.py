import time

import cv2
import numpy as np

from perception.camera import EZBCamera
from perception.depth_estimator import DepthEstimator


def main():
    camera = EZBCamera()
    depth = DepthEstimator()

    if not depth.available:
        print("Depth model is unavailable.")
        return

    try:
        camera.connect()
        print("\n==============================")
        print(" MONOCULAR DEPTH TEST")
        print("==============================")
        print("Q = quit\n")

        while True:
            frame = camera.read_frame()
            start = time.perf_counter()
            depth_map = depth.predict(frame)
            elapsed = time.perf_counter() - start

            h, w = depth_map.shape[:2]
            x1, x2 = int(w * 0.28), int(w * 0.72)
            y1, y2 = int(h * 0.22), int(h * 0.68)

            roi = depth_map[y1:y2, x1:x2]
            valid = roi[
                np.isfinite(roi)
                & (roi > 0.05)
                & (roi < 20.0)
            ]

            if valid.size:
                p25 = float(np.percentile(valid, 25))
                median = float(np.median(valid))
                label = (
                    f"corridor p25={p25:.2f}m "
                    f"median={median:.2f}m "
                    f"inference={elapsed:.2f}s"
                )
            else:
                label = "No valid depth values"

            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 1)
            cv2.putText(
                frame,
                label,
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1,
            )

            finite = depth_map[np.isfinite(depth_map) & (depth_map > 0)]
            if finite.size:
                lo = np.percentile(finite, 5)
                hi = np.percentile(finite, 95)
                normalized = np.clip(
                    (depth_map - lo) / max(hi - lo, 1e-6),
                    0,
                    1,
                )
                depth_image = (normalized * 255).astype(np.uint8)
                depth_image = cv2.applyColorMap(depth_image, cv2.COLORMAP_JET)
                cv2.imshow("Depth Map", depth_image)

            cv2.imshow("Six Camera + Depth ROI", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

    finally:
        camera.disconnect()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
