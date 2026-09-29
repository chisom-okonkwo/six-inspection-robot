class ObstacleAnalyzer:

    DEFAULT_OBSTACLE_LABELS = {
        "chair",
        "couch",
        "bench",
        "backpack",
        "suitcase",
        "potted plant",
        "dining table",
        "bed",
        "dog",
        "cat",
    }

    def __init__(
        self,
        obstacle_labels=None,
        min_area_ratio=0.025,
        min_bottom_ratio=0.55
    ):

        self.obstacle_labels = (
            obstacle_labels
            or self.DEFAULT_OBSTACLE_LABELS
        )

        self.min_area_ratio = (
            min_area_ratio
        )

        self.min_bottom_ratio = (
            min_bottom_ratio
        )

    # ==================================================

    def analyze(
        self,
        frame,
        detections
    ):

        frame_height, frame_width = (
            frame.shape[:2]
        )

        frame_area = (
            frame_width
            * frame_height
        )

        candidates = []

        for detection in detections:

            label = (
                detection["label"]
            )

            # Human detections are controlled by
            # the higher-priority safety system.
            if label == "person":
                continue

            if (
                label
                not in self.obstacle_labels
            ):
                continue

            x1, y1, x2, y2 = (
                detection["bbox"]
            )

            width = max(
                0,
                x2 - x1
            )

            height = max(
                0,
                y2 - y1
            )

            box_area = (
                width * height
            )

            area_ratio = (
                box_area
                / frame_area
            )

            bottom_ratio = (
                y2
                / frame_height
            )

            # Ignore small / distant detections.
            if (
                area_ratio
                < self.min_area_ratio
            ):
                continue

            # Ignore objects that remain high
            # in the camera frame.
            if (
                bottom_ratio
                < self.min_bottom_ratio
            ):
                continue

            center_x = (
                x1 + x2
            ) / 2

            center_ratio = (
                center_x
                / frame_width
            )

            # Ignore objects well outside the
            # approximate forward travel corridor.
            if (
                center_ratio < 0.15
                or
                center_ratio > 0.85
            ):
                continue

            if center_ratio < 0.42:

                side = "left"

            elif center_ratio > 0.58:

                side = "right"

            else:

                side = "center"

            # Heuristic:
            # bigger + lower in image means
            # more likely to block Six.
            score = (
                area_ratio
                * bottom_ratio
            )

            candidates.append({
                **detection,

                "side":
                    side,

                "area_ratio":
                    area_ratio,

                "bottom_ratio":
                    bottom_ratio,

                "score":
                    score,
            })

        if not candidates:

            return None

        return max(
            candidates,
            key=lambda obstacle:
                obstacle["score"]
        )