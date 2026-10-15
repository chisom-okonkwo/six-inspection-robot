from perception.object_policy import (
    OBSTACLE_CLASSES,
    SMALL_FLOOR_CLASSES,
    DEFAULT_OBSTACLE_MIN_CONFIDENCE,
    OBSTACLE_MIN_CONFIDENCE,
)


class ObstacleAnalyzer:

    def __init__(
        self,

        # Approximate width of Six's travel
        # corridor inside the image.
        corridor_left_ratio=0.28,
        corridor_right_ratio=0.72,

        min_bottom_ratio=0.55,

        normal_min_area_ratio=0.008,

        small_min_area_ratio=0.0025,

        min_risk=0.22
    ):

        self.corridor_left_ratio = (
            corridor_left_ratio
        )

        self.corridor_right_ratio = (
            corridor_right_ratio
        )

        self.min_bottom_ratio = (
            min_bottom_ratio
        )

        self.normal_min_area_ratio = (
            normal_min_area_ratio
        )

        self.small_min_area_ratio = (
            small_min_area_ratio
        )

        self.min_risk = min_risk

    # ==================================================

    @staticmethod
    def _clamp01(value):

        return max(
            0.0,
            min(
                1.0,
                value
            )
        )

    # ==================================================

    @staticmethod
    def _overlap(
        start1,
        end1,
        start2,
        end2
    ):

        return max(
            0,
            min(
                end1,
                end2
            )
            -
            max(
                start1,
                start2
            )
        )

    # ==================================================

    def analyze(
        self,
        frame,
        detections
    ):

        height, width = (
            frame.shape[:2]
        )

        frame_area = (
            width * height
        )

        corridor_left = (
            width
            * self.corridor_left_ratio
        )

        corridor_right = (
            width
            * self.corridor_right_ratio
        )

        center_x = (
            width / 2
        )

        candidates = []

        left_score = 0.0
        right_score = 0.0

        for detection in detections:

            label = (
                detection["label"]
            )

            confidence = (
                detection["confidence"]
            )

            # Person safety is handled separately.
            if label == "person":
                continue

            # This is the configurable policy.
            if (
                label
                not in OBSTACLE_CLASSES
            ):
                continue

            minimum_confidence = (
                OBSTACLE_MIN_CONFIDENCE.get(
                    label,
                    DEFAULT_OBSTACLE_MIN_CONFIDENCE
                )
            )

            if (
                confidence
                < minimum_confidence
            ):
                continue

            x1, y1, x2, y2 = (
                detection["bbox"]
            )

            box_width = max(
                1,
                x2 - x1
            )

            box_height = max(
                1,
                y2 - y1
            )

            area_ratio = (
                box_width
                * box_height
                / frame_area
            )

            bottom_ratio = (
                y2 / height
            )

            object_center = (
                (x1 + x2)
                / 2
            )

            center_ratio = (
                object_center
                / width
            )

            # --------------------------------------
            # Does it intersect our path?
            # --------------------------------------

            corridor_overlap_pixels = (
                self._overlap(
                    x1,
                    x2,
                    corridor_left,
                    corridor_right
                )
            )

            corridor_overlap = (
                corridor_overlap_pixels
                / box_width
            )

            if corridor_overlap <= 0:

                # It can remain visible on the side
                # without blocking the robot.
                continue

            # --------------------------------------
            # Proximity
            # --------------------------------------

            if (
                bottom_ratio
                < self.min_bottom_ratio
            ):

                # Likely far away / high in image.
                continue

            # --------------------------------------
            # Small-object handling
            # --------------------------------------

            if (
                label
                in SMALL_FLOOR_CLASSES
            ):

                minimum_area = (
                    self.small_min_area_ratio
                )

            else:

                minimum_area = (
                    self.normal_min_area_ratio
                )

            # Very close objects can matter even
            # with a tiny bounding box.
            if (
                area_ratio
                < minimum_area
                and
                bottom_ratio
                < 0.80
            ):

                continue

            # --------------------------------------
            # Risk score
            # --------------------------------------

            proximity_score = (
                self._clamp01(
                    (
                        bottom_ratio
                        - self.min_bottom_ratio
                    )
                    /
                    (
                        1.0
                        - self.min_bottom_ratio
                    )
                )
            )

            size_score = (
                self._clamp01(
                    area_ratio
                    / 0.10
                )
            )

            center_score = (
                1.0
                -
                self._clamp01(
                    abs(
                        center_ratio
                        - 0.5
                    )
                    / 0.5
                )
            )

            risk = confidence * (
                0.45
                * proximity_score

                +

                0.30
                * corridor_overlap

                +

                0.15
                * size_score

                +

                0.10
                * center_score
            )

            if risk < self.min_risk:
                continue

            # --------------------------------------
            # Determine side
            # --------------------------------------

            if center_ratio < 0.42:

                side = "left"

            elif center_ratio > 0.58:

                side = "right"

            else:

                side = "center"

            # --------------------------------------
            # Measure available left/right space
            # --------------------------------------

            left_overlap = (
                self._overlap(
                    x1,
                    x2,
                    corridor_left,
                    center_x
                )
                / box_width
            )

            right_overlap = (
                self._overlap(
                    x1,
                    x2,
                    center_x,
                    corridor_right
                )
                / box_width
            )

            left_score += (
                risk
                * left_overlap
            )

            right_score += (
                risk
                * right_overlap
            )

            candidates.append({
                **detection,

                "side":
                    side,

                "risk":
                    risk,

                "area_ratio":
                    area_ratio,

                "bottom_ratio":
                    bottom_ratio,

                "corridor_overlap":
                    corridor_overlap,
            })

        if not candidates:

            return None

        primary = max(
            candidates,
            key=lambda item:
                item["risk"]
        )

        # ==========================================
        # ESCAPE DIRECTION
        # ==========================================

        if primary["side"] == "left":

            escape = "right"

        elif primary["side"] == "right":

            escape = "left"

        else:

            # Center obstacle:
            # choose side with less obstruction.
            if left_score <= right_score:

                escape = "left"

            else:

                escape = "right"

        return {
            "primary":
                primary,

            "risk":
                primary["risk"],

            "recommended_escape":
                escape,

            "left_score":
                left_score,

            "right_score":
                right_score,

            "candidate_count":
                len(candidates),
        }