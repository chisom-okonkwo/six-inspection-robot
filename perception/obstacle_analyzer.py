import numpy as np

from perception.object_policy import (
    OBSTACLE_CLASSES,
    DEFAULT_OBSTACLE_MIN_CONFIDENCE,
    OBSTACLE_MIN_CONFIDENCE,
)


class ObstacleAnalyzer:
    """
    Semantic candidate analyzer for ultrasonic fusion.

    NEW responsibility split:

    Camera / YOLO:
        WHAT is in front of Six?
        Is it one of the configured navigation classes
        (currently backpack/book)?
        Which side is it on?
        Does its bounding box overlap the forward corridor?

    HC-SR04:
        WHEN is that candidate physically close enough
        to trigger avoidance?

    Therefore this analyzer deliberately does NOT reject a
    backpack/book just because monocular depth estimates it
    as far away. Distance is metadata here, not the final
    navigation gate.
    """

    def __init__(
        self,
        corridor_left_ratio=0.28,
        corridor_right_ratio=0.72,

        # Require some meaningful overlap with the
        # forward travel corridor.
        min_corridor_overlap=0.10
    ):

        self.corridor_left_ratio = (
            corridor_left_ratio
        )

        self.corridor_right_ratio = (
            corridor_right_ratio
        )

        self.min_corridor_overlap = (
            min_corridor_overlap
        )

    # ==================================================
    # HELPERS
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
            min(end1, end2)
            -
            max(start1, start2)
        )

    @staticmethod
    def _valid_depth_values(
        region
    ):

        if (
            region is None
            or
            region.size == 0
        ):

            return np.array(
                [],
                dtype=np.float32
            )

        mask = (
            np.isfinite(region)
            &
            (region > 0.05)
            &
            (region < 20.0)
        )

        return region[
            mask
        ]

    # ==================================================
    # OPTIONAL CAMERA DEPTH METADATA
    # ==================================================

    def _object_depth(
        self,
        depth_map,
        bbox
    ):
        """
        Estimate object depth for logging/debugging.

        This value does NOT decide whether navigation
        begins. The HC-SR04 handles that.
        """

        if depth_map is None:
            return None

        height, width = (
            depth_map.shape[:2]
        )

        x1, y1, x2, y2 = bbox

        x1 = max(
            0,
            min(width - 1, x1)
        )

        x2 = max(
            0,
            min(width, x2)
        )

        y1 = max(
            0,
            min(height - 1, y1)
        )

        y2 = max(
            0,
            min(height, y2)
        )

        if (
            x2 <= x1
            or
            y2 <= y1
        ):

            return None

        box_height = (
            y2 - y1
        )

        lower_y = int(
            y1
            +
            0.45 * box_height
        )

        region = depth_map[
            lower_y:y2,
            x1:x2
        ]

        valid = (
            self._valid_depth_values(
                region
            )
        )

        if valid.size < 10:
            return None

        return float(
            np.percentile(
                valid,
                25
            )
        )

    # --------------------------------------------------

    def _corridor_clearance(
        self,
        depth_map
    ):
        """
        Optional left/right clearance estimates.

        These remain useful for choosing which direction
        Six should turn, even though ultrasonic decides
        when avoidance starts.
        """

        if depth_map is None:
            return (
                None,
                None
            )

        height, width = (
            depth_map.shape[:2]
        )

        corridor_left = int(
            width
            * self.corridor_left_ratio
        )

        corridor_right = int(
            width
            * self.corridor_right_ratio
        )

        center = (
            corridor_left
            +
            (
                corridor_right
                -
                corridor_left
            )
            // 2
        )

        top = int(
            height * 0.22
        )

        bottom = int(
            height * 0.68
        )

        left_values = (
            self._valid_depth_values(
                depth_map[
                    top:bottom,
                    corridor_left:center
                ]
            )
        )

        right_values = (
            self._valid_depth_values(
                depth_map[
                    top:bottom,
                    center:corridor_right
                ]
            )
        )

        left_clearance = (
            float(
                np.median(
                    left_values
                )
            )
            if left_values.size >= 10
            else None
        )

        right_clearance = (
            float(
                np.median(
                    right_values
                )
            )
            if right_values.size >= 10
            else None
        )

        return (
            left_clearance,
            right_clearance
        )

    # ==================================================
    # MAIN ANALYSIS
    # ==================================================

    def analyze(
        self,
        frame,
        detections,
        depth_map=None
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

        candidates = []

        for detection in detections:

            label = (
                detection["label"]
            )

            confidence = float(
                detection["confidence"]
            )

            # Human safety is handled independently.
            if label == "person":
                continue

            # This is the semantic navigation policy.
            # With your current object_policy.py this
            # means only backpack and book.
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

            center_ratio = (
                (
                    x1 + x2
                )
                / 2
                / width
            )

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

            # Object exists but is clearly beside the
            # projected travel corridor.
            if (
                corridor_overlap
                <
                self.min_corridor_overlap
            ):

                continue

            if center_ratio < 0.42:

                side = "left"

            elif center_ratio > 0.58:

                side = "right"

            else:

                side = "center"

            camera_depth_m = (
                self._object_depth(
                    depth_map,
                    detection["bbox"]
                )
            )

            # Risk is semantic/corridor relevance only.
            # It is NOT the final distance decision.
            risk = (
                confidence
                *
                (
                    0.65
                    +
                    0.35
                    * min(
                        1.0,
                        corridor_overlap
                    )
                )
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

                "distance_m":
                    camera_depth_m,

                "source":
                    "semantic"
            })

        if not candidates:
            return None

        primary = max(
            candidates,
            key=lambda item:
                item["risk"]
        )

        (
            left_clearance,
            right_clearance
        ) = self._corridor_clearance(
            depth_map
        )

        # ==================================================
        # TURN DIRECTION
        # ==================================================

        if (
            left_clearance is not None
            and
            right_clearance is not None
            and
            abs(
                left_clearance
                -
                right_clearance
            )
            >= 0.10
        ):

            recommended_escape = (
                "left"
                if left_clearance
                >
                right_clearance
                else "right"
            )

        elif primary["side"] == "left":

            recommended_escape = (
                "right"
            )

        elif primary["side"] == "right":

            recommended_escape = (
                "left"
            )

        else:

            recommended_escape = (
                "left"
            )

        return {
            "primary":
                primary,

            "risk":
                primary["risk"],

            # Camera depth remains useful for logs and
            # debugging, but does not trigger navigation.
            "distance_m":
                primary["distance_m"],

            "source":
                "semantic",

            "requires_backoff":
                False,

            "recommended_escape":
                recommended_escape,

            "left_score":
                0.0,

            "right_score":
                0.0,

            "left_clearance_m":
                left_clearance,

            "right_clearance_m":
                right_clearance,

            "candidate_count":
                len(candidates),

            "depth_summary":
                None,
        }

    # ==================================================
    # DEBUG OVERLAY
    # ==================================================

    def draw_debug(
        self,
        frame,
        analysis
    ):

        # Keep this intentionally light because YOLO's
        # result.plot() already draws object boxes.
        return frame
