import threading
import time

import cv2
import numpy as np


class ProgressMonitor(threading.Thread):
    """
    Detect possible "commanded forward but not making progress"
    conditions using the robot's raw camera frames.

    This is a fallback, not odometry. It combines sustained
    low visual motion with the forward depth estimate when
    available. A physical range sensor/IMU will be more reliable.
    """

    def __init__(
        self,
        state,
        stuck_event,
        obstacle_event,
        safety_stop_event,
        shutdown_event,
        sample_interval=0.25,
        min_motion_score=0.65,
        min_features=8,
        close_depth_m=0.90,
        close_stuck_seconds=1.50,
        normal_stuck_seconds=3.25,
        max_close_depth_change_m=0.08,
    ):
        super().__init__(
            name="ProgressMonitor",
            daemon=True
        )

        self.state = state
        self.stuck_event = stuck_event
        self.obstacle_event = obstacle_event
        self.safety_stop_event = safety_stop_event
        self.shutdown_event = shutdown_event

        self.sample_interval = sample_interval
        self.min_motion_score = min_motion_score
        self.min_features = min_features

        self.close_depth_m = close_depth_m
        self.close_stuck_seconds = close_stuck_seconds
        self.normal_stuck_seconds = normal_stuck_seconds
        self.max_close_depth_change_m = max_close_depth_change_m

    @staticmethod
    def _prepare(frame):
        height, width = frame.shape[:2]

        target_width = 320
        scale = (
            target_width / width
            if width > target_width
            else 1.0
        )

        if scale != 1.0:
            frame = cv2.resize(
                frame,
                (
                    target_width,
                    int(height * scale)
                ),
                interpolation=cv2.INTER_AREA
            )

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        return cv2.GaussianBlur(
            gray,
            (5, 5),
            0
        )

    def _motion_score(
        self,
        previous_gray,
        current_gray
    ):
        points = cv2.goodFeaturesToTrack(
            previous_gray,
            maxCorners=100,
            qualityLevel=0.01,
            minDistance=7,
            blockSize=7
        )

        if points is None:
            return 0.0, 0

        next_points, status, _ = (
            cv2.calcOpticalFlowPyrLK(
                previous_gray,
                current_gray,
                points,
                None,
                winSize=(21, 21),
                maxLevel=3,
                criteria=(
                    cv2.TERM_CRITERIA_EPS
                    |
                    cv2.TERM_CRITERIA_COUNT,
                    30,
                    0.01
                )
            )
        )

        if (
            next_points is None
            or status is None
        ):
            return 0.0, 0

        mask = status.flatten() == 1

        good_old = (
            points[mask]
            .reshape(-1, 2)
        )

        good_new = (
            next_points[mask]
            .reshape(-1, 2)
        )

        count = len(good_old)

        if count == 0:
            return 0.0, 0

        displacement = np.linalg.norm(
            good_new - good_old,
            axis=1
        )

        return (
            float(np.median(displacement)),
            count
        )

    def run(self):
        print(
            "[PROGRESS] Monitor online"
        )

        previous_gray = None
        previous_frame_id = -1

        low_motion_since = None
        depth_at_low_motion_start = None

        try:
            while (
                not
                self.shutdown_event.is_set()
            ):
                snapshot = (
                    self.state.snapshot()
                )

                motion_state = (
                    snapshot["motion_state"]
                )

                frame_id = (
                    snapshot[
                        "perception_frame_id"
                    ]
                )

                should_monitor = (
                    motion_state == "PATROLLING"
                    and
                    not self.safety_stop_event.is_set()
                    and
                    not self.obstacle_event.is_set()
                )

                if not should_monitor:
                    previous_gray = None
                    low_motion_since = None
                    depth_at_low_motion_start = None

                    # The motion worker owns stuck_event
                    # while actively recovering.
                    if (
                        motion_state
                        not in (
                            "STUCK_RECOVERY",
                            "BACKING_OFF",
                            "OBSTACLE_AVOID"
                        )
                    ):
                        self.stuck_event.clear()

                    self.state.set_progress(
                        motion_score=0.0,
                        feature_count=0,
                        status="INACTIVE"
                    )

                    self.shutdown_event.wait(
                        self.sample_interval
                    )
                    continue

                if frame_id == previous_frame_id:
                    self.shutdown_event.wait(
                        0.05
                    )
                    continue

                frame = (
                    self.state.get_raw_frame()
                )

                if frame is None:
                    self.shutdown_event.wait(
                        0.05
                    )
                    continue

                current_gray = (
                    self._prepare(frame)
                )

                previous_frame_id = frame_id

                if previous_gray is None:
                    previous_gray = current_gray

                    self.shutdown_event.wait(
                        self.sample_interval
                    )
                    continue

                (
                    score,
                    feature_count
                ) = self._motion_score(
                    previous_gray,
                    current_gray
                )

                previous_gray = current_gray

                corridor_depth = (
                    snapshot.get(
                        "corridor_depth_m"
                    )
                )

                close_surface = (
                    corridor_depth is not None
                    and
                    corridor_depth
                    <= self.close_depth_m
                )

                low_motion = (
                    score
                    < self.min_motion_score
                    or
                    feature_count
                    < self.min_features
                )

                if low_motion:
                    if low_motion_since is None:
                        low_motion_since = (
                            time.perf_counter()
                        )
                        depth_at_low_motion_start = (
                            corridor_depth
                        )

                    elapsed = (
                        time.perf_counter()
                        - low_motion_since
                    )

                    required_duration = (
                        self.close_stuck_seconds
                        if close_surface
                        else self.normal_stuck_seconds
                    )

                    # When a close depth estimate exists,
                    # require it to remain roughly stable.
                    # If it is changing substantially,
                    # the robot is probably still moving.
                    close_depth_stable = True

                    if (
                        close_surface
                        and
                        corridor_depth is not None
                        and
                        depth_at_low_motion_start
                        is not None
                    ):
                        close_depth_stable = (
                            abs(
                                corridor_depth
                                -
                                depth_at_low_motion_start
                            )
                            <=
                            self.max_close_depth_change_m
                        )

                    status = (
                        "LOW_MOTION_CLOSE"
                        if close_surface
                        else "LOW_MOTION"
                    )

                    self.state.set_progress(
                        motion_score=score,
                        feature_count=feature_count,
                        status=status
                    )

                    if (
                        elapsed >= required_duration
                        and
                        (
                            not close_surface
                            or close_depth_stable
                        )
                        and
                        not self.stuck_event.is_set()
                    ):
                        self.stuck_event.set()

                        self.state.set_progress(
                            motion_score=score,
                            feature_count=
                                feature_count,
                            status="STUCK"
                        )

                        print()
                        print(
                            "[PROGRESS] "
                            "Possible no-progress condition"
                        )
                        print(
                            "[PROGRESS] "
                            f"Optical-flow score: "
                            f"{score:.3f}"
                        )

                        if corridor_depth is not None:
                            print(
                                "[PROGRESS] "
                                "Forward depth: "
                                f"{corridor_depth:.2f}m"
                            )

                else:
                    low_motion_since = None
                    depth_at_low_motion_start = None

                    self.state.set_progress(
                        motion_score=score,
                        feature_count=feature_count,
                        status="MOVING"
                    )

                self.shutdown_event.wait(
                    self.sample_interval
                )

        except Exception as error:
            print(
                "[PROGRESS] ERROR:",
                error
            )

            self.state.set_error(
                error
            )

        finally:
            print(
                "[PROGRESS] Monitor stopped"
            )
