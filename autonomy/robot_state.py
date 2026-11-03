import threading
import time


class RobotState:

    def __init__(self):

        self._lock = threading.Lock()

        # ==================================================
        # PERSON
        # ==================================================

        self.person_detected = False
        self.person_confidence = 0.0

        self.person_detected_at = None
        self.person_cleared_at = None

        # ==================================================
        # CAMERA / VISION OBSTACLE
        # ==================================================

        self.obstacle_detected = False

        self.obstacle_label = None
        self.obstacle_confidence = 0.0

        self.obstacle_side = None
        self.obstacle_area_ratio = 0.0

        self.obstacle_risk = 0.0
        self.obstacle_distance_m = None
        self.obstacle_source = None

        self.obstacle_escape_direction = None
        self.obstacle_requires_backoff = False

        self.obstacle_left_score = 0.0
        self.obstacle_right_score = 0.0

        self.left_clearance_m = None
        self.right_clearance_m = None

        # ==================================================
        # DEPTH
        # ==================================================

        self.corridor_depth_m = None
        self.corridor_median_depth_m = None

        # ==================================================
        # HC-SR04
        # ==================================================

        self.ultrasonic_status = "NOT_STARTED"

        self.ultrasonic_raw_cm = None
        self.ultrasonic_distance_cm = None

        self.ultrasonic_valid = False
        self.ultrasonic_close = False

        # True when the camera has recently identified
        # a backpack/book candidate.
        self.ultrasonic_vision_candidate = False

        # True when the fused camera + range gate has
        # actually authorized obstacle avoidance.
        self.ultrasonic_navigation_confirmed = False

        self.ultrasonic_updated_at = None

        # ==================================================
        # PROGRESS / STUCK DETECTION
        # ==================================================

        self.visual_motion_score = 0.0
        self.visual_feature_count = 0
        self.progress_status = "UNKNOWN"

        # ==================================================
        # PERCEPTION
        # ==================================================

        self.perception_fps = 0.0
        self.perception_ready = False

        self.perception_frame_id = 0

        self.perception_status = "NOT_STARTED"
        self.camera_status = "DISCONNECTED"
        self.ai_status = "NOT_LOADED"
        self.depth_status = "NOT_LOADED"

        self.latest_frame = None
        self.latest_raw_frame = None

        # ==================================================
        # SYSTEM
        # ==================================================

        self.safety_state = "CLEAR"
        self.motion_state = "DISABLED"
        self.system_state = "STARTING"

        self.last_error = None

    # ==================================================
    # PERCEPTION UPDATE
    # ==================================================

    def update_perception(
        self,
        person_detected,
        confidence,
        fps,
        frame=None,
        raw_frame=None
    ):

        now = time.perf_counter()

        with self._lock:

            previous = (
                self.person_detected
            )

            self.person_detected = (
                person_detected
            )

            self.person_confidence = (
                confidence
            )

            self.perception_fps = fps
            self.perception_ready = True

            self.perception_frame_id += 1

            if (
                person_detected
                and
                not previous
            ):
                self.person_detected_at = (
                    now
                )

            if (
                not person_detected
                and
                previous
            ):
                self.person_cleared_at = (
                    now
                )

            if frame is not None:
                self.latest_frame = (
                    frame.copy()
                )

            if raw_frame is not None:
                self.latest_raw_frame = (
                    raw_frame.copy()
                )

    # ==================================================
    # CAMERA OBSTACLE UPDATE
    # ==================================================

    def update_obstacle(
        self,
        detected,
        label=None,
        confidence=0.0,
        side=None,
        area_ratio=0.0,
        risk=0.0,
        distance_m=None,
        source=None,
        escape_direction=None,
        requires_backoff=False,
        left_score=0.0,
        right_score=0.0,
        left_clearance_m=None,
        right_clearance_m=None
    ):

        with self._lock:

            self.obstacle_detected = (
                detected
            )

            self.obstacle_label = label
            self.obstacle_confidence = (
                confidence
            )

            self.obstacle_side = side
            self.obstacle_area_ratio = (
                area_ratio
            )

            self.obstacle_risk = risk
            self.obstacle_distance_m = (
                distance_m
            )

            self.obstacle_source = source

            self.obstacle_escape_direction = (
                escape_direction
            )

            self.obstacle_requires_backoff = (
                requires_backoff
            )

            self.obstacle_left_score = (
                left_score
            )

            self.obstacle_right_score = (
                right_score
            )

            self.left_clearance_m = (
                left_clearance_m
            )

            self.right_clearance_m = (
                right_clearance_m
            )

    # ==================================================
    # DEPTH
    # ==================================================

    def set_corridor_depth(
        self,
        p25_m=None,
        median_m=None
    ):

        with self._lock:

            self.corridor_depth_m = (
                p25_m
            )

            self.corridor_median_depth_m = (
                median_m
            )

    # ==================================================
    # HC-SR04
    # ==================================================

    def set_ultrasonic_status(
        self,
        value
    ):

        with self._lock:

            self.ultrasonic_status = (
                value
            )

    def update_ultrasonic(
        self,
        raw_cm=None,
        distance_cm=None,
        valid=False,
        close=False,
        vision_candidate=False,
        navigation_confirmed=False
    ):

        with self._lock:

            self.ultrasonic_raw_cm = (
                raw_cm
            )

            self.ultrasonic_distance_cm = (
                distance_cm
            )

            self.ultrasonic_valid = bool(
                valid
            )

            self.ultrasonic_close = bool(
                close
            )

            self.ultrasonic_vision_candidate = bool(
                vision_candidate
            )

            self.ultrasonic_navigation_confirmed = bool(
                navigation_confirmed
            )

            self.ultrasonic_updated_at = (
                time.perf_counter()
            )

    # ==================================================
    # PROGRESS / STUCK
    # ==================================================

    def set_progress(
        self,
        motion_score,
        feature_count,
        status
    ):

        with self._lock:

            self.visual_motion_score = (
                float(motion_score)
            )

            self.visual_feature_count = (
                int(feature_count)
            )

            self.progress_status = (
                status
            )

    # ==================================================
    # SIMPLE STATE SETTERS
    # ==================================================

    def set_perception_status(
        self,
        value
    ):

        with self._lock:

            self.perception_status = (
                value
            )

    def set_camera_status(
        self,
        value
    ):

        with self._lock:

            self.camera_status = value

    def set_ai_status(
        self,
        value
    ):

        with self._lock:

            self.ai_status = value

    def set_depth_status(
        self,
        value
    ):

        with self._lock:

            self.depth_status = value

    def set_safety(
        self,
        value
    ):

        with self._lock:

            self.safety_state = value

    def set_motion(
        self,
        value
    ):

        with self._lock:

            self.motion_state = value

    def set_system(
        self,
        value
    ):

        with self._lock:

            self.system_state = value

    def set_error(
        self,
        error
    ):

        with self._lock:

            self.last_error = (
                str(error)
            )

    # ==================================================
    # SNAPSHOT
    # ==================================================

    def snapshot(self):

        with self._lock:

            return {
                "person_detected":
                    self.person_detected,

                "person_confidence":
                    self.person_confidence,

                "person_detected_at":
                    self.person_detected_at,

                "person_cleared_at":
                    self.person_cleared_at,

                "obstacle_detected":
                    self.obstacle_detected,

                "obstacle_label":
                    self.obstacle_label,

                "obstacle_confidence":
                    self.obstacle_confidence,

                "obstacle_side":
                    self.obstacle_side,

                "obstacle_area_ratio":
                    self.obstacle_area_ratio,

                "obstacle_risk":
                    self.obstacle_risk,

                "obstacle_distance_m":
                    self.obstacle_distance_m,

                "obstacle_source":
                    self.obstacle_source,

                "obstacle_escape_direction":
                    self.obstacle_escape_direction,

                "obstacle_requires_backoff":
                    self.obstacle_requires_backoff,

                "obstacle_left_score":
                    self.obstacle_left_score,

                "obstacle_right_score":
                    self.obstacle_right_score,

                "left_clearance_m":
                    self.left_clearance_m,

                "right_clearance_m":
                    self.right_clearance_m,

                "corridor_depth_m":
                    self.corridor_depth_m,

                "corridor_median_depth_m":
                    self.corridor_median_depth_m,

                "ultrasonic_status":
                    self.ultrasonic_status,

                "ultrasonic_raw_cm":
                    self.ultrasonic_raw_cm,

                "ultrasonic_distance_cm":
                    self.ultrasonic_distance_cm,

                "ultrasonic_valid":
                    self.ultrasonic_valid,

                "ultrasonic_close":
                    self.ultrasonic_close,

                "ultrasonic_vision_candidate":
                    self.ultrasonic_vision_candidate,

                "ultrasonic_navigation_confirmed":
                    self.ultrasonic_navigation_confirmed,

                "ultrasonic_updated_at":
                    self.ultrasonic_updated_at,

                "visual_motion_score":
                    self.visual_motion_score,

                "visual_feature_count":
                    self.visual_feature_count,

                "progress_status":
                    self.progress_status,

                "perception_fps":
                    self.perception_fps,

                "perception_ready":
                    self.perception_ready,

                "perception_frame_id":
                    self.perception_frame_id,

                "perception_status":
                    self.perception_status,

                "camera_status":
                    self.camera_status,

                "ai_status":
                    self.ai_status,

                "depth_status":
                    self.depth_status,

                "safety_state":
                    self.safety_state,

                "motion_state":
                    self.motion_state,

                "system_state":
                    self.system_state,

                "last_error":
                    self.last_error,
            }

    # ==================================================
    # FRAMES
    # ==================================================

    def get_frame(self):

        with self._lock:

            if self.latest_frame is None:
                return None

            return (
                self.latest_frame.copy()
            )

    def get_raw_frame(self):

        with self._lock:

            if (
                self.latest_raw_frame
                is None
            ):
                return None

            return (
                self.latest_raw_frame.copy()
            )
