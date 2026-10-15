import threading
import time


class RobotState:

    def __init__(self):

        self._lock = (
            threading.Lock()
        )

        # ==========================================
        # PERSON
        # ==========================================

        self.person_detected = False
        self.person_confidence = 0.0

        self.person_detected_at = None
        self.person_cleared_at = None

        # ==========================================
        # OBSTACLE
        # ==========================================

        self.obstacle_detected = False

        self.obstacle_label = None
        self.obstacle_confidence = 0.0
        self.obstacle_side = None

        self.obstacle_area_ratio = 0.0
        self.obstacle_risk = 0.0

        self.obstacle_escape_direction = None

        self.obstacle_left_score = 0.0
        self.obstacle_right_score = 0.0

        # ==========================================
        # PERCEPTION
        # ==========================================

        self.perception_fps = 0.0
        self.perception_ready = False

        # Lets navigation wait for genuinely fresh
        # perception after physically moving.
        self.perception_frame_id = 0

        # ==========================================
        # SYSTEM STATES
        # ==========================================

        self.perception_status = (
            "NOT_STARTED"
        )

        self.camera_status = (
            "DISCONNECTED"
        )

        self.ai_status = (
            "NOT_LOADED"
        )

        self.safety_state = "CLEAR"

        self.motion_state = (
            "DISABLED"
        )

        self.system_state = (
            "STARTING"
        )

        self.last_error = None

        self.latest_frame = None

    # ==================================================

    def update_perception(
        self,
        person_detected,
        confidence,
        fps,
        frame=None
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

    # ==================================================

    def update_obstacle(
        self,
        detected,
        label=None,
        confidence=0.0,
        side=None,
        area_ratio=0.0,
        risk=0.0,
        escape_direction=None,
        left_score=0.0,
        right_score=0.0
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

            self.obstacle_escape_direction = (
                escape_direction
            )

            self.obstacle_left_score = (
                left_score
            )

            self.obstacle_right_score = (
                right_score
            )

    # ==================================================

    def set_perception_status(
        self,
        value
    ):

        with self._lock:
            self.perception_status = value

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
            self.last_error = str(
                error
            )

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

                "obstacle_escape_direction":
                    self.obstacle_escape_direction,

                "obstacle_left_score":
                    self.obstacle_left_score,

                "obstacle_right_score":
                    self.obstacle_right_score,

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

    def get_frame(self):

        with self._lock:

            if self.latest_frame is None:

                return None

            return (
                self.latest_frame.copy()
            )