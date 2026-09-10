import threading
import time


class RobotState:

    def __init__(self):

        self._lock = threading.Lock()

        self.person_detected = False
        self.person_confidence = 0.0

        self.person_detected_at = None
        self.person_cleared_at = None

        self.perception_fps = 0.0
        self.perception_ready = False

        self.safety_state = "CLEAR"
        self.motion_state = "DISABLED"
        self.system_state = "STARTING"

        self.perception_status = "NOT_STARTED"
        self.camera_status = "DISCONNECTED"
        self.ai_status = "NOT_LOADED"

        self.last_error = None

        self.latest_frame = None

    # --------------------------------------------------

    def update_perception(
        self,
        person_detected,
        confidence,
        fps,
        frame=None
    ):

        now = time.perf_counter()

        with self._lock:

            previous = self.person_detected

            self.person_detected = (
                person_detected
            )

            self.person_confidence = (
                confidence
            )

            self.perception_fps = fps
            self.perception_ready = True

            if person_detected and not previous:
                self.person_detected_at = now

            if not person_detected and previous:
                self.person_cleared_at = now

            if frame is not None:
                self.latest_frame = frame.copy()

    # --------------------------------------------------

    def set_safety(self, value):

        with self._lock:
            self.safety_state = value

    def set_motion(self, value):

        with self._lock:
            self.motion_state = value

    def set_system(self, value):

        with self._lock:
            self.system_state = value

    def set_error(self, error):

        with self._lock:
            self.last_error = str(error)

    def set_perception_status(self, value):

        with self._lock:
            self.perception_status = value


    def set_camera_status(self, value):

        with self._lock:
            self.camera_status = value


    def set_ai_status(self, value):

        with self._lock:
            self.ai_status = value

    # --------------------------------------------------

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

                "perception_fps":
                    self.perception_fps,

                "perception_ready":
                    self.perception_ready,

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

    def get_frame(self):

        with self._lock:

            if self.latest_frame is None:
                return None

            return self.latest_frame.copy()
