import threading
import time

from perception.camera import EZBCamera
from perception.detector import SafetyDetector
from perception.obstacle_analyzer import ObstacleAnalyzer


class PerceptionWorker(threading.Thread):

    def __init__(
        self,
        state,
        person_event,
        obstacle_event,
        shutdown_event,
        ready_event,
        startup_failed_event,
        model_path="yolo26n.pt",

        # This is the YOLO inference floor.
        # Person and obstacle policies apply
        # their own stricter thresholds later.
        confidence=0.25,

        detection_frames=2,
        clear_frames=4,

        obstacle_frames=2,
        obstacle_clear_frames=3
    ):

        super().__init__(
            name="PerceptionWorker",
            daemon=True
        )

        self.state = state

        self.person_event = person_event
        self.obstacle_event = obstacle_event

        self.shutdown_event = shutdown_event

        self.ready_event = ready_event

        self.startup_failed_event = (
            startup_failed_event
        )

        self.model_path = model_path
        self.confidence = confidence

        # Number of consecutive frames required
        # to confirm a person.
        self.detection_frames = (
            detection_frames
        )

        # Number of consecutive clear frames
        # required to clear person state.
        self.clear_frames = (
            clear_frames
        )

        # Same hysteresis mechanism for obstacles.
        self.obstacle_frames = (
            obstacle_frames
        )

        self.obstacle_clear_frames = (
            obstacle_clear_frames
        )

    # ==================================================
    # INTERNAL STATE UPDATE
    # ==================================================

    def _update_obstacle_state(
        self,
        obstacle_analysis
    ):
        """
        Copy the obstacle analyzer's output into
        the shared RobotState.

        obstacle_analysis has the structure:

        {
            "primary": {...},
            "risk": ...,
            "recommended_escape": ...,
            "left_score": ...,
            "right_score": ...,
            "candidate_count": ...
        }
        """

        if obstacle_analysis is None:

            if not self.obstacle_event.is_set():

                self.state.update_obstacle(
                    detected=False
                )

            # If hysteresis still considers an obstacle
            # active, retain the previous metadata.
            return

        primary = (
            obstacle_analysis[
                "primary"
            ]
        )

        self.state.update_obstacle(
            detected=
                self.obstacle_event.is_set(),

            label=
                primary["label"],

            confidence=
                primary["confidence"],

            side=
                primary["side"],

            area_ratio=
                primary["area_ratio"],

            risk=
                obstacle_analysis[
                    "risk"
                ],

            escape_direction=
                obstacle_analysis[
                    "recommended_escape"
                ],

            left_score=
                obstacle_analysis[
                    "left_score"
                ],

            right_score=
                obstacle_analysis[
                    "right_score"
                ]
        )

    # ==================================================
    # THREAD
    # ==================================================

    def run(self):

        camera = None

        try:

            # ==========================================
            # STEP 1 — LOAD AI
            # ==========================================

            self.state.set_perception_status(
                "STARTING"
            )

            self.state.set_ai_status(
                "LOADING"
            )

            print()
            print(
                "[PERCEPTION] Loading AI model..."
            )

            load_start = (
                time.perf_counter()
            )

            detector = SafetyDetector(
                model_path=
                    self.model_path,

                confidence=
                    self.confidence
            )

            obstacle_analyzer = (
                ObstacleAnalyzer()
            )

            load_time = (
                time.perf_counter()
                - load_start
            )

            self.state.set_ai_status(
                "READY"
            )

            print(
                "[PERCEPTION] AI model loaded "
                f"in {load_time:.2f}s"
            )

            if self.shutdown_event.is_set():
                return

            # ==========================================
            # STEP 2 — CONNECT CAMERA
            # ==========================================

            self.state.set_camera_status(
                "CONNECTING"
            )

            camera = EZBCamera(
                timeout=5.0
            )

            print(
                "[PERCEPTION] Connecting camera..."
            )

            camera_start = (
                time.perf_counter()
            )

            camera.connect()

            camera_time = (
                time.perf_counter()
                - camera_start
            )

            self.state.set_camera_status(
                "CONNECTED"
            )

            print(
                "[PERCEPTION] Camera connected "
                f"in {camera_time:.2f}s"
            )

            # ==========================================
            # STEP 3 — FIRST FRAME
            # ==========================================

            print(
                "[PERCEPTION] "
                "Waiting for first frame..."
            )

            frame_start = (
                time.perf_counter()
            )

            first_frame = (
                camera.read_frame()
            )

            frame_time = (
                time.perf_counter()
                - frame_start
            )

            print(
                "[PERCEPTION] "
                "First frame received "
                f"in {frame_time:.2f}s"
            )

            # ==========================================
            # STEP 4 — FIRST AI INFERENCE
            # ==========================================

            print(
                "[PERCEPTION] Running first "
                "AI inference..."
            )

            inference_start = (
                time.perf_counter()
            )

            # ------------------------------------------
            # One YOLO scene inference
            # ------------------------------------------

            detections, result = (
                detector.detect_scene(
                    first_frame
                )
            )

            # ------------------------------------------
            # Person policy
            # ------------------------------------------

            people = (
                detector.get_people(
                    detections
                )
            )

            # ------------------------------------------
            # Obstacle policy
            # ------------------------------------------

            obstacle_analysis = (
                obstacle_analyzer.analyze(
                    first_frame,
                    detections
                )
            )

            inference_time = (
                time.perf_counter()
                - inference_start
            )

            print(
                "[PERCEPTION] "
                "First inference completed "
                f"in {inference_time:.2f}s"
            )

            # ==========================================
            # INITIAL PERSON STATE
            # ==========================================

            best_person_confidence = 0.0

            if people:

                best_person_confidence = max(
                    detection["confidence"]
                    for detection
                    in people
                )

                # Fail-safe:
                # If a person exists at startup,
                # autonomous movement must not begin.
                self.person_event.set()

            else:

                self.person_event.clear()

            # ==========================================
            # INITIAL OBSTACLE STATE
            # ==========================================

            if (
                obstacle_analysis
                is not None
            ):

                # Startup is deliberately conservative:
                # if an obstacle already blocks the
                # corridor, treat it as confirmed.
                self.obstacle_event.set()

                self._update_obstacle_state(
                    obstacle_analysis
                )

            else:

                self.obstacle_event.clear()

                self.state.update_obstacle(
                    detected=False
                )

            # ==========================================
            # STORE FIRST FRAME
            # ==========================================

            annotated_frame = (
                result.plot()
            )

            self.state.update_perception(
                person_detected=
                    self.person_event.is_set(),

                confidence=
                    best_person_confidence,

                fps=0.0,

                frame=
                    annotated_frame
            )

            self.state.set_perception_status(
                "READY"
            )

            # Tell autonomous_app.py that
            # perception initialization succeeded.
            self.ready_event.set()

            print()
            print(
                "[PERCEPTION] SYSTEM READY"
            )
            print()

            # ==========================================
            # NORMAL LOOP STATE
            # ==========================================

            detection_counter = (
                1
                if people
                else 0
            )

            clear_counter = (
                0
                if people
                else 1
            )

            obstacle_counter = (
                1
                if obstacle_analysis
                is not None
                else 0
            )

            obstacle_clear_counter = (
                0
                if obstacle_analysis
                is not None
                else 1
            )

            fps = 0.0

            previous_time = (
                time.perf_counter()
            )

            # ==========================================
            # NORMAL PERCEPTION LOOP
            # ==========================================

            while (
                not
                self.shutdown_event.is_set()
            ):

                # --------------------------------------
                # CAMERA
                # --------------------------------------

                frame = (
                    camera.read_frame()
                )

                # --------------------------------------
                # YOLO — ONE INFERENCE
                # --------------------------------------

                detections, result = (
                    detector.detect_scene(
                        frame
                    )
                )

                # ======================================
                # PERSON ANALYSIS
                # ======================================

                people = (
                    detector.get_people(
                        detections
                    )
                )

                raw_person_detected = (
                    len(people) > 0
                )

                best_person_confidence = (
                    0.0
                )

                if people:

                    best_person_confidence = max(
                        detection["confidence"]
                        for detection
                        in people
                    )

                # ======================================
                # OBSTACLE ANALYSIS
                # ======================================

                obstacle_analysis = (
                    obstacle_analyzer.analyze(
                        frame,
                        detections
                    )
                )

                raw_obstacle_detected = (
                    obstacle_analysis
                    is not None
                )

                # ======================================
                # FPS
                # ======================================

                now = (
                    time.perf_counter()
                )

                frame_time = (
                    now
                    - previous_time
                )

                previous_time = now

                instantaneous_fps = (
                    1 / frame_time
                    if frame_time > 0
                    else 0
                )

                if fps == 0:

                    fps = (
                        instantaneous_fps
                    )

                else:

                    fps = (
                        0.90 * fps
                        +
                        0.10
                        * instantaneous_fps
                    )

                # ======================================
                # PERSON HYSTERESIS
                # ======================================

                if raw_person_detected:

                    detection_counter += 1
                    clear_counter = 0

                else:

                    clear_counter += 1
                    detection_counter = 0

                # --------------------------------------
                # Confirm person
                # --------------------------------------

                if (
                    not
                    self.person_event.is_set()
                    and
                    detection_counter
                    >= self.detection_frames
                ):

                    self.person_event.set()

                    print(
                        "[PERCEPTION] "
                        "Confirmed person detection"
                    )

                # --------------------------------------
                # Confirm person cleared
                # --------------------------------------

                if (
                    self.person_event.is_set()
                    and
                    clear_counter
                    >= self.clear_frames
                ):

                    self.person_event.clear()

                    print(
                        "[PERCEPTION] "
                        "Person no longer visible"
                    )

                # ======================================
                # OBSTACLE HYSTERESIS
                # ======================================

                if raw_obstacle_detected:

                    obstacle_counter += 1

                    obstacle_clear_counter = 0

                else:

                    obstacle_clear_counter += 1

                    obstacle_counter = 0

                # --------------------------------------
                # Confirm obstacle
                # --------------------------------------

                if (
                    not
                    self.obstacle_event.is_set()
                    and
                    obstacle_counter
                    >= self.obstacle_frames
                ):

                    self.obstacle_event.set()

                    print()
                    print(
                        "[PERCEPTION] "
                        "Obstacle confirmed"
                    )

                    if (
                        obstacle_analysis
                        is not None
                    ):

                        primary = (
                            obstacle_analysis[
                                "primary"
                            ]
                        )

                        print(
                            "[PERCEPTION] "
                            f"Object: "
                            f"{primary['label']}"
                        )

                        print(
                            "[PERCEPTION] "
                            f"Risk: "
                            f"{obstacle_analysis['risk']:.3f}"
                        )

                        print(
                            "[PERCEPTION] "
                            "Recommended escape: "
                            f"{obstacle_analysis['recommended_escape']}"
                        )

                # --------------------------------------
                # Confirm obstacle cleared
                # --------------------------------------

                if (
                    self.obstacle_event.is_set()
                    and
                    obstacle_clear_counter
                    >= self.obstacle_clear_frames
                ):

                    self.obstacle_event.clear()

                    self.state.update_obstacle(
                        detected=False
                    )

                    print(
                        "[PERCEPTION] "
                        "Travel corridor clear"
                    )

                # ======================================
                # UPDATE OBSTACLE SHARED STATE
                # ======================================

                self._update_obstacle_state(
                    obstacle_analysis
                )

                # ======================================
                # ANNOTATED FRAME
                # ======================================

                annotated_frame = (
                    result.plot()
                )

                # ======================================
                # UPDATE SHARED PERCEPTION STATE
                # ======================================

                self.state.update_perception(
                    person_detected=
                        self.person_event.is_set(),

                    confidence=
                        best_person_confidence,

                    fps=fps,

                    frame=
                        annotated_frame
                )

        except Exception as error:

            print()
            print(
                "[PERCEPTION] "
                "START/RUNTIME ERROR:"
            )

            print(
                error
            )

            print()

            self.state.set_error(
                error
            )

            self.state.set_perception_status(
                "ERROR"
            )

            self.startup_failed_event.set()

            # Fail-safe:
            # perception failure disables
            # autonomous movement.
            self.person_event.set()

        finally:

            if camera is not None:

                camera.disconnect()

            print(
                "[PERCEPTION] Worker stopped"
            )