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
        confidence=0.45,
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
        # before confirming a person.
        self.detection_frames = detection_frames

        # Number of clear frames required
        # before clearing a person detection.
        self.clear_frames = clear_frames

        # Same concept for obstacles.
        self.obstacle_frames = obstacle_frames
        self.obstacle_clear_frames = (
            obstacle_clear_frames
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

            load_start = time.perf_counter()

            detector = SafetyDetector(
                model_path=self.model_path,
                confidence=self.confidence
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

            camera_start = time.perf_counter()

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
                "[PERCEPTION] Waiting for first frame..."
            )

            frame_start = time.perf_counter()

            first_frame = camera.read_frame()

            frame_time = (
                time.perf_counter()
                - frame_start
            )

            print(
                "[PERCEPTION] First frame received "
                f"in {frame_time:.2f}s"
            )

            # ==========================================
            # STEP 4 — FIRST AI INFERENCE / WARMUP
            # ==========================================

            print(
                "[PERCEPTION] Running first "
                "AI inference..."
            )

            inference_start = (
                time.perf_counter()
            )

            # Detect the complete scene once.
            detections, result = (
                detector.detect_scene(
                    first_frame
                )
            )

            # Separate people from the complete
            # detection list.
            people = detector.get_people(
                detections
            )

            # Analyze non-person detections to
            # determine whether something blocks
            # the robot's path.
            obstacle = (
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
                "[PERCEPTION] First inference "
                f"completed in "
                f"{inference_time:.2f}s"
            )

            # ==========================================
            # INITIAL PERSON STATE
            # ==========================================

            best_person_confidence = 0.0

            if people:

                best_person_confidence = max(
                    detection["confidence"]
                    for detection in people
                )

                # Fail-safe startup behavior:
                # if a person is already visible when
                # autonomy starts, do not begin moving.
                self.person_event.set()

            else:

                self.person_event.clear()

            # ==========================================
            # INITIAL OBSTACLE STATE
            # ==========================================

            if obstacle is not None:

                self.obstacle_event.set()

                self.state.update_obstacle(
                    detected=True,
                    label=obstacle["label"],
                    confidence=(
                        obstacle["confidence"]
                    ),
                    side=obstacle["side"],
                    area_ratio=(
                        obstacle["area_ratio"]
                    )
                )

            else:

                self.obstacle_event.clear()

                self.state.update_obstacle(
                    detected=False
                )

            # ==========================================
            # STORE FIRST ANNOTATED FRAME
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
            # perception startup succeeded.
            self.ready_event.set()

            print()
            print(
                "[PERCEPTION] SYSTEM READY"
            )
            print()

            # ==========================================
            # NORMAL PERCEPTION LOOP
            # ==========================================

            # Prime counters based on the
            # startup inference.
            detection_counter = (
                1 if people else 0
            )

            clear_counter = (
                0 if people else 1
            )

            obstacle_counter = (
                1
                if obstacle is not None
                else 0
            )

            obstacle_clear_counter = (
                0
                if obstacle is not None
                else 1
            )

            fps = 0.0

            previous_time = (
                time.perf_counter()
            )

            while not self.shutdown_event.is_set():

                # ==================================
                # CAMERA
                # ==================================

                frame = camera.read_frame()

                # ==================================
                # AI — ONE SCENE INFERENCE
                # ==================================

                detections, result = (
                    detector.detect_scene(
                        frame
                    )
                )

                # ==================================
                # PERSON ANALYSIS
                # ==================================

                people = detector.get_people(
                    detections
                )

                raw_person_detected = (
                    len(people) > 0
                )

                best_person_confidence = 0.0

                if people:

                    best_person_confidence = max(
                        detection["confidence"]
                        for detection in people
                    )

                # ==================================
                # OBSTACLE ANALYSIS
                # ==================================

                obstacle = (
                    obstacle_analyzer.analyze(
                        frame,
                        detections
                    )
                )

                raw_obstacle_detected = (
                    obstacle is not None
                )

                # ==================================
                # FPS
                # ==================================

                now = time.perf_counter()

                frame_time = (
                    now - previous_time
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

                    # Exponential moving average.
                    fps = (
                        0.90 * fps
                        + 0.10
                        * instantaneous_fps
                    )

                # ==================================
                # PERSON HYSTERESIS
                # ==================================

                if raw_person_detected:

                    detection_counter += 1
                    clear_counter = 0

                else:

                    clear_counter += 1
                    detection_counter = 0

                # Confirm person.

                if (
                    not self.person_event.is_set()
                    and
                    detection_counter
                    >= self.detection_frames
                ):

                    self.person_event.set()

                    print(
                        "[PERCEPTION] "
                        "Confirmed person detection"
                    )

                # Confirm person cleared.

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

                # ==================================
                # OBSTACLE HYSTERESIS
                # ==================================

                if raw_obstacle_detected:

                    obstacle_counter += 1
                    obstacle_clear_counter = 0

                else:

                    obstacle_clear_counter += 1
                    obstacle_counter = 0

                # Confirm obstacle.

                if (
                    not self.obstacle_event.is_set()
                    and
                    obstacle_counter
                    >= self.obstacle_frames
                ):

                    self.obstacle_event.set()

                    print(
                        "[PERCEPTION] "
                        "Obstacle confirmed"
                    )

                # Confirm obstacle cleared.

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
                        "Path appears clear"
                    )

                # ==================================
                # UPDATE OBSTACLE STATE
                # ==================================

                if obstacle is not None:

                    self.state.update_obstacle(
                        detected=
                            self.obstacle_event.is_set(),

                        label=
                            obstacle["label"],

                        confidence=
                            obstacle["confidence"],

                        side=
                            obstacle["side"],

                        area_ratio=
                            obstacle["area_ratio"]
                    )

                elif not self.obstacle_event.is_set():

                    self.state.update_obstacle(
                        detected=False
                    )

                # If obstacle_event is still set but
                # this individual frame is clear,
                # intentionally keep the previous
                # obstacle metadata until hysteresis
                # officially declares the path clear.

                # ==================================
                # UPDATE PERSON / FRAME STATE
                # ==================================

                annotated_frame = (
                    result.plot()
                )

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
            print(error)
            print()

            self.state.set_error(
                error
            )

            self.state.set_perception_status(
                "ERROR"
            )

            self.startup_failed_event.set()

            # Fail safe:
            # perception failure should prevent
            # autonomous movement.
            self.person_event.set()

        finally:

            if camera is not None:

                camera.disconnect()

            print(
                "[PERCEPTION] Worker stopped"
            )