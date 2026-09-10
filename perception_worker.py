import threading
import time

from camera import EZBCamera
from detector import SafetyDetector


class PerceptionWorker(threading.Thread):

    def __init__(
        self,
        state,
        person_event,
        shutdown_event,
        ready_event,
        startup_failed_event,
        model_path="yolo26n.pt",
        confidence=0.45,
        detection_frames=2,
        clear_frames=4
    ):

        super().__init__(
            name="PerceptionWorker",
            daemon=True
        )

        self.state = state

        self.person_event = person_event
        self.shutdown_event = shutdown_event

        # NEW
        self.ready_event = ready_event
        self.startup_failed_event = (
            startup_failed_event
        )

        self.model_path = model_path
        self.confidence = confidence

        self.detection_frames = detection_frames
        self.clear_frames = clear_frames

    # --------------------------------------------------

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
            # STEP 3 — GET FIRST FRAME
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

            detections, result = (
                detector.detect_people(
                    first_frame
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
            # PERCEPTION IS NOW ACTUALLY READY
            # ==========================================

            person_detected = (
                len(detections) > 0
            )

            best_confidence = 0.0

            if detections:

                best_confidence = max(
                    detection["confidence"]
                    for detection in detections
                )

            annotated_frame = (
                result.plot()
            )

            self.state.update_perception(
                person_detected=
                    person_detected,

                confidence=
                    best_confidence,

                fps=0.0,

                frame=
                    annotated_frame
            )

            self.state.set_perception_status(
                "READY"
            )

            # THIS is what tells main()
            # that startup succeeded.
            self.ready_event.set()

            print()
            print(
                "[PERCEPTION] SYSTEM READY"
            )
            print()

            # ==========================================
            # NORMAL PERCEPTION LOOP
            # ==========================================

            detection_counter = 0
            clear_counter = 0

            fps = 0.0

            previous_time = (
                time.perf_counter()
            )

            while not self.shutdown_event.is_set():

                # ----------------------------------
                # CAMERA
                # ----------------------------------

                frame = camera.read_frame()

                # ----------------------------------
                # AI
                # ----------------------------------

                detections, result = (
                    detector.detect_people(
                        frame
                    )
                )

                raw_person_detected = (
                    len(detections) > 0
                )

                best_confidence = 0.0

                if detections:

                    best_confidence = max(
                        detection["confidence"]
                        for detection in detections
                    )

                # ----------------------------------
                # FPS
                # ----------------------------------

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

                    fps = instantaneous_fps

                else:

                    fps = (
                        0.90 * fps
                        + 0.10
                        * instantaneous_fps
                    )

                # ----------------------------------
                # DETECTION HYSTERESIS
                # ----------------------------------

                if raw_person_detected:

                    detection_counter += 1
                    clear_counter = 0

                else:

                    clear_counter += 1
                    detection_counter = 0

                # Person becomes confirmed

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

                # Person becomes confirmed clear

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

                # ----------------------------------
                # DISPLAY FRAME
                # ----------------------------------

                annotated_frame = (
                    result.plot()
                )

                self.state.update_perception(
                    person_detected=
                        self.person_event.is_set(),

                    confidence=
                        best_confidence,

                    fps=fps,

                    frame=
                        annotated_frame
                )

        except Exception as error:

            print()
            print(
                "[PERCEPTION] START/RUNTIME ERROR:"
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

            # Fail safe.
            self.person_event.set()

        finally:

            if camera is not None:

                camera.disconnect()

            print(
                "[PERCEPTION] Worker stopped"
            )