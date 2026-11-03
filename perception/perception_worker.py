import threading
import time

import numpy as np

from perception.camera import EZBCamera
from perception.detector import SafetyDetector
from perception.depth_estimator import DepthEstimator
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
        confidence=0.25,
        detection_frames=2,
        clear_frames=4,
        obstacle_frames=2,
        obstacle_clear_frames=3,
        depth_every_n_frames=3
    ):
        super().__init__(
            name="PerceptionWorker",
            daemon=True
        )

        self.state = state

        self.person_event = person_event
        self.obstacle_event = (
            obstacle_event
        )

        self.shutdown_event = (
            shutdown_event
        )

        self.ready_event = ready_event

        self.startup_failed_event = (
            startup_failed_event
        )

        self.model_path = model_path
        self.confidence = confidence

        self.detection_frames = (
            detection_frames
        )

        self.clear_frames = (
            clear_frames
        )

        self.obstacle_frames = (
            obstacle_frames
        )

        self.obstacle_clear_frames = (
            obstacle_clear_frames
        )

        self.depth_every_n_frames = max(
            1,
            int(depth_every_n_frames)
        )

    # ==================================================
    # SHARED OBSTACLE STATE
    # ==================================================

    def _update_obstacle_state(
        self,
        analysis
    ):
        if analysis is None:
            if not self.obstacle_event.is_set():
                self.state.update_obstacle(
                    detected=False
                )

            return

        primary = (
            analysis["primary"]
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
                analysis["risk"],

            distance_m=
                analysis["distance_m"],

            source=
                analysis["source"],

            escape_direction=
                analysis[
                    "recommended_escape"
                ],

            requires_backoff=
                analysis[
                    "requires_backoff"
                ],

            left_score=
                analysis[
                    "left_score"
                ],

            right_score=
                analysis[
                    "right_score"
                ],

            left_clearance_m=
                analysis[
                    "left_clearance_m"
                ],

            right_clearance_m=
                analysis[
                    "right_clearance_m"
                ]
        )

    # ==================================================
    # CLASS-AGNOSTIC FORWARD DEPTH
    # ==================================================

    def _update_corridor_depth(
        self,
        depth_map
    ):
        """
        Store a forward-corridor depth metric even
        when YOLO did not classify an object.

        This is useful for plain walls.
        """

        if depth_map is None:
            self.state.set_corridor_depth(
                None,
                None
            )
            return

        height, width = (
            depth_map.shape[:2]
        )

        x1 = int(
            width * 0.28
        )
        x2 = int(
            width * 0.72
        )

        y1 = int(
            height * 0.22
        )
        y2 = int(
            height * 0.68
        )

        region = depth_map[
            y1:y2,
            x1:x2
        ]

        valid = region[
            np.isfinite(region)
            &
            (region > 0.05)
            &
            (region < 20.0)
        ]

        if valid.size < 30:
            self.state.set_corridor_depth(
                None,
                None
            )
            return

        p25 = float(
            np.percentile(
                valid,
                25
            )
        )

        median = float(
            np.median(valid)
        )

        self.state.set_corridor_depth(
            p25_m=p25,
            median_m=median
        )

    # ==================================================
    # THREAD
    # ==================================================

    def run(self):

        camera = None

        try:
            # ==========================================
            # LOAD OBJECT DETECTOR
            # ==========================================

            self.state.set_perception_status(
                "STARTING"
            )

            self.state.set_ai_status(
                "LOADING"
            )

            print()
            print(
                "[PERCEPTION] "
                "Loading AI model..."
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

            self.state.set_ai_status(
                "READY"
            )

            # ==========================================
            # OPTIONAL DEPTH
            # ==========================================

            self.state.set_depth_status(
                "LOADING"
            )

            depth_estimator = (
                DepthEstimator()
            )

            if depth_estimator.available:
                self.state.set_depth_status(
                    "READY"
                )
            else:
                self.state.set_depth_status(
                    "DISABLED"
                )

            if self.shutdown_event.is_set():
                return

            # ==========================================
            # CAMERA
            # ==========================================

            self.state.set_camera_status(
                "CONNECTING"
            )

            camera = EZBCamera(
                timeout=5.0
            )

            print(
                "[PERCEPTION] "
                "Connecting camera..."
            )

            camera.connect()

            self.state.set_camera_status(
                "CONNECTED"
            )

            print(
                "[PERCEPTION] "
                "Camera connected"
            )

            # ==========================================
            # FIRST FRAME / WARMUP
            # ==========================================

            first_frame = (
                camera.read_frame()
            )

            print(
                "[PERCEPTION] "
                "Running first AI inference..."
            )

            detections, result = (
                detector.detect_scene(
                    first_frame
                )
            )

            people = (
                detector.get_people(
                    detections
                )
            )

            depth_map = (
                depth_estimator.predict(
                    first_frame
                )
                if depth_estimator.available
                else None
            )

            self._update_corridor_depth(
                depth_map
            )

            obstacle_analysis = (
                obstacle_analyzer.analyze(
                    first_frame,
                    detections,
                    depth_map=
                        depth_map
                )
            )

            best_person_confidence = (
                0.0
            )

            if people:
                best_person_confidence = max(
                    item["confidence"]
                    for item in people
                )
                self.person_event.set()
            else:
                self.person_event.clear()

            if (
                obstacle_analysis
                is not None
            ):
                self.obstacle_event.set()

                self._update_obstacle_state(
                    obstacle_analysis
                )
            else:
                self.obstacle_event.clear()

                self.state.update_obstacle(
                    detected=False
                )

            annotated_frame = (
                result.plot()
            )

            annotated_frame = (
                obstacle_analyzer.draw_debug(
                    annotated_frame,
                    obstacle_analysis
                )
            )

            self.state.update_perception(
                person_detected=
                    self.person_event.is_set(),

                confidence=
                    best_person_confidence,

                fps=0.0,

                frame=
                    annotated_frame,

                raw_frame=
                    first_frame
            )

            self.state.set_perception_status(
                "READY"
            )

            self.ready_event.set()

            print()
            print(
                "[PERCEPTION] SYSTEM READY"
            )
            print()

            # ==========================================
            # COUNTERS
            # ==========================================

            person_counter = (
                1 if people else 0
            )

            person_clear_counter = (
                0 if people else 1
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

            frame_number = 0

            latest_depth_map = (
                depth_map
            )

            # ==========================================
            # MAIN PERCEPTION LOOP
            # ==========================================

            while (
                not
                self.shutdown_event.is_set()
            ):
                frame = (
                    camera.read_frame()
                )

                frame_number += 1

                detections, result = (
                    detector.detect_scene(
                        frame
                    )
                )

                # --------------------------------------
                # PEOPLE
                # --------------------------------------

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
                        item["confidence"]
                        for item in people
                    )

                # --------------------------------------
                # DEPTH
                # --------------------------------------

                if (
                    depth_estimator.available
                    and
                    (
                        frame_number
                        %
                        self.depth_every_n_frames
                        == 0
                    )
                ):
                    latest_depth_map = (
                        depth_estimator.predict(
                            frame
                        )
                    )

                    self._update_corridor_depth(
                        latest_depth_map
                    )

                # --------------------------------------
                # OBSTACLE FUSION
                # --------------------------------------

                obstacle_analysis = (
                    obstacle_analyzer.analyze(
                        frame,
                        detections,
                        depth_map=
                            latest_depth_map
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

                instant_fps = (
                    1 / frame_time
                    if frame_time > 0
                    else 0
                )

                fps = (
                    instant_fps
                    if fps == 0
                    else
                    0.90 * fps
                    +
                    0.10 * instant_fps
                )

                # ======================================
                # PERSON HYSTERESIS
                # ======================================

                if raw_person_detected:
                    person_counter += 1
                    person_clear_counter = 0
                else:
                    person_clear_counter += 1
                    person_counter = 0

                if (
                    not self.person_event.is_set()
                    and
                    person_counter
                    >= self.detection_frames
                ):
                    self.person_event.set()

                    print(
                        "[PERCEPTION] "
                        "Confirmed person detection"
                    )

                if (
                    self.person_event.is_set()
                    and
                    person_clear_counter
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

                if (
                    not self.obstacle_event.is_set()
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
                            f"Source: "
                            f"{obstacle_analysis['source']}"
                        )

                        distance = (
                            obstacle_analysis[
                                "distance_m"
                            ]
                        )

                        if distance is not None:
                            print(
                                "[PERCEPTION] "
                                "Approx depth: "
                                f"{distance:.2f}m"
                            )

                        print(
                            "[PERCEPTION] "
                            "Escape: "
                            f"{obstacle_analysis['recommended_escape']}"
                        )

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

                self._update_obstacle_state(
                    obstacle_analysis
                )

                # ======================================
                # RAW + ANNOTATED FRAMES
                # ======================================

                annotated_frame = (
                    result.plot()
                )

                annotated_frame = (
                    obstacle_analyzer.draw_debug(
                        annotated_frame,
                        obstacle_analysis
                    )
                )

                self.state.update_perception(
                    person_detected=
                        self.person_event.is_set(),

                    confidence=
                        best_person_confidence,

                    fps=fps,

                    frame=
                        annotated_frame,

                    raw_frame=
                        frame
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

            # Fail safe.
            self.person_event.set()

        finally:
            if camera is not None:
                camera.disconnect()

            print(
                "[PERCEPTION] Worker stopped"
            )
