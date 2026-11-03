import argparse
import threading
import time

import cv2

from pathlib import Path

from robot.ezb import EZB
from robot.six_robot import SixRobot

from autonomy.robot_state import RobotState
from autonomy.safety_supervisor import SafetySupervisor
from autonomy.motion_worker import MotionWorker
from autonomy.progress_monitor import ProgressMonitor
from autonomy.ultrasonic_worker import UltrasonicWorker

from perception.perception_worker import (
    PerceptionWorker
)

from inspection.incident_logger import (
    IncidentLogger
)

from inspection.inspection_manager import (
    InspectionManager
)


PROJECT_ROOT = (
    Path(__file__)
    .resolve()
    .parent
    .parent
)

DEFAULT_MODEL = (
    PROJECT_ROOT
    / "models"
    / "yolo26n.pt"
)


# ======================================================
# STARTUP WAIT
# ======================================================

def wait_for_perception(
    state,
    ready_event,
    failed_event,
    shutdown_event,
    timeout=120
):
    print()
    print(
        "[SYSTEM] Waiting for perception "
        "pipeline to initialize..."
    )

    start = time.perf_counter()
    last_status = start

    while (
        not
        shutdown_event.is_set()
    ):
        if ready_event.is_set():
            elapsed = (
                time.perf_counter()
                - start
            )

            print(
                "[SYSTEM] Perception ready "
                f"after {elapsed:.2f}s"
            )

            return True

        if failed_event.is_set():
            snapshot = (
                state.snapshot()
            )

            print(
                "[SYSTEM] "
                "Perception startup failed:"
            )

            print(
                snapshot["last_error"]
            )

            return False

        elapsed = (
            time.perf_counter()
            - start
        )

        if elapsed >= timeout:
            snapshot = (
                state.snapshot()
            )

            print(
                "[SYSTEM] "
                "Perception startup timed out."
            )

            print(
                "[SYSTEM] AI:",
                snapshot["ai_status"]
            )

            print(
                "[SYSTEM] Depth:",
                snapshot["depth_status"]
            )

            print(
                "[SYSTEM] Camera:",
                snapshot["camera_status"]
            )

            return False

        if (
            time.perf_counter()
            - last_status
            >= 5.0
        ):
            snapshot = (
                state.snapshot()
            )

            print(
                "[SYSTEM] Still starting... "
                f"AI={snapshot['ai_status']} "
                f"Depth={snapshot['depth_status']} "
                f"Camera={snapshot['camera_status']} "
                f"Elapsed={elapsed:.1f}s"
            )

            last_status = (
                time.perf_counter()
            )

        shutdown_event.wait(
            0.10
        )

    return False


# ======================================================
# DISPLAY
# ======================================================

def draw_status(
    frame,
    snapshot,
    mode
):
    obstacle_label = (
        snapshot["obstacle_label"]
        if snapshot["obstacle_detected"]
        else "NO"
    )

    obstacle_distance = (
        snapshot[
            "obstacle_distance_m"
        ]
    )

    corridor_depth = (
        snapshot[
            "corridor_depth_m"
        ]
    )

    obstacle_distance_text = (
        f"{obstacle_distance:.2f}m"
        if obstacle_distance is not None
        else "-"
    )

    corridor_depth_text = (
        f"{corridor_depth:.2f}m"
        if corridor_depth is not None
        else "-"
    )

    ultrasonic_cm = (
        snapshot.get(
            "ultrasonic_distance_cm"
        )
    )

    ultrasonic_text = (
        f"{ultrasonic_cm:.1f} cm"
        if ultrasonic_cm is not None
        else "-"
    )

    lines = [
        f"Mode: {mode.upper()}",

        (
            "AI: "
            f"{snapshot['ai_status']}"
        ),

        (
            "Depth: "
            f"{snapshot['depth_status']}"
        ),

        (
            "Ultrasonic: "
            f"{snapshot.get('ultrasonic_status', '-')}"
        ),

        (
            "Range: "
            f"{ultrasonic_text}"
        ),

        (
            "Range navigation: YES"
            if snapshot.get(
                "ultrasonic_navigation_confirmed",
                False
            )
            else "Range navigation: NO"
        ),

        (
            "Person: YES"
            if snapshot["person_detected"]
            else "Person: NO"
        ),

        (
            "Obstacle: "
            f"{obstacle_label}"
        ),

        (
            "Obstacle distance: "
            f"{obstacle_distance_text}"
        ),

        (
            "Forward depth: "
            f"{corridor_depth_text}"
        ),

        (
            "Escape: "
            f"{snapshot['obstacle_escape_direction']}"
        ),

        (
            "Motion: "
            f"{snapshot['motion_state']}"
        ),

        (
            "Progress: "
            f"{snapshot['progress_status']}"
        ),

        (
            "Optical flow: "
            f"{snapshot['visual_motion_score']:.2f}"
        ),

        (
            "Vision FPS: "
            f"{snapshot['perception_fps']:.1f}"
        ),
    ]

    y = 24

    for line in lines:
        cv2.putText(
            frame,
            line,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 255, 255),
            1
        )

        y += 22

    return frame


# ======================================================
# APPLICATION
# ======================================================

def main():
    parser = argparse.ArgumentParser(
        description=(
            "Autonomous EZ-Robot Six "
            "Safety Inspection System"
        )
    )

    parser.add_argument(
        "--mode",
        choices=[
            "observe",
            "patrol"
        ],
        default="observe"
    )

    parser.add_argument(
        "--model",
        default=str(
            DEFAULT_MODEL
        )
    )

    # Low YOLO candidate floor.
    parser.add_argument(
        "--confidence",
        type=float,
        default=0.25
    )

    parser.add_argument(
        "--clear-delay",
        type=float,
        default=2.0
    )

    args = parser.parse_args()

    # ==================================================
    # EVENTS
    # ==================================================

    shutdown_event = (
        threading.Event()
    )

    person_event = (
        threading.Event()
    )

    # Camera-only semantic candidate:
    # "I see a configured navigation object such as
    # a backpack/book in the travel corridor."
    vision_obstacle_event = (
        threading.Event()
    )

    # Camera + ultrasonic confirmation:
    # "The configured object is now physically close
    # enough that MotionWorker should avoid it."
    navigation_obstacle_event = (
        threading.Event()
    )

    # Ultrasonic-only close-surface event:
    # used for walls / unclassified surfaces at the
    # dedicated wall threshold.
    wall_event = (
        threading.Event()
    )

    stuck_event = (
        threading.Event()
    )

    safety_stop_event = (
        threading.Event()
    )

    perception_ready_event = (
        threading.Event()
    )

    perception_failed_event = (
        threading.Event()
    )

    # ==================================================
    # STATE
    # ==================================================

    state = RobotState()

    state.set_system(
        "STARTING"
    )

    # ==================================================
    # INSPECTION LOGGING
    # ==================================================

    incident_logger = (
        IncidentLogger()
    )

    inspection_manager = (
        InspectionManager(
            state=state,
            logger=incident_logger,
            shutdown_event=
                shutdown_event
        )
    )

    # ==================================================
    # NON-MOTOR WORKERS
    # ==================================================

    perception = (
        PerceptionWorker(
            state=state,

            person_event=
                person_event,

            obstacle_event=
                vision_obstacle_event,

            shutdown_event=
                shutdown_event,

            ready_event=
                perception_ready_event,

            startup_failed_event=
                perception_failed_event,

            model_path=
                args.model,

            confidence=
                args.confidence
        )
    )

    safety = (
        SafetySupervisor(
            state=state,

            person_event=
                person_event,

            safety_stop_event=
                safety_stop_event,

            shutdown_event=
                shutdown_event,

            clear_delay=
                args.clear_delay
        )
    )

    progress = (
        ProgressMonitor(
            state=state,

            stuck_event=
                stuck_event,

            obstacle_event=
                navigation_obstacle_event,

            safety_stop_event=
                safety_stop_event,

            shutdown_event=
                shutdown_event
        )
    )

    threads = [
        perception,
        safety,
        progress,
        inspection_manager
    ]

    ezb = None
    six = None
    ultrasonic = None
    motion = None

    emergency_release = False

    try:
        perception.start()
        safety.start()
        progress.start()
        inspection_manager.start()

        if not wait_for_perception(
            state,
            perception_ready_event,
            perception_failed_event,
            shutdown_event,
            timeout=120
        ):
            raise RuntimeError(
                "Perception did not "
                "become ready."
            )

        # ==============================================
        # PATROL
        # ==============================================

        if args.mode == "patrol":
            print()
            print(
                "[SYSTEM] Connecting "
                "motor controller..."
            )

            ezb = EZB()

            ezb.connect()

            six = SixRobot(
                ezb
            )

            # ==========================================
            # ULTRASONIC RANGE FUSION
            # ==========================================
            #
            # Backpack/book navigation:
            #     camera candidate
            #       +
            #     HC-SR04 <= 45 cm
            #       ->
            #     navigation_obstacle_event
            #
            # Wall / unknown close surface:
            #     HC-SR04 <= 15 cm
            #       ->
            #     wall_event

            ultrasonic = UltrasonicWorker(
                ezb=ezb,
                state=state,

                vision_obstacle_event=
                    vision_obstacle_event,

                navigation_obstacle_event=
                    navigation_obstacle_event,

                wall_event=
                    wall_event,

                shutdown_event=
                    shutdown_event,

                trigger_port=20,
                echo_port=21,

                object_trigger_distance_cm=45,
                object_clear_distance_cm=55,

                wall_trigger_distance_cm=15,
                wall_clear_distance_cm=22,

                object_confirm_readings=2,
                object_clear_readings=2,

                wall_confirm_readings=2,
                wall_clear_readings=2
            )

            threads.append(
                ultrasonic
            )

            ultrasonic.start()

            motion = MotionWorker(
                robot=six,

                state=state,

                safety_stop_event=
                    safety_stop_event,

                obstacle_event=
                    navigation_obstacle_event,

                wall_event=
                    wall_event,

                stuck_event=
                    stuck_event,

                shutdown_event=
                    shutdown_event
            )

            threads.append(
                motion
            )

            motion.start()

        else:
            state.set_motion(
                "DISABLED"
            )

        state.set_system(
            "RUNNING"
        )

        print()
        print(
            "=============================="
        )
        print(
            " AUTONOMOUS SAFETY SYSTEM"
        )
        print(
            "=============================="
        )
        print()

        print(
            f"Mode: {args.mode.upper()}"
        )

        print(
            "Q = graceful shutdown"
        )

        print(
            "E = emergency servo release"
        )

        # ==============================================
        # UI LOOP
        # ==============================================

        while (
            not
            shutdown_event.is_set()
        ):
            frame = (
                state.get_frame()
            )

            if frame is not None:
                snapshot = (
                    state.snapshot()
                )

                frame = draw_status(
                    frame,
                    snapshot,
                    args.mode
                )

                cv2.imshow(
                    "Six Autonomous "
                    "Safety Inspector",
                    frame
                )

            key = (
                cv2.waitKey(1)
                & 0xFF
            )

            if key == ord("q"):
                print(
                    "[SYSTEM] "
                    "Graceful shutdown requested"
                )

                shutdown_event.set()
                break

            if key == ord("e"):
                print(
                    "[SYSTEM] "
                    "EMERGENCY RELEASE"
                )

                emergency_release = True

                shutdown_event.set()

                if ezb is not None:
                    ezb.release_all_servos()

                break

            time.sleep(
                0.005
            )

    except KeyboardInterrupt:
        print()
        print(
            "[SYSTEM] CTRL+C"
        )

        shutdown_event.set()

    except Exception as error:
        print()
        print(
            "[SYSTEM] ERROR:",
            error
        )

        state.set_error(
            error
        )

        shutdown_event.set()

    finally:
        state.set_system(
            "SHUTTING_DOWN"
        )

        shutdown_event.set()

        print(
            "[SYSTEM] "
            "Waiting for workers..."
        )

        for thread in threads:
            thread.join(
                timeout=3.0
            )

        if six is not None:
            if emergency_release:
                pass

            elif (
                person_event.is_set()
                or
                navigation_obstacle_event.is_set()
                or
                wall_event.is_set()
                or
                stuck_event.is_set()
            ):
                six.halt()

            else:
                print(
                    "[SYSTEM] "
                    "Returning Six to stable stance"
                )

                six.stop()

        if ezb is not None:
            ezb.disconnect()

        cv2.destroyAllWindows()

        state.set_system(
            "STOPPED"
        )

        print(
            "[SYSTEM] Shutdown complete"
        )


if __name__ == "__main__":
    main()
