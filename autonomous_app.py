import argparse
import threading
import time

import cv2

from ezb import EZB
from six_robot import SixRobot

from robot_state import RobotState
from perception_worker import PerceptionWorker
from safety_supervisor import SafetySupervisor
from motion_worker import MotionWorker


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

    print(
        f"[SYSTEM] Startup timeout: "
        f"{timeout}s"
    )

    start = time.perf_counter()

    last_status_time = start

    while not shutdown_event.is_set():

        # ======================================
        # SUCCESS
        # ======================================

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

        # ======================================
        # FAILURE
        # ======================================

        if failed_event.is_set():

            snapshot = state.snapshot()

            print(
                "[SYSTEM] Perception startup "
                "failed."
            )

            print(
                "[SYSTEM] Error:",
                snapshot["last_error"]
            )

            return False

        # ======================================
        # TIMEOUT
        # ======================================

        elapsed = (
            time.perf_counter()
            - start
        )

        if elapsed >= timeout:

            snapshot = state.snapshot()

            print()
            print(
                "[SYSTEM] Perception startup "
                "timed out."
            )

            print(
                "[SYSTEM] AI:",
                snapshot["ai_status"]
            )

            print(
                "[SYSTEM] Camera:",
                snapshot["camera_status"]
            )

            print(
                "[SYSTEM] Perception:",
                snapshot[
                    "perception_status"
                ]
            )

            return False

        # ======================================
        # PERIODIC STATUS
        # ======================================

        now = time.perf_counter()

        if (
            now - last_status_time
            >= 5.0
        ):

            snapshot = state.snapshot()

            print(
                "[SYSTEM] Still starting... "
                f"AI={snapshot['ai_status']} "
                f"Camera="
                f"{snapshot['camera_status']} "
                f"Perception="
                f"{snapshot['perception_status']} "
                f"Elapsed={elapsed:.1f}s"
            )

            last_status_time = now

        # Event.wait() is preferable to
        # busy spinning.
        shutdown_event.wait(
            0.10
        )

    return False


def draw_status(frame, snapshot, mode):

    lines = [
        f"Mode: {mode.upper()}",

        (
            "AI: "
            f"{snapshot['ai_status']}"
        ),

        (
            "Camera: "
            f"{snapshot['camera_status']}"
        ),

        (
            "Perception: "
            f"{snapshot['perception_status']}"
        ),

        (
            "Person: YES"
            if snapshot["person_detected"]
            else "Person: NO"
        ),

        (
            "Confidence: "
            f"{snapshot['person_confidence']:.2f}"
        ),

        (
            "Safety: "
            f"{snapshot['safety_state']}"
        ),

        (
            "Motion: "
            f"{snapshot['motion_state']}"
        ),

        (
            "Vision FPS: "
            f"{snapshot['perception_fps']:.1f}"
        ),
    ]

    y = 25

    for line in lines:

        cv2.putText(
            frame,
            line,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        y += 27

    return frame


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
        default="observe",
        help=(
            "observe = AI only, no movement. "
            "patrol = autonomous movement."
        )
    )

    parser.add_argument(
        "--model",
        default="yolo26n.pt"
    )

    parser.add_argument(
        "--confidence",
        type=float,
        default=0.45
    )

    parser.add_argument(
        "--clear-delay",
        type=float,
        default=2.0
    )

    args = parser.parse_args()

    # ==========================================
    # SHARED SYNCHRONIZATION OBJECTS
    # ==========================================

    shutdown_event = threading.Event()

    person_event = threading.Event()

    safety_stop_event = threading.Event()


    # Perception startup synchronization
    perception_ready_event = (
        threading.Event()
    )

    perception_failed_event = (
        threading.Event()
    )

    state = RobotState()

    state.set_system(
        "STARTING"
    )

    # ==========================================
    # PERCEPTION
    # ==========================================

    perception = PerceptionWorker(
        state=state,
        person_event=person_event,
        shutdown_event=shutdown_event,

        ready_event=
            perception_ready_event,

        startup_failed_event=
            perception_failed_event,

        model_path=args.model,
        confidence=args.confidence
    )

    safety = SafetySupervisor(
        state=state,
        person_event=person_event,
        safety_stop_event=
            safety_stop_event,
        shutdown_event=shutdown_event,
        clear_delay=args.clear_delay
    )

    threads = [
        perception,
        safety
    ]

    ezb = None
    six = None
    motion = None

    emergency_release = False

    try:

        # Start AI before motion.
        perception.start()
        safety.start()

        if not wait_for_perception(
            state,
            perception_ready_event,
            perception_failed_event,
            shutdown_event,
            timeout=120
        ):

            raise RuntimeError(
                "Perception did not become "
                "ready in time."
            )

        # ======================================
        # PATROL MODE
        # ======================================

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

            motion = MotionWorker(
                robot=six,
                state=state,
                safety_stop_event=
                    safety_stop_event,
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
        print("==============================")
        print(" AUTONOMOUS SAFETY SYSTEM")
        print("==============================")
        print()
        print(
            f"Mode: {args.mode.upper()}"
        )
        print()
        print("Q = graceful shutdown")
        print(
            "E = emergency servo release"
        )
        print()

        # ======================================
        # MAIN UI LOOP
        # ======================================

        while not shutdown_event.is_set():

            frame = state.get_frame()

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

            # ----------------------------------
            # GRACEFUL QUIT
            # ----------------------------------

            if key == ord("q"):

                print(
                    "[SYSTEM] "
                    "Graceful shutdown requested"
                )

                shutdown_event.set()

                break

            # ----------------------------------
            # EMERGENCY RELEASE
            # ----------------------------------

            if key == ord("e"):

                print(
                    "[SYSTEM] "
                    "EMERGENCY RELEASE REQUESTED"
                )

                emergency_release = True

                shutdown_event.set()

                if ezb is not None:

                    ezb.release_all_servos()

                break

            time.sleep(0.005)

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

        state.set_error(error)

        shutdown_event.set()

    finally:

        state.set_system(
            "SHUTTING_DOWN"
        )

        shutdown_event.set()

        print(
            "[SYSTEM] Waiting for workers..."
        )

        for thread in threads:

            thread.join(
                timeout=3.0
            )

        # ======================================
        # ROBOT CLEANUP
        # ======================================

        if six is not None:

            if emergency_release:

                # Already released.
                pass

            elif person_event.is_set():

                # Do not unexpectedly move toward
                # neutral while somebody is nearby.
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
