import time

import cv2

from camera import EZBCamera
from detector import SafetyDetector


def main():

    camera = EZBCamera()

    detector = SafetyDetector(
        confidence=0.45
    )

    previous_person_state = None

    frame_count = 0

    start_time = time.time()

    try:

        camera.connect()

        print()
        print("==============================")
        print(" SIX AI PERCEPTION TEST")
        print("==============================")
        print()
        print(
            "Stand in front of Six "
            "and see if it detects you."
        )
        print()
        print("Press Q to quit.")
        print()

        while True:

            # -----------------------------------
            # Camera
            # -----------------------------------

            frame = camera.read_frame()

            # -----------------------------------
            # AI
            # -----------------------------------

            detections, result = (
                detector.detect_people(frame)
            )

            person_detected = (
                len(detections) > 0
            )

            # -----------------------------------
            # Status changes
            # -----------------------------------

            if (
                person_detected
                != previous_person_state
            ):

                if person_detected:

                    best = max(
                        detections,
                        key=lambda x:
                        x["confidence"]
                    )

                    print(
                        "[SAFETY] PERSON DETECTED "
                        f"({best['confidence']:.2f})"
                    )

                else:

                    print(
                        "[SAFETY] Area clear"
                    )

                previous_person_state = (
                    person_detected
                )

            # -----------------------------------
            # Draw YOLO output
            # -----------------------------------

            display_frame = (
                result.plot()
            )

            frame_count += 1

            elapsed = (
                time.time()
                - start_time
            )

            fps = (
                frame_count / elapsed
                if elapsed > 0
                else 0
            )

            # -----------------------------------
            # Visual safety state
            # -----------------------------------

            if person_detected:

                status = (
                    "PERSON DETECTED"
                )

            else:

                status = (
                    "AREA CLEAR"
                )

            cv2.putText(
                display_frame,
                status,
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

            cv2.putText(
                display_frame,
                f"FPS: {fps:.1f}",
                (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            # -----------------------------------
            # Window
            # -----------------------------------

            cv2.imshow(
                "Six Safety Perception",
                display_frame
            )

            key = (
                cv2.waitKey(1)
                & 0xFF
            )

            if key == ord("q"):
                break

    except KeyboardInterrupt:

        print()
        print(
            "[SYSTEM] CTRL+C detected"
        )

    finally:

        camera.disconnect()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
