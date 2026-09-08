import time

import cv2

from camera import EZBCamera


def main():

    camera = EZBCamera()

    frame_count = 0
    start_time = time.time()

    try:

        camera.connect()

        print()
        print("==============================")
        print(" EZ-B CAMERA TEST")
        print("==============================")
        print()
        print("Controls:")
        print("  Q = quit")
        print("  S = save snapshot")
        print()

        while True:

            frame = camera.read_frame()

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

            cv2.putText(
                frame,
                f"FPS: {fps:.1f}",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

            cv2.imshow(
                "EZ-B Six Camera",
                frame
            )

            key = (
                cv2.waitKey(1)
                & 0xFF
            )

            if key == ord("q"):
                break

            if key == ord("s"):

                filename = (
                    f"snapshot_"
                    f"{int(time.time())}.jpg"
                )

                cv2.imwrite(
                    filename,
                    frame
                )

                print(
                    f"[CAMERA] Saved {filename}"
                )

    except KeyboardInterrupt:

        print()
        print("[CAMERA] CTRL+C")

    finally:

        camera.disconnect()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
