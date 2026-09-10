import time

import numpy as np

from detector import SafetyDetector


def main():

    print()
    print("==============================")
    print(" AI STARTUP BENCHMARK")
    print("==============================")
    print()

    # ----------------------------------------
    # MODEL LOAD
    # ----------------------------------------

    start = time.perf_counter()

    detector = SafetyDetector(
        model_path="yolo26n.pt",
        confidence=0.45
    )

    load_time = (
        time.perf_counter()
        - start
    )

    print(
        f"Model load: "
        f"{load_time:.3f} seconds"
    )

    # Fake 320x320 OpenCV-style frame.
    frame = np.zeros(
        (320, 320, 3),
        dtype=np.uint8
    )

    # ----------------------------------------
    # FIRST INFERENCE
    # ----------------------------------------

    start = time.perf_counter()

    detector.detect_people(
        frame
    )

    first_inference = (
        time.perf_counter()
        - start
    )

    print(
        f"First inference: "
        f"{first_inference:.3f} seconds"
    )

    # ----------------------------------------
    # SUBSEQUENT INFERENCES
    # ----------------------------------------

    samples = []

    for i in range(10):

        start = time.perf_counter()

        detector.detect_people(
            frame
        )

        elapsed = (
            time.perf_counter()
            - start
        )

        samples.append(
            elapsed
        )

    average = (
        sum(samples)
        / len(samples)
    )

    fps = (
        1 / average
        if average > 0
        else 0
    )

    print(
        f"Average inference: "
        f"{average:.3f} seconds"
    )

    print(
        f"Approx AI-only FPS: "
        f"{fps:.2f}"
    )


if __name__ == "__main__":
    main()
