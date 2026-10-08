import argparse
import time

from robot.ezb import EZB
from robot.six_robot import SixRobot


def main():

    parser = argparse.ArgumentParser(
        description=(
            "Test continuous Six gait"
        )
    )

    parser.add_argument(
        "movement",
        choices=[
            "stand",
            "forward",
            "backward",
            "left",
            "right"
        ]
    )

    parser.add_argument(
        "--cycles",
        type=int,
        default=1
    )

    parser.add_argument(
        "--speed",
        type=float,
        default=1.0
    )

    args = parser.parse_args()

    ezb = EZB()

    try:

        ezb.connect()

        six = SixRobot(
            ezb
        )

        print()
        print("==============================")
        print(" CONTINUOUS GAIT TEST")
        print("==============================")
        print()

        print(
            f"Movement: "
            f"{args.movement}"
        )

        print(
            f"Cycles: "
            f"{args.cycles}"
        )

        print(
            f"Speed multiplier: "
            f"{args.speed}"
        )

        print()

        if args.movement == "stand":

            six.stand()

        elif args.movement == "forward":

            six.stand()

            time.sleep(
                1
            )

            six.forward(
                cycles=args.cycles,
                speed=args.speed
            )

            six.stop()

        elif args.movement == "backward":

            six.stand()

            time.sleep(
                1
            )

            six.backward(
                cycles=args.cycles,
                speed=args.speed
            )

            six.stop()

        elif args.movement == "left":

            six.stand()

            time.sleep(
                1
            )

            six.turn_left(
                cycles=args.cycles,
                speed=args.speed
            )

            six.stop()

        elif args.movement == "right":

            six.stand()

            time.sleep(
                1
            )

            six.turn_right(
                cycles=args.cycles,
                speed=args.speed
            )

            six.stop()

    except KeyboardInterrupt:

        print()
        print(
            "[TEST] CTRL+C"
        )

        try:

            ezb.release_all_servos()

        except Exception:
            pass

    finally:

        ezb.disconnect()


if __name__ == "__main__":
    main()