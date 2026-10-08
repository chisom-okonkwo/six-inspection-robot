import argparse
import time

from robot.ezb import EZB
from robot.six_robot import SixRobot


def main():

    parser = argparse.ArgumentParser(
        description=(
            "Test exact ARC Six gait"
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
        "--movement-speed",
        type=int,
        default=255,
        help=(
            "ARC-style movement speed "
            "0-255. Use 255 first."
        )
    )

    parser.add_argument(
        "--describe",
        action="store_true"
    )

    args = parser.parse_args()

    ezb = EZB()

    try:

        ezb.connect()

        six = SixRobot(
            ezb,
            movement_speed=
                args.movement_speed
        )

        action_names = {
            "stand": "Stop",
            "forward": "Forward",
            "backward": "Reverse",
            "left": "Left",
            "right": "Right",
        }

        action_name = (
            action_names[
                args.movement
            ]
        )

        if args.describe:

            six.gait.describe_action(
                action_name
            )

        print()
        print("==============================")
        print(" ARC SIX GAIT TEST")
        print("==============================")
        print()

        print(
            "Movement:",
            args.movement
        )

        print(
            "Cycles:",
            args.cycles
        )

        print(
            "ARC movement speed:",
            args.movement_speed
        )

        print()

        # Put Six into the actual ARC
        # STAND frame first.
        six.stand()

        time.sleep(
            0.5
        )

        if args.movement == "stand":

            return

        if args.movement == "forward":

            six.forward(
                cycles=args.cycles
            )

        elif args.movement == "backward":

            six.backward(
                cycles=args.cycles
            )

        elif args.movement == "left":

            six.turn_left(
                cycles=args.cycles
            )

        elif args.movement == "right":

            six.turn_right(
                cycles=args.cycles
            )

        # Return using ARC's own Stop action.
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