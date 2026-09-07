import argparse

from ezb import EZB
from six_robot import SixRobot


def main():

    parser = argparse.ArgumentParser(
        description="Direct Python control for EZ-Robot Six"
    )

    parser.add_argument(
        "command",
        choices=[
            "stand",
            "forward",
            "backward",
            "left",
            "right",
            "stop",
            "release"
        ]
    )

    parser.add_argument(
        "--cycles",
        type=int,
        default=1
    )

    args = parser.parse_args()

    ezb = EZB()

    try:

        ezb.connect()

        six = SixRobot(ezb)

        if args.command == "stand":

            six.stand()

        elif args.command == "forward":

            six.stand()
            six.forward(args.cycles)
            six.stop()

        elif args.command == "backward":

            six.stand()
            six.backward(args.cycles)
            six.stop()

        elif args.command == "left":

            six.stand()
            six.turn_left(args.cycles)
            six.stop()

        elif args.command == "right":

            six.stand()
            six.turn_right(args.cycles)
            six.stop()

        elif args.command == "stop":

            six.stop()

        elif args.command == "release":

            six.emergency_stop()

    except KeyboardInterrupt:

        print()
        print("CTRL+C detected - releasing servos")

        try:
            ezb.release_all_servos()
        except Exception:
            pass

    except Exception as error:

        print()
        print("ERROR:")
        print(error)

        try:
            ezb.release_all_servos()
        except Exception:
            pass

        raise

    finally:

        ezb.disconnect()


if __name__ == "__main__":
    main()
