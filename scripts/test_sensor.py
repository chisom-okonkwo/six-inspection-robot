import argparse
import statistics
import time
from collections import deque

from robot.ezb import EZB


def main():

    parser = argparse.ArgumentParser(
        description=(
            "Direct HC-SR04 test through EZ-B v4"
        )
    )

    parser.add_argument(
        "--trigger",
        type=int,
        default=20,
        help="EZ-B trigger digital port"
    )

    parser.add_argument(
        "--echo",
        type=int,
        default=21,
        help="EZ-B echo digital port"
    )

    parser.add_argument(
        "--interval",
        type=float,
        default=0.10,
        help=(
            "Seconds between readings. "
            "Keep >= 0.07."
        )
    )

    args = parser.parse_args()

    interval = max(
        0.07,
        args.interval
    )

    history = deque(
        maxlen=5
    )

    ezb = EZB()

    try:

        ezb.connect()

        print()
        print(
            "================================"
        )

        print(
            " HC-SR04 SENSOR TEST"
        )

        print(
            "================================"
        )

        print(
            f"TRIG = D{args.trigger}"
        )

        print(
            f"ECHO = D{args.echo}"
        )

        print()

        print(
            "Move a flat object toward/away "
            "from the sensor."
        )

        print(
            "CTRL+C = stop"
        )

        print()

        while True:

            raw_cm = (
                ezb.get_ping(
                    args.trigger,
                    args.echo
                )
            )

            valid = (
                2 <= raw_cm < 255
            )

            if valid:

                history.append(
                    raw_cm
                )

                filtered = (
                    statistics.median(
                        history
                    )
                )

                print(
                    f"raw={raw_cm:3d} cm  "
                    f"median={filtered:5.1f} cm"
                )

            else:

                print(
                    f"raw={raw_cm:3d}  "
                    "NO VALID ECHO"
                )

            time.sleep(
                interval
            )

    except KeyboardInterrupt:

        print()
        print(
            "[TEST] Stopped"
        )

    finally:

        ezb.disconnect()


if __name__ == "__main__":
    main()
