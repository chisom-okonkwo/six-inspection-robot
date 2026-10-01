import json

from pathlib import Path


PROJECT_ROOT = (
    Path(__file__)
    .resolve()
    .parent
    .parent
)

LOG_DIR = (
    PROJECT_ROOT
    / "data"
    / "logs"
)


def main():

    logs = list(
        LOG_DIR.glob(
            "inspection_*.jsonl"
        )
    )

    if not logs:

        print(
            "No inspection logs found."
        )

        return

    latest = max(
        logs,
        key=lambda path:
            path.stat().st_mtime
    )

    print()
    print("==============================")
    print(" LATEST INSPECTION REPORT")
    print("==============================")
    print()

    print(
        f"Log: {latest.name}"
    )

    print()

    count = 0

    with open(
        latest,
        "r",
        encoding="utf-8"
    ) as file:

        for line in file:

            if not line.strip():
                continue

            event = json.loads(
                line
            )

            count += 1

            print(
                f"[{event['timestamp']}]"
            )

            print(
                f"  Event: "
                f"{event['event_type']}"
            )

            if event.get(
                "person_detected"
            ):

                print(
                    "  Person confidence: "
                    f"{event.get('person_confidence', 0):.2f}"
                )

            if event.get(
                "obstacle_detected"
            ):

                print(
                    "  Obstacle: "
                    f"{event.get('obstacle_label')}"
                )

                print(
                    "  Side: "
                    f"{event.get('obstacle_side')}"
                )

                print(
                    "  Confidence: "
                    f"{event.get('obstacle_confidence', 0):.2f}"
                )

            print(
                "  Motion: "
                f"{event.get('motion_state')}"
            )

            if event.get(
                "snapshot"
            ):

                print(
                    "  Snapshot: "
                    f"{event['snapshot']}"
                )

            print()

    print(
        f"Total recorded events: "
        f"{count}"
    )


if __name__ == "__main__":
    main()