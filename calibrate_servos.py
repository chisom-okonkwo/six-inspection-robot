import json
import time

from ezb import EZB
from six_config import LEGS, NEUTRAL


TEST_OFFSET = 10

OUTPUT_FILE = "servo_directions.json"


def ask_yes_no(question):

    while True:

        answer = input(question + " [y/n]: ").strip().lower()

        if answer in ("y", "yes"):
            return True

        if answer in ("n", "no"):
            return False

        print("Please enter y or n.")


def neutralize(robot):

    positions = {}

    for leg in LEGS.values():

        positions[leg["hip"]] = NEUTRAL
        positions[leg["knee"]] = NEUTRAL

    robot.set_servo_positions(positions)

    time.sleep(1)


def main():

    print()
    print("======================================")
    print(" SIX SERVO DIRECTION CALIBRATION")
    print("======================================")
    print()
    print("IMPORTANT:")
    print("Lift Six so ALL six feet are off the floor.")
    print("Keep hands away from moving joints.")
    print()
    input("Press ENTER when Six is safely supported...")

    robot = EZB()

    directions = {}

    try:

        robot.connect()

        print()
        print("Moving all Six servos to neutral...")
        print()

        neutralize(robot)

        for leg_id, leg in LEGS.items():

            print()
            print("-----------------------------------")
            print(f"{leg_id}: {leg['name']}")
            print("-----------------------------------")

            hip = leg["hip"]
            knee = leg["knee"]

            directions[leg_id] = {}

            # -----------------------------------
            # HIP TEST
            # -----------------------------------

            print()
            print(f"Testing HIP servo D{hip}")
            print()
            print(
                f"I will move D{hip} from "
                f"{NEUTRAL}° to {NEUTRAL + TEST_OFFSET}°"
            )

            input("Press ENTER to perform test...")

            robot.set_servo_position(
                hip,
                NEUTRAL + TEST_OFFSET
            )

            time.sleep(1)

            positive_is_forward = ask_yes_no(
                "Did the FOOT move toward the FRONT of the robot?"
            )

            directions[leg_id]["hip_forward_sign"] = (
                1 if positive_is_forward else -1
            )

            robot.set_servo_position(hip, NEUTRAL)

            time.sleep(0.7)

            # -----------------------------------
            # KNEE TEST
            # -----------------------------------

            print()
            print(f"Testing outer/leg servo D{knee}")
            print()
            print(
                f"I will move D{knee} from "
                f"{NEUTRAL}° to {NEUTRAL + TEST_OFFSET}°"
            )

            input("Press ENTER to perform test...")

            robot.set_servo_position(
                knee,
                NEUTRAL + TEST_OFFSET
            )

            time.sleep(1)

            positive_is_up = ask_yes_no(
                "Did this movement LIFT the foot upward?"
            )

            directions[leg_id]["knee_lift_sign"] = (
                1 if positive_is_up else -1
            )

            robot.set_servo_position(knee, NEUTRAL)

            time.sleep(0.7)

        # Save result

        with open(OUTPUT_FILE, "w") as file:
            json.dump(directions, file, indent=4)

        print()
        print("======================================")
        print(" CALIBRATION COMPLETE")
        print("======================================")
        print()

        print(json.dumps(directions, indent=4))

        print()
        print(f"Saved to: {OUTPUT_FILE}")

    except KeyboardInterrupt:

        print()
        print("Calibration interrupted.")

        robot.release_all_servos()

    finally:

        robot.disconnect()


if __name__ == "__main__":
    main()
