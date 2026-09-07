from ezb import EZB


robot = EZB()

try:
    robot.connect()

    robot.set_servo_position(0, 90)
    robot.set_servo_position(1, 90)
    robot.release_all_servos()

    uid = robot.get_unique_id()

    print("EZ-B unique ID:", uid)

except KeyboardInterrupt:
    print("Emergency stop requested")

finally:
    try:
        robot.release_all_servos()
    except Exception:
        pass

    robot.disconnect()
