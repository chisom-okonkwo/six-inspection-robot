from pathlib import Path

from robot.arc_gait import (
    ArcGaitPlayer
)


ROBOT_DIR = (
    Path(__file__)
    .resolve()
    .parent
)

DEFAULT_GAIT_FILE = (
    ROBOT_DIR
    / "gaits"
    / "six_stock_gait.AutoPosition"
)


class SixRobot:

    def __init__(
        self,
        ezb,
        gait_file=DEFAULT_GAIT_FILE,
        movement_speed=255
    ):

        self.ezb = ezb

        self.gait = ArcGaitPlayer(
            ezb=ezb,
            auto_position_file=
                gait_file,
            movement_speed=
                movement_speed
        )

    # ==================================================
    # SPEED
    # ==================================================

    def set_movement_speed(
        self,
        speed
    ):

        self.gait.set_movement_speed(
            speed
        )

    # ==================================================
    # STANDING / STOP
    # ==================================================

    def stand(
        self,
        interrupt_events=None
    ):

        print(
            "[SIX] ARC STAND"
        )

        return self.gait.play_action(
            "Stop",
            cycles=1,
            interrupt_events=
                interrupt_events
        )

    def stop(self):

        print(
            "[SIX] ARC STOP"
        )

        return self.gait.play_action(
            "Stop",
            cycles=1
        )

    def halt(self):
        """
        Stop sending new position commands.

        Servos continue holding the last
        commanded ARC gait position.
        """

        print(
            "[SIX] Motion halted"
        )

    def emergency_stop(self):

        print(
            "[SIX] "
            "EMERGENCY SERVO RELEASE"
        )

        self.ezb.release_all_servos()

    # ==================================================
    # ARC MOVEMENT ACTIONS
    # ==================================================

    def forward(
        self,
        cycles=1,
        interrupt_events=None
    ):

        return self.gait.play_action(
            "Forward",
            cycles=cycles,
            interrupt_events=
                interrupt_events
        )

    def backward(
        self,
        cycles=1,
        interrupt_events=None
    ):

        return self.gait.play_action(
            "Reverse",
            cycles=cycles,
            interrupt_events=
                interrupt_events
        )

    def turn_left(
        self,
        cycles=1,
        interrupt_events=None
    ):

        return self.gait.play_action(
            "Left",
            cycles=cycles,
            interrupt_events=
                interrupt_events
        )

    def turn_right(
        self,
        cycles=1,
        interrupt_events=None
    ):

        return self.gait.play_action(
            "Right",
            cycles=cycles,
            interrupt_events=
                interrupt_events
        )