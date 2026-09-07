import json
import time

from six_config import (
    LEGS,
    NEUTRAL,
    HIP_STRIDE,
    KNEE_LIFT,
    TRIPOD_A,
    TRIPOD_B
)


class SixRobot:

    def __init__(
        self,
        ezb,
        calibration_file="servo_directions.json"
    ):

        self.ezb = ezb

        with open(calibration_file, "r") as file:
            self.directions = json.load(file)

        # We begin from a known neutral pose.
        self.current_positions = {}

        for leg in LEGS.values():

            self.current_positions[leg["hip"]] = NEUTRAL
            self.current_positions[leg["knee"]] = NEUTRAL


    # ==================================================
    # LOW LEVEL MOTION
    # ==================================================

    def _move_smooth(
        self,
        target,
        duration=0.30,
        steps=12
    ):

        """
        Smoothly interpolate several servos to a target pose.
        """

        start = self.current_positions.copy()

        delay = duration / steps

        for step in range(1, steps + 1):

            ratio = step / steps

            positions = {}

            for port, target_position in target.items():

                start_position = start.get(
                    port,
                    target_position
                )

                position = round(
                    start_position
                    + (
                        target_position
                        - start_position
                    )
                    * ratio
                )

                position = max(
                    1,
                    min(180, position)
                )

                positions[port] = position

            self.ezb.set_servo_positions(
                positions
            )

            time.sleep(delay)

        self.current_positions.update(target)


    def _all_neutral_pose(self):

        pose = {}

        for leg in LEGS.values():

            pose[leg["hip"]] = NEUTRAL
            pose[leg["knee"]] = NEUTRAL

        return pose


    # ==================================================
    # BASIC POSES
    # ==================================================

    def stand(self):

        print("[SIX] Standing")

        self._move_smooth(
            self._all_neutral_pose(),
            duration=0.8,
            steps=24
        )


    def stop(self):

        print("[SIX] Stop")

        # Return feet to stable neutral pose.
        self.stand()


    def emergency_stop(self):

        print("[SIX] EMERGENCY STOP")

        self.ezb.release_all_servos()


    # ==================================================
    # LEG HELPERS
    # ==================================================

    def _lift_tripod(
        self,
        tripod,
        amount=KNEE_LIFT
    ):

        pose = {}

        for leg_id in tripod:

            leg = LEGS[leg_id]

            sign = self.directions[
                leg_id
            ]["knee_lift_sign"]

            pose[leg["knee"]] = (
                NEUTRAL
                + sign * amount
            )

        self._move_smooth(
            pose,
            duration=0.22,
            steps=10
        )


    def _lower_tripod(self, tripod):

        pose = {}

        for leg_id in tripod:

            leg = LEGS[leg_id]

            pose[leg["knee"]] = NEUTRAL

        self._move_smooth(
            pose,
            duration=0.22,
            steps=10
        )


    def _set_hip_stride(
        self,
        stride_by_leg
    ):

        """
        stride_by_leg example:

        {
            "FL": +1,
            "MR": +1,
            "RL": +1,

            "FR": -1,
            "ML": -1,
            "RR": -1
        }

        +1 = move foot forward
        -1 = move foot backward
        """

        pose = {}

        for leg_id, stride_direction in stride_by_leg.items():

            leg = LEGS[leg_id]

            forward_sign = self.directions[
                leg_id
            ]["hip_forward_sign"]

            angle = (
                NEUTRAL
                + forward_sign
                * HIP_STRIDE
                * stride_direction
            )

            pose[leg["hip"]] = angle

        self._move_smooth(
            pose,
            duration=0.28,
            steps=12
        )


    # ==================================================
    # GENERAL TRIPOD GAIT
    # ==================================================

    def _gait_cycle(
        self,
        movement_direction
    ):

        """
        movement_direction contains desired foot direction
        for each leg.

        +1 = forward step
        -1 = backward step
        """

        # --------------------------
        # PHASE 1
        # Tripod A lifts
        # --------------------------

        self._lift_tripod(TRIPOD_A)

        # A swings toward desired destination.
        # B pushes against floor in opposite direction.

        phase1 = {}

        for leg_id in TRIPOD_A:

            phase1[leg_id] = (
                movement_direction[leg_id]
            )

        for leg_id in TRIPOD_B:

            phase1[leg_id] = (
                -movement_direction[leg_id]
            )

        self._set_hip_stride(phase1)

        # Put tripod A down.

        self._lower_tripod(TRIPOD_A)

        # --------------------------
        # PHASE 2
        # Tripod B lifts
        # --------------------------

        self._lift_tripod(TRIPOD_B)

        phase2 = {}

        for leg_id in TRIPOD_B:

            phase2[leg_id] = (
                movement_direction[leg_id]
            )

        for leg_id in TRIPOD_A:

            phase2[leg_id] = (
                -movement_direction[leg_id]
            )

        self._set_hip_stride(phase2)

        # Put tripod B down.

        self._lower_tripod(TRIPOD_B)


    # ==================================================
    # PUBLIC MOVEMENT COMMANDS
    # ==================================================

    def forward(self, cycles=1):

        direction = {}

        for leg_id, leg in LEGS.items():
            if leg["side"] == "left":
                direction[leg_id] = -1
            else:
                direction[leg_id] = 1

        for _ in range(cycles):
            self._gait_cycle(direction)


    def backward(self, cycles=1):

        direction = {}

        for leg_id, leg in LEGS.items():
            if leg["side"] == "left":
                direction[leg_id] = 1
            else:
                direction[leg_id] = -1

        for _ in range(cycles):
            self._gait_cycle(direction)


    def turn_left(self, cycles=1):

        direction = {
            leg_id: 1
            for leg_id in LEGS
        }

        for _ in range(cycles):
            self._gait_cycle(direction)


    def turn_right(self, cycles=1):

        direction = {
            leg_id: -1
            for leg_id in LEGS
        }

        for _ in range(cycles):
            self._gait_cycle(direction)