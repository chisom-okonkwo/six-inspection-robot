import json
import time

from six_config import (
    LEGS,
    NEUTRAL,
    HIP_STRIDE,
    KNEE_LIFT,
    TRIPOD_A,
    TRIPOD_B,
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

        self.current_positions = {}

        for leg in LEGS.values():
            self.current_positions[leg["hip"]] = NEUTRAL
            self.current_positions[leg["knee"]] = NEUTRAL

    # ==================================================
    # INTERRUPTION
    # ==================================================

    @staticmethod
    def _normalize_events(interrupt_events):

        if interrupt_events is None:
            return []

        if hasattr(interrupt_events, "is_set"):
            return [interrupt_events]

        return list(interrupt_events)

    @staticmethod
    def _interrupted(events):

        return any(
            event.is_set()
            for event in events
        )

    def _interruptible_sleep(
        self,
        duration,
        events
    ):
        """
        Sleep while checking interruption events.

        Checking roughly every 10 ms prevents a long
        time.sleep() from delaying a safety stop.
        """

        if not events:
            time.sleep(duration)
            return True

        end_time = time.perf_counter() + duration

        while time.perf_counter() < end_time:

            if self._interrupted(events):
                return False

            remaining = (
                end_time - time.perf_counter()
            )

            time.sleep(
                min(0.01, max(0, remaining))
            )

        return True

    # ==================================================
    # LOW LEVEL MOTION
    # ==================================================

    def _move_smooth(
        self,
        target,
        duration=0.30,
        steps=12,
        interrupt_events=None
    ):

        events = self._normalize_events(
            interrupt_events
        )

        start = self.current_positions.copy()

        delay = duration / steps

        for step in range(1, steps + 1):

            if self._interrupted(events):
                return False

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

            # This is important:
            # remember the LAST position actually sent,
            # not merely the final intended target.
            self.current_positions.update(
                positions
            )

            completed_sleep = (
                self._interruptible_sleep(
                    delay,
                    events
                )
            )

            if not completed_sleep:
                return False

        return True

    # ==================================================
    # POSES
    # ==================================================

    def _all_neutral_pose(self):

        pose = {}

        for leg in LEGS.values():
            pose[leg["hip"]] = NEUTRAL
            pose[leg["knee"]] = NEUTRAL

        return pose

    def stand(
        self,
        interrupt_events=None
    ):

        print("[SIX] Standing")

        return self._move_smooth(
            self._all_neutral_pose(),
            duration=0.8,
            steps=24,
            interrupt_events=interrupt_events
        )

    def stop(self):

        print("[SIX] Normal stop")

        return self.stand()

    def halt(self):
        """
        Immediately stop issuing new movement commands.

        Servos continue holding their most recently
        commanded positions.

        This is different from releasing them.
        """

        print("[SIX] Motion halted")

    def emergency_stop(self):

        print("[SIX] EMERGENCY SERVO RELEASE")

        self.ezb.release_all_servos()

    # ==================================================
    # LEG HELPERS
    # ==================================================

    def _lift_tripod(
        self,
        tripod,
        amount=KNEE_LIFT,
        interrupt_events=None
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

        return self._move_smooth(
            pose,
            duration=0.22,
            steps=10,
            interrupt_events=interrupt_events
        )

    def _lower_tripod(
        self,
        tripod,
        interrupt_events=None
    ):

        pose = {}

        for leg_id in tripod:

            leg = LEGS[leg_id]

            pose[leg["knee"]] = NEUTRAL

        return self._move_smooth(
            pose,
            duration=0.22,
            steps=10,
            interrupt_events=interrupt_events
        )

    def _set_hip_stride(
        self,
        stride_by_leg,
        interrupt_events=None
    ):

        pose = {}

        for leg_id, stride_direction in (
            stride_by_leg.items()
        ):

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

        return self._move_smooth(
            pose,
            duration=0.28,
            steps=12,
            interrupt_events=interrupt_events
        )

    # ==================================================
    # TRIPOD GAIT
    # ==================================================

    def _gait_cycle(
        self,
        movement_direction,
        interrupt_events=None
    ):

        # --------------------------
        # Tripod A
        # --------------------------

        if not self._lift_tripod(
            TRIPOD_A,
            interrupt_events=interrupt_events
        ):
            return False

        phase1 = {}

        for leg_id in TRIPOD_A:
            phase1[leg_id] = (
                movement_direction[leg_id]
            )

        for leg_id in TRIPOD_B:
            phase1[leg_id] = (
                -movement_direction[leg_id]
            )

        if not self._set_hip_stride(
            phase1,
            interrupt_events=interrupt_events
        ):
            return False

        if not self._lower_tripod(
            TRIPOD_A,
            interrupt_events=interrupt_events
        ):
            return False

        # --------------------------
        # Tripod B
        # --------------------------

        if not self._lift_tripod(
            TRIPOD_B,
            interrupt_events=interrupt_events
        ):
            return False

        phase2 = {}

        for leg_id in TRIPOD_B:
            phase2[leg_id] = (
                movement_direction[leg_id]
            )

        for leg_id in TRIPOD_A:
            phase2[leg_id] = (
                -movement_direction[leg_id]
            )

        if not self._set_hip_stride(
            phase2,
            interrupt_events=interrupt_events
        ):
            return False

        if not self._lower_tripod(
            TRIPOD_B,
            interrupt_events=interrupt_events
        ):
            return False

        return True

    # ==================================================
    # MOVEMENT
    # ==================================================

    def forward(
        self,
        cycles=1,
        interrupt_events=None
    ):

        direction = {
            leg_id: 1
            for leg_id in LEGS
        }

        for _ in range(cycles):

            if not self._gait_cycle(
                direction,
                interrupt_events
            ):
                return False

        return True

    def backward(
        self,
        cycles=1,
        interrupt_events=None
    ):

        direction = {
            leg_id: -1
            for leg_id in LEGS
        }

        for _ in range(cycles):

            if not self._gait_cycle(
                direction,
                interrupt_events
            ):
                return False

        return True

    def turn_left(
        self,
        cycles=1,
        interrupt_events=None
    ):

        direction = {}

        for leg_id, leg in LEGS.items():

            if leg["side"] == "left":
                direction[leg_id] = -1
            else:
                direction[leg_id] = 1

        for _ in range(cycles):

            if not self._gait_cycle(
                direction,
                interrupt_events
            ):
                return False

        return True

    def turn_right(
        self,
        cycles=1,
        interrupt_events=None
    ):

        direction = {}

        for leg_id, leg in LEGS.items():

            if leg["side"] == "left":
                direction[leg_id] = 1
            else:
                direction[leg_id] = -1

        for _ in range(cycles):

            if not self._gait_cycle(
                direction,
                interrupt_events
            ):
                return False

        return True