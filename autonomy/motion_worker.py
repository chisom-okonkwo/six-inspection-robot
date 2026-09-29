import threading
import time


class MotionWorker(threading.Thread):

    def __init__(
        self,
        robot,
        state,
        safety_stop_event,
        obstacle_event,
        shutdown_event,
        max_avoidance_turns=4
    ):

        super().__init__(
            name="MotionWorker",
            daemon=True
        )

        self.robot = robot
        self.state = state

        self.safety_stop_event = (
            safety_stop_event
        )

        self.obstacle_event = (
            obstacle_event
        )

        self.shutdown_event = (
            shutdown_event
        )

        self.max_avoidance_turns = (
            max_avoidance_turns
        )

        # When an obstacle is directly centered,
        # alternate turn directions rather than
        # always choosing the same side.
        self.center_turn_direction = (
            "left"
        )

    # ==================================================
    # TURN SELECTION
    # ==================================================

    def _choose_turn_direction(self):

        snapshot = (
            self.state.snapshot()
        )

        side = snapshot[
            "obstacle_side"
        ]

        # Obstacle appears on left:
        # move away toward right.
        if side == "left":

            return "right"

        # Obstacle appears on right:
        # move away toward left.
        if side == "right":

            return "left"

        # Centered obstacle:
        # alternate directions.
        direction = (
            self.center_turn_direction
        )

        if (
            self.center_turn_direction
            == "left"
        ):

            self.center_turn_direction = (
                "right"
            )

        else:

            self.center_turn_direction = (
                "left"
            )

        return direction

    # ==================================================
    # PERSON SAFETY HOLD
    # ==================================================

    def _wait_for_safety_clear(self):

        self.state.set_motion(
            "PERSON_HOLD"
        )

        self.robot.halt()

        print(
            "[MOTION] Person safety hold"
        )

        while (
            self.safety_stop_event.is_set()
            and
            not self.shutdown_event.is_set()
        ):

            self.shutdown_event.wait(
                0.02
            )

    # ==================================================
    # OBSTACLE AVOIDANCE
    # ==================================================

    def _avoid_obstacle(
        self,
        turn_number
    ):

        snapshot = (
            self.state.snapshot()
        )

        label = (
            snapshot["obstacle_label"]
            or "unknown"
        )

        side = (
            snapshot["obstacle_side"]
            or "center"
        )

        confidence = snapshot[
            "obstacle_confidence"
        ]

        print()
        print(
            "[NAVIGATION] "
            f"Obstacle: {label}"
        )

        print(
            "[NAVIGATION] "
            f"Confidence: "
            f"{confidence:.2f}"
        )

        print(
            "[NAVIGATION] "
            f"Location: {side}"
        )

        direction = (
            self._choose_turn_direction()
        )

        print(
            "[NAVIGATION] "
            f"Avoidance turn: {direction}"
        )

        self.state.set_motion(
            "OBSTACLE_AVOID"
        )

        # ------------------------------------------
        # Stabilize first
        # ------------------------------------------

        recovered = (
            self.robot.stand(
                interrupt_events=[
                    self.safety_stop_event,
                    self.shutdown_event
                ]
            )
        )

        if not recovered:

            return False

        # ------------------------------------------
        # Turn
        # ------------------------------------------
        #
        # IMPORTANT:
        #
        # obstacle_event is intentionally NOT
        # included as an interrupt for the turn.
        #
        # The obstacle caused the turn. If we used
        # obstacle_event here, the turn would cancel
        # itself immediately.

        turn_interrupts = [
            self.safety_stop_event,
            self.shutdown_event
        ]

        if direction == "left":

            completed = (
                self.robot.turn_left(
                    cycles=1,
                    interrupt_events=
                        turn_interrupts
                )
            )

        else:

            completed = (
                self.robot.turn_right(
                    cycles=1,
                    interrupt_events=
                        turn_interrupts
                )
            )

        if completed:

            print(
                "[NAVIGATION] "
                f"Avoidance turn "
                f"{turn_number} complete"
            )

        return completed

    # ==================================================
    # THREAD
    # ==================================================

    def run(self):

        print(
            "[MOTION] Worker online"
        )

        # Normal patrol can be interrupted by:
        #
        # 1. Person safety event
        # 2. Obstacle event
        # 3. Shutdown
        normal_interrupts = [
            self.safety_stop_event,
            self.obstacle_event,
            self.shutdown_event
        ]

        avoidance_turns = 0

        try:

            # ======================================
            # INITIAL STANCE
            # ======================================

            self.state.set_motion(
                "INITIALIZING"
            )

            # Do not let an obstacle prevent Six
            # from reaching a stable initial stance.
            completed = (
                self.robot.stand(
                    interrupt_events=[
                        self.safety_stop_event,
                        self.shutdown_event
                    ]
                )
            )

            if not completed:

                self.robot.halt()

            # ======================================
            # MAIN AUTONOMOUS NAVIGATION LOOP
            # ======================================

            while not self.shutdown_event.is_set():

                # ==================================
                # PRIORITY 1:
                # HUMAN SAFETY
                # ==================================

                if self.safety_stop_event.is_set():

                    avoidance_turns = 0

                    halt_time = (
                        time.perf_counter()
                    )

                    self._wait_for_safety_clear()

                    snapshot = (
                        self.state.snapshot()
                    )

                    detection_time = snapshot[
                        "person_detected_at"
                    ]

                    if detection_time is not None:

                        latency_ms = (
                            halt_time
                            - detection_time
                        ) * 1000

                        print(
                            "[MOTION] "
                            "Software preemption "
                            f"latency: "
                            f"{latency_ms:.1f} ms"
                        )

                    if self.shutdown_event.is_set():
                        break

                    # ----------------------------------
                    # Recover stance after person leaves
                    # ----------------------------------

                    self.state.set_motion(
                        "RECOVERING"
                    )

                    recovered = (
                        self.robot.stand(
                            interrupt_events=[
                                self.safety_stop_event,
                                self.shutdown_event
                            ]
                        )
                    )

                    if not recovered:

                        self.robot.halt()

                    continue

                # ==================================
                # PRIORITY 2:
                # OBSTACLE AVOIDANCE
                # ==================================

                if self.obstacle_event.is_set():

                    avoidance_turns += 1

                    # ----------------------------------
                    # Avoid infinite spinning
                    # ----------------------------------

                    if (
                        avoidance_turns
                        > self.max_avoidance_turns
                    ):

                        self.state.set_motion(
                            "BLOCKED"
                        )

                        self.robot.halt()

                        print()
                        print(
                            "[NAVIGATION] "
                            "Robot appears blocked."
                        )

                        print(
                            "[NAVIGATION] "
                            "Waiting for the path "
                            "to clear."
                        )

                        # Wait until:
                        #
                        # obstacle clears,
                        # a person appears,
                        # or shutdown occurs.
                        while (
                            self.obstacle_event.is_set()
                            and
                            not self.safety_stop_event.is_set()
                            and
                            not self.shutdown_event.is_set()
                        ):

                            self.shutdown_event.wait(
                                0.05
                            )

                        avoidance_turns = 0

                        continue

                    # ----------------------------------
                    # Perform one avoidance turn
                    # ----------------------------------

                    completed = (
                        self._avoid_obstacle(
                            avoidance_turns
                        )
                    )

                    if not completed:

                        self.robot.halt()

                    # Allow perception a brief moment
                    # to observe the new heading.
                    self.shutdown_event.wait(
                        0.20
                    )

                    continue

                # ==================================
                # PRIORITY 3:
                # NORMAL PATROL
                # ==================================

                avoidance_turns = 0

                self.state.set_motion(
                    "PATROLLING"
                )

                completed = (
                    self.robot.forward(
                        cycles=1,
                        interrupt_events=
                            normal_interrupts
                    )
                )

                if not completed:

                    # Something interrupted forward
                    # motion. Do not automatically
                    # return to neutral yet because
                    # the cause may be a person or
                    # obstacle.
                    self.robot.halt()

                    continue

        except Exception as error:

            print(
                "[MOTION] ERROR:",
                error
            )

            self.state.set_error(
                error
            )

            self.robot.halt()

        finally:

            self.state.set_motion(
                "STOPPED"
            )

            self.robot.halt()

            print(
                "[MOTION] Worker stopped"
            )