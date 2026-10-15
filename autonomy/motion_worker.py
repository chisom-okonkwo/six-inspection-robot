import threading
import time


class MotionWorker(
    threading.Thread
):

    def __init__(
        self,
        robot,
        state,
        safety_stop_event,
        obstacle_event,
        shutdown_event,
        max_avoidance_maneuvers=6,
        fresh_frames_required=2
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

        self.max_avoidance_maneuvers = (
            max_avoidance_maneuvers
        )

        self.fresh_frames_required = (
            fresh_frames_required
        )

    # ==================================================
    # FRESH PERCEPTION
    # ==================================================

    def _wait_for_fresh_perception(
        self,
        starting_frame_id,
        timeout=2.5
    ):

        target = (
            starting_frame_id
            + self.fresh_frames_required
        )

        deadline = (
            time.perf_counter()
            + timeout
        )

        while (
            time.perf_counter()
            < deadline
        ):

            if (
                self.shutdown_event.is_set()
                or
                self.safety_stop_event.is_set()
            ):

                return False

            snapshot = (
                self.state.snapshot()
            )

            if (
                snapshot[
                    "perception_frame_id"
                ]
                >= target
            ):

                return True

            self.shutdown_event.wait(
                0.03
            )

        print(
            "[NAVIGATION] "
            "Timed out waiting for "
            "fresh perception"
        )

        return True

    # ==================================================
    # PERSON SAFETY
    # ==================================================

    def _wait_for_person_clear(
        self
    ):

        self.robot.halt()

        self.state.set_motion(
            "PERSON_HOLD"
        )

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
    # MOVEMENT HELPERS
    # ==================================================

    def _strafe(
        self,
        direction
    ):

        interrupts = [
            self.safety_stop_event,
            self.shutdown_event
        ]

        print(
            "[NAVIGATION] "
            f"Strafe {direction}"
        )

        if direction == "left":

            return (
                self.robot.strafe_left(
                    cycles=1,
                    interrupt_events=
                        interrupts
                )
            )

        return (
            self.robot.strafe_right(
                cycles=1,
                interrupt_events=
                    interrupts
            )
        )

    # --------------------------------------------------

    def _rotate(
        self,
        direction
    ):

        interrupts = [
            self.safety_stop_event,
            self.shutdown_event
        ]

        print(
            "[NAVIGATION] "
            f"Rotate {direction}"
        )

        if direction == "left":

            return (
                self.robot.rotate_left(
                    cycles=1,
                    interrupt_events=
                        interrupts
                )
            )

        return (
            self.robot.rotate_right(
                cycles=1,
                interrupt_events=
                    interrupts
            )
        )

    # ==================================================
    # OBSTACLE AVOIDANCE
    # ==================================================

    def _avoid_obstacle(
        self
    ):

        maneuver_count = 0

        # Start by exploiting Six's ability
        # to move sideways.
        use_rotation_next = False

        print()
        print(
            "[NAVIGATION] "
            "Beginning obstacle avoidance"
        )

        while (
            self.obstacle_event.is_set()
            and
            not self.shutdown_event.is_set()
        ):

            # Human detection ALWAYS wins.
            if (
                self.safety_stop_event.is_set()
            ):

                return False

            if (
                maneuver_count
                >= self.max_avoidance_maneuvers
            ):

                self.state.set_motion(
                    "BLOCKED"
                )

                self.robot.halt()

                print()
                print(
                    "[NAVIGATION] "
                    "Unable to clear corridor "
                    "after multiple maneuvers."
                )

                print(
                    "[NAVIGATION] "
                    "Entering safe BLOCKED state."
                )

                # Remain safe until something
                # changes in the scene.
                while (
                    self.obstacle_event.is_set()
                    and
                    not self.safety_stop_event.is_set()
                    and
                    not self.shutdown_event.is_set()
                ):

                    self.shutdown_event.wait(
                        0.10
                    )

                return (
                    not
                    self.obstacle_event.is_set()
                )

            before = (
                self.state.snapshot()
            )

            direction = (
                before.get(
                    "obstacle_escape_direction"
                )
                or
                "left"
            )

            risk_before = (
                before.get(
                    "obstacle_risk"
                )
                or
                0.0
            )

            frame_before = (
                before[
                    "perception_frame_id"
                ]
            )

            print()
            print(
                "[NAVIGATION] "
                f"Obstacle: "
                f"{before.get('obstacle_label')}"
            )

            print(
                "[NAVIGATION] "
                f"Risk: "
                f"{risk_before:.3f}"
            )

            print(
                "[NAVIGATION] "
                f"Escape direction: "
                f"{direction}"
            )

            self.state.set_motion(
                "OBSTACLE_AVOID"
            )

            # Stabilize first.
            stable = (
                self.robot.stand(
                    interrupt_events=[
                        self.safety_stop_event,
                        self.shutdown_event
                    ]
                )
            )

            if not stable:

                return False

            # ======================================
            # SELECT MANEUVER
            # ======================================

            if use_rotation_next:

                completed = (
                    self._rotate(
                        direction
                    )
                )

                maneuver_name = (
                    f"ROTATE_{direction.upper()}"
                )

            else:

                completed = (
                    self._strafe(
                        direction
                    )
                )

                maneuver_name = (
                    f"STRAFE_{direction.upper()}"
                )

            if not completed:

                return False

            maneuver_count += 1

            print(
                "[NAVIGATION] "
                f"Completed "
                f"{maneuver_name}"
            )

            # ======================================
            # WAIT FOR THE CAMERA / AI TO OBSERVE
            # OUR NEW PHYSICAL POSITION
            # ======================================

            self._wait_for_fresh_perception(
                frame_before
            )

            if (
                self.safety_stop_event.is_set()
            ):

                return False

            # Success:
            # obstacle may still be visible,
            # but no longer overlaps our corridor.
            if (
                not
                self.obstacle_event.is_set()
            ):

                print(
                    "[NAVIGATION] "
                    "Travel corridor is clear."
                )

                return True

            after = (
                self.state.snapshot()
            )

            risk_after = (
                after.get(
                    "obstacle_risk"
                )
                or
                0.0
            )

            print(
                "[NAVIGATION] "
                f"New risk: "
                f"{risk_after:.3f}"
            )

            # ======================================
            # DID THE MANEUVER MAKE PROGRESS?
            # ======================================

            # At least ~15% improvement.
            progress = (
                risk_after
                <
                risk_before * 0.85
            )

            if progress:

                print(
                    "[NAVIGATION] "
                    "Obstacle risk decreased."
                )

                # Sidestepping is working.
                # Prefer another short strafe.
                use_rotation_next = False

            else:

                print(
                    "[NAVIGATION] "
                    "Insufficient progress; "
                    "changing maneuver type."
                )

                # A strafe that didn't help is
                # followed by an actual rotation.
                #
                # A rotation that didn't help is
                # followed by a strafe based on the
                # NEW camera view.
                use_rotation_next = (
                    not use_rotation_next
                )

        return True

    # ==================================================
    # THREAD
    # ==================================================

    def run(self):

        print(
            "[MOTION] Worker online"
        )

        normal_interrupts = [
            self.safety_stop_event,
            self.obstacle_event,
            self.shutdown_event
        ]

        try:

            # ======================================
            # INITIALIZE
            # ======================================

            self.state.set_motion(
                "INITIALIZING"
            )

            self.robot.stand(
                interrupt_events=[
                    self.safety_stop_event,
                    self.shutdown_event
                ]
            )

            # ======================================
            # CONTROL LOOP
            # ======================================

            while (
                not
                self.shutdown_event.is_set()
            ):

                # ==================================
                # PRIORITY 1 — HUMAN SAFETY
                # ==================================

                if (
                    self.safety_stop_event.is_set()
                ):

                    self._wait_for_person_clear()

                    if (
                        self.shutdown_event.is_set()
                    ):

                        break

                    self.state.set_motion(
                        "RECOVERING"
                    )

                    self.robot.stand(
                        interrupt_events=[
                            self.safety_stop_event,
                            self.shutdown_event
                        ]
                    )

                    continue

                # ==================================
                # PRIORITY 2 — OBSTACLE
                # ==================================

                if (
                    self.obstacle_event.is_set()
                ):

                    self._avoid_obstacle()

                    continue

                # ==================================
                # PRIORITY 3 — PATROL
                # ==================================

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