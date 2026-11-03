import math
import threading
import time


class MotionWorker(threading.Thread):
    """
    Motion priority:
        1. person safety
        2. wall / very-close unknown surface
        3. recognized object avoidance
        4. stuck/no-progress recovery
        5. normal patrol

    Turn angles are approximate because Six currently has no IMU.
    Desired degrees are converted to ARC Rotate cycles using the
    estimated_degrees_per_rotate_cycle parameter.
    """

    def __init__(
        self,
        robot,
        state,
        safety_stop_event,
        obstacle_event,
        wall_event,
        stuck_event,
        shutdown_event,
        fresh_frames_required=4,
        backoff_cycles=1,
        initial_turn_degrees=50.0,
        turn_increment_degrees=25.0,
        estimated_degrees_per_rotate_cycle=25.0,
        max_turn_extensions=4,
        repeat_obstacle_window_seconds=5.0,
        close_rebackoff_distance_m=0.45,
        rebackoff_every_extensions=2,
        # Wall-specific turn behavior.
        wall_backoff_cycles=1,
        wall_initial_turn_degrees=70.0,
        wall_turn_increment_degrees=25.0,
        wall_max_turn_degrees=160.0,
        wall_repeat_window_seconds=4.0,
        # Stuck recovery.
        stuck_backoff_cycles=1,
        stuck_initial_turn_degrees=90.0,
        stuck_turn_increment_degrees=30.0,
        stuck_max_turn_degrees=180.0,
        stuck_clear_distance_m=1.10,
    ):
        super().__init__(name="MotionWorker", daemon=True)

        self.robot = robot
        self.state = state

        self.safety_stop_event = safety_stop_event
        self.obstacle_event = obstacle_event
        self.wall_event = wall_event
        self.stuck_event = stuck_event
        self.shutdown_event = shutdown_event

        self.fresh_frames_required = max(1, int(fresh_frames_required))

        self.backoff_cycles = max(0, int(backoff_cycles))
        self.initial_turn_degrees = max(1.0, float(initial_turn_degrees))
        self.turn_increment_degrees = max(1.0, float(turn_increment_degrees))
        self.estimated_degrees_per_rotate_cycle = max(
            1.0,
            float(estimated_degrees_per_rotate_cycle),
        )
        self.max_turn_extensions = max(1, int(max_turn_extensions))

        self.repeat_obstacle_window_seconds = max(
            0.0,
            float(repeat_obstacle_window_seconds),
        )
        self.close_rebackoff_distance_m = max(
            0.0,
            float(close_rebackoff_distance_m),
        )
        self.rebackoff_every_extensions = max(
            1,
            int(rebackoff_every_extensions),
        )

        self.wall_backoff_cycles = max(0, int(wall_backoff_cycles))
        self.wall_initial_turn_degrees = max(
            1.0,
            float(wall_initial_turn_degrees),
        )
        self.wall_turn_increment_degrees = max(
            1.0,
            float(wall_turn_increment_degrees),
        )
        self.wall_max_turn_degrees = max(
            self.wall_initial_turn_degrees,
            float(wall_max_turn_degrees),
        )
        self.wall_repeat_window_seconds = max(
            0.0,
            float(wall_repeat_window_seconds),
        )

        self.stuck_backoff_cycles = max(0, int(stuck_backoff_cycles))
        self.stuck_initial_turn_degrees = max(
            1.0,
            float(stuck_initial_turn_degrees),
        )
        self.stuck_turn_increment_degrees = max(
            1.0,
            float(stuck_turn_increment_degrees),
        )
        self.stuck_max_turn_degrees = max(
            self.stuck_initial_turn_degrees,
            float(stuck_max_turn_degrees),
        )
        self.stuck_clear_distance_m = max(
            0.0,
            float(stuck_clear_distance_m),
        )

        self._fallback_turn_direction = "left"

        self._last_avoidance_time = 0.0
        self._last_avoidance_label = None
        self._last_turn_direction = None

        self._last_wall_avoidance_time = 0.0
        self._last_wall_turn_direction = None

    # =========================================================
    # COMMON HELPERS
    # =========================================================

    def _movement_interrupts(self):
        return [
            self.safety_stop_event,
            self.shutdown_event,
        ]

    def _wait_for_fresh_perception(self, starting_frame_id, timeout=4.0):
        target = starting_frame_id + self.fresh_frames_required
        deadline = time.perf_counter() + timeout

        while time.perf_counter() < deadline:
            if (
                self.shutdown_event.is_set()
                or self.safety_stop_event.is_set()
            ):
                return False

            if self.state.snapshot()["perception_frame_id"] >= target:
                return True

            self.shutdown_event.wait(0.03)

        print("[NAVIGATION] Timed out waiting for fresh perception")
        return True

    def _wait_for_person_clear(self):
        self.robot.halt()
        self.state.set_motion("PERSON_HOLD")
        print("[MOTION] Person safety hold")

        while (
            self.safety_stop_event.is_set()
            and not self.shutdown_event.is_set()
        ):
            self.shutdown_event.wait(0.02)

    # =========================================================
    # DIRECTION
    # =========================================================

    def _choose_escape_direction(self, snapshot):
        explicit = snapshot.get("obstacle_escape_direction")

        if explicit in ("left", "right"):
            return explicit

        left_clearance = snapshot.get("left_clearance_m")
        right_clearance = snapshot.get("right_clearance_m")

        if (
            left_clearance is not None
            and right_clearance is not None
            and abs(left_clearance - right_clearance) >= 0.10
        ):
            return (
                "left"
                if left_clearance > right_clearance
                else "right"
            )

        direction = self._fallback_turn_direction
        self._fallback_turn_direction = (
            "right" if direction == "left" else "left"
        )
        return direction

    # =========================================================
    # ROBOT PRIMITIVES
    # =========================================================

    def _stand(self):
        return self.robot.stand(
            interrupt_events=self._movement_interrupts()
        )

    def _backward(self, cycles):
        if cycles <= 0:
            return True

        print(f"[NAVIGATION] Backward x{cycles}")

        return self.robot.backward(
            cycles=cycles,
            interrupt_events=self._movement_interrupts(),
        )

    def _rotate_one(self, direction):
        if direction == "left":
            return self.robot.rotate_left(
                cycles=1,
                interrupt_events=self._movement_interrupts(),
            )

        return self.robot.rotate_right(
            cycles=1,
            interrupt_events=self._movement_interrupts(),
        )

    # =========================================================
    # TURN CONTROL
    # =========================================================

    def _degrees_to_cycles(self, degrees):
        return max(
            1,
            math.ceil(
                float(degrees)
                / self.estimated_degrees_per_rotate_cycle
            ),
        )

    def _rotate_degrees(self, direction, degrees):
        cycles = self._degrees_to_cycles(degrees)

        print()
        print(
            "[NAVIGATION] "
            f"Committed turn: ~{degrees:.0f} degrees {direction}"
        )
        print(
            "[NAVIGATION] "
            f"Using {cycles} ARC rotate cycle(s)"
        )

        self.state.set_motion("TURNING")

        for index in range(1, cycles + 1):
            if (
                self.safety_stop_event.is_set()
                or self.shutdown_event.is_set()
            ):
                return False

            frame_before = self.state.snapshot()["perception_frame_id"]

            print(
                "[NAVIGATION] "
                f"Rotate cycle {index}/{cycles}"
            )

            if not self._rotate_one(direction):
                return False

            self._wait_for_fresh_perception(frame_before)

        return self._stand()

    def _create_clearance(self, cycles):
        self.state.set_motion("BACKING_OFF")
        print()
        print("[NAVIGATION] Creating front-leg clearance")

        if not self._stand():
            return False

        frame_before = self.state.snapshot()["perception_frame_id"]

        if not self._backward(cycles):
            return False

        if not self._stand():
            return False

        self._wait_for_fresh_perception(frame_before)
        return True

    # =========================================================
    # WALL / CLOSE-SURFACE AVOIDANCE
    # =========================================================

    def _avoid_wall(self):
        """
        HC-SR04-only wall/close-surface behavior.

        First detection:
            back up once -> committed turn

        If the wall is still <= wall threshold:
            add turn incrementally until clear or maximum

        A repeated wall detection shortly afterward does not
        back up from scratch; it continues changing heading.
        """
        now = time.perf_counter()

        repeat_wall = (
            self._last_wall_turn_direction in ("left", "right")
            and now - self._last_wall_avoidance_time
            <= self.wall_repeat_window_seconds
        )

        if repeat_wall:
            direction = self._last_wall_turn_direction
        else:
            direction = self._choose_escape_direction(
                self.state.snapshot()
            )

        print()
        print("[NAVIGATION] WALL / CLOSE-SURFACE AVOIDANCE")
        print(f"[NAVIGATION] Turn direction: {direction}")

        turned = 0.0

        if repeat_wall:
            print(
                "[NAVIGATION] Recent wall re-detection: "
                "skip repeated full backoff and extend turn"
            )
            first_turn = self.wall_turn_increment_degrees
        else:
            if not self._create_clearance(self.wall_backoff_cycles):
                return False

            # Even if backing up clears wall_event, still turn.
            # Otherwise Six would walk toward the same wall again.
            first_turn = self.wall_initial_turn_degrees

        if not self._rotate_degrees(direction, first_turn):
            return False

        turned += first_turn
        self._last_wall_turn_direction = direction
        self._last_wall_avoidance_time = time.perf_counter()

        frame_after_turn = self.state.snapshot()["perception_frame_id"]
        self._wait_for_fresh_perception(frame_after_turn)

        if not self.wall_event.is_set():
            print("[NAVIGATION] Wall/close surface is no longer ahead.")
            return True

        while (
            self.wall_event.is_set()
            and turned < self.wall_max_turn_degrees
        ):
            additional = min(
                self.wall_turn_increment_degrees,
                self.wall_max_turn_degrees - turned,
            )

            print(
                "[NAVIGATION] Wall still close; "
                f"adding ~{additional:.0f} degrees"
            )

            if not self._rotate_degrees(direction, additional):
                return False

            turned += additional
            self._last_wall_avoidance_time = time.perf_counter()

            frame_after_extension = (
                self.state.snapshot()["perception_frame_id"]
            )
            self._wait_for_fresh_perception(frame_after_extension)

        if self.wall_event.is_set():
            self.state.set_motion("BLOCKED")
            self.robot.halt()
            print(
                "[NAVIGATION] Wall still present after maximum turn. "
                "Entering BLOCKED."
            )
            return False

        print(
            "[NAVIGATION] Wall avoidance complete. "
            f"Approx requested turn: {turned:.0f} degrees"
        )
        return True

    # =========================================================
    # OBJECT AVOIDANCE MEMORY
    # =========================================================

    def _is_repeat_obstacle(self, label):
        if label is None or self._last_avoidance_label is None:
            return False

        return (
            label == self._last_avoidance_label
            and time.perf_counter() - self._last_avoidance_time
            <= self.repeat_obstacle_window_seconds
        )

    def _remember_avoidance(self, label, direction):
        self._last_avoidance_label = label
        self._last_turn_direction = direction
        self._last_avoidance_time = time.perf_counter()

    # =========================================================
    # RECOGNIZED OBJECT AVOIDANCE
    # =========================================================

    def _avoid_obstacle(self):
        first_snapshot = self.state.snapshot()
        label = first_snapshot.get("obstacle_label")
        repeat_obstacle = self._is_repeat_obstacle(label)

        if (
            repeat_obstacle
            and self._last_turn_direction in ("left", "right")
        ):
            direction = self._last_turn_direction
        else:
            direction = self._choose_escape_direction(first_snapshot)

        print()
        print("[NAVIGATION] Beginning object avoidance")
        print(f"[NAVIGATION] Object: {label}")
        print(f"[NAVIGATION] Escape direction: {direction}")

        if repeat_obstacle:
            print(
                "[NAVIGATION] Same object re-detected recently; "
                "extending turn instead of repeating full backoff"
            )

            if not self._rotate_degrees(
                direction,
                self.turn_increment_degrees,
            ):
                return False

        else:
            if not self._create_clearance(self.backoff_cycles):
                return False

            if not self._rotate_degrees(
                direction,
                self.initial_turn_degrees,
            ):
                return False

        self._remember_avoidance(label, direction)

        frame_after_turn = self.state.snapshot()["perception_frame_id"]
        self._wait_for_fresh_perception(frame_after_turn)

        if not self.obstacle_event.is_set():
            print("[NAVIGATION] New object heading is clear.")
            return True

        for extension in range(1, self.max_turn_extensions + 1):
            current = self.state.snapshot()
            obstacle_distance = current.get("obstacle_distance_m")

            very_close = (
                obstacle_distance is not None
                and obstacle_distance <= self.close_rebackoff_distance_m
            )

            should_rebackoff = (
                very_close
                or extension % self.rebackoff_every_extensions == 0
            )

            if should_rebackoff:
                if not self._create_clearance(1):
                    return False

            if not self._rotate_degrees(
                direction,
                self.turn_increment_degrees,
            ):
                return False

            self._remember_avoidance(label, direction)

            frame_after_extension = (
                self.state.snapshot()["perception_frame_id"]
            )
            self._wait_for_fresh_perception(frame_after_extension)

            if not self.obstacle_event.is_set():
                print("[NAVIGATION] Incremental object turn cleared path.")
                return True

        self.state.set_motion("BLOCKED")
        self.robot.halt()
        print(
            "[NAVIGATION] Object remains after maximum committed turn. "
            "Entering BLOCKED."
        )
        return False

    # =========================================================
    # STUCK RECOVERY
    # =========================================================

    def _recover_from_stuck(self, reason="low_visual_progress"):
        print()
        print("[NAVIGATION] STUCK RECOVERY")
        print(f"[NAVIGATION] Reason: {reason}")

        self.state.set_motion("STUCK_RECOVERY")
        self.stuck_event.clear()

        if not self._create_clearance(self.stuck_backoff_cycles):
            return False

        direction = self._choose_escape_direction(self.state.snapshot())
        turned = 0.0

        if not self._rotate_degrees(
            direction,
            self.stuck_initial_turn_degrees,
        ):
            return False

        turned += self.stuck_initial_turn_degrees

        while turned < self.stuck_max_turn_degrees:
            current = self.state.snapshot()
            corridor_depth = current.get("corridor_depth_m")

            if (
                corridor_depth is not None
                and corridor_depth >= self.stuck_clear_distance_m
                and not self.obstacle_event.is_set()
                and not self.wall_event.is_set()
            ):
                return True

            if corridor_depth is None:
                return True

            if not self._create_clearance(1):
                return False

            remaining = self.stuck_max_turn_degrees - turned
            additional = min(
                self.stuck_turn_increment_degrees,
                remaining,
            )

            if not self._rotate_degrees(direction, additional):
                return False

            turned += additional

        return True

    # =========================================================
    # THREAD
    # =========================================================

    def run(self):
        print("[MOTION] Worker online")

        patrol_interrupts = [
            self.safety_stop_event,
            self.wall_event,
            self.obstacle_event,
            self.stuck_event,
            self.shutdown_event,
        ]

        try:
            self.state.set_motion("INITIALIZING")
            self.robot.stand(
                interrupt_events=self._movement_interrupts()
            )

            while not self.shutdown_event.is_set():
                # 1. Person safety.
                if self.safety_stop_event.is_set():
                    self._wait_for_person_clear()

                    if self.shutdown_event.is_set():
                        break

                    self.state.set_motion("RECOVERING")
                    self._stand()
                    continue

                # 2. Very close wall / unknown surface.
                if self.wall_event.is_set():
                    self._avoid_wall()
                    continue

                # 3. Backpack/book confirmed by camera + range.
                if self.obstacle_event.is_set():
                    self._avoid_obstacle()
                    continue

                # 4. No-progress fallback.
                if self.stuck_event.is_set():
                    self._recover_from_stuck()
                    continue

                # 5. Normal patrol.
                self.state.set_motion("PATROLLING")

                completed = self.robot.forward(
                    cycles=1,
                    interrupt_events=patrol_interrupts,
                )

                if not completed:
                    self.robot.halt()
                    continue

        except Exception as error:
            print("[MOTION] ERROR:", error)
            self.state.set_error(error)
            self.robot.halt()

        finally:
            self.state.set_motion("STOPPED")
            self.robot.halt()
            print("[MOTION] Worker stopped")
