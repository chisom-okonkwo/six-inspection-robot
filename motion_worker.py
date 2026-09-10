import threading
import time


class MotionWorker(threading.Thread):

    def __init__(
        self,
        robot,
        state,
        safety_stop_event,
        shutdown_event
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

        self.shutdown_event = (
            shutdown_event
        )

    # --------------------------------------------------

    def run(self):

        print(
            "[MOTION] Worker online"
        )

        interrupt_events = [
            self.safety_stop_event,
            self.shutdown_event
        ]

        was_safety_holding = False

        try:

            # ----------------------------------
            # Do not start while person present.
            # ----------------------------------

            while (
                self.safety_stop_event.is_set()
                and not self.shutdown_event.is_set()
            ):

                self.state.set_motion(
                    "SAFETY_HOLD"
                )

                self.shutdown_event.wait(
                    0.05
                )

            if self.shutdown_event.is_set():
                return

            # ----------------------------------
            # Stable starting pose
            # ----------------------------------

            self.state.set_motion(
                "INITIALIZING"
            )

            standing_completed = (
                self.robot.stand(
                    interrupt_events=
                        interrupt_events
                )
            )

            if not standing_completed:

                self.robot.halt()

            # ==================================
            # AUTONOMOUS PATROL LOOP
            # ==================================

            while not self.shutdown_event.is_set():

                # ==============================
                # SAFETY HOLD
                # ==============================

                if self.safety_stop_event.is_set():

                    if not was_safety_holding:

                        halt_time = (
                            time.perf_counter()
                        )

                        self.robot.halt()

                        self.state.set_motion(
                            "SAFETY_HOLD"
                        )

                        snapshot = (
                            self.state.snapshot()
                        )

                        detection_time = (
                            snapshot[
                                "person_detected_at"
                            ]
                        )

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

                        print(
                            "[MOTION] "
                            "Waiting for safe condition"
                        )

                        was_safety_holding = True

                    self.shutdown_event.wait(
                        0.02
                    )

                    continue

                # ==============================
                # RECOVER AFTER SAFETY STOP
                # ==============================

                if was_safety_holding:

                    print(
                        "[MOTION] "
                        "Recovering stable stance"
                    )

                    self.state.set_motion(
                        "RECOVERING"
                    )

                    recovered = (
                        self.robot.stand(
                            interrupt_events=
                                interrupt_events
                        )
                    )

                    if not recovered:

                        # Person may have appeared
                        # again during recovery.
                        continue

                    was_safety_holding = False

                # ==============================
                # PATROL
                # ==============================

                self.state.set_motion(
                    "PATROLLING"
                )

                completed = (
                    self.robot.forward(
                        cycles=1,
                        interrupt_events=
                            interrupt_events
                    )
                )

                if not completed:

                    # Do NOT return to neutral here.
                    #
                    # The reason movement stopped may
                    # be a person appearing.
                    self.robot.halt()

                    continue

        except Exception as error:

            print(
                "[MOTION] ERROR:",
                error
            )

            self.state.set_error(error)

            self.robot.halt()

        finally:

            self.state.set_motion(
                "STOPPED"
            )

            self.robot.halt()

            print(
                "[MOTION] Worker stopped"
            )
