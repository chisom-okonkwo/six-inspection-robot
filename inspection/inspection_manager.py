import threading


class InspectionManager(
    threading.Thread
):

    def __init__(
        self,
        state,
        logger,
        shutdown_event,
        poll_interval=0.10
    ):

        super().__init__(
            name="InspectionManager",
            daemon=True
        )

        self.state = state
        self.logger = logger

        self.shutdown_event = (
            shutdown_event
        )

        self.poll_interval = (
            poll_interval
        )

    # ==================================================

    def _log(
        self,
        event_type,
        snapshot,
        include_frame=False,
        details=None
    ):

        frame = None

        if include_frame:

            frame = (
                self.state.get_frame()
            )

        self.logger.log_incident(
            event_type=
                event_type,

            state=
                snapshot,

            frame=
                frame,

            details=
                details
        )

    # ==================================================

    def run(self):

        print(
            "[INSPECTION] "
            "Manager online"
        )

        # Previous states allow us to log
        # state transitions rather than writing
        # the same detection every frame.

        previous_person = False
        previous_obstacle = False

        previous_motion = None
        previous_error = None

        # Record session start.

        snapshot = (
            self.state.snapshot()
        )

        self._log(
            "session_started",
            snapshot
        )

        try:

            while not self.shutdown_event.is_set():

                snapshot = (
                    self.state.snapshot()
                )

                person = snapshot[
                    "person_detected"
                ]

                obstacle = snapshot.get(
                    "obstacle_detected",
                    False
                )

                motion = snapshot[
                    "motion_state"
                ]

                error = snapshot.get(
                    "last_error"
                )

                # ==================================
                # PERSON ENTERS
                # ==================================

                if (
                    person
                    and
                    not previous_person
                ):

                    self._log(
                        "person_detected",
                        snapshot,
                        include_frame=True
                    )

                # ==================================
                # PERSON CLEARS
                # ==================================

                if (
                    not person
                    and
                    previous_person
                ):

                    self._log(
                        "person_cleared",
                        snapshot
                    )

                # ==================================
                # OBSTACLE APPEARS
                # ==================================

                if (
                    obstacle
                    and
                    not previous_obstacle
                ):

                    self._log(
                        "obstacle_detected",
                        snapshot,
                        include_frame=True
                    )

                # ==================================
                # OBSTACLE CLEARS
                # ==================================

                if (
                    not obstacle
                    and
                    previous_obstacle
                ):

                    self._log(
                        "obstacle_cleared",
                        snapshot
                    )

                # ==================================
                # IMPORTANT MOTION STATES
                # ==================================

                if (
                    motion
                    != previous_motion
                ):

                    if motion == (
                        "OBSTACLE_AVOID"
                    ):

                        self._log(
                            "obstacle_avoidance",
                            snapshot
                        )

                    elif motion == "BLOCKED":

                        self._log(
                            "robot_blocked",
                            snapshot,
                            include_frame=True
                        )

                    elif motion == (
                        "PERSON_HOLD"
                    ):

                        self._log(
                            "safety_hold",
                            snapshot
                        )

                # ==================================
                # ERROR
                # ==================================

                if (
                    error is not None
                    and
                    error != previous_error
                ):

                    self._log(
                        "system_error",
                        snapshot,
                        details={
                            "error":
                                str(error)
                        }
                    )

                # ==================================
                # SAVE CURRENT AS PREVIOUS
                # ==================================

                previous_person = (
                    person
                )

                previous_obstacle = (
                    obstacle
                )

                previous_motion = (
                    motion
                )

                previous_error = (
                    error
                )

                self.shutdown_event.wait(
                    self.poll_interval
                )

        finally:

            snapshot = (
                self.state.snapshot()
            )

            self._log(
                "session_ended",
                snapshot
            )

            print(
                "[INSPECTION] "
                "Manager stopped"
            )