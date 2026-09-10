import threading
import time


class SafetySupervisor(threading.Thread):

    def __init__(
        self,
        state,
        person_event,
        safety_stop_event,
        shutdown_event,
        clear_delay=2.0
    ):

        super().__init__(
            name="SafetySupervisor",
            daemon=True
        )

        self.state = state

        self.person_event = person_event
        self.safety_stop_event = (
            safety_stop_event
        )

        self.shutdown_event = shutdown_event

        self.clear_delay = clear_delay

    # --------------------------------------------------

    def run(self):

        print(
            "[SAFETY] Supervisor online"
        )

        clear_since = None

        while not self.shutdown_event.is_set():

            # ======================================
            # PERSON PRESENT
            # ======================================

            if self.person_event.is_set():

                clear_since = None

                if not self.safety_stop_event.is_set():

                    self.safety_stop_event.set()

                    self.state.set_safety(
                        "PERSON_DETECTED"
                    )

                    print()
                    print(
                        "!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                    )
                    print(
                        "[SAFETY] PERSON DETECTED"
                    )
                    print(
                        "[SAFETY] MOTION PREEMPTED"
                    )
                    print(
                        "!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                    )
                    print()

            # ======================================
            # AREA APPEARS CLEAR
            # ======================================

            else:

                if self.safety_stop_event.is_set():

                    if clear_since is None:

                        clear_since = (
                            time.perf_counter()
                        )

                        self.state.set_safety(
                            "WAITING_FOR_CLEAR"
                        )

                    elapsed = (
                        time.perf_counter()
                        - clear_since
                    )

                    if elapsed >= self.clear_delay:

                        self.safety_stop_event.clear()

                        self.state.set_safety(
                            "CLEAR"
                        )

                        clear_since = None

                        print(
                            "[SAFETY] Area confirmed clear"
                        )

                else:

                    clear_since = None

            self.shutdown_event.wait(
                0.02
            )

        print(
            "[SAFETY] Supervisor stopped"
        )
