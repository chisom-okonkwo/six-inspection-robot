import statistics
import threading
import time
from collections import deque


class UltrasonicWorker(threading.Thread):
    """
    HC-SR04 fusion worker.

    Two independent uses of the same range sensor:

    1. OBJECT AVOIDANCE
       Camera sees a configured object (currently backpack/book)
       AND HC-SR04 says it is close enough.

    2. WALL / UNKNOWN CLOSE-SURFACE AVOIDANCE
       No configured camera object is present, but HC-SR04
       reports a very close surface. Default threshold: 15 cm.

    Notes:
    - HC-SR04 cannot identify a literal wall. It only reports
      distance. The wall_event therefore means "very close,
      unclassified surface".
    - Object and wall thresholds are deliberately separate.
    """

    def __init__(
        self,
        ezb,
        state,
        vision_obstacle_event,
        navigation_obstacle_event,
        wall_event,
        shutdown_event,
        trigger_port=20,
        echo_port=21,
        object_trigger_distance_cm=45,
        object_clear_distance_cm=55,
        wall_trigger_distance_cm=15,
        wall_clear_distance_cm=22,
        object_confirm_readings=2,
        object_clear_readings=2,
        wall_confirm_readings=2,
        wall_clear_readings=2,
        filter_size=5,
        active_poll_seconds=0.10,
        idle_poll_seconds=0.40,
        vision_hold_seconds=1.5,
        sensor_failure_fallback_seconds=2.0,
    ):
        super().__init__(name="UltrasonicWorker", daemon=True)

        self.ezb = ezb
        self.state = state

        self.vision_obstacle_event = vision_obstacle_event
        self.navigation_obstacle_event = navigation_obstacle_event
        self.wall_event = wall_event
        self.shutdown_event = shutdown_event

        self.trigger_port = int(trigger_port)
        self.echo_port = int(echo_port)

        self.object_trigger_distance_cm = float(object_trigger_distance_cm)
        self.object_clear_distance_cm = float(object_clear_distance_cm)

        self.wall_trigger_distance_cm = float(wall_trigger_distance_cm)
        self.wall_clear_distance_cm = float(wall_clear_distance_cm)

        if self.object_clear_distance_cm <= self.object_trigger_distance_cm:
            raise ValueError(
                "object_clear_distance_cm must be greater than "
                "object_trigger_distance_cm"
            )

        if self.wall_clear_distance_cm <= self.wall_trigger_distance_cm:
            raise ValueError(
                "wall_clear_distance_cm must be greater than "
                "wall_trigger_distance_cm"
            )

        self.object_confirm_readings = max(1, int(object_confirm_readings))
        self.object_clear_readings = max(1, int(object_clear_readings))
        self.wall_confirm_readings = max(1, int(wall_confirm_readings))
        self.wall_clear_readings = max(1, int(wall_clear_readings))

        self.filter_size = max(1, int(filter_size))
        self.active_poll_seconds = max(0.07, float(active_poll_seconds))
        self.idle_poll_seconds = max(
            self.active_poll_seconds,
            float(idle_poll_seconds),
        )

        self.vision_hold_seconds = max(0.0, float(vision_hold_seconds))
        self.sensor_failure_fallback_seconds = max(
            0.5,
            float(sensor_failure_fallback_seconds),
        )

    @staticmethod
    def _valid_distance(value):
        return value is not None and 2 <= value < 255

    def run(self):
        print("[ULTRASONIC] Worker online")
        print(
            f"[ULTRASONIC] TRIG=D{self.trigger_port}, "
            f"ECHO=D{self.echo_port}"
        )
        print(
            "[ULTRASONIC] Object trigger: "
            f"{self.object_trigger_distance_cm:.0f} cm"
        )
        print(
            "[ULTRASONIC] Wall/unknown-surface trigger: "
            f"{self.wall_trigger_distance_cm:.0f} cm"
        )

        self.state.set_ultrasonic_status("STARTING")

        history = deque(maxlen=self.filter_size)

        object_close_counter = 0
        object_clear_counter = 0
        wall_close_counter = 0
        wall_clear_counter = 0

        last_vision_seen = None
        last_valid_sensor_time = None

        try:
            while not self.shutdown_event.is_set():
                now = time.perf_counter()

                vision_now = self.vision_obstacle_event.is_set()

                if vision_now:
                    last_vision_seen = now

                vision_recent = (
                    vision_now
                    or (
                        last_vision_seen is not None
                        and now - last_vision_seen <= self.vision_hold_seconds
                    )
                )

                poll_interval = (
                    self.active_poll_seconds
                    if (
                        vision_recent
                        or self.navigation_obstacle_event.is_set()
                        or self.wall_event.is_set()
                    )
                    else self.idle_poll_seconds
                )

                try:
                    raw_cm = self.ezb.get_ping(
                        self.trigger_port,
                        self.echo_port,
                    )

                    sensor_valid = self._valid_distance(raw_cm)

                    if sensor_valid:
                        history.append(raw_cm)
                        filtered_cm = float(statistics.median(history))
                        last_valid_sensor_time = now
                        self.state.set_ultrasonic_status("READY")
                    else:
                        filtered_cm = (
                            float(statistics.median(history))
                            if history
                            else None
                        )
                        self.state.set_ultrasonic_status("NO_ECHO")

                except Exception as error:
                    raw_cm = None
                    sensor_valid = False
                    filtered_cm = (
                        float(statistics.median(history))
                        if history
                        else None
                    )
                    self.state.set_ultrasonic_status("ERROR")
                    print("[ULTRASONIC] Read error:", error)

                if sensor_valid and filtered_cm is not None:
                    # ------------------------------
                    # Object threshold hysteresis
                    # ------------------------------
                    if filtered_cm <= self.object_trigger_distance_cm:
                        object_close_counter += 1
                        object_clear_counter = 0
                    elif filtered_cm >= self.object_clear_distance_cm:
                        object_clear_counter += 1
                        object_close_counter = 0
                    else:
                        object_close_counter = 0
                        object_clear_counter = 0

                    # ------------------------------
                    # Wall threshold hysteresis
                    # ------------------------------
                    if filtered_cm <= self.wall_trigger_distance_cm:
                        wall_close_counter += 1
                        wall_clear_counter = 0
                    elif filtered_cm >= self.wall_clear_distance_cm:
                        wall_clear_counter += 1
                        wall_close_counter = 0
                    else:
                        wall_close_counter = 0
                        wall_clear_counter = 0

                object_range_close = (
                    object_close_counter >= self.object_confirm_readings
                )
                object_range_clear = (
                    object_clear_counter >= self.object_clear_readings
                )

                wall_range_close = (
                    wall_close_counter >= self.wall_confirm_readings
                )
                wall_range_clear = (
                    wall_clear_counter >= self.wall_clear_readings
                )

                # ==================================================
                # OBJECT: camera + range
                # ==================================================
                if vision_recent and object_range_close:
                    if not self.navigation_obstacle_event.is_set():
                        print()
                        print(
                            "[ULTRASONIC] Camera + range confirmed "
                            "navigation obstacle"
                        )
                        print(
                            f"[ULTRASONIC] Distance: {filtered_cm:.1f} cm"
                        )

                    self.navigation_obstacle_event.set()

                elif (
                    self.navigation_obstacle_event.is_set()
                    and object_range_clear
                ):
                    self.navigation_obstacle_event.clear()
                    print(
                        "[ULTRASONIC] Object range clear: "
                        f"{filtered_cm:.1f} cm"
                    )

                # ==================================================
                # WALL / UNKNOWN CLOSE SURFACE: range only
                # ==================================================
                # We only call it a wall/unknown-surface hazard when
                # no configured backpack/book candidate is active.
                wall_candidate = (
                    not vision_recent
                    and wall_range_close
                )

                if wall_candidate:
                    if not self.wall_event.is_set():
                        print()
                        print(
                            "[ULTRASONIC] Very close unclassified "
                            "surface detected"
                        )
                        print(
                            "[ULTRASONIC] Treating as wall/close-surface hazard"
                        )
                        print(
                            f"[ULTRASONIC] Distance: {filtered_cm:.1f} cm"
                        )

                    self.wall_event.set()

                elif self.wall_event.is_set() and wall_range_clear:
                    self.wall_event.clear()
                    print(
                        "[ULTRASONIC] Wall/close-surface range clear: "
                        f"{filtered_cm:.1f} cm"
                    )

                # ==================================================
                # SENSOR FAILURE FALLBACK FOR KNOWN CAMERA OBJECTS
                # ==================================================
                no_valid_sensor_for_too_long = (
                    vision_now
                    and (
                        last_valid_sensor_time is None
                        or now - last_valid_sensor_time
                        >= self.sensor_failure_fallback_seconds
                    )
                )

                if no_valid_sensor_for_too_long:
                    self.navigation_obstacle_event.set()
                    self.state.set_ultrasonic_status(
                        "DEGRADED_CAMERA_FALLBACK"
                    )

                self.state.update_ultrasonic(
                    raw_cm=raw_cm,
                    distance_cm=filtered_cm,
                    valid=sensor_valid,
                    close=(
                        filtered_cm is not None
                        and filtered_cm <= self.object_trigger_distance_cm
                    ),
                    vision_candidate=vision_recent,
                    navigation_confirmed=(
                        self.navigation_obstacle_event.is_set()
                    ),
                )

                self.shutdown_event.wait(poll_interval)

        finally:
            self.wall_event.clear()
            self.navigation_obstacle_event.clear()
            self.state.set_ultrasonic_status("STOPPED")
            print("[ULTRASONIC] Worker stopped")
