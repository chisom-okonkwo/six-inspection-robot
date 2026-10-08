import math
import time
import xml.etree.ElementTree as ET


class ArcAutoPosition:

    def __init__(self, file_path):

        self.file_path = file_path

        tree = ET.parse(
            file_path
        )

        self.root = tree.getroot()

        self.ports = []
        self.frames_by_guid = {}
        self.frames_by_title = {}
        self.actions = {}

        self._load_servo_ports()
        self._load_frames()
        self._load_actions()

    # ==================================================
    # HELPERS
    # ==================================================

    @staticmethod
    def _text(
        element,
        name,
        default=None
    ):

        child = element.find(
            name
        )

        if (
            child is None
            or child.text is None
        ):

            return default

        return child.text

    @staticmethod
    def _integer(
        element,
        name,
        default=0
    ):

        value = (
            ArcAutoPosition._text(
                element,
                name
            )
        )

        if value is None:
            return default

        return int(
            value
        )

    @staticmethod
    def _boolean(
        element,
        name,
        default=False
    ):

        value = (
            ArcAutoPosition._text(
                element,
                name
            )
        )

        if value is None:
            return default

        return (
            value.lower()
            == "true"
        )

    @staticmethod
    def _port_number(
        port_name
    ):

        # "D15" -> 15

        if not port_name.startswith(
            "D"
        ):

            raise ValueError(
                "Only digital servo ports "
                f"are supported: {port_name}"
            )

        return int(
            port_name[1:]
        )

    # ==================================================
    # SERVO DESCRIPTORS
    # ==================================================

    def _load_servo_ports(self):

        descriptors = (
            self.root.findall(
                ".//ServoDescriptors/"
                "ServoDescriptor"
            )
        )

        for descriptor in descriptors:

            port_name = (
                descriptor.findtext(
                    "Port"
                )
            )

            self.ports.append(
                self._port_number(
                    port_name
                )
            )

        if not self.ports:

            raise RuntimeError(
                "No servo descriptors found "
                "in AutoPosition file"
            )

        print(
            "[ARC GAIT] Servo ports:",
            self.ports
        )

    # ==================================================
    # FRAMES
    # ==================================================

    def _load_frames(self):

        frames = (
            self.root.findall(
                ".//AutoPositionFrame"
            )
        )

        for frame in frames:

            title = (
                frame.findtext(
                    "Title"
                )
            )

            guid = (
                frame.findtext(
                    "GUID"
                )
            )

            position_elements = (
                frame.findall(
                    "./Positions/int"
                )
            )

            values = [
                int(element.text)
                for element
                in position_elements
            ]

            if (
                len(values)
                != len(self.ports)
            ):

                raise RuntimeError(
                    f"Frame '{title}' has "
                    f"{len(values)} positions "
                    f"but export defines "
                    f"{len(self.ports)} servos"
                )

            positions = dict(
                zip(
                    self.ports,
                    values
                )
            )

            frame_data = {
                "title":
                    title,

                "guid":
                    guid,

                "positions":
                    positions
            }

            self.frames_by_guid[
                guid
            ] = frame_data

            self.frames_by_title[
                title
            ] = frame_data

        print(
            "[ARC GAIT] Loaded "
            f"{len(self.frames_by_guid)} "
            "frames"
        )

    # ==================================================
    # ACTIONS
    # ==================================================

    def _load_actions(self):

        actions = (
            self.root.findall(
                ".//AutoPositionAction"
            )
        )

        for action in actions:

            title = (
                action.findtext(
                    "Title"
                )
            )

            action_type = (
                action.findtext(
                    "ActionType"
                )
            )

            repeats = (
                self._boolean(
                    action,
                    "Repeats"
                )
            )

            sequence = []

            action_frames = (
                action.findall(
                    "./Frames/"
                    "AutoPositionActionFrame"
                )
            )

            for action_frame in (
                action_frames
            ):

                frame_guid = (
                    action_frame.findtext(
                        "FrameGUID"
                    )
                )

                item = {

                    "frame_guid":
                        frame_guid,

                    "delay":
                        self._integer(
                            action_frame,
                            "Delay"
                        ),

                    "steps":
                        self._integer(
                            action_frame,
                            "Steps",
                            1
                        ),

                    "servo_speed":
                        self._integer(
                            action_frame,
                            "ServoSpeed",
                            -1
                        ),

                    "servo_velocity":
                        self._integer(
                            action_frame,
                            "ServoVelocity",
                            -1
                        ),

                    "servo_acceleration":
                        self._integer(
                            action_frame,
                            "ServoAcceleration",
                            -1
                        ),

                    "use_variable_speed":
                        self._boolean(
                            action_frame,
                            "UseVariableSpeed"
                        ),

                    "fast_delay":
                        self._integer(
                            action_frame,
                            "FastDelay"
                        ),

                    "fast_steps":
                        self._integer(
                            action_frame,
                            "FastSteps",
                            1
                        ),

                    "ramp_steps":
                        self._integer(
                            action_frame,
                            "RampSteps",
                            0
                        )
                }

                if frame_guid == "PAUSE":

                    item[
                        "frame_title"
                    ] = "PAUSE"

                else:

                    frame_data = (
                        self.frames_by_guid.get(
                            frame_guid
                        )
                    )

                    if frame_data is None:

                        raise RuntimeError(
                            "Action "
                            f"'{title}' references "
                            "unknown frame GUID "
                            f"{frame_guid}"
                        )

                    item[
                        "frame_title"
                    ] = (
                        frame_data[
                            "title"
                        ]
                    )

                sequence.append(
                    item
                )

            self.actions[
                title
            ] = {

                "title":
                    title,

                "action_type":
                    action_type,

                "repeats":
                    repeats,

                "sequence":
                    sequence
            }

        print(
            "[ARC GAIT] Loaded "
            f"{len(self.actions)} "
            "actions"
        )

    # ==================================================
    # PUBLIC LOOKUP
    # ==================================================

    def get_frame(
        self,
        guid
    ):

        return (
            self.frames_by_guid[
                guid
            ]
        )

    def get_action(
        self,
        title
    ):

        if title not in self.actions:

            raise KeyError(
                "ARC action not found: "
                f"{title}"
            )

        return (
            self.actions[
                title
            ]
        )


# ======================================================
# ARC GAIT PLAYBACK ENGINE
# ======================================================

class ArcGaitPlayer:

    def __init__(
        self,
        ezb,
        auto_position_file,
        movement_speed=255
    ):

        self.ezb = ezb

        self.project = (
            ArcAutoPosition(
                auto_position_file
            )
        )

        self.movement_speed = (
            self._clamp_speed(
                movement_speed
            )
        )

        # ARC's stock STAND frame is the safest
        # initial software reference for Six.

        stand = (
            self.project.frames_by_title.get(
                "STAND"
            )
        )

        if stand is None:

            raise RuntimeError(
                "AutoPosition export does "
                "not contain STAND frame"
            )

        self.current_positions = (
            stand["positions"].copy()
        )

    # ==================================================
    # EVENTS
    # ==================================================

    @staticmethod
    def _normalize_events(
        interrupt_events
    ):

        if interrupt_events is None:

            return []

        if hasattr(
            interrupt_events,
            "is_set"
        ):

            return [
                interrupt_events
            ]

        return list(
            interrupt_events
        )

    @staticmethod
    def _interrupted(
        events
    ):

        return any(
            event.is_set()
            for event in events
        )

    def _interruptible_sleep(
        self,
        seconds,
        events
    ):

        if seconds <= 0:

            return True

        if not events:

            time.sleep(
                seconds
            )

            return True

        end_time = (
            time.perf_counter()
            + seconds
        )

        while (
            time.perf_counter()
            < end_time
        ):

            if self._interrupted(
                events
            ):

                return False

            remaining = (
                end_time
                - time.perf_counter()
            )

            time.sleep(
                min(
                    0.01,
                    max(
                        0,
                        remaining
                    )
                )
            )

        return True

    # ==================================================
    # SPEED
    # ==================================================

    @staticmethod
    def _clamp_speed(
        value
    ):

        return max(
            0,
            min(
                255,
                int(value)
            )
        )

    def set_movement_speed(
        self,
        value
    ):

        self.movement_speed = (
            self._clamp_speed(
                value
            )
        )

    def _get_transition_timing(
        self,
        action_frame
    ):
        """
        ARC exports two transition parameter sets
        for variable-speed movement frames:

            normal:
                Delay / Steps

            fastest:
                FastDelay / FastSteps

        At movement speed 255 we use the exported
        fast values directly.

        At movement speed 0 we use the exported
        normal values directly.

        Intermediate interpolation is an approximation
        because ARC's exact internal mapping between
        those endpoints is not documented publicly.
        """

        normal_delay = (
            action_frame[
                "delay"
            ]
        )

        normal_steps = (
            action_frame[
                "steps"
            ]
        )

        if not action_frame[
            "use_variable_speed"
        ]:

            return (
                normal_delay,
                normal_steps
            )

        fast_delay = (
            action_frame[
                "fast_delay"
            ]
        )

        fast_steps = (
            action_frame[
                "fast_steps"
            ]
        )

        # Exact export endpoint:
        if self.movement_speed >= 255:

            return (
                fast_delay,
                fast_steps
            )

        # Exact export endpoint:
        if self.movement_speed <= 0:

            return (
                normal_delay,
                normal_steps
            )

        # Approximation only for intermediate
        # movement speeds.
        ratio = (
            self.movement_speed
            / 255.0
        )

        delay = round(
            normal_delay
            + (
                fast_delay
                - normal_delay
            )
            * ratio
        )

        steps = round(
            normal_steps
            + (
                fast_steps
                - normal_steps
            )
            * ratio
        )

        return (
            max(1, delay),
            max(1, steps)
        )

    # ==================================================
    # ARC TRANSITION ALGORITHM
    # ==================================================

    def _transition_to(
        self,
        target_positions,
        delay_ms,
        step_size,
        interrupt_events=None
    ):
        """
        Reproduce ARC's documented Auto Position
        Steps/Delay behavior.

        1. Find largest servo travel distance.
        2. Divide that distance by Steps to determine
           number of updates.
        3. Scale every other servo proportionally.
        4. Send all servo positions together.
        5. Wait Delay milliseconds.
        """

        events = (
            self._normalize_events(
                interrupt_events
            )
        )

        active_targets = {}

        for (
            port,
            target
        ) in target_positions.items():

            # ARC value -1 = skip servo.
            if target == -1:

                continue

            # 0 releases PWM in ARC/EZ-B.
            #
            # None of the exported walking frames
            # use zero, so fail loudly rather than
            # unexpectedly releasing a leg.
            if target == 0:

                raise RuntimeError(
                    "Movement frame attempted "
                    f"to release D{port}"
                )

            active_targets[
                port
            ] = target

        if not active_targets:

            return True

        start_positions = {}

        differences = {}

        for (
            port,
            target
        ) in active_targets.items():

            start = (
                self.current_positions.get(
                    port,
                    target
                )
            )

            start_positions[
                port
            ] = start

            differences[
                port
            ] = (
                target - start
            )

        largest_difference = max(
            abs(value)
            for value
            in differences.values()
        )

        step_size = max(
            1,
            int(step_size)
        )

        # Even if already in position, send one
        # target packet to keep our hardware/cache
        # synchronized.
        if largest_difference == 0:

            self.ezb.set_servo_positions(
                active_targets
            )

            self.current_positions.update(
                active_targets
            )

            return (
                self._interruptible_sleep(
                    delay_ms / 1000.0,
                    events
                )
            )

        increments = max(
            1,
            math.ceil(
                largest_difference
                / step_size
            )
        )

        for index in range(
            1,
            increments + 1
        ):

            if self._interrupted(
                events
            ):

                return False

            ratio = (
                index
                / increments
            )

            packet = {}

            for (
                port,
                target
            ) in active_targets.items():

                start = (
                    start_positions[
                        port
                    ]
                )

                difference = (
                    differences[
                        port
                    ]
                )

                if index == increments:

                    # Guarantee exact final frame.
                    position = target

                else:

                    position = round(
                        start
                        + difference
                        * ratio
                    )

                packet[
                    port
                ] = max(
                    1,
                    min(
                        180,
                        position
                    )
                )

            self.ezb.set_servo_positions(
                packet
            )

            self.current_positions.update(
                packet
            )

            if not self._interruptible_sleep(
                delay_ms / 1000.0,
                events
            ):

                return False

        return True

    # ==================================================
    # ACTION PLAYBACK
    # ==================================================

    def play_action(
        self,
        action_name,
        cycles=1,
        interrupt_events=None
    ):

        action = (
            self.project.get_action(
                action_name
            )
        )

        cycles = max(
            1,
            int(cycles)
        )

        for _ in range(
            cycles
        ):

            for action_frame in (
                action["sequence"]
            ):

                if self._interrupted(
                    self._normalize_events(
                        interrupt_events
                    )
                ):

                    return False

                # ARC action pause item.
                if (
                    action_frame[
                        "frame_guid"
                    ]
                    == "PAUSE"
                ):

                    if not self._interruptible_sleep(
                        action_frame[
                            "delay"
                        ] / 1000.0,
                        self._normalize_events(
                            interrupt_events
                        )
                    ):

                        return False

                    continue

                frame = (
                    self.project.get_frame(
                        action_frame[
                            "frame_guid"
                        ]
                    )
                )

                (
                    delay,
                    steps
                ) = (
                    self._get_transition_timing(
                        action_frame
                    )
                )

                completed = (
                    self._transition_to(
                        target_positions=
                            frame["positions"],

                        delay_ms=
                            delay,

                        step_size=
                            steps,

                        interrupt_events=
                            interrupt_events
                    )
                )

                if not completed:

                    return False

        return True

    # ==================================================
    # DEBUG
    # ==================================================

    def describe_action(
        self,
        action_name
    ):

        action = (
            self.project.get_action(
                action_name
            )
        )

        print()
        print(
            "================================"
        )

        print(
            f" ARC ACTION: {action_name}"
        )

        print(
            "================================"
        )

        print(
            "Repeats in ARC:",
            action["repeats"]
        )

        for index, item in enumerate(
            action["sequence"],
            start=1
        ):

            (
                delay,
                steps
            ) = (
                self._get_transition_timing(
                    item
                )
            )

            print(
                f"{index}. "
                f"{item['frame_title']} "
                f"| Delay={delay}ms "
                f"| Steps={steps} "
                f"| Ramp="
                f"{item['ramp_steps']}"
            )

        print()