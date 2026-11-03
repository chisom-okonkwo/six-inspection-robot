import socket
import threading


class EZB:
    """
    Direct TCP interface for EZ-B v4.

    One shared lock protects both:
      - normal command writes
      - request/response transactions such as HC-SR04

    This prevents a servo packet from being interleaved
    while Python is waiting for a sensor response.
    """

    # EZ-B protocol reserves commands 228..251 for
    # HC-SR04 reads, one command value per trigger port.
    CMD_HC_SR04_BASE = 228

    def __init__(
        self,
        ip="192.168.1.1",
        port=23
    ):
        self.ip = ip
        self.port = port

        self.socket = None

        # Keep the existing name so other project code
        # remains compatible.
        self._tx_lock = threading.Lock()

    # ==================================================
    # CONNECTION
    # ==================================================

    def connect(self):

        print(
            f"[EZB] Connecting to "
            f"{self.ip}:{self.port}"
        )

        self.socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        )

        self.socket.settimeout(
            5
        )

        self.socket.connect(
            (
                self.ip,
                self.port
            )
        )

        # EZ-B protocol handshake.
        self.socket.sendall(
            bytes([0x55])
        )

        response = self._recv_exact(
            1
        )

        hardware_id = (
            response[0]
        )

        if hardware_id == 78:

            raise RuntimeError(
                "EZ-B reports another client "
                "is already connected."
            )

        print(
            "[EZB] Connected. "
            f"Hardware ID = {hardware_id}"
        )

        return hardware_id

    # --------------------------------------------------

    def disconnect(self):

        if self.socket:

            try:
                self.socket.close()

            finally:
                self.socket = None

        print(
            "[EZB] Disconnected"
        )

    # ==================================================
    # TCP RECEIVE HELPER
    # ==================================================

    def _recv_exact(
        self,
        count
    ):
        """
        TCP recv() may return fewer bytes than requested.
        Wait until exactly `count` bytes are received.
        """

        if self.socket is None:

            raise RuntimeError(
                "EZ-B is not connected"
            )

        data = bytearray()

        while len(data) < count:

            chunk = self.socket.recv(
                count - len(data)
            )

            if not chunk:

                raise ConnectionError(
                    "EZ-B connection closed "
                    "while waiting for response"
                )

            data.extend(
                chunk
            )

        return bytes(
            data
        )

    # ==================================================
    # GENERAL COMMANDS
    # ==================================================

    def get_unique_id(self):

        if self.socket is None:

            raise RuntimeError(
                "EZ-B is not connected"
            )

        # Request and response must remain atomic.
        with self._tx_lock:

            self.socket.sendall(
                bytes([0x02])
            )

            return self._recv_exact(
                12
            )

    # ==================================================
    # HC-SR04
    # ==================================================

    def get_ping(
        self,
        trigger_port,
        echo_port
    ):
        """
        Read an HC-SR04 through the EZ-B firmware.

        trigger_port:
            Digital trigger port number, 0..23.

        echo_port:
            Digital echo port number, 0..23.

        EZ-B request:
            [228 + trigger_port, echo_port]

        EZ-B response:
            one unsigned byte, distance in centimeters.

        Higher-level code should treat 0 and 255 as
        invalid/no-useful-echo readings.

        Do not poll a normal HC-SR04 faster than about
        once every 60 ms.
        """

        if self.socket is None:

            raise RuntimeError(
                "EZ-B is not connected"
            )

        if not 0 <= trigger_port <= 23:

            raise ValueError(
                "Trigger port must be D0-D23"
            )

        if not 0 <= echo_port <= 23:

            raise ValueError(
                "Echo port must be D0-D23"
            )

        command = (
            self.CMD_HC_SR04_BASE
            + int(trigger_port)
        )

        packet = bytes([
            command,
            int(echo_port)
        ])

        # Keep the entire transaction under one lock.
        with self._tx_lock:

            self.socket.sendall(
                packet
            )

            response = self._recv_exact(
                1
            )

        return int(
            response[0]
        )

    # ==================================================
    # SERVO CONTROL
    # ==================================================

    def set_servo_position(
        self,
        port,
        position
    ):
        """
        Move one EZ-B servo.

        port:     0-23
        position: 1-180 degrees
        """

        if self.socket is None:

            raise RuntimeError(
                "EZ-B is not connected"
            )

        if not 0 <= port <= 23:

            raise ValueError(
                "Servo port must be "
                "between D0 and D23"
            )

        if not 1 <= position <= 180:

            raise ValueError(
                "Servo position must be "
                "between 1 and 180"
            )

        packet = bytes([
            0xAC + port,
            int(position)
        ])

        with self._tx_lock:

            self.socket.sendall(
                packet
            )

    # --------------------------------------------------

    def set_servo_positions(
        self,
        positions
    ):
        """
        Move several servos in one TCP transmission.

        Example:
            {
                0: 90,
                1: 90,
                3: 100
            }
        """

        if self.socket is None:

            raise RuntimeError(
                "EZ-B is not connected"
            )

        packet = bytearray()

        for (
            port,
            position
        ) in positions.items():

            if not 0 <= port <= 23:

                raise ValueError(
                    f"Invalid servo port D{port}"
                )

            if not 1 <= position <= 180:

                raise ValueError(
                    f"Invalid position "
                    f"{position} for D{port}"
                )

            packet.extend([
                0xAC + port,
                int(position)
            ])

        with self._tx_lock:

            self.socket.sendall(
                packet
            )

    # --------------------------------------------------

    def release_all_servos(self):

        if self.socket is None:
            return

        with self._tx_lock:

            self.socket.sendall(
                bytes([0x01])
            )

        print(
            "[EZB] All servos released"
        )
