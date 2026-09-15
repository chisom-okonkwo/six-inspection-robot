import socket
import threading

class EZB:
    def __init__(self, ip="192.168.1.1", port=23):
        self.ip = ip
        self.port = port
        self.socket = None
        self._tx_lock = threading.Lock()

    def connect(self):
        print(f"[EZB] Connecting to {self.ip}:{self.port}")

        self.socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        )

        self.socket.settimeout(5)

        self.socket.connect((self.ip, self.port))

        # EZ-B protocol handshake
        self.socket.sendall(bytes([0x55]))

        response = self.socket.recv(1)

        if not response:
            raise RuntimeError("No handshake response received")

        hardware_id = response[0]

        if hardware_id == 78:
            raise RuntimeError(
                "EZ-B reports another client is already connected."
            )

        print(f"[EZB] Connected. Hardware ID = {hardware_id}")

        return hardware_id

    def disconnect(self):
        if self.socket:
            self.socket.close()
            self.socket = None

        print("[EZB] Disconnected")
        
    
    def get_unique_id(self):
        if not self.socket:
            raise RuntimeError("EZ-B is not connected")

        self.socket.sendall(bytes([0x02]))

        data = self.socket.recv(12)

        return data

    def set_servo_position(self, port, position):
        """
        Move one EZ-B servo.

        port:     0-23
        position: 1-180 degrees
        """

        if self.socket is None:
            raise RuntimeError("EZ-B is not connected")

        if not 0 <= port <= 23:
            raise ValueError("Servo port must be between D0 and D23")

        if not 1 <= position <= 180:
            raise ValueError("Servo position must be between 1 and 180")

        packet = bytes([
            0xAC + port,
            position
        ])

        with self._tx_lock:
            self.socket.sendall(packet)


    def set_servo_positions(self, positions):
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
            raise RuntimeError("EZ-B is not connected")

        packet = bytearray()

        for port, position in positions.items():

            if not 0 <= port <= 23:
                raise ValueError(f"Invalid servo port D{port}")

            if not 1 <= position <= 180:
                raise ValueError(
                    f"Invalid position {position} for D{port}"
                )

            packet.extend([
                0xAC + port,
                int(position)
            ])

        with self._tx_lock:
            self.socket.sendall(packet)


    def release_all_servos(self):

        if self.socket is None:
            return

        with self._tx_lock:
            self.socket.sendall(bytes([0x01]))

        print("[EZB] All servos released")