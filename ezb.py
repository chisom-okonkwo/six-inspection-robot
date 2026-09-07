import socket


class EZB:
    def __init__(self, ip="192.168.1.1", port=23):
        self.ip = ip
        self.port = port
        self.socket = None

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
        if not self.socket:
            raise RuntimeError("EZ-B is not connected")

        if not 0 <= port <= 23:
            raise ValueError("Port must be D0-D23")

        if not 0 <= position <= 180:
            raise ValueError("Servo position must be 0-180")

        command = 0xAC + port

        packet = bytes([
            command,
            position
        ])

        self.socket.sendall(packet)

    