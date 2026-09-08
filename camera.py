import socket
import struct

import cv2
import numpy as np


class EZBCamera:
    """
    Direct TCP camera client for EZ-B v4.

    Default camera connection:
        192.168.1.1:24

    The EZ-B streams JPEG frames beginning with:
        EZIMG
    """

    CAMERA_MARKER = b"EZIMG"

    def __init__(
        self,
        ip="192.168.1.1",
        port=24,
        timeout=5.0
    ):
        self.ip = ip
        self.port = port
        self.timeout = timeout

        self.socket = None
        self.buffer = bytearray()

    # --------------------------------------------------
    # CONNECTION
    # --------------------------------------------------

    def connect(self):

        if self.socket is not None:
            return

        print(
            f"[CAMERA] Connecting to "
            f"{self.ip}:{self.port}"
        )

        sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        )

        sock.settimeout(self.timeout)

        # Reduce latency for video.
        sock.setsockopt(
            socket.IPPROTO_TCP,
            socket.TCP_NODELAY,
            1
        )

        sock.connect(
            (self.ip, self.port)
        )

        self.socket = sock

        print("[CAMERA] Connected")

    def disconnect(self):

        if self.socket is not None:

            try:
                self.socket.shutdown(
                    socket.SHUT_RDWR
                )
            except OSError:
                pass

            self.socket.close()
            self.socket = None

        self.buffer.clear()

        print("[CAMERA] Disconnected")

    # --------------------------------------------------
    # NETWORK READING
    # --------------------------------------------------

    def _receive_more_data(self):

        if self.socket is None:
            raise RuntimeError(
                "Camera is not connected"
            )

        chunk = self.socket.recv(65536)

        if not chunk:
            raise ConnectionError(
                "Camera connection closed"
            )

        self.buffer.extend(chunk)

    # --------------------------------------------------
    # FRAME PARSER
    # --------------------------------------------------

    def read_jpeg(self):
        """
        Wait for one complete EZIMG JPEG packet
        and return the JPEG bytes.

        The Synthiam SDK sample currently parses
        the image length as little-endian UInt32.

        We also recognize the older/documented
        UInt16 representation by checking where
        the JPEG SOI marker begins.
        """

        marker = self.CAMERA_MARKER

        while True:

            # ------------------------------------------
            # Find EZIMG
            # ------------------------------------------

            marker_index = self.buffer.find(
                marker
            )

            if marker_index == -1:

                # Keep enough bytes in case EZIMG
                # is split across TCP packets.
                if len(self.buffer) > len(marker):

                    del self.buffer[
                        :-len(marker) + 1
                    ]

                self._receive_more_data()
                continue

            # Throw away garbage before EZIMG.
            if marker_index > 0:

                del self.buffer[
                    :marker_index
                ]

            # Need enough data to inspect headers.
            if len(self.buffer) < 11:

                self._receive_more_data()
                continue

            # Layout possibilities:
            #
            # UInt16:
            # EZIMG + 2-byte length + FF D8
            #
            # UInt32:
            # EZIMG + 4-byte length + FF D8

            two_byte_jpeg_start = (
                self.buffer[7:9] == b"\xff\xd8"
            )

            four_byte_jpeg_start = (
                self.buffer[9:11] == b"\xff\xd8"
            )

            if four_byte_jpeg_start:

                length_size = 4

                image_size = struct.unpack(
                    "<I",
                    self.buffer[5:9]
                )[0]

            elif two_byte_jpeg_start:

                length_size = 2

                image_size = struct.unpack(
                    "<H",
                    self.buffer[5:7]
                )[0]

            else:
                # This probably wasn't the start of
                # a valid frame. Shift by one byte and
                # resynchronize.
                del self.buffer[0]

                continue

            # Sanity check
            if image_size <= 0:
                del self.buffer[0]
                continue

            if image_size > 2_000_000:
                raise RuntimeError(
                    "Camera reported an unreasonable "
                    f"JPEG size: {image_size}"
                )

            header_size = (
                len(marker)
                + length_size
            )

            packet_size = (
                header_size
                + image_size
            )

            # Wait until complete JPEG arrives.
            while len(self.buffer) < packet_size:
                self._receive_more_data()

            jpeg_data = bytes(
                self.buffer[
                    header_size:packet_size
                ]
            )

            # Remove packet from network buffer.
            del self.buffer[:packet_size]

            return jpeg_data

    # --------------------------------------------------
    # OPENCV FRAME
    # --------------------------------------------------

    def read_frame(self):

        jpeg_data = self.read_jpeg()

        numpy_data = np.frombuffer(
            jpeg_data,
            dtype=np.uint8
        )

        frame = cv2.imdecode(
            numpy_data,
            cv2.IMREAD_COLOR
        )

        if frame is None:
            raise RuntimeError(
                "OpenCV could not decode "
                "EZ-B JPEG frame"
            )

        return frame
