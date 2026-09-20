"""
==============================================================================
SHARED PROTOCOL MODULE
==============================================================================
This file MUST be identical on BOTH the laptop and the Pi.

WHAT IS A PROTOCOL?
-------------------
A protocol is just an agreement about how to send data over a wire so both
sides can decode it the same way. Without one, you send bytes, the other
side has no idea where a message starts or ends.

OUR PROTOCOL (designed for this project)
----------------------------------------
Every message is a JSON object on its own line (terminated by '\n').

Example commands:
    {"cmd":"START_CALIB","expected_rows":3,"step_size":100}\n
    {"cmd":"CONTINUE"}\n

If the JSON has a "len" field with a value greater than 0, that many bytes
of binary data follow IMMEDIATELY after the newline.

Example photo:
    {"type":"CALIB_PHOTO","seq":3,"z_steps":300,"len":50432}\n
    <50432 raw JPEG bytes here>

WHY THIS DESIGN?
----------------
1. JSON for commands  -> human-readable, easy to debug, easy to extend with
                         new fields without breaking old code.
2. '\n' delimiter     -> easy to resync if a byte gets dropped: just skip to
                         the next newline.
3. Length-prefix for  -> JPEG bytes can contain anything (including newlines),
   binary payloads      so we MUST know the exact length before reading.
4. One connection,    -> Same TCP socket carries both commands and photos.
   two framings         The "len" field tells us which mode we're in.
==============================================================================
"""

import json
import socket


class MessageStream:
    """
    Wraps a TCP socket. Knows how to send and receive our protocol messages.

    Usage:
        sock = socket.socket(...)
        sock.connect(...)
        stream = MessageStream(sock)

        # Send a command:
        stream.send_message({"cmd": "START_CALIB", "expected_rows": 3})

        # Send a photo:
        stream.send_message({"type": "CALIB_PHOTO", "seq": 0},
                            binary_payload=jpeg_bytes)

        # Receive whatever the other side sends:
        msg, payload = stream.recv_message()
        # msg is a dict, payload is bytes or None
    """

    def __init__(self, sock):
        self.sock = sock
        # We buffer incoming bytes here because recv() may give us partial
        # data (e.g. half a JSON line) or extra data (e.g. the start of the
        # next message). We need to handle both.
        self._buffer = bytearray()

    # ---------- low-level read helpers ----------

    def _fill_until(self, ready):
        """
        Keep reading from the socket until the condition `ready(buffer)`
        returns True. Raises ConnectionError if the socket closes mid-read.
        """
        while not ready(self._buffer):
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("Connection closed by peer")
            self._buffer.extend(chunk)

    def _read_line(self):
        """Consume bytes up to and including the next '\n'. Return the text."""
        self._fill_until(lambda b: b'\n' in b)
        idx = self._buffer.index(b'\n')
        line = bytes(self._buffer[:idx])
        del self._buffer[: idx + 1]  # drop the line AND the newline
        return line.decode('utf-8')

    def _read_exact(self, n):
        """Consume exactly n bytes from the buffer. Return them."""
        self._fill_until(lambda b: len(b) >= n)
        data = bytes(self._buffer[:n])
        del self._buffer[:n]
        return data

    # ---------- public API ----------

    def recv_message(self):
        """
        Receive ONE message.

        Returns:
            (msg_dict, payload_bytes_or_None)
            (None, None) if the connection closed cleanly.
        """
        try:
            line = self._read_line()
        except ConnectionError:
            return None, None

        msg = json.loads(line)
        payload = None
        payload_len = msg.get('len', 0)
        if payload_len > 0:
            payload = self._read_exact(payload_len)
        return msg, payload

    def send_message(self, msg_dict, binary_payload=None):
        """
        Send ONE message.

        If `binary_payload` is given, the 'len' field is added automatically
        and the bytes are appended after the JSON line.
        """
        # Make a copy so we don't accidentally modify the caller's dict
        msg_to_send = dict(msg_dict)
        if binary_payload is not None:
            msg_to_send['len'] = len(binary_payload)

        line = (json.dumps(msg_to_send) + '\n').encode('utf-8')
        self.sock.sendall(line)
        if binary_payload is not None:
            self.sock.sendall(binary_payload)

    def close(self):
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()
