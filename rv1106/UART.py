#!/usr/bin/env python3
"""
UART helper for RV1106 (Luckfox Pico), standard library only.

UART2_TX_M1 -> GPIO1_B2 (pin 42)
UART2_RX_M1 -> GPIO1_B3 (pin 43)

Example:
    from UART import UART

    with UART("/dev/ttyS2", 921600) as uart:
        uart.send(b"\xAA\x55")
        data = uart.read(13, timeout=0.1)
"""
import fcntl
import os
import select
import struct
import termios
import time

DEFAULT_DEVICE = "/dev/ttyS2"
DEFAULT_BAUDRATE = 921600


class UART:
    def __init__(self, device=DEFAULT_DEVICE, baudrate=DEFAULT_BAUDRATE):
        self.device = device
        self.baudrate = baudrate
        self.fd = None

    # ---- open / close ----------------------------------------------------

    def open(self):
        if self.fd is not None:
            return
        self.fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            self._configure()
        except Exception:
            os.close(self.fd)
            self.fd = None
            raise
        self.flush()

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    @property
    def is_open(self):
        return self.fd is not None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    def _configure(self):
        try:
            speed = getattr(termios, "B%d" % self.baudrate)
        except AttributeError:
            raise ValueError("unsupported baudrate: %d" % self.baudrate)

        iflag, oflag, cflag, lflag, _, _, cc = termios.tcgetattr(self.fd)

        # Raw mode, 8N1, no flow control
        iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP
                   | termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON
                   | termios.IXOFF | termios.IXANY)
        oflag &= ~termios.OPOST
        lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG
                   | termios.IEXTEN)
        cflag &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.CRTSCTS)
        cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0

        termios.tcsetattr(self.fd, termios.TCSANOW,
                          [iflag, oflag, cflag, lflag, speed, speed, cc])

    # ---- send ------------------------------------------------------------

    def send(self, data, drain=True):
        """Write all bytes. If drain, block until they are shifted out."""
        self._check_open()
        view = memoryview(bytes(data))
        while view:
            select.select([], [self.fd], [])
            n = os.write(self.fd, view)
            view = view[n:]
        if drain:
            termios.tcdrain(self.fd)

    # ---- read ------------------------------------------------------------

    def available(self):
        """Number of bytes waiting in the RX buffer."""
        self._check_open()
        buf = fcntl.ioctl(self.fd, termios.FIONREAD, b"\x00\x00\x00\x00")
        return struct.unpack("I", buf)[0]

    def read(self, size, timeout=None):
        """
        Read up to `size` bytes. Returns early with fewer bytes on timeout.
        timeout=None blocks until `size` bytes arrive, 0 returns what is
        already buffered.
        """
        self._check_open()
        deadline = None if timeout is None else time.monotonic() + timeout
        data = bytearray()
        while len(data) < size:
            remaining = None if deadline is None else max(0.0, deadline - time.monotonic())
            ready, _, _ = select.select([self.fd], [], [], remaining)
            if not ready:
                break
            chunk = os.read(self.fd, size - len(data))
            if chunk:
                data += chunk
        return bytes(data)

    def read_all(self):
        """Read everything currently in the RX buffer without waiting."""
        n = self.available()
        return self.read(n, timeout=0) if n else b""

    def read_until(self, terminator=b"\n", max_size=4096, timeout=None):
        """Read until `terminator` is seen, `max_size` reached, or timeout."""
        self._check_open()
        deadline = None if timeout is None else time.monotonic() + timeout
        data = bytearray()
        while len(data) < max_size and not data.endswith(terminator):
            remaining = None if deadline is None else max(0.0, deadline - time.monotonic())
            chunk = self.read(1, timeout=remaining)
            if not chunk:
                break
            data += chunk
        return bytes(data)

    # ---- buffers ---------------------------------------------------------

    def flush(self):
        """Discard both RX and TX buffers."""
        self._check_open()
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def flush_input(self):
        self._check_open()
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def flush_output(self):
        self._check_open()
        termios.tcflush(self.fd, termios.TCOFLUSH)

    def _check_open(self):
        if self.fd is None:
            raise IOError("UART %s is not open" % self.device)
