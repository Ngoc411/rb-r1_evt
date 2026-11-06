#!/usr/bin/env python3
"""
Link from the RV1106 to the ESP32 over UART2 (see UART.py), per "ESP - RV" (HCR1 command table).

Frame (both directions):
    AA 55 | ID | L | DATA (L bytes) | CS,   CS = ~(ID + L + all DATA bytes) & 0xFF
    int16 little-endian, total length L + 5

    ID    name          RV -> ESP32                     ESP32 -> RV
    0x01  ping          L=0                             L=1: STATUS_FLAGS (uint8)
    0x02  torque        L=1: 0 off, 1 on (uint8)        -
    0x03  get_feedback  L=0                             L=20: 10 int16, see FEEDBACK_NAMES
    0x04  send_action   L=12: 6 int16, see ACTION_NAMES -
    0x05  fold_down     L=0                             -

    STATUS_FLAGS bit = 1: 0 ESP32 ok, 1 torque on, 2-7 motor ok (MOTOR_NAMES order, by UART:
    UART1 = right, UART2 = left). get_feedback / send_action are ordered by joint, left first.
    int16 values are raw (scaled integers); their units are set by the ESP32 firmware.

As a library:
    from ESP_link import ESPLink

    with ESPLink("/dev/ttyS2") as esp:
        flags = esp.ping()                  # STATUS_FLAGS or None
        esp.torque(True)
        fb = esp.get_feedback()             # 10 int16 or None after 20 ms
        esp.send_action([0, 0, 0, 0, 0, 0])

From the shell:
    python3 ESP_link.py ping [--raw]
    python3 ESP_link.py torque on
    python3 ESP_link.py get_feedback [-n 1] [--hz 1]
    python3 ESP_link.py send_action <left_hip> <right_hip> <left_knee> <right_knee> <left_wheel> <right_wheel>
    python3 ESP_link.py fold_down
    python3 ESP_link.py                     # interactive prompt
"""
import argparse
import shlex
import struct
import time

from UART import UART, DEFAULT_DEVICE, DEFAULT_BAUDRATE

# ============================== config ==============================

PING_TIMEOUT_S = 0.5            # how long ping waits for the reply
FEEDBACK_TIMEOUT_S = 0.02       # how long get_feedback waits for the reply
CHECK_FRAMES = False            # True: only accept replies with the right header / ID / length /
                                # checksum. False: no checks, use the bytes after AA 55 | ID | L

# commands; must match the ESP32 firmware
CMD_PING = 0x01                 # -> L=0          <- L=1 STATUS_FLAGS
CMD_TORQUE = 0x02               # -> L=1 uint8    no reply
CMD_GET_FEEDBACK = 0x03         # -> L=0          <- L=20, 10 int16
CMD_SEND_ACTION = 0x04          # -> L=12, 6 int16  no reply
CMD_FOLD_DOWN = 0x05            # -> L=0          no reply

# motor order of STATUS_FLAGS bits 2..7: by motor UART (UART1 = right leg, UART2 = left leg)
MOTOR_NAMES = ("right hip", "right knee", "right wheel", "left hip", "left knee", "left wheel")

# get_feedback reply, 10 int16, by joint (left first): 4 absolute joint angles (wheels have
# none), then 6 velocities. Units: pos counts (16384 = 1 turn = 2π rad), vel RPM.
FEEDBACK_NAMES = ("left_hip", "right_hip", "left_knee", "right_knee",
                  "left_hip_velocity", "right_hip_velocity",
                  "left_knee_velocity", "right_knee_velocity",
                  "left_wheel_velocity", "right_wheel_velocity")

# send_action, 6 int16, by joint (left first): 4 joint angle targets (hip, knee),
# then 2 wheel speed targets. Units: angles in counts, wheel speeds in RPM.
ACTION_NAMES = ("left_hip", "right_hip", "left_knee", "right_knee", "left_wheel", "right_wheel")

FEEDBACK_STRUCT = struct.Struct("<%dh" % len(FEEDBACK_NAMES))     # 20 bytes
ACTION_STRUCT = struct.Struct("<%dh" % len(ACTION_NAMES))         # 12 bytes

# ============================= protocol =============================

HEADER = b"\xAA\x55"


def checksum(body):
    return ~sum(body) & 0xFF


def build_frame(cmd, payload=b""):
    body = bytes([cmd, len(payload)]) + payload
    return HEADER + body + bytes([checksum(body)])


# frames that never change: built once
PING_FRAME = build_frame(CMD_PING)                      # AA 55 01 00 FE
FEEDBACK_FRAME = build_frame(CMD_GET_FEEDBACK)          # AA 55 03 00 FC
TORQUE_FRAMES = {True: build_frame(CMD_TORQUE, b"\x01"),     # AA 55 02 01 01 FB
                 False: build_frame(CMD_TORQUE, b"\x00")}    # AA 55 02 01 00 FC
FOLD_DOWN_FRAME = build_frame(CMD_FOLD_DOWN)            # AA 55 05 00 FA


def find_frame(buf, cmd, size):
    """Look for AA 55 | cmd | size | payload | CS in buf. Returns (payload, bytes
    used) or (None, bytes that can be dropped). Bad or foreign frames are skipped."""
    frame_len = 2 + 2 + size + 1
    i = 0
    while True:
        i = buf.find(HEADER, i)
        if i < 0:
            return None, max(0, len(buf) - 1)          # keep a trailing AA
        if len(buf) - i < frame_len:
            return None, i                              # wait for the rest
        body = bytes(buf[i + 2:i + 4 + size])
        if body[0] == cmd and body[1] == size and buf[i + 4 + size] == checksum(body):
            return body[2:], i + frame_len
        i += 1                                          # not ours / corrupt: keep looking


def int16(value, name):
    value = int(round(value))
    if not -32768 <= value <= 32767:
        raise ValueError("%s = %d out of int16 range" % (name, value))
    return value


def decode_status(flags):
    """STATUS_FLAGS -> {"esp_ok": bool, "torque": bool, "motors_ok": [6 bools, MOTOR_NAMES order]}"""
    return {"esp_ok": bool(flags & 0x01),
            "torque": bool(flags & 0x02),
            "motors_ok": [bool(flags & (1 << (2 + i))) for i in range(len(MOTOR_NAMES))]}

# ============================= ESP link =============================


class ESPLink:
    """verbose: print every frame sent and received."""

    def __init__(self, device=DEFAULT_DEVICE, baudrate=DEFAULT_BAUDRATE, verbose=False):
        self.uart = UART(device, baudrate)
        self.verbose = verbose

    # ---- open / close ----

    def open(self):
        self.uart.open()

    def close(self):
        self.uart.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    # ---- low level ----

    def write(self, frame):
        """Queue a frame for sending and return at once (no drain: the kernel sends it
        while we already wait for the reply)."""
        self.uart.send(frame, drain=False)
        if self.verbose:
            print("  tx [%d] %s" % (len(frame), frame.hex(" ").upper()))

    def read_frame(self, cmd, size, timeout):
        """Wait for the reply to cmd (size data bytes); returns the data or None after timeout.
        CHECK_FRAMES = True: only a frame with AA 55, this cmd, this size and a good checksum.
        CHECK_FRAMES = False: no checks, the size bytes after the first AA 55 | ID | L
        (or from byte 0 if there is no AA 55)."""
        frame_len = 2 + 2 + size + 1
        deadline = time.monotonic() + timeout
        buf = bytearray()
        payload = None
        while True:
            if CHECK_FRAMES:
                payload, used = find_frame(buf, cmd, size)
                if payload is not None:
                    break
                del buf[:used]                          # junk before a possible frame
                need = frame_len - len(buf)
            else:
                i = buf.find(HEADER)
                start = i + 4 if i >= 0 else 0
                need = max(i, 0) + frame_len - len(buf)
                if need <= 0:
                    payload = bytes(buf[start:start + size])
                    break
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            # one call that waits until all missing bytes are in (or timeout)
            buf += self.uart.read(need, timeout=remaining)

        if self.verbose:
            self._show_rx(buf, cmd, size, payload)
        return payload

    def _show_rx(self, buf, cmd, size, payload):
        print("  rx [%d] %s" % (len(buf), bytes(buf).hex(" ").upper() or "(nothing)"))
        if CHECK_FRAMES or payload is None:
            return
        i = buf.find(HEADER)
        if i < 0:
            print("    no AA 55 header: data taken from byte 0")
        else:                                           # report only, nothing is rejected
            body = bytes(buf[i + 2:i + 4 + size])
            print("    not checked: ID %02X (expect %02X), L %d (expect %d), CS %02X (expect %02X)"
                  % (body[0], cmd, body[1], size, buf[i + 4 + size], checksum(body)))

    def request(self, frame, cmd, reply_size, timeout):
        """Send a request frame, return the reply data or None."""
        self.uart.flush_input()                         # drop late replies / noise
        self.write(frame)
        return self.read_frame(cmd, reply_size, timeout)

    # ---- commands ----

    def ping(self, timeout=PING_TIMEOUT_S):
        """0x01: returns STATUS_FLAGS (uint8, see decode_status), or None if no reply."""
        payload = self.request(PING_FRAME, CMD_PING, 1, timeout)
        return None if payload is None else payload[0]

    def torque(self, on):
        """0x02: torque on (True) / off (False) for all motors. No reply; check with ping()."""
        self.write(TORQUE_FRAMES[bool(on)])

    def get_feedback(self, timeout=FEEDBACK_TIMEOUT_S):
        """0x03: returns the 10 raw int16 in FEEDBACK_NAMES order
        (pos = [0:4] left/right hip, left/right knee; vel = [4:10] left/right hip, knee, wheel),
        or None if no reply within timeout."""
        payload = self.request(FEEDBACK_FRAME, CMD_GET_FEEDBACK, FEEDBACK_STRUCT.size, timeout)
        return None if payload is None else list(FEEDBACK_STRUCT.unpack(payload))

    def send_action(self, values):
        """0x04: 6 int16 in ACTION_NAMES order (left/right hip, left/right knee, left/right wheel).
        Blind send, no reply."""
        if len(values) != len(ACTION_NAMES):
            raise ValueError("need %d values, got %d" % (len(ACTION_NAMES), len(values)))
        try:
            payload = ACTION_STRUCT.pack(*values)       # fails if not int16
        except struct.error:
            payload = ACTION_STRUCT.pack(*[int16(v, n) for v, n in zip(values, ACTION_NAMES)])
        self.write(build_frame(CMD_SEND_ACTION, payload))

    def fold_down(self):
        """0x05: fold down, no data. Blind send, no reply."""
        self.write(FOLD_DOWN_FRAME)

# ============================= terminal =============================


def show_status(flags):
    s = decode_status(flags)
    print("   ESP32   %s" % ("ok" if s["esp_ok"] else "FAULT"))
    print("   torque  %s" % ("on" if s["torque"] else "off"))
    for name, ok in zip(MOTOR_NAMES, s["motors_ok"]):
        print("   %-12s %s" % (name, "ok" if ok else "FAULT / no reply"))


def cmd_ping(esp, a):
    if a.raw:
        # raw debug: send ping, then dump every byte received for PING_TIMEOUT_S, no frame checks
        esp.uart.flush_input()
        esp.uart.send(PING_FRAME, drain=False)
        print("-> ping tx [%d] %s" % (len(PING_FRAME), PING_FRAME.hex(" ").upper()))
        data = esp.uart.read(4096, timeout=PING_TIMEOUT_S)
        print("-> ping rx [%d] %s" % (len(data), data.hex(" ").upper() if data else "nothing"))
        return
    start = time.monotonic()
    flags = esp.ping()
    ms = (time.monotonic() - start) * 1000
    if flags is None:
        print("-> ping: no reply within %d ms" % (PING_TIMEOUT_S * 1000))
        return
    print("-> ping (%.1f ms): STATUS_FLAGS 0x%02X" % (ms, flags))
    show_status(flags)


def cmd_torque(esp, a):
    esp.torque(a.state == "on")
    print("-> torque %s sent (no reply; check with ping)" % a.state)


def cmd_get_feedback(esp, a):
    for n in range(a.n):
        if n:
            time.sleep(1.0 / a.hz)
        start = time.monotonic()
        fb = esp.get_feedback()
        ms = (time.monotonic() - start) * 1000
        if fb is None:
            print("-> get_feedback: no valid reply within %d ms" % (FEEDBACK_TIMEOUT_S * 1000))
            continue
        print("-> get_feedback (%.1f ms) raw: %s" % (ms, " ".join(str(v) for v in fb)))
        if a.n == 1:
            for name, v in zip(FEEDBACK_NAMES, fb):
                print("   %-21s %6d" % (name, v))


def cmd_send_action(esp, a):
    values = [getattr(a, name) for name in ACTION_NAMES]
    esp.send_action(values)
    print("-> send_action: %s" % "  ".join("%s=%d" % nv for nv in zip(ACTION_NAMES, values)))


def cmd_fold_down(esp, _args):
    esp.fold_down()
    print("-> fold_down sent (no reply)")


def build_parser():
    parser = argparse.ArgumentParser(
        prog="ESP_link.py",
        description="Send commands to the ESP32 over UART. With no command, opens a prompt.",
        epilog="values are raw int16; units are set by the ESP32 firmware")
    parser.add_argument("--device", default=DEFAULT_DEVICE, help="default %(default)s")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUDRATE, help="default %(default)s")
    parser.add_argument("-q", "--quiet", action="store_true", help="don't print raw frames")
    sub = parser.add_subparsers(dest="command", metavar="command")

    p = sub.add_parser("ping", help="0x01: ESP32, torque and motor status")
    p.add_argument("--raw", action="store_true",
                   help="just dump every byte received for %g s" % PING_TIMEOUT_S)
    p.set_defaults(func=cmd_ping)

    p = sub.add_parser("torque", help="0x02: torque on|off for all motors")
    p.add_argument("state", choices=("on", "off"))
    p.set_defaults(func=cmd_torque)

    p = sub.add_parser("get_feedback", help="0x03: 4 pos + 6 vel (raw int16)")
    p.add_argument("-n", type=int, default=1, help="read N times (default %(default)s)")
    p.add_argument("--hz", type=float, default=1.0, help="reads per second when N > 1 (default %(default)s)")
    p.set_defaults(func=cmd_get_feedback)

    p = sub.add_parser("send_action", help="0x04: 6 int16, no reply")
    for name in ACTION_NAMES:           # left_hip right_hip left_knee right_knee left_wheel right_wheel
        p.add_argument(name, type=int)
    p.set_defaults(func=cmd_send_action)

    p = sub.add_parser("fold_down", help="0x05: fold down, no data, no reply")
    p.set_defaults(func=cmd_fold_down)

    return parser


def run(esp, args):
    try:
        args.func(esp, args)
    except ValueError as e:
        print("  error: %s" % e)


def interactive(esp, parser):
    try:
        import readline  # noqa: F401  gives arrow-key editing/history for input()
    except ImportError:
        pass

    parser.print_help()
    print("\ntype a command (e.g. ping), 'help', or 'q' to quit")
    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not line:
            continue
        if line in ("q", "quit", "exit"):
            return
        if line == "help":
            parser.print_help()
            continue
        try:
            args = parser.parse_args(shlex.split(line))
        except SystemExit:              # argparse already printed the error / help
            continue
        if args.command is None:
            print("  error: no command; type help")
            continue
        run(esp, args)


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)

    with ESPLink(args.device, args.baud, verbose=not args.quiet) as esp:
        if args.command is None:
            print("Opened %s @ %d baud, 8N1" % (args.device, args.baud))
            interactive(esp, parser)
        else:
            run(esp, args)


if __name__ == "__main__":
    main()
