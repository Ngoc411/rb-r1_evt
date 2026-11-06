#!/usr/bin/env python3
"""
Operator commands for the policy (obs [6:9] and [25:27]), separate from run_policy.py.

run_policy.py keeps the commands in its own global array COMMANDS (5 floats, order
CMD_NAMES) and the model reads it locally. This file edits that array:
  - you run this file in a second terminal and type new values; it sends them over UDP
  - inside run_policy.py, CommandReceiver (a background thread) writes each one into COMMANDS
(two programs cannot share a Python variable directly, so the values travel over a socket)

If no packet arrives for TIMEOUT_S (sender closed or crashed), the receiver sets
COMMANDS to all zeros, so the robot is told to stand still. The sender resends 10
times per second to keep the command alive.

Terminal (second SSH session):
    python3 command.py
    > vx 0.5          forward 0.5 m/s
    > wz 1.0          turn 1.0 rad/s
    > pitch 0.05      lean body 0.05 rad
    > vx 0.5 wz 1.0   several on one line, sent together in one packet
    > stop            everything back to 0
    > show            print the current command
    > quit            (or Ctrl+C) the policy falls back to 0 after TIMEOUT_S

Import (run_policy.py):
    from command import CommandReceiver
    COMMANDS = [0.0] * 5                # vx vy wz pitch roll
    rx = CommandReceiver(COMMANDS)
    rx.start()                          # COMMANDS now follows command.py
"""
import json
import math
import socket
import threading
import time

HOST = "127.0.0.1"              # this board only; use "0.0.0.0" in the receiver to allow a PC to send
PORT = 9870
TIMEOUT_S = 0.5                 # no packet for this long -> all commands 0
RESEND_HZ = 10

# name -> (min, max); vy is fixed at 0 (not trained)
RANGES = {
    "vx": (-2.0, 2.0),          # m/s
    "vy": (0.0, 0.0),           # m/s
    "wz": (-math.pi, math.pi),  # rad/s
    "pitch": (-0.1, 0.1),       # rad, about +-5.7 deg
    "roll": (-0.1, 0.1),        # rad
}
CMD_NAMES = tuple(RANGES)       # order of the COMMANDS array: vx vy wz pitch roll


def zero_command():
    return {name: 0.0 for name in RANGES}


def clamp_command(cmd):
    """Unknown names are dropped, missing ones are 0, values are clamped to RANGES."""
    out = zero_command()
    for name, (low, high) in RANGES.items():
        v = float(cmd.get(name, 0.0))
        if not math.isfinite(v):
            v = 0.0
        out[name] = max(low, min(high, v))
    return out

# ============================ receiver ==============================


class CommandReceiver:
    """
    Background thread that writes every command from command.py into store,
    a list of 5 floats in CMD_NAMES order (e.g. run_policy.COMMANDS). Sets it
    to all zeros when nothing arrives for timeout_s.
    """

    def __init__(self, store, host=HOST, port=PORT, timeout_s=TIMEOUT_S):
        if len(store) != len(CMD_NAMES):
            raise ValueError("store must have %d values: %s" % (len(CMD_NAMES), " ".join(CMD_NAMES)))
        self.store = store
        self.timeout_s = timeout_s
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.settimeout(0.05)      # wake up regularly to check the timeout
        self.last_rx = None             # time of the last good packet
        self.running = False
        self.thread = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            try:
                data = self.sock.recv(1024)
                cmd = clamp_command(json.loads(data))
            except socket.timeout:
                cmd = None
            except (ValueError, TypeError, AttributeError):
                continue                # bad packet: ignore
            except OSError:
                return                  # socket closed
            if cmd is not None:
                self.store[:] = [cmd[name] for name in CMD_NAMES]   # one atomic list update
                self.last_rx = time.monotonic()
            elif not self.alive and any(self.store):
                self.store[:] = [0.0] * len(CMD_NAMES)

    @property
    def alive(self):
        return self.last_rx is not None and time.monotonic() - self.last_rx <= self.timeout_s

    def close(self):
        self.running = False
        self.sock.close()
        if self.thread is not None:
            self.thread.join(timeout=1)
        self.store[:] = [0.0] * len(CMD_NAMES)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

# ============================== sender ==============================


class CommandSender:
    """Keeps sending the current command at RESEND_HZ from a background thread."""

    def __init__(self, host=HOST, port=PORT):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd = zero_command()
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._resend, daemon=True)
        self.thread.start()

    def set(self, **values):
        with self.lock:
            self.cmd = clamp_command(dict(self.cmd, **values))
            self._send()
            return dict(self.cmd)

    def stop(self):
        with self.lock:
            self.cmd = zero_command()
            self._send()

    def _send(self):
        self.sock.sendto(json.dumps(self.cmd).encode(), self.addr)

    def _resend(self):
        while self.running:
            with self.lock:
                self._send()
            time.sleep(1.0 / RESEND_HZ)

    def close(self):
        self.stop()
        self.running = False
        self.sock.close()


def show(cmd):
    return "vx %+.2f m/s  wz %+.2f rad/s  pitch %+.3f rad  roll %+.3f rad" % (
        cmd["vx"], cmd["wz"], cmd["pitch"], cmd["roll"])


USAGE = "use: vx/wz/pitch/roll <value> (several on one line: vx 0.5 wz 1.0), stop, show, quit"


def parse_pairs(parts):
    """["vx", "0.5", "wz", "1.0"] -> {"vx": 0.5, "wz": 1.0}; prints why and returns None if any pair is bad."""
    if len(parts) % 2:
        print(USAGE)
        return None
    values = {}
    for name, text in zip(parts[0::2], parts[1::2]):
        name = name.lower()
        if name not in RANGES:
            print("unknown command: %s\n%s" % (name, USAGE))
            return None
        try:
            values[name] = float(text)
        except ValueError:
            print("not a number: %s" % text)
            return None
    return values


def main():
    sender = CommandSender()
    print("sending to %s:%d; %s" % (sender.addr + (USAGE[5:],)))
    try:
        while True:
            parts = input("> ").split()
            if not parts:
                continue
            word = parts[0].lower()
            if word in ("quit", "exit", "q"):
                break
            if word == "stop":
                sender.stop()
            elif word == "show":
                pass
            else:
                values = parse_pairs(parts)
                if values is None:
                    continue
                sender.set(**values)            # all pairs of the line in one packet
            print(show(sender.cmd))
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        sender.close()                  # sends zeros once; the policy also times out to 0


if __name__ == "__main__":
    main()
