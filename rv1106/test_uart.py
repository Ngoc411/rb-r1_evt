#!/usr/bin/env python3
"""
UART loopback test: send bytes out of TX, read them back on RX, compare, print everything.

Wiring: jumper TX -> RX on the RV1106 and disconnect the ESP32:
    pin 42 GPIO1_B2 (UART2_TX)  ->  pin 43 GPIO1_B3 (UART2_RX)

    python3 test_uart.py                     # 5 rounds on /dev/ttyS2, 921600 baud
    python3 test_uart.py -n 20 --size 64     # 20 rounds, 64 random bytes each
    python3 test_uart.py --device /dev/ttyS3 --baud 115200

Rounds: 1 = an ESP_link ping frame (AA 55 01 00 FE), 2 = every byte 00..FF,
then random bytes. Random data never contains AA 55, so if the ESP32 is still
connected by mistake it cannot see a command frame.
"""
import argparse
import os
import random
import time

from UART import UART, DEFAULT_DEVICE, DEFAULT_BAUDRATE


def hexdump(data, indent="    "):
    if not data:
        return indent + "(nothing)"
    lines = []
    for i in range(0, len(data), 16):
        chunk = data[i:i + 16]
        lines.append("%s%04X  %s" % (indent, i, chunk.hex(" ").upper()))
    return "\n".join(lines)


def no_header(data):
    """replace any AA 55 pair so the data can never form an ESP_link frame"""
    out = bytearray(data)
    for i in range(1, len(out)):
        if out[i - 1] == 0xAA and out[i] == 0x55:
            out[i] = 0x54
    return bytes(out)


def port_users(device):
    """other processes that have the device open (e.g. rb_runtime on /dev/ttyS2)"""
    target = os.path.realpath(device)
    users = []
    if not os.path.isdir("/proc"):
        return users
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            fds = os.listdir("/proc/%s/fd" % pid)
            if any(os.path.realpath("/proc/%s/fd/%s" % (pid, fd)) == target for fd in fds):
                with open("/proc/%s/cmdline" % pid, "rb") as f:
                    users.append("%s %s" % (pid, f.read().replace(b"\0", b" ").decode(errors="replace").strip()))
        except OSError:
            continue
    return users


def patterns(rounds, size):
    yield "ping frame", bytes.fromhex("AA550100FE")
    yield "bytes 00..FF", no_header(bytes(range(256)))
    for i in range(rounds - 2):
        yield "random %d" % (i + 1), no_header(bytes(random.getrandbits(8) for _ in range(size)))


def main():
    p = argparse.ArgumentParser(description="UART loopback test (jumper TX -> RX).")
    p.add_argument("--device", default=DEFAULT_DEVICE, help="default %(default)s")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUDRATE, help="default %(default)s")
    p.add_argument("-n", type=int, default=5, help="rounds (default %(default)s)")
    p.add_argument("--size", type=int, default=32, help="bytes per random round (default %(default)s)")
    args = p.parse_args()

    users = port_users(args.device)
    if users:
        print("WARNING: %s is also open by:" % args.device)
        for u in users:
            print("    " + u)
        print("  it will steal the bytes; stop it first (rb_runtime: /etc/init.d/S96rbhybrid stop)\n")

    passed = 0
    with UART(args.device, args.baud) as uart:
        print("opened %s @ %d baud, 8N1; expect TX jumpered to RX\n" % (args.device, args.baud))
        for n, (name, tx) in enumerate(patterns(max(args.n, 2), args.size), 1):
            uart.flush_input()
            # wire time is 10 bits per byte; allow plenty more
            timeout = 0.05 + 3 * len(tx) * 10 / args.baud
            start = time.monotonic()
            uart.send(tx)
            rx = uart.read(len(tx), timeout=timeout)
            ms = (time.monotonic() - start) * 1000
            extra = uart.read_all()             # anything beyond what was sent

            ok = rx == tx and not extra
            passed += ok
            print("round %d: %s, %d bytes, %.2f ms -> %s" % (
                n, name, len(tx), ms, "OK" if ok else "FAIL"))
            print("  tx [%d]:\n%s" % (len(tx), hexdump(tx)))
            print("  rx [%d]:\n%s" % (len(rx), hexdump(rx)))
            if extra:
                print("  extra rx [%d]:\n%s" % (len(extra), hexdump(extra)))
            if not ok and rx:
                diff = next((i for i in range(min(len(tx), len(rx))) if tx[i] != rx[i]), None)
                if diff is not None:
                    print("  first difference at byte %d: sent %02X, got %02X" % (diff, tx[diff], rx[diff]))
                elif len(rx) < len(tx):
                    print("  only %d of %d bytes came back" % (len(rx), len(tx)))
            print()

    total = max(args.n, 2)
    print("%d / %d rounds OK" % (passed, total))
    if passed == 0:
        print("nothing came back right: check the TX -> RX jumper, the device, and that no other "
              "program has the port open")


if __name__ == "__main__":
    main()
