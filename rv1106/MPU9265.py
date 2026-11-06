#!/usr/bin/env python3
"""
MPU-6500 IMU ("MPU-92/65" module) over SPI on RV1106 (Luckfox Pico), standard library only.
Reads accel + gyro + temp whenever the caller asks; no filtering, only unit scaling and the
mounting rotation IMU_TO_BODY (chip axes -> robot body axes, set below for this robot).
The caller sets the rate (e.g. the policy loop calls read_ang_proj() each cycle);
--loop HZ is only a test loop.

Wiring (module pin -> RV1106):
    SCL -> pin 49 GPIO1_C1 (SPI0 CLK)
    SDA -> pin 50 GPIO1_C2 (SPI0 MOSI)
    AD0 -> pin 51 GPIO1_C3 (SPI0 MISO)
    NCS -> pin 48 GPIO1_C0 (SPI0 CS0, driven by the SPI controller)

Terminal:
    python3 MPU9265.py                                    # read one sample, print it, exit
    python3 MPU9265.py --loop 100                         # test: read 100 Hz, print every sample
    python3 MPU9265.py --loop 100 --every 10              # read 100 Hz, print 10 lines per second
    python3 MPU9265.py --loop 100 --every_ang_proj 10     # same, angular velocity + projected gravity
    python3 MPU9265.py --ping                             # only read WHO_AM_I and exit

Import:
    from MPU9265 import MPU9265

    with MPU9265() as imu:
        s = imu.read()
        s["accel_g"], s["gyro_dps"], s["temp_c"]     # body axes
        ang_vel, proj_gravity = imu.read_ang_proj()  # body axes
    MPU9265(to_body=((1, 0, 0), (0, 1, 0), (0, 0, 1)))   # chip axes, e.g. to check the mounting
"""
import argparse
import ctypes
import fcntl
import math
import os
import struct
import time

DEFAULT_SPI = "/dev/spidev0.0"
SPI_HZ = 1000000                # 1 MHz: max for writing config registers
SPI_MODE = 0                    # MPU-6500 latches on the rising edge: mode 0 or 3 (ESP32 driver uses 0)

# ============================== SPI =================================

# linux/spi/spidev.h
SPI_IOC_WR_MODE = 0x40016B01
SPI_IOC_WR_BITS_PER_WORD = 0x40016B03
SPI_IOC_WR_MAX_SPEED_HZ = 0x40046B04
SPI_IOC_MESSAGE_1 = 0x40206B00


class _SpiTransfer(ctypes.Structure):
    _fields_ = [("tx_buf", ctypes.c_uint64),
                ("rx_buf", ctypes.c_uint64),
                ("len", ctypes.c_uint32),
                ("speed_hz", ctypes.c_uint32),
                ("delay_usecs", ctypes.c_uint16),
                ("bits_per_word", ctypes.c_uint8),
                ("cs_change", ctypes.c_uint8),
                ("tx_nbits", ctypes.c_uint8),
                ("rx_nbits", ctypes.c_uint8),
                ("word_delay_usecs", ctypes.c_uint8),
                ("pad", ctypes.c_uint8)]


class SPI:
    """Full-duplex transfers on /dev/spidevB.C, CS driven by the SPI controller."""

    def __init__(self, device=DEFAULT_SPI, speed_hz=SPI_HZ, mode=SPI_MODE):
        self.device = device
        self.speed_hz = speed_hz
        self.mode = mode
        self.fd = None

    def open(self):
        if self.fd is not None:
            return
        self.fd = os.open(self.device, os.O_RDWR)
        try:
            fcntl.ioctl(self.fd, SPI_IOC_WR_MODE, struct.pack("B", self.mode))
            fcntl.ioctl(self.fd, SPI_IOC_WR_BITS_PER_WORD, struct.pack("B", 8))
            fcntl.ioctl(self.fd, SPI_IOC_WR_MAX_SPEED_HZ, struct.pack("I", self.speed_hz))
        except Exception:
            self.close()
            raise

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    def transfer(self, data):
        """NSS goes low, data is clocked out while the same number of bytes is read in, NSS goes high."""
        n = len(data)
        tx = ctypes.create_string_buffer(bytes(data), n)
        rx = ctypes.create_string_buffer(n)
        xfer = _SpiTransfer(tx_buf=ctypes.addressof(tx), rx_buf=ctypes.addressof(rx),
                            len=n, speed_hz=self.speed_hz, bits_per_word=8)
        fcntl.ioctl(self.fd, SPI_IOC_MESSAGE_1, xfer)
        return rx.raw

# ============================ registers =============================

SMPLRT_DIV = 0x19               # sample rate divider: output rate = 1 kHz / (1 + value)
CONFIG = 0x1A                   # gyro + temp low-pass filter (DLPF_CFG, bits 2:0)
GYRO_CONFIG = 0x1B              # gyro full-scale range (bits 4:3)
ACCEL_CONFIG = 0x1C             # accel full-scale range (bits 4:3)
ACCEL_CONFIG2 = 0x1D            # accel low-pass filter (A_DLPF_CFG, bits 2:0)
ACCEL_XOUT_H = 0x3B             # 14 bytes: ax ay az temp gx gy gz, big-endian int16
USER_CTRL = 0x6A                # bit 4 I2C_IF_DIS = 1: I2C off, SPI only
PWR_MGMT_1 = 0x6B               # bit 7 reset, bit 6 sleep, bits 2:0 clock source (1 = gyro PLL)
PWR_MGMT_2 = 0x6C               # 1 bit per axis to turn it off; 0 = all 6 axes on
WHO_AM_I = 0x75                 # chip ID, read-only
READ = 0x80                     # set bit 7 of the register address to read

CHIPS = {0x70: "MPU-6500", 0x71: "MPU-9250", 0x73: "MPU-9255"}     # WHO_AM_I value -> chip

# range -> value for bits 4:3 of ACCEL_CONFIG / GYRO_CONFIG; resolution = range / 32768 per count
ACCEL_RANGES_G = {2: 0, 4: 1, 8: 2, 16: 3}              # +-g
GYRO_RANGES_DPS = {250: 0, 500: 1, 1000: 2, 2000: 3}    # +-deg/s

# chip's own low-pass (hardware, before the samples are read)
DLPF_CFG = 3                    # gyro 41 Hz bandwidth, ~5.9 ms delay, 1 kHz internal rate
A_DLPF_CFG = 3                  # accel 44.8 Hz bandwidth, ~4.9 ms delay
SAMPLE_DIV = 0                  # 1 kHz / (1 + 0) = 1 kHz output: a read at any rate gets a sample <= 1 ms old

# IMU mounting: rotation from the chip's axes to the robot body axes, applied in read()
# to accel and gyro (so read_ang_proj() and the CLI are in body axes too; read_raw() is not):
#   v_body = IMU_TO_BODY · v_chip
# body axes: x forward, y left, z up. Row i says which chip axis becomes body axis i:
#   row 0 = body x, row 1 = body y, row 2 = body z
# Entries are 0 / +1 / -1, one non-zero per row and per column, and the determinant must
# be +1 (a real rotation). Determinant -1 is a mirror no mounting can produce, and it
# flips the gyro signs; e.g. on the diagonal, use an even number of -1, never just one.
#
#   chip X/Y/Z arrows match the body (forward, left, up):
#       ((1, 0, 0), (0, 1, 0), (0, 0, 1))
#   chip turned 180 deg about x (upside down, X arrow forward, Y arrow pointing right):
#       ((1, 0, 0), (0, -1, 0), (0, 0, -1))        <- this robot
#   chip turned 180 deg about y (upside down, X arrow pointing back):
#       ((-1, 0, 0), (0, 1, 0), (0, 0, -1))
#   chip turned 180 deg about z (still face up, X arrow pointing back):
#       ((-1, 0, 0), (0, -1, 0), (0, 0, 1))
#
# check with the robot standing upright (python3 MPU9265.py --loop 100 --every_ang_proj 5):
# proj_gravity about (0, 0, -1); nose down -> gx goes toward +1;
# turning left (CCW seen from above) -> wz > 0
IMU_TO_BODY = ((1, 0, 0),       # chip turned 180 deg about x
               (0, -1, 0),
               (0, 0, -1))


def check_rotation(m):
    """Raise ValueError unless m is a 3x3 axis rotation (0/+-1 entries, determinant +1)."""
    if len(m) != 3 or any(len(row) != 3 or any(v not in (-1, 0, 1) for v in row) for row in m):
        raise ValueError("IMU_TO_BODY must be 3x3 with entries 0, 1 or -1: %r" % (m,))
    det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
           - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
           + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    if det != 1:
        raise ValueError("IMU_TO_BODY determinant is %d, must be +1 (a rotation, not a mirror)" % det)

# ============================== IMU =================================


class MPU9265:
    def __init__(self, spi=DEFAULT_SPI, accel_g=4, gyro_dps=1000, speed_hz=SPI_HZ, mode=SPI_MODE,
                 to_body=IMU_TO_BODY):
        if accel_g not in ACCEL_RANGES_G:
            raise ValueError("accel_g must be one of %s" % sorted(ACCEL_RANGES_G))
        if gyro_dps not in GYRO_RANGES_DPS:
            raise ValueError("gyro_dps must be one of %s" % sorted(GYRO_RANGES_DPS))
        check_rotation(to_body)
        self.to_body = to_body                      # chip axes -> body axes, see IMU_TO_BODY
        self.spi = SPI(spi, speed_hz, mode)
        self.accel_g = accel_g
        self.gyro_dps = gyro_dps
        self.accel_scale = accel_g / 32768.0        # g per LSB
        self.gyro_scale = gyro_dps / 32768.0        # deg/s per LSB
        self.chip = None

    # ---- open / close ----------------------------------------------------

    def open(self):
        self.spi.open()
        try:
            who = self.ping()
            if who not in CHIPS:
                raise RuntimeError("ping failed on %s: WHO_AM_I=0x%02x%s" % (
                    self.spi.device, who, " (no answer, check wiring)" if who in (0x00, 0xFF) else ""))
            self.chip = CHIPS[who]
            self._init()
        except Exception:
            self.spi.close()
            raise

    def close(self):
        self.spi.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    # ---- registers -------------------------------------------------------

    def _write(self, reg, value):
        self.spi.transfer(bytes([reg & 0x7F, value]))

    def _read(self, reg, size):
        return self.spi.transfer(bytes([reg | READ]) + bytes(size))[1:]

    def ping(self):
        """Read WHO_AM_I (0x70 MPU-6500, 0x71 MPU-9250). Needs only the SPI open."""
        return self._read(WHO_AM_I, 1)[0]

    def _init(self):
        w = self._write
        w(PWR_MGMT_1, 0x80)                         # reset
        time.sleep(0.1)
        w(USER_CTRL, 0x10)                          # I2C interface off (SPI only)
        w(PWR_MGMT_1, 0x01)                         # wake, clock from gyro PLL
        w(PWR_MGMT_2, 0x00)                         # all accel + gyro axes on
        w(SMPLRT_DIV, SAMPLE_DIV)
        w(CONFIG, DLPF_CFG)
        w(GYRO_CONFIG, GYRO_RANGES_DPS[self.gyro_dps] << 3)
        w(ACCEL_CONFIG, ACCEL_RANGES_G[self.accel_g] << 3)
        w(ACCEL_CONFIG2, A_DLPF_CFG)
        time.sleep(0.05)                            # gyro start-up

    # ---- read ------------------------------------------------------------

    def read_raw(self):
        """returns (ax, ay, az, temp, gx, gy, gz) as raw int16 counts, chip axes (not rotated)"""
        return struct.unpack(">7h", self._read(ACCEL_XOUT_H, 14))

    def _body(self, v):
        """chip-axes vector -> body axes (self.to_body)"""
        return tuple(r[0] * v[0] + r[1] * v[1] + r[2] * v[2] for r in self.to_body)

    def read(self):
        """returns {"accel_g": (x, y, z), "gyro_dps": (x, y, z), "temp_c": float}, body axes"""
        ax, ay, az, t, gx, gy, gz = self.read_raw()
        a, g = self.accel_scale, self.gyro_scale
        return {
            "accel_g": self._body((ax * a, ay * a, az * a)),
            "gyro_dps": self._body((gx * g, gy * g, gz * g)),
            "temp_c": t / 333.87 + 21.0,
        }

    def read_ang_proj(self):
        """
        returns (ang_vel, proj_gravity), both (x, y, z) in body axes (IMU_TO_BODY applied):
          ang_vel:      angular velocity in rad/s
          proj_gravity: unit vector of gravity's direction, (0, 0, -1) when the robot is upright
        proj_gravity comes from the accelerometer alone (no filter), so it is only right
        while the IMU is not accelerating; shaking or impacts tilt it.
        """
        s = self.read()
        ax, ay, az = s["accel_g"]
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm > 1e-6:
            # at rest the accelerometer reads +1 g pointing up; gravity points the other way.
            # Dividing by the length makes it a unit vector, so each component is in [-1, 1];
            # the clamp only guards against float rounding.
            proj_gravity = tuple(max(-1.0, min(1.0, -v / norm)) for v in (ax, ay, az))
        else:
            proj_gravity = (0.0, 0.0, -1.0)         # free fall: no direction to measure
        ang_vel = tuple(math.radians(v) for v in s["gyro_dps"])
        return ang_vel, proj_gravity

# =============================== CLI ================================


def format_sample(s, t=None):
    line = "acc[g] %+6.3f %+6.3f %+6.3f  gyr[dps] %+8.2f %+8.2f %+8.2f  T=%.1fC" % (
        s["accel_g"] + s["gyro_dps"] + (s["temp_c"],))
    return line if t is None else "t=%8.3f  %s" % (t, line)


def format_ang_proj(ang_vel, proj_gravity, t=None):
    line = "ang_vel[rad/s] %+7.3f %+7.3f %+7.3f  proj_gravity %+6.3f %+6.3f %+6.3f" % (
        ang_vel + proj_gravity)
    return line if t is None else "t=%8.3f  %s" % (t, line)


def run(imu, rate_hz, print_hz=None, ang_proj=False, stats_every_s=1.0):
    """
    Test loop until Ctrl+C.
    rate_hz:  how often to read the IMU
    print_hz: how many lines to print per second (None = every sample)
    ang_proj: True prints angular velocity + projected gravity instead of raw accel/gyro
    """
    period = 1.0 / rate_hz
    every = 1 if not print_hz else max(1, round(rate_hz / print_hz))   # print 1 of every N reads
    start = time.monotonic()
    next_tick = start
    stats_at = start + stats_every_s
    samples = cycles = overruns = 0
    worst_s = 0.0

    while True:
        t0 = time.monotonic()
        s = imu.read_ang_proj() if ang_proj else imu.read()
        worst_s = max(worst_s, time.monotonic() - t0)
        cycles += 1
        samples += 1

        if samples % every == 0:
            print(format_ang_proj(*s, t=t0 - start) if ang_proj else format_sample(s, t=t0 - start))

        next_tick += period
        delay = next_tick - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            overruns += 1
            if delay < -period:                     # far behind: restart the schedule, don't burst
                next_tick = time.monotonic()

        now = time.monotonic()
        if now >= stats_at:
            print("--- %.1f Hz, worst read %.2f ms, %d overruns" % (
                cycles / (now - stats_at + stats_every_s), worst_s * 1000, overruns))
            cycles = overruns = 0
            worst_s = 0.0
            stats_at = now + stats_every_s


def build_parser():
    p = argparse.ArgumentParser(description="Read the MPU-6500 over SPI and print it.")
    p.add_argument("--spi", default=DEFAULT_SPI, help="spidev device (default %(default)s)")
    p.add_argument("--hz", type=int, default=SPI_HZ, help="SPI clock (default %(default)s)")
    p.add_argument("--mode", type=int, default=SPI_MODE, choices=range(4), help="SPI mode (default %(default)s)")
    p.add_argument("--loop", type=float, metavar="HZ",
                   help="test loop: read HZ times per second until Ctrl+C (off: read once and exit)")
    p.add_argument("--every", type=float, metavar="HZ",
                   help="with --loop: print HZ lines per second (default: every sample)")
    p.add_argument("--every_ang_proj", type=float, metavar="HZ",
                   help="with --loop: print angular velocity (rad/s) + projected gravity, HZ lines per second")
    p.add_argument("--accel-range", type=int, default=4, choices=sorted(ACCEL_RANGES_G), help="g")
    p.add_argument("--gyro-range", type=int, default=1000, choices=sorted(GYRO_RANGES_DPS), help="deg/s")
    p.add_argument("--ping", type=int, nargs="?", const=5, metavar="N",
                   help="only read WHO_AM_I N times (default 5) and exit")
    return p


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if (args.every or args.every_ang_proj) and not args.loop:
        parser.error("--every / --every_ang_proj need --loop HZ, e.g. --loop 100 --every_ang_proj 10")
    imu = MPU9265(args.spi, args.accel_range, args.gyro_range, args.hz, args.mode)

    if args.ping:
        with imu.spi:
            for i in range(args.ping):
                who = imu.ping()
                print("ping %d: WHO_AM_I=0x%02x  %s" % (i + 1, who, CHIPS.get(who, "no answer / unknown")))
                time.sleep(0.1)
        return

    with imu:
        print("%s on %s (mode %d, %d Hz), accel +-%d g, gyro +-%d dps, body axes IMU_TO_BODY %s" % (
            imu.chip, imu.spi.device, imu.spi.mode, imu.spi.speed_hz, imu.accel_g, imu.gyro_dps,
            imu.to_body))
        if not args.loop:
            print(format_sample(imu.read()))
            print(format_ang_proj(*imu.read_ang_proj()))
            return

        print("test loop %g Hz; Ctrl+C to stop" % args.loop)
        try:
            if args.every_ang_proj:
                run(imu, args.loop, args.every_ang_proj, ang_proj=True)
            else:
                run(imu, args.loop, args.every)
        except KeyboardInterrupt:
            print()


if __name__ == "__main__":
    main()
