#!/usr/bin/env python3
"""
Run the recurrent policy at RATE_HZ (100 Hz). Every cycle:

    get_feedback (10 int16) ─┐
    read_ang_proj (6 float) ─┤
    COMMANDS (5 float)      ─┼─► INPUT FILTER ─► obs[32] ─► MODEL ─► action[6] ─► OUTPUT FILTER ─► send_action (6 int16)
    last_action (6 float)   ─┘                                 │
                                    (last_action = action) ◄───┘

    python3 run_policy.py                 # default /dev/ttyS2
    python3 run_policy.py /dev/ttyS3
    python3 command.py                    # second terminal: edits COMMANDS (vx / wz / pitch / roll)

At start: ping the ESP32 (refuses to run on no reply / fault), then torque on.
Ctrl+C stops the loop: legs hold, wheels 0, then torque off.

INPUT FILTER (input_filter): sources -> obs[32], floats in the trained order and units
    feedback (10 int16): [0:4] left_hip, right_hip, left_knee, right_knee (absolute angles)
                         [4:10] left/right hip, left/right knee, left/right wheel velocities
    obs     what                         from                                   unit
    [0:3]   gyro wx wy wz                read_ang_proj()[0]                     rad/s, body axes
    [3:6]   gravity gx gy gz             read_ang_proj()[1]                     unit vector, upright (0,0,-1)
            (IMU mounting rotation: IMU_TO_BODY in MPU9265.py, applied there)
    [6:9]   vx, vy, wz                   COMMANDS vx, 0, wz                     m/s, -, rad/s
    [9:13]  left_hip  - (-0.7)           sign · feedback[0:4] · 2π/16384        counts -> rad
            right_hip - (-0.7)
            left_knee  - 0.9
            right_knee - 0.9
    [13:19] left/right hip, left/right   sign · feedback[4:10] · 2π/60          RPM -> rad/s
            knee, left/right wheel vel
    [19:25] previous actions             last_action, as the model gave it      raw (unscaled)
    [25]    pitch                        COMMANDS                               rad
    [26]    roll                         COMMANDS                               rad
    [27]    height                       HEIGHT_CMD = 0.16                      m
    [28:32] jump                         0, 0, 0, 0
MODEL (PolicyModel.step):
    obs -> (obs - mean) / divisor (models/beni512_19500_normalizer.json) -> int16 -> NPU
    NPU int16 -> action[6] floats (raw) -> last_action; GRU memory h_out -> h_in (512)
OUTPUT FILTER (output_filter): action [a0..a5] -> 6 int16 for send_action, all absolute targets
    hip target    = -0.7 + 0.25·a0 (left), a1 (right)
    crank target  = (-0.7 + 0.9) + 0.25·a2 (left), a3 (right)    the model's "knee" outputs are cranks
    knee target   = crank target - hip target                    what the ESP32 gets
    wheel target  = (0.8/0.035)·a4 (left), a5 (right)
    sent: sign · [left_hip, right_hip, left_knee, right_knee] · 16384/2π     rad -> counts
          sign · [left_wheel, right_wheel] · 60/2π                         rad/s -> RPM
    sign = JOINT_SIGN (LH RH LK RK LW RW = -1 1 1 -1 1 -1)
"""
import json
import math
import os
import sys
import time

from command import CMD_NAMES, CommandReceiver
from ESP_link import FEEDBACK_TIMEOUT_S, MOTOR_NAMES, ESPLink, decode_status
from MPU9265 import MPU9265
from rknn_model import MODEL_PATH, N_ACTIONS, N_HIDDEN, N_OBS, RKNNModel
from UART import DEFAULT_DEVICE

# ============================== config ==============================

RATE_HZ = 100                   # policy rate (the model was trained at 100 Hz)
PERIOD_S = 1.0 / RATE_HZ

HERE = os.path.dirname(os.path.abspath(__file__))
NORMALIZER_PATH = os.path.join(HERE, "models", "beni512_19500_normalizer.json")

SEND_ACTIONS = False            # True: send the model's actions to the motors. Turn on only
                                # after checking JOINT_SIGN and the printout
TORQUE_ON_START = True          # torque on before the loop, off after it (ESP32 torque command 0x02)

# ---- ESP32 data: already in policy joint order, left first (ESP_link.FEEDBACK_NAMES / ACTION_NAMES)
#   feedback pos [0:4]  left_hip, right_hip, left_knee, right_knee            absolute angle, counts
#   feedback vel [4:10] left/right hip, left/right knee, left/right wheel     velocity, RPM
#   action      [0:6]  left_hip, right_hip, left_knee, right_knee (counts), left_wheel, right_wheel (RPM)
# motor units: pos 16384 counts = 1 output-shaft turn = 2π rad; vel 1 unit = 1 RPM
COUNTS_PER_RAD = 16384 / (2 * math.pi)     # rad -> counts (2607.6); counts -> rad: divide
RPM_PER_RAD_S = 60 / (2 * math.pi)         # rad/s -> RPM (9.549); RPM -> rad/s: divide

# direction of each joint: +1 = ESP32 and policy turn the same way, -1 = opposite.
# input: policy value = sign * ESP32 value;  output: ESP32 value = sign * policy value
#              LH  RH  LK  RK  LW  RW
JOINT_SIGN = (-1,  1,  1, -1,  1, -1)

# ---- trained default pose, rad: obs [9:13] = angle - default ----
DEFAULT_HIP = -3.054
DEFAULT_KNEE = 0.9
DEFAULT_ANGLES = (DEFAULT_HIP, DEFAULT_HIP, DEFAULT_KNEE, DEFAULT_KNEE)     # left/right hip, left/right knee

# ---- startup check (init_check): before the model runs, both hips must be near DEFAULT_HIP ----
INIT_TIME = 4                   # s from program start: when the check feedback is read
INIT_HIP_TOL_DEG = 10.0         # feedback left_hip, right_hip must be within DEFAULT_HIP +- this
INIT_FEEDBACK_TRIES = 5         # get_feedback attempts in total if no reply comes
INIT_CHECK = True               # False: skip the check

# ---- output filter scales (same as training / rb_runtime) ----
LEG_ACTION_SCALE = 0.25             # rad per action unit around the default (hips, cranks)
WHEEL_ACTION_SCALE = 0.8 / 0.035    # rad/s per action unit

# ---- input filter: commands and fixed values ----
# operator command, edited live by command.py: vx (m/s), vy (always 0), wz (rad/s), pitch, roll (rad)
COMMANDS = [0.0] * len(CMD_NAMES)
HEIGHT_CMD = 0.15               # m; trained range 0.15-0.17 (0 would be outside training)
JUMP = (0.0, 0.0, 0.0, 0.0)     # active, phase, height, distance

STATS_EVERY_S = 1.0             # print loop timing + observation summary this often

# =========================== input filter ===========================


def input_filter(feedback, ang_vel, proj_gravity, commands, last_action):
    """
    feedback:      10 int16 from get_feedback (ESP_link.FEEDBACK_NAMES order):
                   [0:4] left_hip, right_hip, left_knee, right_knee         absolute angles
                   [4:10] left/right hip, left/right knee, left/right wheel velocities
    ang_vel, proj_gravity: read_ang_proj(), already in body axes (MPU9265.IMU_TO_BODY)
    commands:      COMMANDS, 5 floats vx vy wz pitch roll
    last_action:   6 floats, the model's previous output
    returns obs:   32 floats, see the table at the top
    """
    # ESP32 direction -> policy direction (JOINT_SIGN: LH RH LK RK LW RW)
    angles = [s * raw / COUNTS_PER_RAD for s, raw in zip(JOINT_SIGN[0:4], feedback[0:4])]    # counts -> rad
    velocities = [s * raw / RPM_PER_RAD_S for s, raw in zip(JOINT_SIGN, feedback[4:10])]     # RPM -> rad/s
    vx, _, wz, pitch, roll = commands

    obs = [0.0] * N_OBS
    obs[0:3] = ang_vel                                      # gyro wx wy wz, rad/s
    obs[3:6] = proj_gravity                                 # gravity gx gy gz, unit vector
    obs[6:9] = (vx, 0.0, wz)                                # vx, vy (always 0), wz
    obs[9:13] = [a - d for a, d in zip(angles, DEFAULT_ANGLES)]     # 9-12: left/right hip - (-0.7),
                                                                     #       left/right knee - 0.9
    obs[13:19] = velocities                                 # 13-18: left/right hip, knee, wheel velocity
    obs[19:25] = last_action                                # previous raw actions
    obs[25] = pitch                                         # body pitch, rad
    obs[26] = roll                                          # body roll, rad
    obs[27] = HEIGHT_CMD                                    # body height, m
    obs[28:32] = JUMP                                       # jump active, phase, height, distance
    return obs

# =============================== model ==============================


class PolicyModel:
    """
    GRU policy on the NPU: obs[32] + h_in[512] -> actions[6] + h_out[512].

    The NPU tensors are int16; step() takes and returns floats and does the
    conversion. The GRU memory is the 512-value h_in buffer: all 0 at start
    (reset), and after every step h_out is copied into it.
    last_action is the output of the previous step, obs [19:25].
    """

    def __init__(self, model_path=MODEL_PATH, normalizer_path=NORMALIZER_PATH):
        with open(normalizer_path) as f:
            n = json.load(f)
        self.mean, self.divisor = n["mean"], n["divisor"]
        if len(self.mean) != N_OBS or len(self.divisor) != N_OBS:
            raise ValueError("normalizer must have %d mean and divisor values" % N_OBS)
        self.model = RKNNModel(model_path)
        self.model.check_io()
        self.reset()

    def reset(self):
        m = self.model
        m.set_hidden([m.h_in.to_raw(0.0)] * N_HIDDEN)      # GRU memory = 0
        self.last_action = [0.0] * N_ACTIONS

    def step(self, obs):
        """obs: 32 floats -> 6 action floats (also kept in last_action)"""
        if len(obs) != N_OBS or not all(math.isfinite(v) for v in obs):
            raise ValueError("invalid observation")
        m = self.model
        # the .rknn has no normalizer inside (the .onnx does): normalize, then float -> int16
        m.set_obs([m.obs.to_raw((v - mu) / d) for v, mu, d in zip(obs, self.mean, self.divisor)])
        m.run()
        action = m.actions.to_real(m.get_outputs())         # int16 -> float
        m.feed_back_hidden()                                # h_in = h_out for the next step
        self.last_action = action
        return action

    def hidden(self):
        """current GRU memory, 512 floats (for inspection)"""
        return self.model.h_in.to_real(self.model.h_in.read())

    def close(self):
        self.model.close()

# =========================== output filter ==========================


def int16(v):
    return max(-32768, min(32767, int(round(v))))


def output_filter(action):
    """
    action:    6 floats, raw model output [a0..a5]
    returns:   6 int16 for send_action, ESP_link.ACTION_NAMES order:
               [left_hip, right_hip, left_knee, right_knee] absolute angles, counts,
               [left_wheel, right_wheel] speeds, RPM
    """
    left_hip = DEFAULT_HIP + LEG_ACTION_SCALE * action[0]                       # rad
    right_hip = DEFAULT_HIP + LEG_ACTION_SCALE * action[1]
    # the model's a2, a3 are crank targets (crank = hip + knee), around the default hip + knee
    left_crank = DEFAULT_HIP + DEFAULT_KNEE + LEG_ACTION_SCALE * action[2]      # rad
    right_crank = DEFAULT_HIP + DEFAULT_KNEE + LEG_ACTION_SCALE * action[3]
    left_knee = left_crank - left_hip                                           # knee = crank - hip
    right_knee = right_crank - right_hip
    left_wheel = WHEEL_ACTION_SCALE * action[4]                                 # rad/s
    right_wheel = WHEEL_ACTION_SCALE * action[5]
    # policy direction -> ESP32 direction (JOINT_SIGN: LH RH LK RK LW RW)
    s = JOINT_SIGN
    return ([int16(s[j] * t * COUNTS_PER_RAD) for j, t in enumerate((left_hip, right_hip, left_knee, right_knee))]
            + [int16(s[4 + j] * t * RPM_PER_RAD_S) for j, t in enumerate((left_wheel, right_wheel))])


def hold_command(feedback):
    """not sending the model: legs keep this feedback's hip and knee angles, wheels 0"""
    left_hip, right_hip, left_knee, right_knee = feedback[0:4]
    return [left_hip, right_hip, left_knee, right_knee, 0, 0]

# =============================== loop ===============================


def init_check(esp, t_start):
    """
    Startup check: wait until t_start + INIT_TIME, then send get_feedback, resending while
    there is no reply, up to INIT_FEEDBACK_TRIES times in total. Check that feedback[0]
    (left_hip) and feedback[1] (right_hip), in rad, are within DEFAULT_HIP +- INIT_HIP_TOL_DEG.
    Returns True if both are, False if they are not or no reply came in all tries.
    """
    wait = t_start + INIT_TIME - time.monotonic()
    if wait > 0:
        print("init check in %.1f s ..." % wait)
        time.sleep(wait)

    fb = None
    for attempt in range(1, INIT_FEEDBACK_TRIES + 1):
        fb = esp.get_feedback(FEEDBACK_TIMEOUT_S)
        if fb is not None:
            break
        print("init check: no feedback reply (try %d/%d)" % (attempt, INIT_FEEDBACK_TRIES))
    if fb is None:
        print("init check FAILED: no feedback reply in %d tries" % INIT_FEEDBACK_TRIES)
        return False

    tol = math.radians(INIT_HIP_TOL_DEG)
    hips = [JOINT_SIGN[j] * fb[j] / COUNTS_PER_RAD for j in (0, 1)]         # left, right, rad
    if all(abs(h - DEFAULT_HIP) <= tol for h in hips):
        print("init check OK: hips %.3f, %.3f rad (default %.3f +- %.1f deg)"
              % (hips[0], hips[1], DEFAULT_HIP, INIT_HIP_TOL_DEG))
        return True
    print("init check FAILED: hips %.3f, %.3f rad, need %.3f +- %.1f deg (%.3f .. %.3f)"
          % (hips[0], hips[1], DEFAULT_HIP, INIT_HIP_TOL_DEG, DEFAULT_HIP - tol, DEFAULT_HIP + tol))
    return False


def get_feedbacks(esp, imu):
    """Returns (feedback, ang_vel, proj_gravity); feedback is the 10 raw int16
    (4 pos + 6 vel), or None if the ESP32 did not answer within FEEDBACK_TIMEOUT_S."""
    feedback = esp.get_feedback(FEEDBACK_TIMEOUT_S)
    ang_vel, proj_gravity = imu.read_ang_proj()
    return feedback, ang_vel, proj_gravity


def send_actions(esp, values):
    """6 int16 in ESP_link.ACTION_NAMES order, blind send."""
    esp.send_action(values)


def fmt(values, digits=2):
    return "[" + " ".join("%+.*f" % (digits, v) for v in values) + "]"


def run(esp, imu, policy, receiver=None):
    state = {}
    try:
        _run(esp, imu, policy, receiver, state)
    finally:
        # sends are blind and the ESP keeps the last one: on any exit, stop the wheels
        # and hold the legs where they are
        if state.get("feedback") is not None:
            send_actions(esp, hold_command(state["feedback"]))
            print("stopped: legs hold, wheels 0")


def _run(esp, imu, policy, receiver, state):
    feedback = hold = esp_cmd = None            # last good feedback, hold command, last model command
    obs = action = None
    start = time.monotonic()
    next_tick = start
    cycles = overruns = missed_reads = steps = 0
    worst_s = worst_feedback_s = worst_model_s = 0.0
    stats_at = start + STATS_EVERY_S

    while True:
        cycle_start = time.monotonic()

        # 1. feedback: ESP32 (10 int16) + IMU; keep the last good one, first one = hold pose
        #    (do NOT remove: everything below needs it)
        fb, ang_vel, proj_gravity = get_feedbacks(esp, imu)
        if fb is None:
            missed_reads += 1                   # keep using the last good feedback
        else:
            feedback = state["feedback"] = fb
            if hold is None:
                hold = hold_command(fb)
        t_feedback = time.monotonic()

        # 2. model: input filter -> NPU -> output filter
        if feedback is not None:
            obs = input_filter(feedback, ang_vel, proj_gravity, list(COMMANDS), policy.last_action)
            print(f"final_obs: {obs}")
            action = policy.step(obs)
            esp_cmd = output_filter(action)
            steps += 1
        t_model = time.monotonic()

        # print(f"final_actions: {esp_cmd}")

        # 3. send: model output if SEND_ACTIONS, else hold pose (legs hold, wheels 0)
        # if SEND_ACTIONS and esp_cmd is not None:
        #     send_actions(esp, esp_cmd)
        # elif hold is not None:
        #     send_actions(esp, hold)

        # 4. measure timing (only for the printout; remove together with step 6)
        now = time.monotonic()
        worst_s = max(worst_s, now - cycle_start)
        worst_feedback_s = max(worst_feedback_s, t_feedback - cycle_start)
        worst_model_s = max(worst_model_s, t_model - t_feedback)
        cycles += 1

        # 5. keep RATE_HZ: sleep until the next tick (do NOT remove)
        next_tick += PERIOD_S
        delay = next_tick - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            overruns += 1
            if delay < -PERIOD_S:               # far behind: restart the schedule, don't burst
                next_tick = time.monotonic()

        # 6. printout once per STATS_EVERY_S (safe to remove, with step 4)
        now = time.monotonic()
        if now >= stats_at:
            print("%.1f Hz, worst cycle %.1f ms (feedback %.1f, model %.1f), %d overruns, %d missed reads"
                  % (cycles / (now - stats_at + STATS_EVERY_S), worst_s * 1000, worst_feedback_s * 1000,
                     worst_model_s * 1000, overruns, missed_reads))
            if obs is None:
                print("  waiting for the first ESP feedback (model not running)")
            else:
                alive = receiver is None or receiver.alive
                print("  cmd %s%s" % (" ".join("%s %+.2f" % c for c in zip(CMD_NAMES, COMMANDS)),
                                      "" if alive else "  (no command.py: all 0)"))
                print("  in : gyro %s  grav %s  angle %s  vel %s" % (
                    fmt(obs[0:3]), fmt(obs[3:6]), fmt(obs[9:13]), fmt(obs[13:19])))
                print("  out: action %s -> esp %s %s  (%d steps)" % (
                    fmt(action), esp_cmd, "SENT" if SEND_ACTIONS else "not sent, holding", steps))
            cycles = overruns = missed_reads = 0
            worst_s = worst_feedback_s = worst_model_s = 0.0
            stats_at = now + STATS_EVERY_S


def main():
    t_start = time.monotonic()                  # INIT_TIME counts from here
    device = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_DEVICE

    policy = PolicyModel()
    print("model %s, GRU memory %d, normalizer %s" % (MODEL_PATH, N_HIDDEN, NORMALIZER_PATH))
    try:
        with MPU9265() as imu, CommandReceiver(COMMANDS) as receiver, ESPLink(device) as esp:
            receiver.start()                    # command.py now edits COMMANDS
            print("IMU %s, commands on udp %d" % (imu.chip, receiver.sock.getsockname()[1]))

            flags = esp.ping()
            if flags is None:
                print("ESP32 does not answer ping on %s; not starting" % device)
                return
            status = decode_status(flags)
            faults = [n for n, ok in zip(MOTOR_NAMES, status["motors_ok"]) if not ok]
            print("ESP32 status 0x%02X: %s, torque %s, motor faults: %s" % (
                flags, "ok" if status["esp_ok"] else "FAULT", "on" if status["torque"] else "off",
                ", ".join(faults) or "none"))
            if not status["esp_ok"] or faults:
                print("not starting")
                return

            if TORQUE_ON_START:
                esp.torque(True)
                print("torque on sent")

            # startup check: at INIT_TIME, feedback (up to INIT_FEEDBACK_TRIES); hips = DEFAULT_HIP +- tol
            if INIT_CHECK and not init_check(esp, t_start):
                esp.torque(False)
                print("torque off sent; stopping")
                return

            print("running %d Hz on %s, actions %s; Ctrl+C to stop" % (
                RATE_HZ, device, "SENT" if SEND_ACTIONS else "NOT sent (legs hold, wheels 0)"))
            try:
                run(esp, imu, policy, receiver)
            except KeyboardInterrupt:
                print()
            finally:
                if TORQUE_ON_START:
                    esp.torque(False)
                    print("torque off sent")
    finally:
        policy.close()


if __name__ == "__main__":
    main()
