#pragma once

#include <cstdint>

/* The USB link to uart_sender.py. It receives a command, decodes it, calls
 * the one function that does it (motor_utils:: for joint commands, motor::
 * for single-motor ones), and replies.
 *
 * PC -> ESP frame: AA 55 CMD LEN PAYLOAD... CS, CS = ~(CMD + LEN + PAYLOAD...).
 * BUS is the UART number (1 or 2), ID 1, 2 or 3 (the joint table in
 * config.hpp); 16-bit values are little-endian. ESP -> PC replies are text
 * lines (see reply()).
 * set_cmd_mit, set_cmd_pos, test_motor, test_motor_dual: action mode only,
 * and only for a motor whose torque is on. set_gains, set_gains_limit,
 * set_mode: config mode only. The others: either mode. */

namespace pc {
inline constexpr uint8_t kSetSimMode = 0x02;      // ACTION: 1 action, 0 config (all torque off)
inline constexpr uint8_t kSetTorqueAll = 0x03;    // ENABLE: every motor on both buses
/* MIT command fields, in register units: KP, KD (u16, 0.001 N*m/rad and
 * 0.001 N*m*s/rad), POS (i16 counts, relative to the offset for position
 * motors), VEL (i16 RPM), TFF (i16, 0.001 N*m). */
inline constexpr uint8_t kSetCmdMit = 0x04;       // BUS ID KP KD POS VEL TFF
inline constexpr uint8_t kSetCmdPos = 0x05;       // BUS ID POS VEL TFF (keeps KP KD)
/* set_cmd_mit, plus a log of the motor's state for SAMPLES (u16) cycles. */
inline constexpr uint8_t kTestMotor = 0x06;       // BUS ID KP KD POS VEL TFF SAMPLES
/* test_motor on that id on every bus at once. */
inline constexpr uint8_t kTestMotorDual = 0x07;   // ID KP KD POS VEL TFF SAMPLES
inline constexpr uint8_t kSetGains = 0x10;        // BUS ID KP(u16) KD(u16)
inline constexpr uint8_t kSetGainsLimit = 0x11;   // BUS ID KP_MAX(u16) KD_MAX(u16)
inline constexpr uint8_t kSetMode = 0x12;        // BUS ID MODE (2 position, 3 MIT)
/* One motor: on holds where it is; off lets it turn freely. */
inline constexpr uint8_t kSetTorque = 0x13;       // BUS ID ENABLE
inline constexpr uint8_t kPing = 0x20;            // BUS ID
inline constexpr uint8_t kGetGains = 0x24;        // BUS ID
inline constexpr uint8_t kGetMode = 0x25;         // BUS ID
inline constexpr uint8_t kGetPosVel = 0x26;       // BUS ID
inline constexpr uint8_t kGetTorque = 0x27;       // BUS ID
inline constexpr uint8_t kGetAll = 0x28;          // BUS ID: registers 0x18..0x35
}  // namespace pc

// builds one line of text like printf and sends it to the PC over uart0, with a \n on the end.
__attribute__((format(printf, 1, 2)))
void reply(const char *format, ...);

/* Reads PC frames and dispatches them. Started after kPcStartDelay. */
void pc_rx_task(void *);
