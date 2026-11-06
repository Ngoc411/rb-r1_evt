#pragma once

#include <cstdint>

#include "instruction.hpp"

/* One joint motor: its registers, and the fundamental functions that set and
 * get them, on top of the packet layer in instruction.hpp. All values are raw
 * register units at the output shaft (protocol doc V1.0.0). */

namespace motor {

inline constexpr int32_t kCountsPerRev = 16384;  // 1 output turn = 2 pi rad = 360 deg

/* Register addresses. */
namespace reg {
inline constexpr uint8_t kId = 0x03;
inline constexpr uint8_t kStatusReturnLevel = 0x10;
inline constexpr uint8_t kTorqueEnable = 0x18;     // RAM 0x18..0x35, read whole by get_all
inline constexpr uint8_t kKp = 0x1A;               // control block 0x1A..0x23:
inline constexpr uint8_t kKd = 0x1C;               //   Kp, Kd, Pos, Spd, Tau
inline constexpr uint8_t kGoalPosition = 0x1E;
inline constexpr uint8_t kGoalSpeed = 0x20;
inline constexpr uint8_t kFeedforwardTorque = 0x22;
inline constexpr uint8_t kPresentPosition = 0x24;
inline constexpr uint8_t kPresentSpeed = 0x26;
inline constexpr uint8_t kPresentCurrent = 0x28;
inline constexpr uint8_t kPresentVoltage = 0x2A;
inline constexpr uint8_t kPresentTemperature = 0x2B;
inline constexpr uint8_t kRegWritePending = 0x2C;
inline constexpr uint8_t kControlMode = 0x2D;
inline constexpr uint8_t kMoving = 0x2E;
inline constexpr uint8_t kEepromLock = 0x2F;
inline constexpr uint8_t kPunch = 0x30;
inline constexpr uint8_t kKpMax = 0x32;
inline constexpr uint8_t kKdMax = 0x34;
inline constexpr uint8_t kRamEnd = 0x35;           // last byte of Kd_max
}  // namespace reg

/* Status byte flags (response byte 4); 0 = no error. */
namespace status {
inline constexpr uint8_t kInstructionError = 0x40;
inline constexpr uint8_t kOverload = 0x20;       // torque turned off
inline constexpr uint8_t kChecksumError = 0x10;
inline constexpr uint8_t kRangeError = 0x08;     // bad length, gain above max, or locked
inline constexpr uint8_t kOverheat = 0x04;       // torque turned off
inline constexpr uint8_t kAngleLimit = 0x02;     // goal position rejected
inline constexpr uint8_t kVoltageError = 0x01;
}  // namespace status

enum class Mode_enum : uint8_t {
    Position = 2,  // multi-turn position; goal speed is the speed limit
    Mit = 3,       // impedance (power-up default)
};

/* Which unicast instructions the motor replies to (register 0x10). */
enum class ReplyLevel_enum : uint8_t {
    PingOnly = 0,
    PingAndRead = 1,
    All = 2,  // factory default
};

struct Gains_struct {
    uint16_t kp;  // 0.001 N*m/rad, default 10000; at most Kp_max
    uint16_t kd;  // 0.001 N*m*s/rad, default 36; at most Kd_max
};

/* MIT mode drives torque = Kp * (position - actual) + Kd * (speed - actual) + torque,
 * so Kp = 0 gives pure speed control. */
struct Command_struct {
    int16_t position;  // counts (kCountsPerRev per turn), +-2 turns
    int16_t speed;     // RPM
    int16_t torque;    // 0.001 N*m feedforward
};

/* One motor: which one it is, and a mirror of its registers, starting at the
 * power-up defaults. The set functions edit the mirror, then write the motor
 * from it; the get functions read the motor into it. Every function holds
 * its bus's lock for the whole exchange, so any task may call them. */
struct Motor_struct {
    // ── which motor ──
    const instruction::Bus_class &bus;
    uint8_t id;

    // ── written by the set functions ──
    bool torque = false;                      // 0x18
    Gains_struct gains = { .kp = 10000, .kd = 36 };  // 0x1A kp, 0x1C kd
    Command_struct command = {};                     // 0x1E pos, 0x20 vel, 0x22 tff
    Mode_enum mode = Mode_enum::Mit;                    // 0x2D
    Gains_struct limits = {};        // 0x32 Kp_max, 0x34 Kd_max: the motor rejects kp, kd above these

    // ── filled by the get functions ──
    uint8_t status = 0;       // byte 4 of every reply (error flags)
    int16_t position = 0;     // 0x24, and bytes 5..6 of every reply (counts)
    int16_t velocity = 0;     // 0x26 (RPM)
    // the rest: get_all only
    int16_t current = 0;      // 0x28 (mA)
    uint8_t voltage = 0;      // 0x2A (0.1 V)
    uint8_t temperature = 0;  // 0x2B (deg C)
    bool reg_pending = false; // 0x2C: a REG_WRITE waits for ACTION
    bool moving = false;      // 0x2E: moving to a goal in Position mode
    bool eeprom_locked = false;  // 0x2F
    uint16_t punch = 0;       // 0x30
};

/* ── set: edit m from the input, then instruction writes the motor from m ──
 * Each sends one WRITE and ignores the reply: the motor is set to
 * ReplyLevel_enum::PingAndRead at start-up, so it sends none. */

// pos vel tff (0x1E, 6 bytes). The motor turns torque on.
esp_err_t set_command(Motor_struct &m, const Command_struct &command);
// kp kd (0x1A, 4 bytes). Torque stays as it is.
esp_err_t set_gains(Motor_struct &m, const Gains_struct &gains);
// kp kd pos vel tff (0x1A, 10 bytes). The motor turns torque on.
esp_err_t set_gains_and_command(Motor_struct &m, const Gains_struct &gains, const Command_struct &command);
// On holds the shaft; off lets it turn freely (0x18).
esp_err_t set_torque(Motor_struct &m, bool on);
// Control mode (0x2D); not kept over power-off.
esp_err_t set_mode(Motor_struct &m, Mode_enum mode);
// Kp_max Kd_max (0x32, 4 bytes): kp, kd above these are capped by the motor.
esp_err_t set_gains_limit(Motor_struct &m, const Gains_struct &limits);

/* ── setup ──
 * Which instructions the motor replies to (0x10). Not a field of m: sent once
 * at start-up. Until it takes effect the motor may answer this write, so wait
 * for that reply before the next frame. Kept until power-off unless saved
 * with instruction::Bus_class::reboot. */
esp_err_t set_reply_level(Motor_struct &m, ReplyLevel_enum level);

/* ── get: send the request, then fill m from the reply ──
 * Each waits up to timeout_ticks for the reply, which also fills m.status
 * and, if the reply carries it, m.position, and returns:
 *   ESP_OK                    reply received; m is filled
 *   ESP_ERR_TIMEOUT           no reply
 *   ESP_ERR_INVALID_RESPONSE  reply had a bad checksum or did not match
 *   ESP_ERR_INVALID_ARG       id is instruction::kBroadcastId or above (no reply possible)
 *   other                     UART error
 * On any error m is left unchanged. */

// Checks that the motor answers: status, position.
esp_err_t ping(Motor_struct &m, TickType_t timeout_ticks);
// position, velocity (0x24..0x27).
esp_err_t get_pos_vel(Motor_struct &m, TickType_t timeout_ticks);
// kp kd in use (0x1A..0x1D).
esp_err_t get_gains(Motor_struct &m, TickType_t timeout_ticks);
// Control mode in use (0x2D), or the raw value if the motor reports another.
esp_err_t get_mode(Motor_struct &m, TickType_t timeout_ticks);
// Torque on or off (0x18); the motor turns it off by itself on overload or overheat.
esp_err_t get_torque(Motor_struct &m, TickType_t timeout_ticks);
// Every register from 0x18 to 0x35 (30 bytes) into every field of m.
esp_err_t get_all(Motor_struct &m, TickType_t timeout_ticks);

}  // namespace motor
