#include "pc_link.hpp"

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstdio>
#include <span>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "config.hpp"
#include "frame_parser.hpp"
#include "little_endian.hpp"
#include "motor.hpp"
#include "motor_utils.hpp"

// send back text line to PC via USB. With the RV link on UART0 instead, it
// sends nothing: only HCR1 frames may go there.
void reply(const char *format, ...) {
    if constexpr (kUseRvLink) {
        return;
    }
    std::array<char, 160> line;
    va_list args;
    va_start(args, format);
    const int n = vsnprintf(line.data(), line.size() - 1U, format, args);
    va_end(args);
    if (n < 0) {
        return;
    }
    size_t length = std::min(static_cast<size_t>(n), line.size() - 2U);
    line[length++] = '\n';
    uart0.write(std::span(reinterpret_cast<const uint8_t *>(line.data()), length));
}

namespace {

constexpr size_t kMaxPcPayload = 14;  // test_motor - 14 bytes

/* ============================= decoding ============================= */

/* KP KD (u16 each) */
motor::Gains_struct read_gains(const uint8_t *bytes) {
    return { .kp = get_u16(&bytes[0]), .kd = get_u16(&bytes[2]) };
}

/* POS VEL TFF (i16 each) */
motor::Command_struct read_command(const uint8_t *bytes) {
    return { .position = get_i16(&bytes[0]), .speed = get_i16(&bytes[2]),
             .torque = get_i16(&bytes[4]) };
}

/* True if the payload has the right size; else replies an error. */
bool check_size(const char *name, std::span<const uint8_t> payload, size_t size) {
    if (payload.size() != size) {
        reply("ERR %s: payload", name);
        return false;
    }
    return true;
}

/* Replies ERR when the called function refused (why is not nullptr). */
void reply_if_refused(const char *name, const char *why) {
    if (why != nullptr) {
        reply("ERR %s: %s", name, why);
    }
}

/* The motor for BUS ID (payload[0], payload[1]), or nullptr after an error. */
motor::Motor_struct *find_motor(const char *name, std::span<const uint8_t> payload) {
    Joint_struct *j = find_joint(payload[0], payload[1]);
    if (j == nullptr) {
        reply("ERR %s: bad bus or id", name);
        return nullptr;
    }
    return &j->motor;
}

/* The RESULT line that ends a single-motor command. */
void report(const char *name, const motor::Motor_struct &m, esp_err_t err) {
    reply("RESULT %s bus=%d id=%u: %s", name, m.bus.uart().port(), m.id, esp_err_to_name(err));
}

/* ============================= commands ============================= */

/* True in config mode; else replies an error. */
bool check_config(const char *name) {
    if (motor_utils::in_action()) {
        reply("ERR %s: action mode; send config first", name);
        return false;
    }
    return true;
}

/* action / config: ACTION */
void handle_sim_mode(std::span<const uint8_t> payload) {
    if (!check_size("sim_mode", payload, 1U)) {
        return;
    }
    if (payload[0] > 1U) {
        reply("ERR sim_mode: must be 0 (config) or 1 (action)");
        return;
    }
    const bool action = payload[0] != 0U;
    motor_utils::set_action(action);
    reply("MODE %s", action ? "action" : "config");
}

/* torque: ENABLE */
void handle_torque(std::span<const uint8_t> payload) {
    if (!check_size("torque", payload, 1U)) {
        return;
    }
    if (payload[0] > 1U) {
        reply("ERR torque: must be 0 (off) or 1 (on)");
        return;
    }
    const bool on = payload[0] != 0U;
    if (const char *why = motor_utils::set_torque_all(on)) {
        reply("ERR torque: %s", why);
        reply("TORQUE off");
        return;
    }
    reply("TORQUE %s", on ? "on" : "off");
}

/* settorque: BUS ID ENABLE */
void handle_set_torque(std::span<const uint8_t> payload) {
    if (!check_size("set_torque", payload, 3U)) {
        return;
    }
    if (payload[2] > 1U) {
        reply("ERR set_torque: must be 0 (off) or 1 (on)");
        return;
    }
    const bool on = payload[2] != 0U;
    if (const char *why = motor_utils::set_torque(payload[0], payload[1], on)) {
        reply("ERR set_torque: %s", why);
        return;
    }
    reply("MOTOR_TORQUE bus=%u id=%u on=%u", payload[0], payload[1], on ? 1U : 0U);
}

/* The joint for BUS ID with a joint command turned into motor values, if in
 * action mode and its torque is on; else nullptr after an error. */
Joint_struct *joint_with_command(const char *name, std::span<const uint8_t> payload,
                          const motor::Command_struct &command, motor::Command_struct &raw) {
    if (!motor_utils::in_action()) {
        reply("ERR %s: config mode; send action first", name);
        return nullptr;
    }
    Joint_struct *j = find_joint(payload[0], payload[1]);
    if (j == nullptr) {
        reply("ERR %s: bad bus or id", name);
        return nullptr;
    }
    if (!j->motor.torque) {
        reply("ERR %s: this motor's torque is off; send settorque on first", name);
        return nullptr;
    }
    if (const char *why = motor_utils::to_motor(*j, command, raw)) {
        reply("ERR %s: %s", name, why);
        return nullptr;
    }
    return j;
}

/* set_cmd_mit: BUS ID KP KD POS VEL TFF → set command and gains */
void handle_set_cmd_mit(std::span<const uint8_t> payload) {
    if (!check_size("set_cmd_mit", payload, 12U)) {
        return;
    }
    motor::Command_struct raw;
    if (Joint_struct *j = joint_with_command("set_cmd_mit", payload, read_command(&payload[6]), raw)) {
        const esp_err_t err = motor::set_gains_and_command(j->motor, read_gains(&payload[2]), raw);
        if (err != ESP_OK) {
            report("set_cmd_mit", j->motor, err);
        }
    }
}

/* set_cmd_pos: BUS ID POS VEL TFF → set command */
void handle_set_cmd_pos(std::span<const uint8_t> payload) {
    if (!check_size("set_cmd_pos", payload, 8U)) {
        return;
    }
    motor::Command_struct raw;
    if (Joint_struct *j = joint_with_command("set_cmd_pos", payload, read_command(&payload[2]), raw)) {
        const esp_err_t err = motor::set_command(j->motor, raw);
        if (err != ESP_OK) {
            report("set_cmd_pos", j->motor, err);
        }
    }
}

/* test_motor: BUS ID KP KD POS VEL TFF SAMPLES */
void handle_test_motor(std::span<const uint8_t> payload) {
    if (!check_size("test_motor", payload, 14U)) {
        return;
    }
    reply_if_refused("test_motor",
                     motor_utils::test_motor(payload[0], payload[1], read_gains(&payload[2]),
                                             read_command(&payload[6]), get_u16(&payload[12])));
}

/* test_motor_dual: ID KP KD POS VEL TFF SAMPLES */
void handle_test_motor_dual(std::span<const uint8_t> payload) {
    if (!check_size("test_motor_dual", payload, 13U)) {
        return;
    }
    reply_if_refused("test_motor_dual",
                     motor_utils::test_motor_dual(payload[0], read_gains(&payload[1]),
                                                  read_command(&payload[5]),
                                                  get_u16(&payload[11])));
}

/* setgains: BUS ID KP KD */
void handle_set_gains(std::span<const uint8_t> payload) {
    if (!check_size("set_gains", payload, 6U) || !check_config("set_gains")) {
        return;
    }
    if (motor::Motor_struct *m = find_motor("set_gains", payload)) {
        report("set_gains", *m, motor::set_gains(*m, read_gains(&payload[2])));
    }
}

/* setgainslimit: BUS ID KP_MAX KD_MAX */
void handle_set_gains_limit(std::span<const uint8_t> payload) {
    if (!check_size("set_gains_limit", payload, 6U) || !check_config("set_gains_limit")) {
        return;
    }
    if (motor::Motor_struct *m = find_motor("set_gains_limit", payload)) {
        report("set_gains_limit", *m, motor::set_gains_limit(*m, read_gains(&payload[2])));
    }
}

/* setmode: BUS ID MODE (2 position, 3 MIT) */
void handle_set_mode(std::span<const uint8_t> payload) {
    if (!check_size("set_mode", payload, 3U) || !check_config("set_mode")) {
        return;
    }
    const auto mode = static_cast<motor::Mode_enum>(payload[2]);
    if (mode != motor::Mode_enum::Position && mode != motor::Mode_enum::Mit) {
        reply("ERR set_mode: mode must be 2 (position) or 3 (MIT)");
        return;
    }
    if (motor::Motor_struct *m = find_motor("set_mode", payload)) {
        report("set_mode", *m, motor::set_mode(*m, mode));
    }
}

/* ping: BUS ID */
void handle_ping(std::span<const uint8_t> payload) {
    if (!check_size("ping", payload, 2U)) {
        return;
    }
    if (motor::Motor_struct *m = find_motor("ping", payload)) {
        report("ping", *m, motor::ping(*m, kReplyTimeout));
    }
}

/* getgains: BUS ID */
void handle_get_gains(std::span<const uint8_t> payload) {
    if (!check_size("get_gains", payload, 2U)) {
        return;
    }
    motor::Motor_struct *m = find_motor("get_gains", payload);
    if (m == nullptr) {
        return;
    }
    const esp_err_t err = motor::get_gains(*m, kReplyTimeout);
    if (err == ESP_OK) {
        reply("GAINS bus=%u id=%u kp=%u kd=%u", payload[0], m->id, m->gains.kp, m->gains.kd);
    }
    report("get_gains", *m, err);
}

/* getposvel: BUS ID. Raw motor values, plus joint values while its torque is
 * on (the offset is set when it is turned on): jpos in counts, jvel_x100 in
 * 0.01 RPM at the output shaft. */
void handle_get_pos_vel(std::span<const uint8_t> payload) {
    if (!check_size("get_pos_vel", payload, 2U)) {
        return;
    }
    Joint_struct *j = find_joint(payload[0], payload[1]);
    if (j == nullptr) {
        reply("ERR get_pos_vel: bad bus or id");
        return;
    }
    const motor::Motor_struct &m = j->motor;
    const esp_err_t err = motor::get_pos_vel(j->motor, kReplyTimeout);
    if (err == ESP_OK) {
        if (m.torque) {
            int32_t joint_position;
            int32_t joint_velocity_x100;
            motor_utils::to_joint_feedback(*j, m.position, m.velocity, joint_position,
                                           joint_velocity_x100);
            reply("POSVEL bus=%u id=%u pos=%d vel=%d jpos=%ld jvel_x100=%ld", payload[0], m.id,
                  m.position, m.velocity, static_cast<long>(joint_position),
                  static_cast<long>(joint_velocity_x100));
        } else {
            reply("POSVEL bus=%u id=%u pos=%d vel=%d", payload[0], m.id, m.position,
                  m.velocity);
        }
    }
    report("get_pos_vel", m, err);
}

/* gettorque: BUS ID */
void handle_get_torque(std::span<const uint8_t> payload) {
    if (!check_size("get_torque", payload, 2U)) {
        return;
    }
    motor::Motor_struct *m = find_motor("get_torque", payload);
    if (m == nullptr) {
        return;
    }
    const esp_err_t err = motor::get_torque(*m, kReplyTimeout);
    if (err == ESP_OK) {
        reply("MOTOR_TORQUE bus=%u id=%u on=%u", payload[0], m->id, m->torque ? 1U : 0U);
    }
    report("get_torque", *m, err);
}

/* getmode: BUS ID */
void handle_get_mode(std::span<const uint8_t> payload) {
    if (!check_size("get_mode", payload, 2U)) {
        return;
    }
    motor::Motor_struct *m = find_motor("get_mode", payload);
    if (m == nullptr) {
        return;
    }
    const esp_err_t err = motor::get_mode(*m, kReplyTimeout);
    if (err == ESP_OK) {
        const char *name = m->mode == motor::Mode_enum::Position ? "position"
                         : m->mode == motor::Mode_enum::Mit      ? "mit"
                                                            : "unknown";
        reply("CTRLMODE bus=%u id=%u mode=%u (%s)", payload[0], m->id,
              static_cast<unsigned>(m->mode), name);
    }
    report("get_mode", *m, err);
}

/* getall: BUS ID. Every register from 0x18 to 0x35, raw, in two lines: what
 * is set (ALL_CTRL) and what the motor measures (ALL_STATE). */
void handle_get_all(std::span<const uint8_t> payload) {
    if (!check_size("get_all", payload, 2U)) {
        return;
    }
    motor::Motor_struct *m = find_motor("get_all", payload);
    if (m == nullptr) {
        return;
    }
    const esp_err_t err = motor::get_all(*m, kReplyTimeout);
    if (err == ESP_OK) {
        reply("ALL_CTRL bus=%u id=%u torque=%u mode=%u kp=%u kd=%u cmd_pos=%d cmd_vel=%d "
              "cmd_tff=%d kp_max=%u kd_max=%u",
              payload[0], m->id, m->torque ? 1U : 0U, static_cast<unsigned>(m->mode),
              m->gains.kp, m->gains.kd, m->command.position, m->command.speed,
              m->command.torque, m->limits.kp, m->limits.kd);
        reply("ALL_STATE bus=%u id=%u status=0x%02X pos=%d spd=%d cur=%d volt=%u temp=%u "
              "reg=%u moving=%u lock=%u punch=%u",
              payload[0], m->id, m->status, m->position, m->velocity, m->current, m->voltage,
              m->temperature, m->reg_pending ? 1U : 0U, m->moving ? 1U : 0U,
              m->eeprom_locked ? 1U : 0U, m->punch);
    }
    report("get_all", *m, err);
}

}  // namespace

/* ============================= receive task ============================= */

void pc_rx_task(void *) {
    FrameParser_class<kMaxPcPayload> parser;

    /* Drop whatever the PC sent during start-up. */
    ESP_ERROR_CHECK(uart0.flush_input());

    while (true) {
        /* The PC writes a frame in one go, so a gap inside one means it was
         * cut off; drop it rather than let it swallow the next frame. */
        const TickType_t timeout = parser.in_frame() ? kPcByteTimeout : portMAX_DELAY;
        uint8_t byte;
        size_t received = 0;
        if (uart0.read(std::span(&byte, 1), timeout, &received) != ESP_OK) {
            continue;
        }
        if (received == 0U) {
            parser.reset();
            continue;
        }
        if (!parser.push(byte)) {
            continue;
        }

        const std::span<const uint8_t> payload = parser.payload();
        switch (parser.command()) {
        case pc::kSetSimMode:    handle_sim_mode(payload); break;
        case pc::kSetTorqueAll:  handle_torque(payload); break;
        case pc::kSetTorque:     handle_set_torque(payload); break;
        case pc::kSetCmdMit:     handle_set_cmd_mit(payload); break;
        case pc::kSetCmdPos:     handle_set_cmd_pos(payload); break;
        case pc::kTestMotor:     handle_test_motor(payload); break;
        case pc::kTestMotorDual: handle_test_motor_dual(payload); break;
        case pc::kSetGains:      handle_set_gains(payload); break;
        case pc::kSetGainsLimit: handle_set_gains_limit(payload); break;
        case pc::kSetMode:       handle_set_mode(payload); break;
        case pc::kPing:          handle_ping(payload); break;
        case pc::kGetGains:      handle_get_gains(payload); break;
        case pc::kGetMode:       handle_get_mode(payload); break;
        case pc::kGetPosVel:     handle_get_pos_vel(payload); break;
        case pc::kGetTorque:     handle_get_torque(payload); break;
        case pc::kGetAll:        handle_get_all(payload); break;
        default:                 reply("ERR command 0x%02X: unknown", parser.command()); break;
        }
    }
}
