#include "motor_utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <span>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "UART.hpp"
#include "pc_link.hpp"  // reply()
#include "instruction.hpp"

namespace motor_utils {

namespace {

/* ============================= state ============================= */

/* Guards the state below (mode, tests, the joints' structs and offsets), so
 * the 100 Hz feedback reads and the PC's commands never see each other's
 * half-done changes. Each bus is guarded by the fundamental functions
 * themselves. */
SemaphoreHandle_t utils_lock = nullptr;

class Lock_class {
public:
    Lock_class() { xSemaphoreTake(utils_lock, portMAX_DELAY); }
    ~Lock_class() { xSemaphoreGive(utils_lock); }
    Lock_class(const Lock_class &) = delete;
    Lock_class &operator=(const Lock_class &) = delete;
};

bool action_mode = false;  // action mode: motion commands and tests allowed; else config mode

constexpr const char *kNotAction = "config mode; send action first";
constexpr const char *kMotorTorqueOff = "this motor's torque is off; send settorque on first";

/* A test_motor run in progress, one per bus (joints[b]). */
struct ActiveTest_struct {
    bool active = false;
    Joint_struct *joint = nullptr;  // the tested joint
    uint32_t samples_left = 0;
    uint32_t samples = 0;
    uint32_t errors = 0;
    int64_t start_us = 0;
};
std::array<ActiveTest_struct, 2> tests;

constexpr size_t kEchoFrameMax = 17;  // longest frame the echo cache holds (10-byte write)

/* Last frame echoed for one motor: [0] writes and other frames, [1] reads, so
 * a test's read every cycle is not echoed every time. */
using EchoSlots_alias = std::array<std::array<uint8_t, kEchoFrameMax>, 2>;
std::array<std::array<EchoSlots_alias, 3>, 2> echoed;  // like joints

/* ============================= lookups ============================= */

int bus_number(size_t b) {
    return joints[b][0].motor.bus.uart().port();
}

/* Which bus (index into joints) j is on. */
size_t bus_of(const Joint_struct &j) {
    for (size_t b = 0; b < joints.size(); ++b) {
        for (const Joint_struct &other : joints[b]) {
            if (&other == &j) {
                return b;
            }
        }
    }
    return 0;
}

int16_t clamp_i16(int32_t value) {
    return static_cast<int16_t>(std::clamp<int32_t>(value, INT16_MIN, INT16_MAX));
}

/* ============================= TX echo ============================= */

/* Echoes a motor frame to the PC as one text line, e.g.
 * "TX1: FF FF 02 09 03 1E 00 08 00 00 00 00 CB\n" (1 = UART_NUM_1), but only
 * when it differs from the last frame of its kind (read or not) for that
 * motor: a test's read every 10 ms in hex text would flood the PC link and
 * bury the LOG lines. Called by the instruction layer, inside the bus's lock. */
/* Sends "<c0><c1><port>: XX XX ...\n" to the PC, e.g. "TX1: FF FF 01 ..." */
void send_hex_line(char c0, char c1, const Uart_class &uart, std::span<const uint8_t> bytes) {
    static constexpr char hex[] = "0123456789ABCDEF";
    std::array<uint8_t, 208> line;
    size_t n = 0;

    line[n++] = c0;
    line[n++] = c1;
    line[n++] = static_cast<uint8_t>('0' + uart.port());
    line[n++] = ':';
    for (size_t k = 0; k < bytes.size() && n + 4U <= line.size(); ++k) {
        line[n++] = ' ';
        line[n++] = hex[bytes[k] >> 4];
        line[n++] = hex[bytes[k] & 0x0FU];
    }
    line[n++] = '\n';
    uart0.write(std::span(line).first(n));
}

/* In config mode every frame is echoed; in action mode, only one that
 * differs from the last of its kind for that motor. */
void echo_tx_frame(const Uart_class &uart, std::span<const uint8_t> frame) {
    for (size_t b = 0; b < joints.size(); ++b) {
        for (size_t i = 0; i < joints[b].size(); ++i) {
            const motor::Motor_struct &m = joints[b][i].motor;
            if (m.bus.uart().port() != uart.port() || m.id != frame[2] ||
                frame.size() > kEchoFrameMax) {
                continue;
            }
            auto &last = echoed[b][i][frame[4] == instruction::kInstRead ? 1U : 0U];
            if (action_mode && std::equal(frame.begin(), frame.end(), last.begin())) {
                return;
            }
            std::copy(frame.begin(), frame.end(), last.begin());
        }
    }
    send_hex_line('T', 'X', uart, frame);
}

/* Shows what arrived while a reply was awaited, e.g. "RX1: FF FF 01 07 ..."
 * (the first 64 bytes), or "RX1:" if nothing did: always in config mode,
 * and in action mode only when the reply was not found (the test logs'
 * reads would flood the PC link). */
void echo_rx_bytes(const Uart_class &uart, std::span<const uint8_t> bytes, bool found) {
    if (found && action_mode) {
        return;
    }
    send_hex_line('R', 'X', uart, bytes);
}

/* ============================= test_motor log ============================= */

void end_test(size_t b, const char *reason) {
    ActiveTest_struct &test = tests[b];
    if (!test.active) {
        return;
    }
    reply("LOG_END bus=%d id=%u samples=%lu errors=%lu reason=%s", bus_number(b),
          test.joint->motor.id, static_cast<unsigned long>(test.samples),
          static_cast<unsigned long>(test.errors), reason);
    test.active = false;
}

/* LOG_START tells the PC to open a CSV. The logged measurements are raw; the
 * PC turns them into its own frame: position = dir * (pos - offset) relative to
 * the torque-on pose (offset 0 for speed joints), speed = dir * raw / gear
 * ratio; current stays raw (its sign does not follow the turning direction).
 * A test running on this bus ends first. */
void start_test(size_t b, Joint_struct &j, uint32_t samples, int64_t start_us) {
    end_test(b, "replaced");
    tests[b] = { .active = true, .joint = &j, .samples_left = samples,
                 .samples = 0, .errors = 0, .start_us = start_us };
    const int offset = j.control == Control_enum::Position ? j.offset : 0;
    reply("LOG_START bus=%d id=%u samples=%lu offset=%d dir=%d", bus_number(b), j.motor.id,
          static_cast<unsigned long>(samples), offset, j.direction);
}

/* Reads every register of the tested motor into its struct (get_all) and
 * sends one CSV-ready line from the struct:
 *   LOG,bus,id,t_us,kp,kd,cmd_pos,cmd_vel,cmd_tff,pos,spd,cur,volt,temp,status,err
 * t_us counts from LOG_START. Commanded values are joint values, as the PC
 * set them; measured ones are raw, and empty with err set when the read
 * fails. */
void log_sample(size_t b) {
    ActiveTest_struct &test = tests[b];
    Joint_struct &j = *test.joint;
    const motor::Motor_struct &m = j.motor;
    const auto t_us = static_cast<unsigned long>(esp_timer_get_time() - test.start_us);
    const esp_err_t err = motor::get_all(j.motor, kReplyTimeout);
    const motor::Command_struct cmd = to_joint(j, m.command);
    ++test.samples;
    if (err != ESP_OK) {
        ++test.errors;
        reply("LOG,%d,%u,%lu,%u,%u,%d,%d,%d,,,,,,,%s", bus_number(b), m.id, t_us,
              m.gains.kp, m.gains.kd, cmd.position, cmd.speed, cmd.torque,
              esp_err_to_name(err));
    } else {
        reply("LOG,%d,%u,%lu,%u,%u,%d,%d,%d,%d,%d,%d,%u,%u,%u,OK", bus_number(b), m.id, t_us,
              m.gains.kp, m.gains.kd, cmd.position, cmd.speed, cmd.torque, m.position,
              m.velocity, m.current, m.voltage, m.temperature, m.status);
    }
    if (--test.samples_left == 0U) {
        end_test(b, "done");
    }
}

/* ============================= torque ============================= */

/* Torque off on every motor. A motor still replying to every write (as after
 * its power-up) answers each one, so wait for that reply to finish before the
 * next frame. */
void turn_torque_off() {
    for (auto &bus_joints : joints) {
        for (Joint_struct &j : bus_joints) {
            ESP_ERROR_CHECK(motor::set_torque(j.motor, false));
            vTaskDelay(kReplySettle);
        }
    }
}

/* Every motor replies only to PING and READ from now on, so the set
 * functions' writes get no reply that could collide with the next frame on
 * the shared wire. Until it takes effect the motor answers it; wait for that. */
void set_reply_levels() {
    for (auto &bus_joints : joints) {
        for (Joint_struct &j : bus_joints) {
            ESP_ERROR_CHECK(motor::set_reply_level(j.motor, motor::ReplyLevel_enum::PingAndRead));
            vTaskDelay(kReplySettle);
        }
    }
}

/* Reads m's gains and present position, then tells it to hold there with
 * those gains, which turns its torque on without moving it. */
esp_err_t hold_in_place(motor::Motor_struct &m) {
    esp_err_t err = motor::get_gains(m, kReplyTimeout);
    if (err == ESP_OK) {
        err = motor::get_pos_vel(m, kReplyTimeout);
    }
    if (err != ESP_OK) {
        return err;
    }
    return motor::set_gains_and_command(m, m.gains, { .position = m.position, .speed = 0, .torque = 0 });
}

/* Turns j's torque on without moving it: a speed joint gets kSpeedGains and
 * speed 0; a position joint keeps its motor's gains, its present position
 * becomes its offset (its 0), and it is told to hold there. Either is read
 * first, so a motor that does not answer is not turned on. */
esp_err_t turn_on_in_place(Joint_struct &j) {
    motor::Motor_struct &m = j.motor;
    if (j.control == Control_enum::Speed) {
        const esp_err_t err = motor::get_pos_vel(m, kReplyTimeout);
        if (err != ESP_OK) {
            return err;
        }
        j.offset = 0;
        return motor::set_gains_and_command(m, kSpeedGains, {});
    }
    const esp_err_t err = hold_in_place(m);
    if (err != ESP_OK) {
        return err;
    }
    j.offset = m.position;
    reply("OFFSET bus=%d id=%u pos=%d", bus_number(bus_of(j)), m.id, m.position);
    return ESP_OK;
}

/* turn_on_in_place for every motor whose torque is off. The reply level is
 * set again in case a motor was power-cycled. If a motor does not answer,
 * torque is turned off again on all of them. */
const char *turn_torque_on_in_place() {
    set_reply_levels();
    for (size_t b = 0; b < joints.size(); ++b) {
        for (Joint_struct &j : joints[b]) {
            if (j.motor.torque) {
                continue;  // keeps its offset and command
            }
            if (const esp_err_t err = turn_on_in_place(j); err != ESP_OK) {
                reply("ERR torque on: bus=%d id=%u read failed: %s", bus_number(b), j.motor.id,
                      esp_err_to_name(err));
                for (size_t t = 0; t < tests.size(); ++t) {
                    end_test(t, "torque_off");
                }
                turn_torque_off();
                return "a motor did not answer; torque stays off";
            }
        }
    }
    return nullptr;
}

/* ============================= the 100 Hz loop ============================= */

void set_leds(uint32_t level) {
    ESP_ERROR_CHECK(gpio_set_level(LED1, level));
    ESP_ERROR_CHECK(gpio_set_level(LED2, level));
}

/* Start-up, then every 10 ms. The loop only reads feedback: every command is
 * sent once, when the PC asks, and the motor keeps it. In action mode it reads
 * and logs each running test's motor (nothing at all when no test runs) and
 * blinks the LEDs; in config mode it reads nothing and the LEDs stay on. */
void loop_task(void *) {
    {
        Lock_class lock;
        /* The motors may still hold torque from before an ESP reset. After
         * these, only what the PC asks for is sent. */
        turn_torque_off();
        set_reply_levels();
    }

    uint32_t cycle = 0;
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        /* After a long wait for the lock (torque on), start the period from
         * now instead of catching up. */
        if (xTaskDelayUntil(&last_wake, kControlPeriod) == pdFALSE) {
            last_wake = xTaskGetTickCount();
        }
        Lock_class lock;
        if (!action_mode) {
            set_leds(1);
            continue;
        }
        for (size_t b = 0; b < tests.size(); ++b) {
            if (tests[b].active) {
                log_sample(b);
            }
        }
        set_leds((cycle / kLedToggleCycles) % 2U);
        ++cycle;
    }
}

/* ============================= checks ============================= */

/* The joint for bus and id, if in action mode, its torque is on and command
 * fits it, with the motor command in raw; else why not. */
const char *checked_joint(int bus, uint8_t id, const motor::Command_struct &command, Joint_struct *&j,
                          motor::Command_struct &raw) {
    if (!action_mode) {
        return kNotAction;
    }
    j = find_joint(bus, id);
    if (j == nullptr) {
        return "bad bus or id";
    }
    if (!j->motor.torque) {
        return kMotorTorqueOff;
    }
    return to_motor(*j, command, raw);
}

}  // namespace

/* ============================= joint <-> motor values ============================= */

const char *to_motor(const Joint_struct &j, const motor::Command_struct &command, motor::Command_struct &out) {
    int32_t position = j.direction * static_cast<int32_t>(command.position);
    if (j.control == Control_enum::Position) {
        position += j.offset;
    }
    if (position < INT16_MIN || position > INT16_MAX) {
        return "pos plus the torque-on offset is outside the motor's range";
    }
    out = { .position = static_cast<int16_t>(position),
            .speed = clamp_i16(j.direction * command.speed),
            .torque = clamp_i16(j.direction * command.torque) };
    return nullptr;
}

motor::Command_struct to_joint(const Joint_struct &j, const motor::Command_struct &command) {
    int32_t position = command.position;
    if (j.control == Control_enum::Position) {
        position -= j.offset;
    }
    return { .position = clamp_i16(j.direction * position),
             .speed = clamp_i16(j.direction * command.speed),
             .torque = clamp_i16(j.direction * command.torque) };
}

void to_joint_feedback(const Joint_struct &j, int16_t position, int16_t velocity,
                       int32_t &joint_position, int32_t &joint_velocity_x100) {
    int32_t raw = position;
    if (j.control == Control_enum::Position) {
        raw -= j.offset;
    }
    joint_position = j.direction * raw;
    joint_velocity_x100 = static_cast<int32_t>(
        std::lround(j.direction * velocity * 100.0f / j.gear_ratio));
}

/* ============================= public functions ============================= */

void start() {
    utils_lock = xSemaphoreCreateMutex();
    configASSERT(utils_lock != nullptr);
    if constexpr (!kUseRvLink) {  // the RV link takes no text
        instruction::set_tx_hook(echo_tx_frame);
        instruction::set_rx_hook(echo_rx_bytes);
    }
    configASSERT(xTaskCreate(loop_task, "motor_loop", 4096, nullptr, 5, nullptr) == pdPASS);
}

void set_action(bool on) {
    Lock_class lock;
    if (on == action_mode) {
        return;
    }
    action_mode = on;
    if (!on) {
        for (size_t b = 0; b < tests.size(); ++b) {
            end_test(b, "config");
        }
        turn_torque_off();
    }
}

bool in_action() {
    Lock_class lock;
    return action_mode;
}

const char *set_torque_all(bool on) {
    Lock_class lock;
    if (on) {
        return turn_torque_on_in_place();
    }
    for (size_t b = 0; b < tests.size(); ++b) {
        end_test(b, "torque_off");
    }
    turn_torque_off();
    return nullptr;
}

const char *set_torque(int bus, uint8_t id, bool on) {
    Lock_class lock;
    Joint_struct *j = find_joint(bus, id);
    if (j == nullptr) {
        return "bad bus or id";
    }
    motor::Motor_struct &m = j->motor;
    if (!on) {
        const size_t b = bus_of(*j);
        if (tests[b].joint == j) {
            end_test(b, "torque_off");
        }
        ESP_ERROR_CHECK(motor::set_torque(m, false));
        return nullptr;
    }
    if (m.torque) {
        return nullptr;  // keeps its offset and command
    }
    /* In case it was power-cycled and replies to every write again. */
    ESP_ERROR_CHECK(motor::set_reply_level(m, motor::ReplyLevel_enum::PingAndRead));
    vTaskDelay(kReplySettle);
    const esp_err_t err = turn_on_in_place(*j);
    return err == ESP_OK ? nullptr : esp_err_to_name(err);
}

const char *test_motor(int bus, uint8_t id, const motor::Gains_struct &gains,
                       const motor::Command_struct &command, uint16_t samples) {
    if (samples == 0U) {
        return "duration must be at least one cycle";
    }
    Lock_class lock;
    Joint_struct *j;
    motor::Command_struct raw;
    if (const char *why = checked_joint(bus, id, command, j, raw)) {
        return why;
    }
    const esp_err_t err = motor::set_gains_and_command(j->motor, gains, raw);
    if (err != ESP_OK) {
        return esp_err_to_name(err);
    }
    const size_t b = bus_of(*j);
    for (size_t other = 0; other < tests.size(); ++other) {
        if (other != b) {
            end_test(other, "replaced");
        }
    }
    start_test(b, *j, samples, esp_timer_get_time());
    return nullptr;
}

const char *test_motor_dual(uint8_t id, const motor::Gains_struct &gains,
                            const motor::Command_struct &command, uint16_t samples) {
    if (samples == 0U) {
        return "duration must be at least one cycle";
    }
    Lock_class lock;
    std::array<Joint_struct *, 2> tested;
    std::array<motor::Command_struct, 2> raw;
    for (size_t b = 0; b < joints.size(); ++b) {
        if (const char *why = checked_joint(bus_number(b), id, command, tested[b], raw[b])) {
            return why;
        }
    }
    const int64_t start_us = esp_timer_get_time();
    for (size_t b = 0; b < joints.size(); ++b) {
        const esp_err_t err = motor::set_gains_and_command(tested[b]->motor, gains, raw[b]);
        if (err != ESP_OK) {
            return esp_err_to_name(err);
        }
        start_test(b, *tested[b], samples, start_us);
    }
    return nullptr;
}

}  // namespace motor_utils
