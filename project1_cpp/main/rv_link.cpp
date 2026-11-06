#include "rv_link.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "freertos/FreeRTOS.h"
#include "config.hpp"
#include "frame_parser.hpp"
#include "little_endian.hpp"
#include "motor.hpp"
#include "motor_utils.hpp"

namespace {

constexpr size_t kMaxRvPayload = 12;  // SEND_ACTION

/* ============================= joints ============================= */

struct JointRef_struct {
    int bus;
    uint8_t id;
};

/* The order of GET_FEEDBACK and SEND_ACTION: the first kJointsWithPos have a
 * position (hips, knees); all of them have a velocity. */
constexpr std::array<JointRef_struct, 6> kDataOrder = {{
    { kLeftBus, kHipId }, { kRightBus, kHipId },
    { kLeftBus, kKneeId }, { kRightBus, kKneeId },
    { kLeftBus, kWheelId }, { kRightBus, kWheelId },
}};
constexpr size_t kJointsWithPos = 4;

/* The order of STATUS_FLAGS bits 2..7. */
constexpr std::array<JointRef_struct, 6> kStatusOrder = {{
    { kRightBus, kHipId }, { kRightBus, kKneeId }, { kRightBus, kWheelId },
    { kLeftBus, kHipId }, { kLeftBus, kKneeId }, { kLeftBus, kWheelId },
}};

/* FOLD_DOWN's hips and where they go, in joint counts. */
constexpr int16_t deg_to_counts(int32_t deg) {
    return static_cast<int16_t>(deg * motor::kCountsPerRev / 360);
}
constexpr std::array<JointRef_struct, 2> kFoldJoints = {{ { kRightBus, kHipId }, { kLeftBus, kHipId } }};
constexpr std::array<int16_t, 2> kFoldCounts = { deg_to_counts(kFoldRightHipDeg),
                                                 deg_to_counts(kFoldLeftHipDeg) };
static_assert(kFoldCounts[0] == 7964 && kFoldCounts[1] == -7964, "175 deg = 7964 counts");

Joint_struct &joint(const JointRef_struct &ref) {
    Joint_struct *j = find_joint(ref.bus, ref.id);
    configASSERT(j != nullptr);  // the tables above match config.hpp
    return *j;
}

/* ============================= sending ============================= */

/* Sends AA 55 ID L DATA CS to the RV. */
void send_frame(uint8_t id, std::span<const uint8_t> data) {
    std::array<uint8_t, 5 + 20> frame;  // GET_FEEDBACK's reply is the longest
    const size_t length = std::min(data.size(), frame.size() - 5U);
    frame[0] = 0xAA;
    frame[1] = 0x55;
    frame[2] = id;
    frame[3] = static_cast<uint8_t>(length);
    uint8_t sum = static_cast<uint8_t>(id + length);
    for (size_t i = 0; i < length; ++i) {
        frame[4 + i] = data[i];
        sum = static_cast<uint8_t>(sum + data[i]);
    }
    frame[4 + length] = static_cast<uint8_t>(~sum);
    uart0.write(std::span(frame).first(5 + length));
}

/* ============================= values ============================= */

/* RV value → motor value: the raw value, only a position joint's position
 * moved by its offset (its 0 is the torque-on pose). No direction sign, no
 * scaling. False if the position does not fit the motor's range. */
bool to_raw(const Joint_struct &j, const motor::Command_struct &command, motor::Command_struct &raw) {
    int32_t position = command.position;
    if (j.control == Control_enum::Position) {
        position += j.offset;
    }
    if (position < INT16_MIN || position > INT16_MAX) {
        return false;
    }
    raw = { .position = static_cast<int16_t>(position), .speed = command.speed,
            .torque = command.torque };
    return true;
}

/* Motor position → RV value: the reverse of to_raw. */
int16_t from_raw_position(const Joint_struct &j, int16_t position) {
    int32_t value = position;
    if (j.control == Control_enum::Position) {
        value -= j.offset;
    }
    return static_cast<int16_t>(std::clamp<int32_t>(value, INT16_MIN, INT16_MAX));
}

/* RV commands to these joints, each sent once with set_command (its kp kd
 * stay). All or nothing on range: if one does not fit its motor, none is
 * sent. A motor whose torque is off is skipped, as the write would turn it
 * on without setting its offset. */
template <size_t N>
void send_commands(const std::array<JointRef_struct, N> &refs,
                   const std::array<motor::Command_struct, N> &commands) {
    std::array<motor::Command_struct, N> raw;
    for (size_t i = 0; i < N; ++i) {
        if (!to_raw(joint(refs[i]), commands[i], raw[i])) {
            return;
        }
    }
    for (size_t i = 0; i < N; ++i) {
        motor::Motor_struct &m = joint(refs[i]).motor;
        if (m.torque) {
            (void)motor::set_command(m, raw[i]);
        }
    }
}

/* ============================= commands ============================= */

/* PING: L 0 → STATUS_FLAGS. */
void handle_ping(std::span<const uint8_t> payload) {
    if (!payload.empty()) {
        return;
    }
    uint8_t flags = rv::kStatusEsp;
    bool all_on = true;
    for (size_t i = 0; i < kStatusOrder.size(); ++i) {
        motor::Motor_struct &m = joint(kStatusOrder[i]).motor;
        all_on = all_on && m.torque;
        if (motor::ping(m, kRvReplyTimeout) == ESP_OK && m.status == 0U) {
            flags |= static_cast<uint8_t>(1U << (rv::kStatusFirstMotorBit + i));
        }
    }
    if (all_on) {
        flags |= rv::kStatusTorque;
    }
    send_frame(rv::kPing, std::span(&flags, 1));
}

/* TORQUE: L 1, 0 off / 1 on. */
void handle_torque(std::span<const uint8_t> payload) {
    if (payload.size() != 1U || payload[0] > 1U) {
        return;
    }
    (void)motor_utils::set_torque_all(payload[0] != 0U);  // a failure shows in PING
}

/* GET_FEEDBACK: L 0 → pos[4] vel[6]: positions from the torque-on pose,
 * velocities raw. */
void handle_get_feedback(std::span<const uint8_t> payload) {
    if (!payload.empty()) {
        return;
    }
    std::array<uint8_t, 20> data;
    for (size_t i = 0; i < kDataOrder.size(); ++i) {
        Joint_struct &j = joint(kDataOrder[i]);
        (void)motor::get_pos_vel(j.motor, kRvReplyTimeout);  // on error the struct keeps the last values
        if (i < kJointsWithPos) {
            put_i16(&data[2 * i], from_raw_position(j, j.motor.position));
        }
        put_i16(&data[2 * kJointsWithPos + 2 * i], j.motor.velocity);
    }
    send_frame(rv::kGetFeedback, data);
}

/* SEND_ACTION: L 12, hip L R, knee L R positions, wheel L R speeds. A
 * position joint holds its position (speed 0); a wheel turns at its speed. */
void handle_send_action(std::span<const uint8_t> payload) {
    if (payload.size() != 2U * kDataOrder.size()) {
        return;
    }
    std::array<motor::Command_struct, kDataOrder.size()> commands;
    for (size_t i = 0; i < kDataOrder.size(); ++i) {
        const int16_t value = get_i16(&payload[2 * i]);
        commands[i] = i < kJointsWithPos
                          ? motor::Command_struct{ .position = value, .speed = 0, .torque = 0 }
                          : motor::Command_struct{ .position = 0, .speed = value, .torque = 0 };
    }
    send_commands(kDataOrder, commands);
}

/* FOLD_DOWN: L 0, both hips to their fold position, through
 * motor::set_command: target position, and target speed kFoldHipSpeed with
 * the sign of that position (right +, left -). */
void handle_fold_down(std::span<const uint8_t> payload) {
    if (!payload.empty()) {
        return;
    }
    constexpr auto speed_toward = [](int16_t position) {
        return static_cast<int16_t>(position < 0 ? -kFoldHipSpeed : kFoldHipSpeed);
    };
    send_commands(kFoldJoints, std::array<motor::Command_struct, 2>{{
        { .position = kFoldCounts[0], .speed = speed_toward(kFoldCounts[0]), .torque = 0 },
        { .position = kFoldCounts[1], .speed = speed_toward(kFoldCounts[1]), .torque = 0 },
    }});
}

}  // namespace

/* ============================= receive task ============================= */

void rv_rx_task(void *) {
    FrameParser_class<kMaxRvPayload> parser;

    /* Drop whatever arrived during start-up. */
    ESP_ERROR_CHECK(uart0.flush_input());

    while (true) {
        /* A gap inside a frame means it was cut off; drop it rather than let
         * it swallow the next frame. */
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
        case rv::kPing:        handle_ping(payload); break;
        case rv::kTorque:      handle_torque(payload); break;
        case rv::kGetFeedback: handle_get_feedback(payload); break;
        case rv::kSendAction:  handle_send_action(payload); break;
        case rv::kFoldDown:    handle_fold_down(payload); break;
        default:               break;  // unknown ID: no reply, no text on this link
        }
    }
}
