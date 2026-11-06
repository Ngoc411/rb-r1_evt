#pragma once

#include <array>
#include <cstdint>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "UART.hpp"
#include "motor.hpp"
#include "instruction.hpp"

/* Hardware and tuning: pins, UARTs, timings, and the six joints (at the end). */

inline constexpr gpio_num_t LED1 = GPIO_NUM_38;
inline constexpr gpio_num_t LED2 = GPIO_NUM_37;

/* Speed motors start with Kp = 0, so turning torque on does not pull them to a
 * position; Kd (default 36) is then the speed gain. set_cmd_mit can change both. */
inline constexpr motor::Gains_struct kSpeedGains = { .kp = 0, .kd = 36 };

/* Which controller is on UART0 (GPIO43/44, 921600): false = the PC
 * (pc_link, uart_sender.py), true = the RV1106 / Luckfox (rv_link). Only one:
 * they share the port, and the RV link takes no text (no replies, no TX/RX
 * echo, no ESP log). */
inline constexpr bool kUseRvLink = true;

inline constexpr TickType_t kControlPeriod = pdMS_TO_TICKS(10);  // 100 Hz; CONTROL_HZ in uart_sender.py
inline constexpr TickType_t kPcStartDelay = pdMS_TO_TICKS(500);  // start until PC commands are read
inline constexpr TickType_t kReplyTimeout = pdMS_TO_TICKS(2000);   // for ping and reads
inline constexpr TickType_t kRvReplyTimeout = pdMS_TO_TICKS(20);  // rv_link's reads: a dead motor delays its reply this long
inline constexpr TickType_t kReplySettle = pdMS_TO_TICKS(20);    // >= 10 ms for a reply to finish
inline constexpr TickType_t kPcByteTimeout = pdMS_TO_TICKS(50);  // max gap inside a PC frame
inline constexpr uint32_t kLedToggleCycles = 25;                  // 500 ms -> 1 Hz blink

inline constexpr Uart_class uart1({
    .port = UART_NUM_1,
    .baud_rate = 1000000,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_control = UART_HW_FLOWCTRL_DISABLE,
    .tx_pin = GPIO_NUM_17,
    .rx_pin = GPIO_NUM_17,  // single wire: replies come back on the TX pin
    .rts_pin = GPIO_NUM_NC,
    .rx_buffer_size = 512,
    .tx_buffer_size = 0,
});

inline constexpr Uart_class uart2({
    .port = UART_NUM_2,
    .baud_rate = 1000000,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_control = UART_HW_FLOWCTRL_DISABLE,
    .tx_pin = GPIO_NUM_15,
    .rx_pin = GPIO_NUM_15,  // single wire: replies come back on the TX pin
    .rts_pin = GPIO_NUM_NC,
    .rx_buffer_size = 512,
    .tx_buffer_size = 0,
});

/* PC link through the CP2102. Shares the console UART, so the PC also
 * receives the ESP log output on TX (the boot messages before init() stay at
 * 115200). A dual test sends two LOG lines every 10 ms, about 12 KB/s, more
 * than 115200 baud carries; the TX buffer lets a reply return at once. Baud
 * must match BAUD in uart_sender.py. */
inline constexpr Uart_class uart0({
    .port = UART_NUM_0,
    .baud_rate = 921600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_control = UART_HW_FLOWCTRL_DISABLE,
    .tx_pin = GPIO_NUM_43,
    .rx_pin = GPIO_NUM_44,
    .rts_pin = GPIO_NUM_NC,
    .rx_buffer_size = 256,
    .tx_buffer_size = 4096,
});

inline constexpr instruction::Bus_class bus1(uart1);
inline constexpr instruction::Bus_class bus2(uart2);

/* ============================= the six joints ============================= */

/* How a joint is used. Position: its position 0 is where it was when torque
 * was turned on (the offset). Speed: Kp = 0, so only speed matters (Kd is the
 * speed gain), and no offset is added. */
enum class Control_enum { Position, Speed }; // must call Control_enum::Speed instead of just Speed like plain enum

/* One joint: its motor (the register mirror) and how it is mounted. The
 * motor_utils functions turn joint values into motor values with these
 * before calling the fundamental functions. */
struct Joint_struct {
    motor::Motor_struct motor;
    Control_enum control;
    int8_t direction;  // +1, or -1 when mounted mirrored
    float gear_ratio;  // motor turns per output turn; feedback speed is / this (commands are not)
    int16_t offset;    // motor position (counts) when torque was turned on
};

/* joints[0] = bus 1, joints[1] = bus 2; ids 1, 2, 3 on each. id 1's gear
 * ratio is not known yet (1.0). */
inline std::array<std::array<Joint_struct, 3>, 2> joints = {{
    {{
        { .motor = { .bus = bus1, .id = 1 }, .control = Control_enum::Speed,    .direction = -1, .gear_ratio = 1.0f, .offset = 0 },
        { .motor = { .bus = bus1, .id = 2 }, .control = Control_enum::Position, .direction = 1,  .gear_ratio = 7.5f, .offset = 0 },
        { .motor = { .bus = bus1, .id = 3 }, .control = Control_enum::Position, .direction = 1,  .gear_ratio = 9.0f, .offset = 0 },
    }},
    {{
        { .motor = { .bus = bus2, .id = 1 }, .control = Control_enum::Speed,    .direction = 1,  .gear_ratio = 1.0f, .offset = 0 },
        { .motor = { .bus = bus2, .id = 2 }, .control = Control_enum::Position, .direction = -1, .gear_ratio = 7.5f, .offset = 0 },
        { .motor = { .bus = bus2, .id = 3 }, .control = Control_enum::Position, .direction = -1, .gear_ratio = 9.0f, .offset = 0 },
    }},
}};

/* Where each joint is on the robot (rv_link): UART1 is the right leg, UART2
 * the left; on each, id 1 the wheel, id 2 the hip, id 3 the knee (crank). */
inline constexpr int kRightBus = 1;
inline constexpr int kLeftBus = 2;
inline constexpr uint8_t kWheelId = 1;
inline constexpr uint8_t kHipId = 2;
inline constexpr uint8_t kKneeId = 3;

/* FOLD_DOWN (rv_link 0x05): where each hip goes, in degrees of the output
 * shaft from the pose when torque was turned on, raw motor direction (no
 * direction sign). rv_link turns them into counts; nothing else uses degrees. */
inline constexpr int32_t kFoldRightHipDeg = 175;
inline constexpr int32_t kFoldLeftHipDeg = -175;
/* FOLD_DOWN's target speed (speed register, 1 = 1 RPM of the motor), its sign
 * that of each hip's target position: right +15, left -15. */
inline constexpr int16_t kFoldHipSpeed = 15;

/* The joint on UART number bus (1 or 2) with that motor id, or nullptr. */
inline Joint_struct *find_joint(int bus, uint8_t id) {
    for (auto &bus_joints : joints) {
        for (Joint_struct &j : bus_joints) {
            if (j.motor.bus.uart().port() == bus && j.motor.id == id) {
                return &j;
            }
        }
    }
    return nullptr;
}
