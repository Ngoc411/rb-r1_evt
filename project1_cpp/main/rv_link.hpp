#pragma once

#include <cstdint>

/* The UART0 link to the RV1106 / Luckfox (HCR1, doc "ESP - RV"), used instead
 * of pc_link when kUseRvLink is set. It receives a command, does it at once
 * (nothing is repeated by the 100 Hz loop), and replies with a binary frame
 * if the command has a reply. Nothing else is sent on UART0: no text.
 *
 * Frame, both ways: AA 55 ID L DATA... CS, CS = ~(ID + L + DATA...) & 0xFF.
 * int16 values are little-endian. Every value is the raw register value: no
 * rad, deg or RPM conversion, and no direction sign (mirrored joints are not
 * flipped, unlike pc_link):
 *   pos  counts: 16384 per output-shaft turn; for hips and knees 0 = the pose
 *        when torque was turned on (the offset is added / subtracted).
 *   vel  the speed register (1 = 1 RPM of the motor, before the gear ratio).
 * Joints: left = UART2, right = UART1; hip id 2, knee (crank) id 3, wheel
 * id 1 (config.hpp). A bad frame, a wrong L or an unknown ID gets no reply. */

namespace rv {
/* L 0 → reply L 1: STATUS_FLAGS (bits below). Pings all six motors. */
inline constexpr uint8_t kPing = 0x01;
/* L 1: 0 every motor's torque off, 1 on (each holds where it is, and that
 * pose is its position 0; if one does not answer, all stay off). No reply. */
inline constexpr uint8_t kTorque = 0x02;
/* L 0 → reply L 20: int16 pos[4] = left hip, right hip, left knee, right knee;
 * then int16 vel[6] = left hip, right hip, left knee, right knee, left wheel,
 * right wheel. A motor that does not answer keeps its last values. */
inline constexpr uint8_t kGetFeedback = 0x03;
/* L 12: int16 left hip, right hip, left knee, right knee (pos), left wheel,
 * right wheel (vel). Sent once to each motor whose torque is on; if a
 * position does not fit the motor's range, nothing is sent. No reply. */
inline constexpr uint8_t kSendAction = 0x04;
/* L 0: both hips to kFoldRightHipDeg / kFoldLeftHipDeg (config.hpp), turned
 * into counts, with target speed kFoldHipSpeed signed like each target
 * (right +15, left -15). Only a hip whose torque is on moves. No reply. */
inline constexpr uint8_t kFoldDown = 0x05;

/* STATUS_FLAGS: 1 = normal / on. */
inline constexpr uint8_t kStatusEsp = 1U << 0;     // the ESP32 (always 1: no fault state yet)
inline constexpr uint8_t kStatusTorque = 1U << 1;  // every motor's torque is on
/* Bits 2..7, one per motor (answers a ping, no error flags): right (UART1)
 * hip, knee, wheel, then left (UART2) hip, knee, wheel. */
inline constexpr unsigned kStatusFirstMotorBit = 2;
}  // namespace rv

/* Reads RV frames and does them. Started after kPcStartDelay. */
void rv_rx_task(void *);
