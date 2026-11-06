#pragma once

#include <cstdint>

/* 16-bit values as 2 bytes, low byte first: the order of both the motor
 * registers and the PC frames. */

// write a uint16_t as 2 bytes
inline void put_u16(uint8_t *bytes, uint16_t value) {
    bytes[0] = static_cast<uint8_t>(value & 0xFFU);
    bytes[1] = static_cast<uint8_t>(value >> 8);
}

// write an int16_t as 2 bytes
inline void put_i16(uint8_t *bytes, int16_t value) {
    put_u16(bytes, static_cast<uint16_t>(value));
}

// turn 2 bytes to uint16_t
inline uint16_t get_u16(const uint8_t *bytes) {
    return static_cast<uint16_t>(bytes[0] | (bytes[1] << 8));
}

// turn 2 bytes to int16_t
inline int16_t get_i16(const uint8_t *bytes) {
    return static_cast<int16_t>(get_u16(bytes));
}
