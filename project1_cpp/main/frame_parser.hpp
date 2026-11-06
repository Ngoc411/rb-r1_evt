#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

/* Receives the frames of both links on UART0 (pc_link, rv_link), one byte at
 * a time:
 *   AA 55 ID LEN DATA... CS      CS = ~(ID + LEN + DATA...) & 0xFF
 * MaxPayload is the longest DATA the link uses; a longer LEN is dropped. */
template <size_t MaxPayload>
class FrameParser_class {
public:
    // Feeds one received byte; returns true when it completes a frame with a
    // valid checksum. The header resynchronizes after a split frame.
    bool push(uint8_t byte) {
        if ((used_ == 0U && byte != kHeader0) ||
            (used_ == 3U && byte > MaxPayload)) {
            used_ = (byte == kHeader0) ? 1U : 0U;
            return false;
        }
        if (used_ == 1U && byte != kHeader1) {
            used_ = (byte == kHeader0) ? 1U : 0U;
            return false;
        }
        frame_[used_++] = byte;
        if (used_ < 4U || used_ < 5U + frame_[3]) {
            return false;
        }
        used_ = 0;

        uint8_t sum = 0;
        for (size_t i = 2; i < 4U + frame_[3]; ++i) {
            sum = static_cast<uint8_t>(sum + frame_[i]);
        }
        return static_cast<uint8_t>(~sum) == frame_[4U + frame_[3]];
    }

    // True while part of a frame has been received.
    bool in_frame() const { return used_ != 0U; }
    // Drops a partly received frame.
    void reset() { used_ = 0; }

    uint8_t command() const { return frame_[2]; }
    std::span<const uint8_t> payload() const { return std::span(frame_).subspan(4, frame_[3]); }

private:
    static constexpr uint8_t kHeader0 = 0xAA;
    static constexpr uint8_t kHeader1 = 0x55;

    std::array<uint8_t, 5 + MaxPayload> frame_{};
    size_t used_ = 0;
};
