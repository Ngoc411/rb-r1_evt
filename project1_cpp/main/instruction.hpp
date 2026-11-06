#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "UART.hpp"

/* Packet layer of the joint motor protocol: FF FF ID LEN INST PARAMS... CS.
 * Knows the instruction set but not what the registers mean; see motor.hpp. */

namespace instruction {

/* Instruction codes, byte 4 of a frame. */
inline constexpr uint8_t kInstPing = 0x01;
inline constexpr uint8_t kInstRead = 0x02;
inline constexpr uint8_t kInstWrite = 0x03;
inline constexpr uint8_t kInstReg = 0x04;
inline constexpr uint8_t kInstAction = 0x05;
inline constexpr uint8_t kInstReset = 0x06;
inline constexpr uint8_t kInstReboot = 0x64;
inline constexpr uint8_t kInstSync = 0x83;

inline constexpr uint8_t kBroadcastId = 0xFE;
inline constexpr uint8_t kMaxId = 0xFD;
inline constexpr size_t kMaxReadLength = 248;

struct Response_struct {
    uint8_t id;
    uint8_t length;
    uint8_t status;       /* Driver status flags from response byte 4. */
    bool has_position;    /* False for motors whose replies leave it out. */
    int16_t position;     /* Output shaft position in counts, if has_position. */
    uint8_t pre_len;
    uint8_t instruction;
    uint8_t parameter1;   /* Echoed register address, or zero. */
    std::array<uint8_t, kMaxReadLength> data; /* READ_DATA payload, when requested. */
    size_t data_length; // unit: byte
};

struct SyncEntry_struct {
    uint8_t id;
    std::span<const uint8_t> data;  /* Exactly data_length bytes. */
};

/* Called with every complete instruction frame after it is sent. */
using TxHook_alias = void (*)(const Uart_class &uart, std::span<const uint8_t> frame);
void set_tx_hook(TxHook_alias hook);

/* Called after every wait for a reply with the bytes received meanwhile (the
 * first 64), and whether the reply was found in them. */
using RxHook_alias = void (*)(const Uart_class &uart, std::span<const uint8_t> bytes, bool found);
void set_rx_hook(RxHook_alias hook);

/* Creates one lock per UART; call once at start-up, before any task uses a Bus. */
void init_locks();

/* Motors sharing one UART. PING and READ require response storage. For WRITE,
 * REG_WRITE, ACTION, and RESET, response may be nullptr to send without
 * reading a reply. The motor's Status Return Level register (0x10) controls
 * whether it transmits one; when it is 2, supply response storage to consume
 * the reply. Broadcasts do not reply. timeout_ticks is measured in FreeRTOS
 * ticks; 0 means do not wait, which always times out since replies arrive
 * after the 0x05 delay. Reading a reply needs the UART's RX pin wired. */
class Bus_class {
public:
    constexpr explicit Bus_class(const Uart_class &uart) : uart_(uart) {}

    esp_err_t ping(uint8_t id, TickType_t timeout_ticks, Response_struct &response) const;
    esp_err_t read(uint8_t id, uint8_t address, uint8_t length,
                   TickType_t timeout_ticks, Response_struct &response) const;
    /* Writes data to consecutive registers starting at address. */
    esp_err_t write(uint8_t id, uint8_t address, std::span<const uint8_t> data,
                    TickType_t timeout_ticks = 0,
                    Response_struct *response = nullptr) const;
    /* Like write, but the motor holds the data until ACTION. */
    esp_err_t reg_write(uint8_t id, uint8_t address, std::span<const uint8_t> data,
                        TickType_t timeout_ticks = 0,
                        Response_struct *response = nullptr) const;
    esp_err_t action(uint8_t id, TickType_t timeout_ticks = 0,
                     Response_struct *response = nullptr) const;
    /* Restores factory settings. */
    esp_err_t reset(uint8_t id, TickType_t timeout_ticks = 0,
                    Response_struct *response = nullptr) const;
    /* Saves settings to flash and restarts the motor; never replies. */
    esp_err_t reboot(uint8_t id) const;
    /* Writes data_length bytes at address to each entry's motor, in one
     * broadcast frame; never replies. */
    esp_err_t sync_write(uint8_t address, uint8_t data_length,
                         std::span<const SyncEntry_struct> entries) const;

    const Uart_class &uart() const { return uart_; }

    /* Held by the motor functions for one whole exchange (edit + send, or
     * send + fill), so tasks sharing this bus never interleave frames. */
    void lock() const;
    void unlock() const;

private:
    // send_frame, then receive_reply if response storage is given.
    esp_err_t send_and_receive(uint8_t id, uint8_t instruction,
                               std::span<const uint8_t> params, uint8_t parameter1,
                               size_t reply_data_length, TickType_t timeout_ticks,
                               Response_struct *response) const;
    // Packs FF FF ID LEN INST PARAMS CS around params and sends it on the UART.
    esp_err_t send_frame(uint8_t id, uint8_t instruction,
                         std::span<const uint8_t> params) const;
    // Reads the motor's reply to the frame just sent and checks it.
    esp_err_t receive_reply(uint8_t id, uint8_t instruction, uint8_t request_length,
                            uint8_t parameter1, size_t data_length,
                            TickType_t timeout_ticks, Response_struct &response) const;

    const Uart_class &uart_;
};

/* Holds a bus's lock from construction to the end of the scope. */
class BusLock_class {
public:
    explicit BusLock_class(const Bus_class &bus) : bus_(bus) { bus_.lock(); }
    ~BusLock_class() { bus_.unlock(); }
    BusLock_class(const BusLock_class &) = delete;
    BusLock_class &operator=(const BusLock_class &) = delete;

private:
    const Bus_class &bus_;
};

}  // namespace instruction
