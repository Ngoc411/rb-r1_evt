#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

struct UartConfig_struct {
    uart_port_t port;   // define COM
    int baud_rate;      // baud rate - default 1Mbs
    uart_word_length_t data_bits;   // data length - in bit
    uart_parity_t parity;           // parity - default no parity
    uart_stop_bits_t stop_bits;     // default 1 stop bit
    uart_hw_flowcontrol_t flow_control;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;  // == tx_pin: single-wire half duplex on one pin
    gpio_num_t rts_pin; // use for half duplex mode with an RS-485 transceiver
    int rx_buffer_size;
    int tx_buffer_size;
};

class Uart_class {
public:
    // another way to write: 
    // Uart(const UartConfig &config) {
    //     config_ = config;      // copy the parameter into the member
    // }

    constexpr explicit Uart_class(const UartConfig_struct &config) : config_(config) {} // constructor

    // Installs the UART. A configured RTS pin enables RS-485 half-duplex mode;
    // GPIO_NUM_NC leaves the UART in normal mode. rx_pin == tx_pin selects
    // single-wire mode: the pin is driven only while sending and is otherwise
    // released (with pull-up) so the other end can answer on it.
    esp_err_t init() const;

    // Queues bytes for transmission. On a shared line (single wire or RS-485)
    // it also waits until the last stop bit is sent, so the other end can
    // answer; a plain UART returns once the bytes are queued.
    // In single-wire mode the pin also receives what is sent; that echo is
    // discarded, so reads return only what the other end sends.
    esp_err_t write(std::span<const uint8_t> data,
                    size_t *bytes_written = nullptr) const;

    // Reads up to buffer.size() bytes. A timeout with no data is a successful
    // read with *bytes_read set to zero.
    esp_err_t read(std::span<uint8_t> buffer, TickType_t timeout_ticks,
                   size_t *bytes_read = nullptr) const;

    // Discards bytes received but not yet read.
    esp_err_t flush_input() const;

    uart_port_t port() const { return config_.port; }
    const UartConfig_struct &config() const { return config_; }
    bool single_wire() const { return config_.rx_pin == config_.tx_pin; }
    bool shared_line() const { return single_wire() || config_.rts_pin != GPIO_NUM_NC; }

private:
    // Single-wire mode: hand the pin to the UART's TX output, or let it go.
    void drive_line() const;
    void release_line() const;
    // Single-wire mode: spins until the last stop bit of the length bytes is
    // out. uart_wait_tx_done sleeps on an interrupt instead, and the tens of
    // us it takes to wake drown a motor that answers at once.
    esp_err_t wait_tx_idle(size_t length) const;
    // Single-wire mode: reads and drops the length bytes just sent, and no
    // more, so a reply that follows at once is kept.
    esp_err_t drop_echo(size_t length) const;

    UartConfig_struct config_;
};
