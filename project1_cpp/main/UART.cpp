#include "UART.hpp"

#include <algorithm>
#include <array>

#include "esp_timer.h"
#include "hal/gpio_ll.h"
#include "hal/uart_ll.h"
#include "soc/gpio_struct.h"

esp_err_t Uart_class::init() const
{
    if (config_.baud_rate <= 0 ||
        config_.tx_pin == GPIO_NUM_NC ||
        config_.rx_buffer_size <= 0 || config_.tx_buffer_size < 0 ||
        config_.flow_control != UART_HW_FLOWCTRL_DISABLE) {
        return ESP_ERR_INVALID_ARG;
    }

    uart_config_t uart_config = {};
    uart_config.baud_rate = config_.baud_rate;
    uart_config.data_bits = config_.data_bits;
    uart_config.parity = config_.parity;
    uart_config.stop_bits = config_.stop_bits;
    uart_config.flow_ctrl = config_.flow_control;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    esp_err_t err = uart_param_config(config_.port, &uart_config); // set config for real uart
    if (err != ESP_OK) {
        return err;
    }

    err = uart_driver_install(config_.port, config_.rx_buffer_size,
                              config_.tx_buffer_size, 0, nullptr, 0);
    if (err != ESP_OK) {
        return err;
    }

    err = uart_set_pin(config_.port, config_.tx_pin, config_.rx_pin,
                       config_.rts_pin, UART_PIN_NO_CHANGE);
    /* An RTS pin selects RS-485 half-duplex; without one, use normal UART. */
    if (err == ESP_OK && config_.rts_pin != GPIO_NUM_NC) {
        err = uart_set_mode(config_.port, UART_MODE_RS485_HALF_DUPLEX);
    }
    /* Single wire: uart_set_pin routed TX and RX to the same pin; keep it
     * readable, held high while idle, and released until the first write. */
    if (err == ESP_OK && single_wire()) {
        err = gpio_input_enable(config_.tx_pin);
        if (err == ESP_OK) {
            err = gpio_pullup_en(config_.tx_pin);
        }
        if (err == ESP_OK) {
            release_line();
        }
    }

    if (err != ESP_OK) {
        uart_driver_delete(config_.port);
    }
    return err;
}

esp_err_t Uart_class::write(std::span<const uint8_t> data, size_t *bytes_written) const
{
    if (bytes_written != nullptr) {
        *bytes_written = 0;
    }
    if (data.empty()) {
        return ESP_ERR_INVALID_ARG;
    }

    if (single_wire()) {
        drive_line();
    }
    const int written = uart_write_bytes(config_.port, data.data(), data.size());
    esp_err_t err = (written < 0) ? ESP_FAIL : ESP_OK;
    // On a shared line, wait for transmission to finish so the line is free
    // before the other end answers. A plain UART (the PC link) does not wait,
    // so a reply never stalls the motor task that sent it.
    if (err == ESP_OK && shared_line()) {
        err = single_wire() ? wait_tx_idle(static_cast<size_t>(written))
                            : uart_wait_tx_done(config_.port, pdMS_TO_TICKS(1000));
    }
    if (single_wire()) {
        // Free the pin before the other end answers, then drop the echo of
        // what was just sent. Flushing instead would also drop the start of a
        // reply that comes quickly.
        release_line();
        if (err == ESP_OK) {
            err = drop_echo(static_cast<size_t>(written));
        }
    }
    if (written < 0) {
        return ESP_FAIL;
    }
    if (bytes_written != nullptr) {
        *bytes_written = static_cast<size_t>(written);
    }
    return err;
}

/* TX stays routed to the pin through the GPIO matrix; only who controls its
 * output enable changes: the UART (always on) or GPIO_ENABLE (cleared). */
void Uart_class::drive_line() const
{
    gpio_ll_set_output_enable_ctrl(&GPIO, config_.tx_pin, true, false);
}

void Uart_class::release_line() const
{
    gpio_ll_output_disable(&GPIO, config_.tx_pin);
    gpio_ll_set_output_enable_ctrl(&GPIO, config_.tx_pin, false, false);
}

esp_err_t Uart_class::wait_tx_idle(size_t length) const
{
    // Twice the frame's time on the wire, plus 1 ms.
    const int64_t limit_us = esp_timer_get_time() + 1000 +
                             static_cast<int64_t>(length) * 20'000'000 / config_.baud_rate;
    uart_dev_t *hw = UART_LL_GET_HW(config_.port);
    while (!uart_ll_is_tx_idle(hw)) {
        if (esp_timer_get_time() > limit_us) {
            return ESP_ERR_TIMEOUT;
        }
    }
    return ESP_OK;
}

esp_err_t Uart_class::drop_echo(size_t length) const
{
    // The echo is complete once the stop bit is out, but reaches the read
    // buffer only after the RX idle timeout (about 10 bytes long).
    constexpr TickType_t kEchoTimeout = 2;
    std::array<uint8_t, 32> scratch;
    while (length > 0U) {
        const int received = uart_read_bytes(config_.port, scratch.data(),
                                             static_cast<uint32_t>(std::min(length, scratch.size())),
                                             kEchoTimeout);
        if (received < 0) {
            return ESP_FAIL;
        }
        if (received == 0) {
            return ESP_OK;  // echo missing; a reader skips what is left of it
        }
        length -= static_cast<size_t>(received);
    }
    return ESP_OK;
}

esp_err_t Uart_class::read(std::span<uint8_t> buffer, TickType_t timeout_ticks,
                     size_t *bytes_read) const
{
    if (bytes_read != nullptr) {
        *bytes_read = 0;
    }
    if (buffer.empty()) {
        return ESP_ERR_INVALID_ARG;
    }

    const int received = uart_read_bytes(config_.port, buffer.data(),
                                         static_cast<uint32_t>(buffer.size()),
                                         timeout_ticks);
    if (received < 0) {
        return ESP_FAIL;
    }
    if (bytes_read != nullptr) {
        *bytes_read = static_cast<size_t>(received);
    }
    return ESP_OK;
}

esp_err_t Uart_class::flush_input() const
{
    return uart_flush_input(config_.port);
}
