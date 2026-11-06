#include "instruction.hpp"

#include <algorithm>
#include <optional>

namespace instruction {

namespace {

constexpr uint8_t kHeader = 0xFF;

constexpr size_t kMaxParams = 253;
constexpr size_t kMaxFrame = 259;

TxHook_alias tx_hook = nullptr;
RxHook_alias rx_hook = nullptr;

std::array<SemaphoreHandle_t, UART_NUM_MAX> bus_locks = {};  // one per UART port

/* ============================= helper functions ============================= */

uint8_t checksum(std::span<const uint8_t> bytes)
{
    uint8_t sum = 0;
    for (const uint8_t byte : bytes) {
        sum = static_cast<uint8_t>(sum + byte);
    }
    return static_cast<uint8_t>(~sum);
}

/* Packs the params of WRITE_DATA and REG_WRITE: the start address, then the
 * data. Sends nothing. Returns how many bytes of params it filled, or 0 if
 * the data does not fit. */
size_t pack_address_and_data(uint8_t address, std::span<const uint8_t> data,
                             std::array<uint8_t, kMaxParams> &params)
{
    if (data.empty() || data.size() > kMaxParams - 1U) {
        return 0U;
    }
    params[0] = address;
    std::copy(data.begin(), data.end(), &params[1]);
    return data.size() + 1U;
}

}  // namespace

void set_tx_hook(TxHook_alias hook)
{
    tx_hook = hook;
}

void set_rx_hook(RxHook_alias hook)
{
    rx_hook = hook;
}

void init_locks()
{
    for (SemaphoreHandle_t &lock : bus_locks) {
        lock = xSemaphoreCreateMutex();
        configASSERT(lock != nullptr);
    }
}

void Bus_class::lock() const
{
    xSemaphoreTake(bus_locks[uart_.port()], portMAX_DELAY);
}

void Bus_class::unlock() const
{
    xSemaphoreGive(bus_locks[uart_.port()]);
}

/* Packs the full frame FF FF ID LEN INST PARAMS CS around params and sends it. */
esp_err_t Bus_class::send_frame(uint8_t id, uint8_t instruction,
                    std::span<const uint8_t> params) const
{
    if (id > kBroadcastId || params.size() > kMaxParams) {
        return ESP_ERR_INVALID_ARG;
    }

    const size_t frame_length = params.size() + 6U;
    std::array<uint8_t, kMaxFrame> frame;
    frame[0] = kHeader;
    frame[1] = kHeader;
    frame[2] = id;
    frame[3] = static_cast<uint8_t>(params.size() + 2U);
    frame[4] = instruction;
    std::copy(params.begin(), params.end(), &frame[5]);
    frame[frame_length - 1U] = checksum(std::span(frame).subspan(2, params.size() + 3U));

    const auto packet = std::span<const uint8_t>(frame).first(frame_length);
    size_t written = 0;
    esp_err_t err = uart_.write(packet, &written); // write only sends frame_length bytes, not the whole buffer
    if (err != ESP_OK) return err;
    if (written != frame_length) return ESP_FAIL;
    if (tx_hook != nullptr) tx_hook(uart_, packet);
    return ESP_OK;
}

/* Reads and checks the motor's reply to the frame just sent (unicast only). */
esp_err_t Bus_class::receive_reply(uint8_t id, uint8_t instruction, uint8_t request_length,
                       uint8_t parameter1, size_t data_length,
                       TickType_t timeout_ticks, Response_struct &response) const
{
    if (id > kMaxId || data_length > kMaxReadLength) {
        return ESP_ERR_INVALID_ARG;
    }

    std::array<uint8_t, kMaxFrame> frame;
    frame[0] = kHeader;
    frame[1] = kHeader;  // frame[2] onward is read in

    /* Ticks left before timeout_ticks expires; 0 means do not wait. */
    const TickType_t start = xTaskGetTickCount();
    auto remaining = [&]() -> std::optional<TickType_t> {
        if (timeout_ticks == 0U) return timeout_ticks;
        const TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed >= timeout_ticks) return std::nullopt;
        return timeout_ticks - elapsed;
    };

    /* Everything received, in order, for rx_hook when no reply is found. */
    std::array<uint8_t, 64> seen;
    size_t seen_count = 0;

    /* Read into frame until it holds target bytes. */
    size_t used = 0;
    auto fill = [&](size_t target) -> esp_err_t {
        while (used < target) {
            const auto ticks = remaining();
            if (!ticks) return ESP_ERR_TIMEOUT;

            size_t received = 0;
            esp_err_t err = uart_.read(std::span(frame).subspan(used, target - used),
                                       *ticks, &received);
            if (err != ESP_OK) return err;
            if (received == 0U) return ESP_ERR_TIMEOUT;
            for (size_t k = 0; k < received && seen_count < seen.size(); ++k) {
                seen[seen_count++] = frame[used + k];
            }
            used += received;
        }
        return ESP_OK;
    };

    /* Until the timeout: find FF, take the first byte after it that is not
     * FF as the ID, and check the frame. One FF is enough: a motor that
     * answers at once starts while this pin still holds the line, which hides
     * its first FF; an extra FF is a glitch as it starts sending. Anything
     * that is not the expected reply (a leftover echo, noise) is skipped, and
     * the search goes on; ID, Length and CheckSum must all match.
     *
     * Two layouts: the doc's, with the position (Length = M + 7), and the one
     * without it (Length = M + 5) that some motors send:
     *   FF FF ID LEN STATUS [POS_L POS_H] PRE_LEN INST PARAM1 DATA... CS */
    bool skipped = false;
    auto fail = [&](esp_err_t err) {
        if (rx_hook != nullptr) rx_hook(uart_, std::span(seen).first(seen_count), false);
        return (err == ESP_ERR_TIMEOUT && skipped) ? ESP_ERR_INVALID_RESPONSE : err;
    };
    size_t at = 0;  // where PRE_LEN is: 7 with the position, 5 without
    while (true) {
        bool header = false;
        while (true) {
            used = 2U;
            if (esp_err_t err = fill(3U); err != ESP_OK) return fail(err);
            if (frame[2] == kHeader) {
                header = true;
            } else if (header) {
                break;
            }
        }

        /* Then read Length, and use it to read the rest of the frame. */
        if (esp_err_t err = fill(4U); err != ESP_OK) return fail(err);
        if (frame[2] != id) {
            skipped = true;
            continue;
        }
        if (frame[3] == data_length + 7U) {
            at = 7U;
        } else if (frame[3] == data_length + 5U) {
            at = 5U;
        } else {
            skipped = true;
            continue;
        }
        const size_t target = static_cast<size_t>(frame[3]) + 4U;
        if (esp_err_t err = fill(target); err != ESP_OK) return fail(err);
        if (checksum(std::span(frame).subspan(2, target - 3U)) != frame[target - 1U] ||
            frame[at] != request_length || frame[at + 1U] != instruction ||
            frame[at + 2U] != parameter1) {
            skipped = true;
            continue;
        }
        break;
    }
    if (rx_hook != nullptr) rx_hook(uart_, std::span(seen).first(seen_count), true);

    response.id = id;
    response.length = frame[3];
    response.status = frame[4];
    response.has_position = at == 7U;
    response.position = response.has_position ? static_cast<int16_t>(frame[5] | (frame[6] << 8)) : 0;
    response.pre_len = frame[at];
    response.instruction = frame[at + 1U];
    response.parameter1 = frame[at + 2U];
    response.data_length = data_length;
    std::copy_n(&frame[at + 3U], data_length, response.data.begin());
    return ESP_OK;
}

/* send_frame, then receive_reply when response storage is given. */
esp_err_t Bus_class::send_and_receive(uint8_t id, uint8_t instruction,
                       std::span<const uint8_t> params, uint8_t parameter1,
                       size_t reply_data_length, TickType_t timeout_ticks,
                       Response_struct *response) const
{
    if (response == nullptr) {
        return send_frame(id, instruction, params);
    }
    if (id > kMaxId) {
        return ESP_ERR_INVALID_ARG;  // broadcasts never reply
    }

    /* Drop replies to earlier instructions that were never read. */
    esp_err_t err = uart_.flush_input();
    if (err != ESP_OK) return err;
    err = send_frame(id, instruction, params);
    if (err != ESP_OK) return err;
    return receive_reply(id, instruction, static_cast<uint8_t>(params.size() + 2U),
                   parameter1, reply_data_length, timeout_ticks, *response);
}

/* ============================= core functions ============================= */

esp_err_t Bus_class::ping(uint8_t id, TickType_t timeout_ticks, Response_struct &response) const
{
    return send_and_receive(id, kInstPing, {}, 0U, 0U, timeout_ticks, &response);
}

esp_err_t Bus_class::read(uint8_t id, uint8_t address, uint8_t length,
                    TickType_t timeout_ticks, Response_struct &response) const
{
    if (length == 0U || length > kMaxReadLength) {
        return ESP_ERR_INVALID_ARG;
    }
    const std::array<uint8_t, 2> params = { address, length };
    return send_and_receive(id, kInstRead, params, address, length, timeout_ticks, &response);
}

esp_err_t Bus_class::write(uint8_t id, uint8_t address, std::span<const uint8_t> data,
                     TickType_t timeout_ticks, Response_struct *response) const
{
    std::array<uint8_t, kMaxParams> params;
    const size_t length = pack_address_and_data(address, data, params);
    if (length == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    return send_and_receive(id, kInstWrite, std::span(params).first(length), address, 0U,
                   timeout_ticks, response);
}

esp_err_t Bus_class::reg_write(uint8_t id, uint8_t address, std::span<const uint8_t> data,
                         TickType_t timeout_ticks, Response_struct *response) const
{
    std::array<uint8_t, kMaxParams> params;
    const size_t length = pack_address_and_data(address, data, params); // count only bytes are filled
    if (length == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    return send_and_receive(id, kInstReg, std::span(params).first(length), address, 0U,
                   timeout_ticks, response);
}

esp_err_t Bus_class::action(uint8_t id, TickType_t timeout_ticks, Response_struct *response) const
{
    return send_and_receive(id, kInstAction, {}, 0U, 0U, timeout_ticks, response);
}

esp_err_t Bus_class::reset(uint8_t id, TickType_t timeout_ticks, Response_struct *response) const
{
    return send_and_receive(id, kInstReset, {}, 0U, 0U, timeout_ticks, response);
}

esp_err_t Bus_class::reboot(uint8_t id) const
{
    return send_frame(id, kInstReboot, {});
}

esp_err_t Bus_class::sync_write(uint8_t address, uint8_t data_length,
                          std::span<const SyncEntry_struct> entries) const
{
    if (entries.empty() || data_length == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Length byte = (L + 1) * K + 4 must fit in one byte. */
    if (entries.size() > (kMaxParams - 2U) / (1U + data_length)) {
        return ESP_ERR_INVALID_ARG;
    }

    std::array<uint8_t, kMaxParams> params;
    params[0] = address;
    params[1] = data_length;
    size_t pos = 2U; // 2: byte5 + byte6

    for (const SyncEntry_struct &entry : entries) {
        if (entry.id > kMaxId || entry.data.size() != data_length) {
            return ESP_ERR_INVALID_ARG;
        }
        params[pos++] = entry.id;
        std::copy(entry.data.begin(), entry.data.end(), &params[pos]);
        pos += data_length;
    }
    return send_frame(kBroadcastId, kInstSync, std::span(params).first(pos));
}

}  // namespace instruction
