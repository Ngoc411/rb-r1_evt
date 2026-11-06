#include "motor.hpp"

#include <array>

#include "little_endian.hpp"  // registers are little-endian

namespace motor {

namespace {

// pack Kp and Kd into 4 bytes
void put_gains(uint8_t *bytes, const Gains_struct &gains)
{
    put_u16(&bytes[0], gains.kp);
    put_u16(&bytes[2], gains.kd);
}

// pack Pos, Vel, Tff into 6 bytes
void put_command(uint8_t *bytes, const Command_struct &command)
{
    put_u16(&bytes[0], static_cast<uint16_t>(command.position));
    put_u16(&bytes[2], static_cast<uint16_t>(command.speed));
    put_u16(&bytes[4], static_cast<uint16_t>(command.torque));
}

// unpack 4 bytes into Kp and Kd
Gains_struct take_gains(const uint8_t *bytes)
{
    return { .kp = get_u16(&bytes[0]), .kd = get_u16(&bytes[2]) };
}

/* Every reply carries the status flags, and most the present position. */
void take_reply(Motor_struct &m, const instruction::Response_struct &response)
{
    m.status = response.status;
    if (response.has_position) {
        m.position = response.position;
    }
}

/* READ length bytes from address; on success also takes the reply's status
 * and position into m. */
esp_err_t read(Motor_struct &m, uint8_t address, uint8_t length, TickType_t timeout_ticks,
               instruction::Response_struct &response)
{
    const esp_err_t err = m.bus.read(m.id, address, length, timeout_ticks, response);
    if (err == ESP_OK) {
        take_reply(m, response);
    }
    return err;
}

}  // namespace

/* ============================= set ============================= */

esp_err_t set_command(Motor_struct &m, const Command_struct &command)
{
    instruction::BusLock_class lock(m.bus);  // the whole exchange, so no other frame cuts in
    m.command = command;
    m.torque = true;  // a write that holds the goal position turns torque on
    std::array<uint8_t, 6> data;
    put_command(data.data(), m.command);
    return m.bus.write(m.id, reg::kGoalPosition, data);
}

esp_err_t set_gains(Motor_struct &m, const Gains_struct &gains)
{
    instruction::BusLock_class lock(m.bus);  // the whole exchange, so no other frame cuts in
    m.gains = gains;
    std::array<uint8_t, 4> data;
    put_gains(data.data(), m.gains);
    return m.bus.write(m.id, reg::kKp, data);
}

esp_err_t set_gains_and_command(Motor_struct &m, const Gains_struct &gains, const Command_struct &command)
{
    instruction::BusLock_class lock(m.bus);  // the whole exchange, so no other frame cuts in
    m.gains = gains;
    m.command = command;
    m.torque = true;  // a write that holds the goal position turns torque on
    std::array<uint8_t, 10> data;
    put_gains(&data[0], m.gains);
    put_command(&data[4], m.command);
    return m.bus.write(m.id, reg::kKp, data);
}

esp_err_t set_torque(Motor_struct &m, bool on)
{
    instruction::BusLock_class lock(m.bus);  // the whole exchange, so no other frame cuts in
    m.torque = on;
    const std::array<uint8_t, 1> data = { static_cast<uint8_t>(m.torque ? 1U : 0U) };
    return m.bus.write(m.id, reg::kTorqueEnable, data);
}

esp_err_t set_mode(Motor_struct &m, Mode_enum mode)
{
    instruction::BusLock_class lock(m.bus);  // the whole exchange, so no other frame cuts in
    m.mode = mode;
    const std::array<uint8_t, 1> data = { static_cast<uint8_t>(m.mode) };
    return m.bus.write(m.id, reg::kControlMode, data);
}

esp_err_t set_gains_limit(Motor_struct &m, const Gains_struct &limits)
{
    instruction::BusLock_class lock(m.bus);  // the whole exchange, so no other frame cuts in
    m.limits = limits;
    std::array<uint8_t, 4> data;
    put_gains(data.data(), m.limits);  // Kp_max, Kd_max are laid out like Kp, Kd
    return m.bus.write(m.id, reg::kKpMax, data);
}

/* ============================= setup ============================= */

esp_err_t set_reply_level(Motor_struct &m, ReplyLevel_enum level)
{
    instruction::BusLock_class lock(m.bus);  // the whole exchange, so no other frame cuts in
    const std::array<uint8_t, 1> data = { static_cast<uint8_t>(level) };
    return m.bus.write(m.id, reg::kStatusReturnLevel, data);
}

/* ============================= get ============================= */

esp_err_t ping(Motor_struct &m, TickType_t timeout_ticks)
{
    instruction::BusLock_class lock(m.bus);  // the whole exchange, so no other frame cuts in
    instruction::Response_struct response;
    const esp_err_t err = m.bus.ping(m.id, timeout_ticks, response);
    if (err == ESP_OK) {
        take_reply(m, response);
    }
    return err;
}

esp_err_t get_pos_vel(Motor_struct &m, TickType_t timeout_ticks)
{
    instruction::BusLock_class lock(m.bus);  // the whole exchange, so no other frame cuts in
    instruction::Response_struct response;
    const esp_err_t err = read(m, reg::kPresentPosition, 4U, timeout_ticks, response);
    if (err != ESP_OK) return err;

    m.position = get_i16(&response.data[0]);
    m.velocity = get_i16(&response.data[2]);
    return ESP_OK;
}

esp_err_t get_gains(Motor_struct &m, TickType_t timeout_ticks)
{
    instruction::BusLock_class lock(m.bus);  // the whole exchange, so no other frame cuts in
    instruction::Response_struct response;
    const esp_err_t err = read(m, reg::kKp, 4U, timeout_ticks, response);
    if (err != ESP_OK) return err;

    m.gains = take_gains(&response.data[0]);
    return ESP_OK;
}

esp_err_t get_mode(Motor_struct &m, TickType_t timeout_ticks)
{
    instruction::BusLock_class lock(m.bus);  // the whole exchange, so no other frame cuts in
    instruction::Response_struct response;
    const esp_err_t err = read(m, reg::kControlMode, 1U, timeout_ticks, response);
    if (err != ESP_OK) return err;

    m.mode = static_cast<Mode_enum>(response.data[0]);
    return ESP_OK;
}

esp_err_t get_torque(Motor_struct &m, TickType_t timeout_ticks)
{
    instruction::BusLock_class lock(m.bus);  // the whole exchange, so no other frame cuts in
    instruction::Response_struct response;
    const esp_err_t err = read(m, reg::kTorqueEnable, 1U, timeout_ticks, response);
    if (err != ESP_OK) return err;

    m.torque = response.data[0] != 0U;
    return ESP_OK;
}

esp_err_t get_all(Motor_struct &m, TickType_t timeout_ticks)
{
    instruction::BusLock_class lock(m.bus);  // the whole exchange, so no other frame cuts in
    constexpr uint8_t kLength = reg::kRamEnd - reg::kTorqueEnable + 1U;  // 30
    instruction::Response_struct response;
    const esp_err_t err = read(m, reg::kTorqueEnable, kLength, timeout_ticks, response);
    if (err != ESP_OK) return err;

    // the byte of register address in the reply
    auto at = [&](uint8_t address) { return &response.data[address - reg::kTorqueEnable]; };
    m.torque = *at(reg::kTorqueEnable) != 0U;
    m.gains = take_gains(at(reg::kKp));
    m.command = { .position = get_i16(at(reg::kGoalPosition)),
                  .speed = get_i16(at(reg::kGoalSpeed)),
                  .torque = get_i16(at(reg::kFeedforwardTorque)) };
    m.position = get_i16(at(reg::kPresentPosition));
    m.velocity = get_i16(at(reg::kPresentSpeed));
    m.current = get_i16(at(reg::kPresentCurrent));
    m.voltage = *at(reg::kPresentVoltage);
    m.temperature = *at(reg::kPresentTemperature);
    m.reg_pending = *at(reg::kRegWritePending) != 0U;
    m.mode = static_cast<Mode_enum>(*at(reg::kControlMode));
    m.moving = *at(reg::kMoving) != 0U;
    m.eeprom_locked = *at(reg::kEepromLock) != 0U;
    m.punch = get_u16(at(reg::kPunch));
    m.limits = take_gains(at(reg::kKpMax));  // Kp_max, Kd_max are laid out like Kp, Kd
    return ESP_OK;
}

}  // namespace motor
