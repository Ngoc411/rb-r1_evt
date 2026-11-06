#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.hpp"
#include "motor_utils.hpp"
#include "esp_log.h"
#include "pc_link.hpp"
#include "rv_link.hpp"
#include "instruction.hpp"

/* uart_sender.py on the PC controls everything over USB (sim mode).
 *  - Config mode (the start-up state): nothing is sent to the motors except
 *    what the PC asks for (setgains, setmode, ping, getall, ...).
 *  - Action mode: every motor whose torque is on (settorque, or torque on for
 *    all; it holds where it is, and that pose is its position 0) takes the
 *    motion commands. Each is sent once and the motor keeps it; the 100 Hz
 *    loop only reads feedback for the running tests (test_motor*).
 *    Back to config mode turns every motor's torque off.
 * PC commands are accepted after kPcStartDelay.
 * With kUseRvLink (config.hpp) the RV1106 takes UART0 instead (rv_link):
 * binary commands done at once, binary replies, no text.
 *
 * Files:
 *   config.hpp         pins, UARTs, timings, the six joints (Joint_struct, joints)
 *   motor.*            fundamental: Motor_struct (one motor's registers) and
 *                      its set / get functions (motor::)
 *   motor_utils.*      non-fundamental: joint <-> motor values, modes,
 *                      torque on/off, the 100 Hz loop, tests (motor_utils::)
 *   pc_link.*          the USB link to uart_sender.py: receive, call, reply
 *   rv_link.*          the UART0 link to the RV1106 (HCR1), instead of pc_link
 *   frame_parser.hpp   receives AA 55 ID LEN DATA CS frames (both links)
 *   instruction.*      the packet layer on the motor bus (instruction::)
 *   UART.*             the ports */

extern "C" void app_main(void) {
    if constexpr (kUseRvLink) {
        esp_log_level_set("*", ESP_LOG_NONE);  // UART0 carries only HCR1 frames
    }
    ESP_ERROR_CHECK(uart1.init());
    ESP_ERROR_CHECK(uart2.init());
    ESP_ERROR_CHECK(uart0.init());
    instruction::init_locks();

    gpio_config_t config = {};
    config.pin_bit_mask = (1ULL << LED1) | (1ULL << LED2);
    config.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&config));
    ESP_ERROR_CHECK(gpio_set_level(LED1, 0));
    ESP_ERROR_CHECK(gpio_set_level(LED2, 0));

    /* The motor task turns every motor's torque off, then starts in config
     * mode: it sends nothing the PC does not ask for. */
    motor_utils::start();

    vTaskDelay(kPcStartDelay);
    if constexpr (kUseRvLink) {
        configASSERT(xTaskCreate(rv_rx_task, "rv_rx_task", 4096,
                                 nullptr, 6, nullptr) == pdPASS);
    } else {
        configASSERT(xTaskCreate(pc_rx_task, "pc_rx_task", 4096,
                                 nullptr, 6, nullptr) == pdPASS);
    }
}
