idf_component_register(
    SRCS "motor_utils.cpp" "main.cpp" "pc_link.cpp" "rv_link.cpp" "instruction.cpp" "motor.cpp" "UART.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_uart esp_driver_spi esp_timer
)
