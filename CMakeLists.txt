cmake_minimum_required(VERSION 3.13)
project(final_demo C CXX ASM)

set(PICO_BOARD pico_w)

add_executable(final_demo
    src/main.c
    src/servo.c
    src/imu.c
    src/ultrasonic.c
    src/barcode.c
)

target_include_directories(final_demo PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(final_demo
    pico_stdlib
    hardware_i2c
    hardware_pwm
    hardware_adc
    hardware_pio
    hardware_spi
    hardware_uart
)

pico_set_program_name(final_demo "final_demo")
pico_set_program_version(final_demo "0.1")
pico_enable_stdio_uart(final_demo 1)
pico_enable_stdio_usb(final_demo 1)
pico_add_extra_outputs(final_demo)
