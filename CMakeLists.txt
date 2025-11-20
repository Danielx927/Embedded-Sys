cmake_minimum_required(VERSION 3.13)
project(final_demo C CXX ASM)

set(PICO_BOARD pico_w)

add_executable(final_demo
    src/main.c
    src/imu.c
    src/motors.c
    src/encoders.c
    src/line_sensor.c
    src/barcode.c
    src/ultrasonic.c
    src/navigation.c
    src/multicore_obstacle.c
    src/utils.c
)

target_include_directories(final_demo PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(final_demo
    pico_stdlib
    pico_multicore
    pico_sync
    hardware_i2c
    hardware_pwm
    hardware_gpio
    hardware_irq
    hardware_adc
)

pico_set_program_name(final_demo "final_demo")
pico_set_program_version(final_demo "0.1")
pico_enable_stdio_uart(final_demo 1)
pico_enable_stdio_usb(final_demo 1)
pico_add_extra_outputs(final_demo)
