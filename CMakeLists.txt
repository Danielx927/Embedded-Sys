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
    src/telemetry.c
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
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mqtt
)

pico_set_program_name(final_demo "final_demo")
pico_set_program_version(final_demo "0.1")
pico_enable_stdio_uart(final_demo 1)
pico_enable_stdio_usb(final_demo 1)
pico_add_extra_outputs(final_demo)

# List of test executables
set(TEST_FILES B1 I1 I2 L1 L2 M1 M2-L M2-R M2-U O1 O2 Q1 UC01 UC02 UC03 UC04 UC05 UC06 UC07 UC08 UC09 UC10 UC11)

# Add each test as a separate executable
foreach(TEST_NAME ${TEST_FILES})
    add_executable(${TEST_NAME} test/${TEST_NAME}.c)
    target_include_directories(${TEST_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
    target_link_libraries(${TEST_NAME}
        pico_stdlib
        pico_multicore
        pico_sync
        hardware_i2c
        hardware_pwm
        hardware_gpio
        hardware_irq
        hardware_adc
    )
    pico_set_program_name(${TEST_NAME} "${TEST_NAME}")
    pico_set_program_version(${TEST_NAME} "0.1")
    pico_enable_stdio_uart(${TEST_NAME} 1)
    pico_enable_stdio_usb(${TEST_NAME} 1)
    pico_add_extra_outputs(${TEST_NAME})
endforeach()
