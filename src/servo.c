#include "servo.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include "config.h"

// Initialize the servo (sets up PWM slice and default center pulse)
void init_servo(void) {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_set_clkdiv(slice, 125.0f);
    pwm_set_wrap(slice, 20000);
    pwm_set_enabled(slice, true);

    set_servo_pulse(SERVO_CENTER);
    sleep_ms(700);
}

// Set servo position by pulse width (us)
void set_servo_pulse(uint16_t us) {
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;

    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    uint chan  = pwm_gpio_to_channel(SERVO_PIN);
    uint16_t level = (us * 20000) / 20000;
    pwm_set_chan_level(slice, chan, level);
}

// Convert a pulse width to an angle in degrees based on calibration constants
float servo_pulse_to_angle(uint16_t pulse_us) {
    int16_t diff = pulse_us - SERVO_CENTER;
    return (float)diff * SERVO_DEG_PER_PULSE;
}
