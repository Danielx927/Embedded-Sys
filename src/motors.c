#include "motors.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <math.h>
#include <stdint.h>
#include "config.h"

void setup_pwm(uint32_t gpio)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice_num, PWM_WRAP);
    pwm_set_clkdiv(slice_num, PWM_CLKDIV);  /* Adjust clock divider to reduce frequency */
    pwm_set_enabled(slice_num, true);
    pwm_set_gpio_level(gpio, 0);  /* Start at 0 duty */
}

void motors_init(void) {
    setup_pwm(PWM_M1A);
    setup_pwm(PWM_M1B);
    setup_pwm(PWM_M2A);
    setup_pwm(PWM_M2B);
}

void motors_stop(void) {
    motors_set_speed(0.0f, 0.0f);
}

void motors_set_speed(float left_duty, float right_duty) {
    uint16_t left_level = (uint16_t)(fabsf(left_duty) * (float)PWM_WRAP);
    uint16_t right_level = (uint16_t)(fabsf(right_duty) * (float)PWM_WRAP);

    if (right_duty > 0.0f) {
        pwm_set_gpio_level(PWM_M1A, right_level);
        pwm_set_gpio_level(PWM_M1B, 0);
    } else if (right_duty < 0.0f) {
        pwm_set_gpio_level(PWM_M1A, 0);
        pwm_set_gpio_level(PWM_M1B, right_level);
    } else {
        pwm_set_gpio_level(PWM_M1A, 0);
        pwm_set_gpio_level(PWM_M1B, 0);
    }

    if (left_duty > 0.0f) {
        pwm_set_gpio_level(PWM_M2A, left_level);
        pwm_set_gpio_level(PWM_M2B, 0);
    } else if (left_duty < 0.0f) {
        pwm_set_gpio_level(PWM_M2A, 0);
        pwm_set_gpio_level(PWM_M2B, left_level);
    } else {
        pwm_set_gpio_level(PWM_M2A, 0);
        pwm_set_gpio_level(PWM_M2B, 0);
    }
}