#include "encoders.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include <stdint.h>
#include <stdbool.h>
#include "config.h"


volatile uint32_t g_left_pulses = 0;
volatile uint32_t g_right_pulses = 0;

void encoders_init(void) {
    gpio_init(LEFT_ENCODER);
    gpio_set_dir(LEFT_ENCODER, GPIO_IN);
    gpio_pull_up(LEFT_ENCODER);
    gpio_set_irq_enabled(LEFT_ENCODER, GPIO_IRQ_EDGE_RISE, true);
    
    gpio_init(RIGHT_ENCODER);
    gpio_set_dir(RIGHT_ENCODER, GPIO_IN);
    gpio_pull_up(RIGHT_ENCODER);
    gpio_set_irq_enabled(RIGHT_ENCODER, GPIO_IRQ_EDGE_RISE, true);
}

uint32_t encoders_get_left_pulses(void) { return g_left_pulses; }
uint32_t encoders_get_right_pulses(void) { return g_right_pulses; }
void encoders_reset(void) { g_left_pulses = 0; g_right_pulses = 0; }

void encoder_callback(uint gpio, uint32_t events)
{
    if (gpio == LEFT_ENCODER) {
        g_left_pulses++;
    } else if (gpio == RIGHT_ENCODER) {
        g_right_pulses++;
    }
}

void encoders_filter_init(speed_filter_t * const p_filter)
{
    uint8_t i;
    for (i = 0U; i < SPEED_FILTER_WINDOW; i++)
    {
        p_filter->buffer[0][i] = 0.0f;
        p_filter->buffer[1][i] = 0.0f;
    }
    p_filter->index = 0U;
    p_filter->full = FALSE;
}

void encoders_filter_update(speed_filter_t * const p_filter,
                                float left_speed,
                                float right_speed,
                                float *filtered_left,
                                float *filtered_right)
{
    uint8_t idx = p_filter->index;
    uint8_t i;
    float left_sum = 0.0f;
    float right_sum = 0.0f;
    uint8_t divisor;

    /* Store new samples */
    p_filter->buffer[0][idx] = left_speed;
    p_filter->buffer[1][idx] = right_speed;

    /* Update index */
    p_filter->index = (p_filter->index + 1) % SPEED_FILTER_WINDOW;
    if (p_filter->index == 0U) {
        p_filter->full = TRUE;
    }

    /* Calculate sum */
    divisor = p_filter->full ? SPEED_FILTER_WINDOW : (p_filter->index + 1);
    for (i = 0U; i < divisor; i++)
    {
        left_sum += p_filter->buffer[0][i];
        right_sum += p_filter->buffer[1][i];
    }

    /* Calculate average */
    *filtered_left = left_sum / (float)divisor;
    *filtered_right = right_sum / (float)divisor;
}

