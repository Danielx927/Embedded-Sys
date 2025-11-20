#include "line_sensor.h"
#include "hardware/adc.h"
#include <stdint.h>
#include "config.h"


uint16_t line_history[2] = {1600, 1600};
uint8_t line_idx = 0;

void line_sensor_init(void) {
    adc_init();
    adc_gpio_init(LEFT_IR_ADC_GPIO);
}

uint16_t line_sensor_read_filtered(void) {
    adc_select_input(LEFT_IR_ADC_CHANNEL);
    uint16_t line_raw = adc_read();
    
    line_history[line_idx] = line_raw;
    line_idx = (line_idx + 1) % 2;
    
    return (line_history[0] + line_history[1]) / 2;
}

float line_sensor_compute_error(uint16_t sensor_value) {
    return (float)(sensor_value - LINE_SETPOINT) / LINE_SCALE;
}