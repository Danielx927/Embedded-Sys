#include "utils.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdint.h>

uint64_t micros_now(void) {
    return to_us_since_boot(get_absolute_time());
}

uint32_t millis_now(void) {
    return (uint32_t)(micros_now() / 1000);
}

void i2c_write_register(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(i2c, addr, buf, 2, false);
}

void i2c_read_registers(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    i2c_write_blocking(i2c, addr, &reg, 1, true);
    i2c_read_blocking(i2c, addr, buf, len, false);
}