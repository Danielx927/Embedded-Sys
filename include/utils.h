/**
 * @file    utils.h
 * @brief   Utility functions
 * @author  Embedded Systems
 * @date    2025
 */

#ifndef UTILS_H
#define UTILS_H

#include "config.h"

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

/**
 * @brief Get current time in microseconds
 * @return Time in microseconds
 */
uint64_t micros_now(void);

/**
 * @brief Get current time in milliseconds
 * @return Time in milliseconds
 */
uint32_t millis_now(void);

/**
 * @brief Write a single byte to I2C register
 * @param[in] i2c  I2C instance
 * @param[in] addr Device address
 * @param[in] reg  Register address
 * @param[in] val  Value to write
 */
void i2c_write_register(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t val);

/**
 * @brief Read multiple bytes from I2C registers
 * @param[in]  i2c  I2C instance
 * @param[in]  addr Device address
 * @param[in]  reg  Starting register address
 * @param[out] buf  Buffer to store data
 * @param[in]  len  Number of bytes to read
 */
void i2c_read_registers(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);

#endif /* UTILS_H */