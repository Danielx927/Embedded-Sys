/**
 * @file    barcode.h
 * @brief   Barcode scanning and decoding interface
 * @author  Embedded Systems
 * @date    2025
 */

#ifndef BARCODE_H
#define BARCODE_H

#include "config.h"
#include "navigation.h"

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

/**
 * @brief Initialize barcode scanner
 */
void barcode_init(void);

/**
 * @brief Check for barcode timeout and process if ready
 * @param[in]  current_time Current time in milliseconds
 * @param[out] turn_dir     Detected turn direction
 * @param[out] turn_start   Time to start turn (ms)
 * @return true if barcode detected and processed
 */
bool_t barcode_check_and_process(uint32_t current_time, 
                                turn_dir_t *turn_dir,
                                uint32_t *turn_start);

/**
 * @brief Get ISR call count for debugging
 * @return Number of ISR calls
 */
uint32_t barcode_get_isr_count(void);

/**
 * @brief Get number of stored widths
 * @return Width count
 */
int barcode_get_width_count(void);

#endif /* BARCODE_H */