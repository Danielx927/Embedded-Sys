#ifndef BARCODE_H
#define BARCODE_H

#include "config.h"
#include "navigation.h"
#include "hardware/gpio.h"
#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/
extern volatile uint64_t last_edge_us;
extern volatile bool     last_level;
extern volatile uint32_t widths[MAX_WIDTHS];
extern volatile int      num_widths;
extern volatile uint32_t isr_call_count;  // Debug: count ISR calls
extern char decoded_barcode[10];
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

int uint32_cmp(const void *a, const void *b);
void ir_isr(uint gpio, uint32_t events);
int get_bar_digit(int bar_pos1, int bar_pos2);
char decode_char(const int binary[9]);
char reverse_map(char c);
void str_reverse(char *str);
float find_threshold(uint32_t *local_widths, int start, int end);
char decode_local(uint32_t *widths);
void decode_and_process(void);

#endif /* BARCODE_H */