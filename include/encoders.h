#ifndef ENCODERS_H
#define ENCODERS_H

#include "config.h"
#include <stdint.h>
#include "hardware/gpio.h"

/*******************************************************************************
 * TYPE DEFINITIONS
 ******************************************************************************/

/**
 * @brief Speed moving average filter structure
 */
typedef struct
{
    float buffer[2][SPEED_FILTER_WINDOW];
    uint8_t index;
    bool_t full;
} speed_filter_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

/**
 * @brief Initialize encoders and interrupts
 */
void encoders_init(void);

/**
 * @brief Get left encoder pulse count
 * @return Pulse count
 */
uint32_t encoders_get_left_pulses(void);

/**
 * @brief Get right encoder pulse count
 * @return Pulse count
 */
uint32_t encoders_get_right_pulses(void);

/**
 * @brief Reset encoder counts
 */
void encoders_reset(void);

void encoder_callback(uint gpio, uint32_t events);

/**
 * @brief Initialize speed filter
 * @param[out] p_filter Pointer to speed filter structure
 */
void encoders_filter_init(speed_filter_t *p_filter);

/**
 * @brief Update speed filter with new measurements
 * @param[in,out] p_filter        Pointer to speed filter
 * @param[in]     left_speed      Current left speed (m/s)
 * @param[in]     right_speed     Current right speed (m/s)
 * @param[out]    filtered_left   Filtered left speed (m/s)
 * @param[out]    filtered_right  Filtered right speed (m/s)
 */
void encoders_filter_update(speed_filter_t *p_filter,
                            float left_speed,
                            float right_speed,
                            float *filtered_left,
                            float *filtered_right);


void encoders_filter_init(speed_filter_t * const p_filter);

void encoders_filter_update(speed_filter_t * const p_filter,
                                float left_speed,
                                float right_speed,
                                float *filtered_left,
                                float *filtered_right);
                            
#endif /* ENCODERS_H */