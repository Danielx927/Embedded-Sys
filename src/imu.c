#include "servo.h"
#include "barcode.h"
#include "imu.h"
#include "ultrasonic.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "utils.h"
#include "config.h"

void imu_init(void) {
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    accel_init();
    mag_init();
}

void accel_init(void){
    /* Power on, 50Hz ODR, all axes enabled */
    i2c_write_register(I2C_PORT, ACCEL_I2C_ADDR, CTRL_REG1_A, 0x57U);
    
    /* High resolution, +/-2g, big endian */
    i2c_write_register(I2C_PORT, ACCEL_I2C_ADDR, CTRL_REG4_A, 0x08U);
}

void mag_init(void){
    /* 15Hz ODR, default gain */
    i2c_write_register(I2C_PORT, MAG_I2C_ADDR, CRA_REG_M, 0x10U);
    
    /* Continuous conversion mode */
    i2c_write_register(I2C_PORT, MAG_I2C_ADDR, MR_REG_M, 0x00U);
}

void imu_read_accel(){
    uint8_t buf[6];
    i2c_read_registers(I2C_PORT, ACCEL_I2C_ADDR, OUT_X_L_A | AUTO_INCREMENT, buf, 6);
    
    g_raw_data.accel_x = (int16_t)((((uint16_t)buf[1]) << 8) | buf[0]);
    g_raw_data.accel_y = (int16_t)((((uint16_t)buf[3]) << 8) | buf[2]);
    g_raw_data.accel_z = (int16_t)((((uint16_t)buf[5]) << 8) | buf[4]);
}

void imu_read_mag(){
    uint8_t buf[6];
    i2c_read_registers(I2C_PORT, MAG_I2C_ADDR, OUT_X_H_M | AUTO_INCREMENT, buf, 6);
    
    g_raw_data.mag_x = (int16_t)((((uint16_t)buf[0]) << 8) | buf[1]);
    g_raw_data.mag_y = (int16_t)((((uint16_t)buf[4]) << 8) | buf[5]);
    g_raw_data.mag_z = (int16_t)((((uint16_t)buf[2]) << 8) | buf[3]);
}

void imu_filter_init(moving_avg_filter_t *p_filter){
    memset(p_filter->accel_buffer, 0, sizeof(p_filter->accel_buffer));
    memset(p_filter->mag_buffer, 0, sizeof(p_filter->mag_buffer));
    p_filter->buffer_index = 0U;
    p_filter->buffer_full = FALSE;
}

void imu_filter_update(moving_avg_filter_t *p_filter, sensor_data_t *p_raw){
    uint8_t idx = p_filter->buffer_index;
    
    p_filter->accel_buffer[0U][idx] = (int32_t)p_raw->accel_x;
    p_filter->accel_buffer[1U][idx] = (int32_t)p_raw->accel_y;
    p_filter->accel_buffer[2U][idx] = (int32_t)p_raw->accel_z;
    
    p_filter->mag_buffer[0U][idx] = (int32_t)p_raw->mag_x;
    p_filter->mag_buffer[1U][idx] = (int32_t)p_raw->mag_y;
    p_filter->mag_buffer[2U][idx] = (int32_t)p_raw->mag_z;
    
    p_filter->buffer_index = (p_filter->buffer_index + 1U) % FILTER_WINDOW_SIZE;
    if (p_filter->buffer_index == 0U)
    {
        p_filter->buffer_full = TRUE;
    }
}

void imu_filter_average(moving_avg_filter_t *p_filter, filtered_data_t *p_filtered){
    int32_t accel_sum[3U] = {0, 0, 0};
    int32_t mag_sum[3U] = {0, 0, 0};
    uint8_t divisor = p_filter->buffer_full ? FILTER_WINDOW_SIZE : p_filter->buffer_index;
    uint8_t i;
    
    for (i = 0U; i < divisor; i++)
    {
        accel_sum[0U] += p_filter->accel_buffer[0U][i];
        accel_sum[1U] += p_filter->accel_buffer[1U][i];
        accel_sum[2U] += p_filter->accel_buffer[2U][i];
        
        mag_sum[0U] += p_filter->mag_buffer[0U][i];
        mag_sum[1U] += p_filter->mag_buffer[1U][i];
        mag_sum[2U] += p_filter->mag_buffer[2U][i];
    }
    
    /* Calculate average */
    p_filtered->accel_x = accel_sum[0U] / (int32_t)divisor;
    p_filtered->accel_y = accel_sum[1U] / (int32_t)divisor;
    p_filtered->accel_z = accel_sum[2U] / (int32_t)divisor;
    
    p_filtered->mag_x = mag_sum[0U] / (int32_t)divisor;
    p_filtered->mag_y = mag_sum[1U] / (int32_t)divisor;
    p_filtered->mag_z = mag_sum[2U] / (int32_t)divisor;
}

float imu_compute_heading(filtered_data_t *p_data)
{
    float heading = atan2f((float)p_data->mag_y, (float)p_data->mag_x);
    return heading;
}

float imu_calibrate_heading(moving_avg_filter_t *p_filter, 
                           sensor_data_t *p_raw_data) {
    float sum_heading = 0.0f;
    for (uint8_t i = 0; i < CALIB_SAMPLES; i++) {
        imu_read_accel(p_raw_data);
        imu_read_mag(p_raw_data);
        imu_filter_update(p_filter, p_raw_data);
        
        filtered_data_t filtered;
        imu_filter_average(p_filter, &filtered);
        sum_heading += imu_compute_heading(&filtered);
        sleep_ms(50);
    }
    return sum_heading / (float)CALIB_SAMPLES;
}