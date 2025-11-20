#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <math.h>

/*******************************************************************************
 * BOOLEAN TYPE
 ******************************************************************************/
typedef uint8_t bool_t;
#define TRUE  (1U)
#define FALSE (0U)

/*******************************************************************************
 * I2C CONFIGURATION
 ******************************************************************************/
#define I2C_PORT            i2c1
#define I2C_SDA_PIN         26
#define I2C_SCL_PIN         27
#define I2C_FREQ            400000

/* I2C Addresses */
#define ACCEL_I2C_ADDR      (0x19U)
#define MAG_I2C_ADDR        (0x1EU)

/* Accelerometer Registers */
#define CTRL_REG1_A         (0x20U)
#define CTRL_REG4_A         (0x23U)
#define OUT_X_L_A           (0x28U)
#define AUTO_INCREMENT      (0x80U)

/* Magnetometer Registers */
#define CRA_REG_M           (0x00U)
#define MR_REG_M            (0x02U)
#define OUT_X_H_M           (0x03U)

/*******************************************************************************
 * MOTOR CONFIGURATION
 ******************************************************************************/
#define PWM_M1A             8
#define PWM_M1B             9
#define PWM_M2A             11
#define PWM_M2B             10
#define PWM_WRAP            65535U
#define PWM_CLKDIV          44.0f
#define MIN_DUTY            0.3f
#define RIGHT_DUTY_FACTOR   0.95f
#define INITIAL_BASE_DUTY   0.30f

/*******************************************************************************
 * ENCODER CONFIGURATION
 ******************************************************************************/
#define LEFT_ENCODER        4
#define RIGHT_ENCODER       16
#define PULSES_PER_REV      20.0f
#define WHEEL_DIAMETER_M    0.06f
#define WHEEL_CIRC_M        (M_PI * WHEEL_DIAMETER_M)
#define DIST_PER_PULSE      (WHEEL_CIRC_M / PULSES_PER_REV)

/*******************************************************************************
 * IR SENSOR CONFIGURATION
 ******************************************************************************/
#define LEFT_IR_ADC_GPIO    28
#define LEFT_IR_ADC_CHANNEL 2
#define RIGHT_IR_DIGITAL_GPIO 0

/*******************************************************************************
 * ULTRASONIC CONFIGURATION
 ******************************************************************************/
#define TRIG_PIN            3
#define ECHO_PIN            2
#define SERVO_PIN           15
#define SERVO_CENTER        1500
#define SERVO_LEFT_START    1550
#define SERVO_RIGHT_START   1450
#define SERVO_LEFT_MAX      2000
#define SERVO_RIGHT_MAX     1000
#define SERVO_STEP          10
#define SERVO_SCAN_DELAY_MS 500
#define SERVO_PULSE_PER_30DEG 260.0f
#define SERVO_DEG_PER_PULSE (30.0f / SERVO_PULSE_PER_30DEG)

/*******************************************************************************
 * PID CONSTANTS
 ******************************************************************************/
/* Speed PID */
#define SPEED_KP            0.10f
#define SPEED_KI            0.06f
#define SPEED_KD            0.04f

/* Heading PID - Ramp-up */
#define HEADING_KP_RAMP     0.02f
#define HEADING_KI_RAMP     0.0f
#define HEADING_KD_RAMP     0.8f

/* Heading PID - Steady-state */
#define HEADING_KP_STEADY   0.02f
#define HEADING_KI_STEADY   0.0f
#define HEADING_KD_STEADY   1.2f

/* Line PID */
#define LINE_KP             0.12f
#define LINE_KI             0.003f
#define LINE_KD             0.08f
#define LINE_SETPOINT       1600
#define LINE_SCALE          1000.0f
#define LINE_ERROR_THRESHOLD 1200.0f

/* PID Limits */
#define INTEGRAL_LIMIT      0.3f

/*******************************************************************************
 * NAVIGATION CONSTANTS
 ******************************************************************************/
#define TARGET_SPEED_MS     0.08f
#define RAMP_TIME_SEC       2.5f
#define TURN_SPEED_MS       0.08f
#define TURN_DURATION_MS    1500
#define RECOVERY_TURN_RATE  0.1f
#define RECOVERY_TURN_RATE_SWITCHED 0.2f
#define RECOVERY_SPEED_SWITCHED 0.16f
#define RECOVERY_DISABLE_TIME_MS 1800
#define RECOVERY_TIMEOUT_MS 2250
#define CURVE_DETECTION_THRESHOLD 600.0f
#define CURVE_HISTORY_SIZE 5
#define MIN_HEADING_CHANGE_RATE 0.3f

/*******************************************************************************
 * OBSTACLE DETECTION
 ******************************************************************************/
#define OBSTACLE_THRESHOLD  20.0f
#define OBSTACLE_STOP_TIME_MS 10000
#define DISTANCE_JUMP_THRESHOLD 2.0f

/*******************************************************************************
 * DETOUR NAVIGATION
 ******************************************************************************/
#define DETOUR_SAFETY_MARGIN_CM  10.0f
#define DETOUR_MAX_SIDEWAYS_CM   25.0f
#define DETOUR_PARALLEL_DIST_CM  35.0f
#define DETOUR_TURN_DURATION_MS  1500
#define DETOUR_TURN_SPEED        0.08f
#define DETOUR_MOVE_SPEED        0.08f
#define DETOUR_HEADING_TOLERANCE 0.10f
#define IMU_HEADING_CHECK_INTERVAL_MS 200
#define DETOUR_HEADING_CORRECTION 0.05f
#define DETOUR_SPOT_TURN_DUTY 0.4f

/* Empirical IMU turn deltas */
#define PERP_LEFT_DELTA  (3.55f - 2.59f)
#define PERP_RIGHT_DELTA (4.48f - 3.55f)

/*******************************************************************************
 * BARCODE PARAMETERS
 ******************************************************************************/
#define MAX_WIDTHS         200
#define SCAN_TIMEOUT_MS    10000
#define BARCODE_PROCESS_TIMEOUT_MS 100
#define TURN_DELAY_MS      1000
#define LONG_WIDTH_US      2000000
#define MIN_VALID_WIDTH_US 5000
#define THRESHOLD_MULTIPLIER 2.0f
#define MIN_EDGE_US        15000

/*******************************************************************************
 * FILTER CONFIGURATION
 ******************************************************************************/
#define FILTER_WINDOW_SIZE  15U
#define SPEED_FILTER_WINDOW 10U
#define NUM_AXES            3U

/*******************************************************************************
 * TIMING CONSTANTS
 ******************************************************************************/
#define SAMPLE_DELAY_MS     200U
#define CALIB_SAMPLES       20U

#endif /* CONFIG_H */