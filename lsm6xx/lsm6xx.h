/**
 ******************************************************************************
 * @file    lsm6xx.h
 * @brief   This file contains all the function prototypes for
 *          the lsm6xx.c file. Supports two different IMUs; LSM6DS3TR and
 *LSMDSO32
 *
 * @author Sean McDougall
 * @date June-2025
 ******************************************************************************
 */

#ifndef __LSM6XX_H
#define __LSM6XX_H
#ifdef __cplusplus
extern "C" {
#endif

// update the below with the MCU-specific HAL
// #include "stm32g0xx_hal.h"

#define LSM6DSO32

// Select the target device by defining one of:
// - LSM6DS3TR_C
// - LSM6DSO32
#if !defined(LSM6DS3TR_C) && !defined(LSM6DSO32)
#error                                                                         \
    "Please define either LSM6DS3TR_C or LSM6DSO32 before including this header."
#endif

// Register addresses generated with ChatGPT
// Has been validated against datasheet so shouldn't have any hallucinations

// =======================
// Shared Registers
// =======================
#define REG_FUNC_CFG_ACCESS 0x01
#define REG_INT1_CTRL 0x0D
#define REG_INT2_CTRL 0x0E
#define REG_WHO_AM_I 0x0F

#define REG_CTRL1_XL 0x10
#define REG_CTRL2_G 0x11
#define REG_CTRL3_C 0x12
#define REG_CTRL4_C 0x13
#define REG_CTRL5_C 0x14
#define REG_CTRL6_C 0x15
#define REG_CTRL7_G 0x16
#define REG_CTRL8_XL 0x17
#define REG_CTRL9_XL 0x18
#define REG_CTRL10_C 0x19

#define REG_WAKE_UP_SRC 0x1B
#define REG_TAP_SRC 0x1C
#define REG_D6D_SRC 0x1D
#define REG_STATUS_REG 0x1E

#define REG_OUT_TEMP_L 0x20
#define REG_OUT_TEMP_H 0x21
#define REG_OUTX_L_G 0x22
#define REG_OUTX_H_G 0x23
#define REG_OUTY_L_G 0x24
#define REG_OUTY_H_G 0x25
#define REG_OUTZ_L_G 0x26
#define REG_OUTZ_H_G 0x27
// OUTZ_L_XL on LSM6DS3TR, OUTZ_L_A on LSMDSO32
#define REG_OUTX_L_XL 0x28
#define REG_OUTX_H_XL 0x29
#define REG_OUTY_L_XL 0x2A
#define REG_OUTY_H_XL 0x2B
#define REG_OUTZ_L_XL 0x2C
#define REG_OUTZ_H_XL 0x2D

#define REG_X_OFS_USR 0x73
#define REG_Y_OFS_USR 0x74
#define REG_Z_OFS_USR 0x75

#define REG_TAP_THS_6D 0x59
#define REG_INT_DUR2 0x5A
#define REG_WAKE_UP_THS 0x5B
#define REG_WAKE_UP_DUR 0x5C
#define REG_FREE_FALL 0x5D
#define REG_MD1_CFG 0x5E
#define REG_MD2_CFG 0x5F

#define LSM6XX_ADR 0x6B

// =======================
// Device-specific Registers
// =======================

#ifdef LSM6DS3TR_C

#define REG_SENSOR_SYNC_TIME 0x04
#define REG_SENSOR_SYNC_RATIO 0x05
#define REG_FIFO_CTRL1 0x06
#define REG_FIFO_CTRL2 0x07
#define REG_FIFO_CTRL3 0x08
#define REG_FIFO_CTRL4 0x09
#define REG_FIFO_CTRL5 0x0A
#define REG_DRDY_PULSE_CFG_G 0x0B
#define REG_MASTER_CONFIG 0x1A
#define REG_STEP_COUNTER_L 0x4B
#define REG_STEP_COUNTER_H 0x4C
#define REG_FUNC_SRC1 0x53
#define REG_FUNC_SRC2 0x54
#define REG_WRIST_TILT_IA 0x55

#define WHO_AM_I 0x6A

#elif defined(LSM6DSO32)

#define REG_PIN_CTRL 0x02
#define REG_FIFO_CTRL1 0x07
#define REG_FIFO_CTRL2 0x08
#define REG_FIFO_CTRL3 0x09
#define REG_FIFO_CTRL4 0x0A
#define REG_COUNTER_BDR_REG1 0x0B
#define REG_COUNTER_BDR_REG2 0x0C
#define REG_ALL_INT_SRC 0x1A
#define REG_FIFO_STATUS1 0x3A
#define REG_FIFO_STATUS2 0x3B
#define REG_TIMESTAMP0 0x40
#define REG_TIMESTAMP1 0x41
#define REG_TIMESTAMP2 0x42
#define REG_TAP_CFG1 0x57
#define REG_TAP_CFG2 0x58
#define REG_FIFO_DATA_OUT_TAG 0x78
#define REG_FIFO_DATA_OUT_X_L 0x79
#define REG_FIFO_DATA_OUT_X_H 0x7A
#define REG_FIFO_DATA_OUT_Y_L 0x7B
#define REG_FIFO_DATA_OUT_Y_H 0x7C
#define REG_FIFO_DATA_OUT_Z_L 0x7D
#define REG_FIFO_DATA_OUT_Z_H 0x7E
#define REG_I3C_BUS_AVB 0x62
#define REG_INTERNAL_FREQ_FINE 0x63

#define WHO_AM_I 0x6C

#endif // LSM6DSO32

// =======================
// States and Enums
// =======================
typedef enum {
  LSM6XX_OK,
  LSM6XX_FAIL,
  LSM6XX_BUSY,
} LSM6XX_STATES;

typedef enum {
  LSM_ACCEL_4G = 0,
  LSM_ACCEL_32G = 1,
  LSM_ACCEL_8G = 2,
  LSM_ACCEL_16G = 3
} LSM6XX_ACCEL_FS;

typedef enum {
  // register map is FS[1:0], FS_125
  // for some reason
  // if we combine it to be [FS, FS, FS_125], less overhead for driver
  // implementation
  // i.e. value of 0 maps to [0, 0, 0]
  // value of 4 maps to [1, 0, 0]
  // enums by default ascend numerically but we need to skip a few numbers
  LSM_GYRO_250 = 0,
  LSM_GYRO_125 = 1,
  LSM_GYRO_500 = 2,
  LSM_GYRO_1000 = 4,
  LSM_GYRO_2000 = 6,
} LSM6XX_GYRO_FS;

typedef enum lsm_accel_rate {
  LSM_ACCEL_12HZ = 1,
  LSM_ACCEL_26HZ = 2,
  LSM_ACCEL_52HZ = 3,
  LSM_ACCEL_104HZ = 4,
  LSM_ACCEL_208HZ = 5,
  LSM_ACCEL_416HZ = 6,
  LSM_ACCEL_833HZ = 7,
  LSM_ACCEL_1K66HZ = 8,
  LSM_ACCEL_3K33HZ = 9,
  LSM_ACCEL_6K66HZ = 10,
  LSM_ACCEL_1HZ6 = 11,
} LSM6XX_ACCEL_ODR;

typedef enum lsm_gyro_rate {
  LSM_GYRO_12HZ = 1,
  LSM_GYRO_26HZ = 2,
  LSM_GYRO_52HZ = 3,
  LSM_GYRO_104HZ = 4,
  LSM_GYRO_208HZ = 5,
  LSM_GYRO_416HZ = 6,
  LSM_GYRO_833HZ = 7,
  LSM_GYRO_1K66HZ = 8,
  LSM_GYRO_3K33HZ = 9,
  LSM_GYRO_6K66HZ = 10,
} LSM6XX_GYRO_ODR;

typedef enum lsm_int_lines {
  INT1,
  INT2,
} LSM6XX_INT;

// units are in mG (0.00981 m/s2)
typedef enum lsm_freefall_thres {
  LSM_FF_312 = 0,
  LSM_FF_438 = 1,
  LSM_FF_500 = 2,
} LSM6XX_FF_THRES;

// Data structure for raw sensor readings
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} LSM6XX_RAW;

// Data structure for scaled sensor readings
typedef struct {
  float x;
  float y;
  float z;
} LSM6XX_DATA;

// struct for calibration

typedef struct {
  int8_t xl_hw_x;
  int8_t xl_hw_y;
  int8_t xl_hw_z;
  int16_t g_sw_x;
  int16_t g_sw_y;
  int16_t g_sw_z;
} LSM6XX_CAL;

// =======================
// Public Functions
// =======================

/**
 * @brief  Initializes LSM6XX Sensor
 * @param  hi2c I2C Handler address
 * @retval Initialization status:
 *           - FAILED: Was not abe to communicate with sensor
 *           - OK: Sensor initialized OK and ready to use
 */
LSM6XX_STATES
LSM6XX_Init(I2C_HandleTypeDef *xi2c);

/**
 * @brief  Sets accelearion sensitivity
 * @param  lsm_accel range of accelerometer
 * @param lsm_rate rate of accelerometer sampling
 * @retval Initialization status:
 *           - FAILED: Couldn't change register values (has sensor been
 * initialized?)
 *           - OK: Acceleration correctly updated
 */

LSM6XX_STATES LSM6XX_set_accel_config(LSM6XX_ACCEL_FS fs, LSM6XX_ACCEL_ODR odr);

/**
 * @brief  Sets gyroscope sensitivity
 * @param  range range of gyroscope
 * @param rate rate of gyroscope sampling
 * @retval Initialization status:
 *           - FAILED: Couldn't change register values (has sensor been
 * initialized?)
 *           - OK: Acceleration correctly updated
 */

LSM6XX_STATES LSM6XX_set_gyro_config(LSM6XX_GYRO_FS fs, LSM6XX_GYRO_ODR odr);

/**
 * @brief  Enables the free fall (FF) interrupt mask on the IMU
 * @param  int_line which interrupt pin
 * @param ff_thres trigger point for the interrupt
 * @param duration ODR interval to sample over. max 4 bits
 * @retval Status:
 *           - FAILED: Couldn't enable pin
 *           - OK: Interrupt mask correctly set
 */

LSM6XX_STATES LSM6XX_set_ff(LSM6XX_INT int_line, LSM6XX_FF_THRES ff_thres,
                            uint8_t duration);

/**
 * @brief  Calibrates both accel and gyroscope values. This only has to be done
 * once Testing should be done when connected to a debugger. The code will ask
 * you to place the IMU on each axis, to record a baseline This is then stored
 * in the IMU's register. This should only be done when developing, and not used
 * in any production code
 * @param prev_cal: Previous calibration data
 * @retval Status:
 *           - FAILED: Couldn't connect to the sensor and update the values
 *           - OK: Baseline calibration completed.
 */

LSM6XX_STATES LSM6XX_calibrate(LSM6XX_CAL *prev_cal);

/**
 * @brief  Read latest acceleration data
 * @param  buf 3 struct representing x,y and z directions
 * [0] = x_axis
 * [1] = y_axis
 * [2] = z_axis
 * @retval Status:
 *           - FAILED: No data returned
 *           - OK: Data OKfully returned
 */
LSM6XX_STATES LSM6XX_get_accel(LSM6XX_DATA *buf);

/**
 * @brief  Read latest gyroscope data
 * @param  buf 3 struct representing x,y and z directions
 * @retval Status:
 *           - FAILED: No data returned
 *           - OK: Data OKfully returned
 */
LSM6XX_STATES LSM6XX_get_gyro(LSM6XX_DATA *buf);

uint8_t free_fall_detect();

#ifdef __cplusplus
}
#endif
#endif //__LSM6XX_H