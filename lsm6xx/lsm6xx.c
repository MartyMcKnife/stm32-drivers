/**
 ******************************************************************************
 * @file    lsm6xx.c
 * @brief   This file contains all the drivers to interface with a ST IMU
 *
 * Currently supports both a LSM6DS3TR (budget, lower G range (16g)) and
 *LSMDSO32 (more expesnive, higher g range (32g))
 *
 * @author Sean McDougall
 * @date June-2025
 ******************************************************************************
 */
#include "lsm6xx.h"

// private functions
static HAL_StatusTypeDef lsm6xx_write_reg(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef lsm6xx_read_reg(uint8_t reg, uint8_t *ret_val);
static LSM6XX_STATES lsm6xx_get_accel_raw(LSM6XX_RAW *buf);
static LSM6XX_STATES lsm6xx_get_gyro_raw(LSM6XX_RAW *buf);

// static handlers
static I2C_HandleTypeDef *l_hi2c;
static uint8_t lsm_addr = LSM6XX_ADR << 1;
static uint8_t cur_xl_fs;
static uint8_t cur_g_fs;
static LSM6XX_CAL *cur_cal = {0};

// Handlers for Read/Write
static HAL_StatusTypeDef lsm6xx_write_reg(uint8_t reg, uint8_t value) {
  return HAL_I2C_Mem_Write(l_hi2c, lsm_addr, reg, I2C_MEMADD_SIZE_8BIT, &value,
                           1, 10);
}
static HAL_StatusTypeDef lsm6xx_read_reg(uint8_t reg, uint8_t *ret_val) {
  return HAL_I2C_Mem_Read(l_hi2c, lsm_addr, reg, I2C_MEMADD_SIZE_8BIT, ret_val,
                          1, 10);
}

static HAL_StatusTypeDef lsm6dxx_read_regs(uint8_t reg, uint8_t *ret_val,
                                           uint16_t len) {
  return HAL_I2C_Mem_Read(l_hi2c, lsm_addr, reg, I2C_MEMADD_SIZE_8BIT, ret_val,
                          len, 10);
}

// handlers to map range to sensitivity
static float lsm6xx_get_xl_sensitivity(LSM6XX_ACCEL_FS val) {
  switch (val) {
  case LSM_ACCEL_4G:
    return 0.000122f; // 4g / 32768 LSB
    break;
  case LSM_ACCEL_8G:
    return 0.000244f; // 8g / 32768 LSB
    break;
  case LSM_ACCEL_16G:
    return 0.000488f; // 16g / 32768 LSB
    break;
  case LSM_ACCEL_32G:
    return 0.000976f; // 32g / 32768 LSB
    break;

  default:
    return 0;
    break;
  }
}

static float lsm6xx_get_gy_sensitivity(LSM6XX_GYRO_FS val) {
  switch (val) {
  case LSM_GYRO_125:
    return 0.004375f; // 125dps / 32768 LSB * 1.142857
    break;
  case LSM_GYRO_250:
    return 0.008750f; // 250dps / 32768 LSB * 1.142857
    break;
  case LSM_GYRO_500:
    return 0.017500f; // 500dps / 32768 LSB * 1.142857
    break;
  case LSM_GYRO_1000:
    return 0.035000f; // 1000dps / 32768 LSB * 1.142857
    break;
  case LSM_GYRO_2000:
    return 0.070000f; // 2000dps / 32768 LSB * 1.142857
    break;
  default:
    return 0;
    break;
  }
}

LSM6XX_STATES LSM6XX_Init(I2C_HandleTypeDef *xi2c) {
  l_hi2c = xi2c;

  // check if device at correct address
  if (HAL_I2C_IsDeviceReady(l_hi2c, lsm_addr, 3, 10) != HAL_OK) {
    return LSM6XX_FAIL;
  }

  uint8_t whoami_buf;

  // check if device buffer matches to what we expect
  if (lsm6xx_read_reg(REG_WHO_AM_I, &whoami_buf) != HAL_OK) {
    return LSM6XX_BUSY;
  } else {
    if (whoami_buf != WHO_AM_I) {
      return LSM6XX_FAIL;
    }
  }

  // reset our device to known state
  lsm6xx_write_reg(REG_CTRL3_C, 1);

  // delay to let device clear
  HAL_Delay(100);

  return LSM6XX_OK;
}

LSM6XX_STATES LSM6XX_set_accel_config(LSM6XX_ACCEL_FS fs,
                                      LSM6XX_ACCEL_ODR odr) {
  // register is 8 bits long; first 4 are our rate, next 3 are scale

  uint8_t pkt = ((fs << 2) | (odr << 4));
  uint8_t chk_pkt;

  lsm6xx_write_reg(REG_CTRL1_XL, pkt);
  lsm6xx_read_reg(REG_CTRL1_XL, &chk_pkt);

  if (chk_pkt == pkt) {
    cur_xl_fs = fs;
    return LSM6XX_OK;
  } else {
    return LSM6XX_FAIL;
  }
}

LSM6XX_STATES LSM6XX_set_gyro_config(LSM6XX_GYRO_FS fs, LSM6XX_GYRO_ODR odr) {
  // register is 8 bits long; first 4 are our rate, next 3 are scale

  uint8_t pkt = ((fs << 1) | (odr << 4));
  uint8_t chk_pkt;

  lsm6xx_write_reg(REG_CTRL2_G, pkt);
  lsm6xx_read_reg(REG_CTRL2_G, &chk_pkt);

  if ((chk_pkt) == pkt) {
    cur_g_fs = fs;
    return LSM6XX_OK;
  } else {
    return LSM6XX_FAIL;
  }
}

LSM6XX_STATES LSM6XX_set_ff(LSM6XX_INT int_line, LSM6XX_FF_THRES ff_thres,
                            uint8_t duration) {
// enable interrupts globally - only on LSM6DSO32
// not really sure why it is on the tap register
#ifdef LSM6DSO32
  lsm6xx_write_reg(REG_TAP_CFG2, (1 << 7));
#endif
  // TODO: Check LSM6DS3TR datasheet and implement interrupt enable

  // enable FF interrupt on given interrupt line
  if (int_line == INT1) {
    lsm6xx_write_reg(REG_MD1_CFG, (1 << 4));
  } else {
    lsm6xx_write_reg(REG_MD2_CFG, (1 << 4));
  }

  uint8_t cfg = (duration << 3) | (ff_thres);

  // set config
  lsm6xx_write_reg(REG_FREE_FALL, cfg);

  // check
  uint8_t chk_pkt;
  lsm6xx_read_reg(REG_FREE_FALL, &chk_pkt);
  if (chk_pkt == cfg) {
    return LSM6XX_OK;
  } else {
    return LSM6XX_FAIL;
  }
}

LSM6XX_STATES lsm6xx_get_accel_raw(LSM6XX_RAW *buf) {
  // check to make sure accel data is ready to be processed (always should be)
  uint8_t chk_pkt;
  lsm6xx_read_reg(REG_STATUS_REG, &chk_pkt);
  if ((chk_pkt >> 2) != 1) {
    return LSM6XX_FAIL;
  }

  uint8_t raw_data[6];

  if (lsm6dxx_read_regs(REG_OUTX_L_XL, raw_data, 6) != HAL_OK) {
    return LSM6XX_FAIL;
  }

  buf->x = (int16_t)(raw_data[1] << 8 | raw_data[0]);
  buf->y = (int16_t)(raw_data[3] << 8 | raw_data[2]);
  buf->z = (int16_t)(raw_data[5] << 8 | raw_data[4]);

  return LSM6XX_OK;
}

LSM6XX_STATES LSM6XX_get_accel(LSM6XX_DATA *buf) {
  LSM6XX_RAW raw_buff;
  if (lsm6xx_get_accel_raw(&raw_buff) != LSM6XX_FAIL) {
    float sf = lsm6xx_get_xl_sensitivity(cur_xl_fs);

    buf->x = raw_buff.x * sf * 9.81f;
    buf->y = raw_buff.y * sf * 9.81f;
    buf->z = raw_buff.z * sf * 9.81f;

    return LSM6XX_OK;

  } else {
    return LSM6XX_FAIL;
  }
}

LSM6XX_STATES lsm6xx_get_gyro_raw(LSM6XX_RAW *buf) {
  // same implmentation as accel

  uint8_t chk_pkt;
  lsm6xx_read_reg(REG_STATUS_REG, &chk_pkt);
  if ((chk_pkt & (1 << 1)) != 2) {
    return LSM6XX_FAIL;
  }

  uint8_t raw_data[6];

  if (lsm6dxx_read_regs(REG_OUTX_L_G, raw_data, 6) != HAL_OK) {
    return LSM6XX_FAIL;
  }

  // take into account calibration readings
  buf->x = (int16_t)(raw_data[1] << 8 | raw_data[0]) - cur_cal->g_sw_x;
  buf->y = (int16_t)(raw_data[3] << 8 | raw_data[2]) - cur_cal->g_sw_y;
  buf->z = (int16_t)(raw_data[5] << 8 | raw_data[4]) - cur_cal->g_sw_z;

  return LSM6XX_OK;
}

LSM6XX_STATES LSM6XX_get_gyro(LSM6XX_DATA *buf) {
  LSM6XX_RAW raw_buff;
  if (lsm6xx_get_gyro_raw(&raw_buff) != LSM6XX_FAIL) {
    float sf = lsm6xx_get_gy_sensitivity(cur_xl_fs);

    buf->x = raw_buff.x * sf;
    buf->y = raw_buff.y * sf;
    buf->z = raw_buff.z * sf;

    return LSM6XX_OK;

  } else {
    return LSM6XX_FAIL;
  }
}

static int32_t lsm6xx_collect_samples(uint8_t axis, uint8_t samples) {
  int32_t running_total = 0;
  LSM6XX_RAW data;

  for (uint8_t i = 0; i < samples; i++) {
    lsm6xx_get_accel_raw(&data);
    switch (axis) {
    case 0:
      running_total += data.x;
      break;
    case 1:
      running_total += data.y;
      break;
    case 2:
      running_total += data.z;
      break;
      break;
    }
    HAL_Delay(1);
  }

  return running_total;
}

LSM6XX_STATES LSM6XX_calibrate(LSM6XX_CAL *prev_cal) {

  // -----------------------
  // CALIBRATE ACCELEROMETER
  //        OVERVIEW
  // -----------------------
  // This function is only to be used in development
  // This function takes 100 raw readings for each axis
  // Place a breakpoint on the next line after each axis calibration
  // Once each breakpoint is run, it will be saved to the IMU so no processing
  // has to be done

  // feel free to increase this value, but don't make it too high!
  uint8_t CALIBRATION_SAMPLES = 100;

  //------------------
  // ACCEL CALIBRATION
  //------------------

  // check if we already have calibration data for the accel
  // if we do just use that
  if (prev_cal->xl_hw_x != 0) {
    uint8_t res;
    lsm6xx_write_reg(REG_X_OFS_USR, prev_cal->xl_hw_x);
    lsm6xx_write_reg(REG_Y_OFS_USR, prev_cal->xl_hw_y);
    lsm6xx_write_reg(REG_Z_OFS_USR, prev_cal->xl_hw_z);

    lsm6xx_read_reg(REG_Z_OFS_USR, &res);
    // gyro calibration is software only
    cur_cal = prev_cal;
    return LSM6XX_OK;
  } else {
    int32_t total;
    int16_t cal_val_pos;
    int16_t cal_val_neg;

    float xl_sf = lsm6xx_get_xl_sensitivity(cur_xl_fs);

    int8_t axis_update[3];

    // loop through 3 axis
    for (uint8_t axis = 0; axis < 3; axis++) {
      // !! PLACE BREAKPOINT HERE !!
      // !! PLACE FC FLAT ON SIDE (AXIS POSITIVE) !!
      total = lsm6xx_collect_samples(axis, CALIBRATION_SAMPLES);
      cal_val_pos = total / CALIBRATION_SAMPLES;

      // !! PLACE BREAKPOINT HERE !!
      // !! PLACE FC FLAT AND UPSIDE DOWN ON SIDE (AXIS NEGATIVE) !!

      total = lsm6xx_collect_samples(axis, CALIBRATION_SAMPLES);
      cal_val_neg = total / CALIBRATION_SAMPLES;

      int16_t offset = cal_val_pos + cal_val_neg;

      int8_t hw_offset = (offset * (xl_sf * 1000.0)) / 0.976f;

      axis_update[axis] = hw_offset;

      lsm6xx_write_reg(REG_X_OFS_USR + axis, hw_offset);
    }

    // update calibration struct for x values
    prev_cal->xl_hw_x = axis_update[0];
    prev_cal->xl_hw_y = axis_update[1];
    prev_cal->xl_hw_z = axis_update[2];

    //-----------------
    // GYRO CALIBRATION
    //-----------------

    // !! PLACE BREAKPOINT HERE !!
    // !! MAKE SURE THE FC IS COMPLETELY STILL !!

    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;
    LSM6XX_RAW gyro_data;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
      lsm6xx_get_gyro_raw(&gyro_data);

      sum_x += gyro_data.x;
      sum_y += gyro_data.y;
      sum_z += gyro_data.z;

      HAL_Delay(1);
    }

    prev_cal->g_sw_x = sum_x / CALIBRATION_SAMPLES;
    prev_cal->g_sw_y = sum_y / CALIBRATION_SAMPLES;
    prev_cal->g_sw_z = sum_z / CALIBRATION_SAMPLES;

    return LSM6XX_OK;
  }
}

uint8_t free_fall_detect() {
  uint8_t reg;

  lsm6xx_read_reg(REG_ALL_INT_SRC, &reg);

  if ((reg & 0x01) == 1) {
    return 1;
  } else {
    return 0;
  }
}