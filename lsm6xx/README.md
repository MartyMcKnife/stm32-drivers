# MS5607

MS5607 is a C library for interfacing with either a LSM6DSO32 or LSM6DS3TR accelerometer, through an I2C connection

## Installation

Place `lsm6xx.c` file into the `Core/Src` folder
Place `lsm6xx.h` file into the `Core/Inc` folder, and include the appropiate HAL

## Usage

The library must be intialized with the I2C interface. This takes a reference to the I2C handler

`LSM6XX_Init(&hi2c1);`

The library will either return `LSM6XX_OK` or `LSM6XX_FAIL` depending on the outcome of the request

The following methods can be called to configure the accelerometer. The driver also provides a way to set the free fall interrupt:

```C
LSM6XX_STATES LSM6XX_set_accel_config(LSM6XX_ACCEL_FS fs LSM6XX_ACCEL_ODR odr);
LSM6XX_STATES LSM6XX_set_gyro_config(LSM6XX_GYRO_FS fs LSM6XX_GYRO_ODR odr);
LSM6XX_STATES LSM6XX_set_ff(LSM6XX_INT int_line, LSM6XX_FF_THRES ff_thres, uint8_t duration);
```

The following methods can be called to get data out of the accelerometer
All require a reference to the corresponding struct

```C
LSM6XX_STATES LSM6XX_get_accel(LSM6XX_DATA *buf);
LSM6XX_STATES LSM6XX_get_gyro(LSM6XX_DATA *buf);
```

The accelerometer can also be calibrated using the `LSM6XX_calibrate()` function. This takes an optional struct if the device has already been calibrated. Take a look at the `lsm6xx.c` file for usage instructions

## Example

```C
  /* USER CODE BEGIN 2 */
  LSM6XX_Init(&hi2c1);
  LSM6XX_set_accel_config(LSM_ACCEL_32G, LSM_ACCEL_208HZ);
  LSM6XX_set_gyro_config(LSM_GYRO_1000, LSM_GYRO_208HZ);

  LSM6XX_CAL cal_set = {.xl_hw_x = 20,
                        .xl_hw_y = -50,
                        .xl_hw_z = 44,
                        .g_sw_x = 25,
                        .g_sw_y = -10,
                        .g_sw_z = -30};

  LSM6XX_calibrate(&cal_set);

  LSM6XX_DATA accel_buff;
  LSM6XX_DATA gyro_buff;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    LSM6XX_get_accel(&accel_buff);
    LSM6XX_get_gyro(&gyro_buff);
    HAL_Delay(500);
  }
  /* USER CODE END 3 */
```
