# MS5607

MS5607 is a C library for interfacing with a MS5607 Barometer, through an SPI connection

## Installation

Place `ms5607.c` file into the `Core/Src` folder
Place `ms5607.h` file into the `Core/Inc` folder, and include the appropiate HAL

## Usage

The library must be intialized with the SPI interface. This takes a reference to the SPI handler, as well as the CS pin

`MS5607_Init(&hspi2, SPI2_CS_GPIO_Port, SPI2_CS_Pin);`

The library will either return `MS5607_SUCCESS` or `MS5607_FAIL` depending on the outcome of the request

The following methods can be called:

``
MS5607_STATE get_temperature(MS5607_TEMPERATURE *temperature);
MS5607_STATE get_pressure(MS5607_PRESSURE *pressure);
void get_altitude(MS5607_ALTITUDE *altitude, uint32_t baseline_pressure);
``

All require a reference to the corresponding struct

## Example

``
  /* USER CODE BEGIN 2 */
  MS5607_Init(&hspi2, SPI2_CS_GPIO_Port, SPI2_CS_Pin);
  MS5607_ALTITUDE altitude;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    get_altitude(&altitude, 102400);
    HAL_Delay(500);
  }
  /* USER CODE END 3 */
``
