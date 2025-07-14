/**
 ******************************************************************************
 * @file    ms5607.c
 * @brief   This file contains all the drivers to interface with a MS5607
 *Barometer, through an SPI connection
 *
 * @author Sean McDougall
 * @date June-2025
 ******************************************************************************
 */
#include "ms5607.h"

MS5607_STATE MS5607_Init(SPI_HandleTypeDef *xhspi, GPIO_TypeDef *port,
                         uint16_t pin) {
  m_hspi = xhspi;
  CS_PORT = port;
  CS_PIN = pin;

  enable_cs();
  uint8_t SPI_TRANSMIT = OP_RESET;
  HAL_SPI_Transmit(m_hspi, &SPI_TRANSMIT, 1, 10);
  HAL_Delay(3);
  disable_cs();
  getPROM(&PROM_DATA);

  if (PROM_DATA.reserved == 0x00 || PROM_DATA.reserved == 0xff) {
    return MS5607_FAIL;
  }
  uint16_t crc_check = crc4(&PROM_DATA);

  if ((crc_check & 0xF) != (PROM_DATA.crc & 0xF)) {
    return MS5607_FAIL;
  } else {
    return MS5607_SUCCESS;
  }
}

MS5607_STATE get_temperature(MS5607_TEMPERATURE *temperature) {
  // been more than 2ms; update data
  if (HAL_GetTick() - raw_data.time_captured >= 2 || raw_data.pressure == 0) {
    getRaw(&raw_data);
  }

  // if either value is 0, codes did not work correctly
  if (raw_data.pressure == 0 || raw_data.temperature == 0) {
    return MS5607_FAIL;
  }

  int32_t dt;
  int32_t temp;

  dt = raw_data.temperature - ((int32_t)(PROM_DATA.tref << 8));

  temp = 2000 + ((int64_t)(dt * PROM_DATA.temp_sens) >> 23);

  temperature->dT = dt;
  temperature->temp = temp;

  return MS5607_SUCCESS;
}

MS5607_STATE get_pressure(MS5607_PRESSURE *pressure) {
  int64_t off;
  int64_t sens;
  int32_t p;

  MS5607_TEMPERATURE last_temp;

  // will also call raw_data reading
  if (get_temperature(&last_temp) == MS5607_FAIL) {
    return MS5607_FAIL;
  }

  off = ((int64_t)PROM_DATA.off_t1 << 17) +
        ((int64_t)(PROM_DATA.tco * last_temp.dT) >> 6);

  sens = ((int64_t)PROM_DATA.sens_t1 << 16) +
         ((int64_t)(PROM_DATA.tcs * last_temp.dT) >> 7);

  // low temperature compensation
  if (last_temp.temp < 2000) {
    int32_t temp_off = last_temp.temp - 2000;
    int64_t off2;
    int64_t sens2;

    off2 = (61 * (int64_t)temp_off * (int64_t)temp_off) >> 4;
    sens2 = 2 * ((int64_t)temp_off * (int64_t)temp_off);

    // freezing temperature compensation
    if (last_temp.temp <= -1500) {
      int32_t temp_off_cold = last_temp.temp + 1500;
      off2 += 15 * (temp_off_cold * temp_off_cold);
      sens2 += 8 * (temp_off_cold * temp_off_cold);
    }

    off -= off2;
    sens -= sens2;
  }
  p = ((((int64_t)raw_data.pressure * sens) >> 21) - off) >> 15;

  pressure->offset = off;
  pressure->sens = sens;
  pressure->pressure = p;

  return MS5607_SUCCESS;
}

// calculated from https://www.mide.com/air-pressure-at-altitude-calculator
void get_altitude(MS5607_ALTITUDE *altitude, uint32_t baseline_pressure) {
  MS5607_TEMPERATURE temperature;
  MS5607_PRESSURE pressure;

  get_temperature(&temperature);
  get_pressure(&pressure);

  if (temperature.temp < 2000) {
    int32_t t2 = ((int64_t)temperature.dT * (int64_t)temperature.dT) >> 31;
    temperature.temp -= t2;
  }

  float p0 = (baseline_pressure) / (float)pressure.pressure;

  int32_t height =
      153.84615 * (pow(p0, 0.19) - 1) * ((temperature.temp / 1000) + 273.15);

  // check to see if we are in stratosphere - calculation is probably wrong
  if (height > 11500) {
    height = 11000 + ((8.31432 * (temperature.temp / 1000 - 71.5) * log(p0)) /
                      (-0.284));
  }

  altitude->altitude = height;
  altitude->pressure = pressure.pressure;
  altitude->temp = temperature.temp;
}

void set_OSR(MS5607_OSR_RANGES osr_level) { OSR_LEVEL = osr_level; }

static void getPROM(MS5607_PROM_DATA *prom) {
  uint16_t *prom_pointer;
  prom_pointer = (uint16_t *)prom;

  for (uint8_t i = 0; i < 8; i++) {
    uint8_t PROM_CODE = PROM_ADDR_CONV(i);
    enable_cs();
    HAL_SPI_Transmit(m_hspi, &PROM_CODE, 1, 10);
    HAL_SPI_Receive(m_hspi, prom_pointer, 2, 50);
    disable_cs();
    prom_pointer++;
  }

  // swap bit order; from 12345678 to 43218754
  // ms5607 pipes out MSB, but stm32 reads LSB
  // conversion to ensure actual integer numbers are correct
  // code taken from stm32 MS5607 driver on github
  prom_pointer = (uint16_t *)prom;
  for (uint8_t i = 0; i < 8; i++) {
    uint8_t *toSwap = (uint8_t *)prom_pointer;
    uint8_t secondByte = toSwap[0];
    toSwap[0] = toSwap[1];
    toSwap[1] = secondByte;
    prom_pointer++;
  }
}

static void getRaw(MS5607_RAW *raw) {
  uint8_t spi_tx = OP_CONV_D1 | OSR_LEVEL;
  uint8_t recv_buf[3] = {0};

  enable_cs();
  // conversion time is
  // 0.54ms for 256 OSR
  // 1.06ms for 512 OSR
  // 2.08ms for 1024 OSR
  // 4.13ms for 2048 OSR
  // 8.22ms for 4096 OSR
  HAL_SPI_Transmit(m_hspi, &spi_tx, 1, 10);
  // dummy spi receive - will send high signal if conversion complete, and code
  // can continue
  while (recv_buf[0] != 0xFF) {
    HAL_SPI_Receive(m_hspi, recv_buf, 1, 10);
  }
  disable_cs();

  spi_tx = OP_ADC_READ;
  enable_cs();
  HAL_SPI_Transmit(m_hspi, &spi_tx, 1, 10);
  HAL_SPI_Receive(m_hspi, recv_buf, 3, 10);
  disable_cs();

  // swap bit order and push into our struct
  raw->pressure = (uint32_t)recv_buf[0] << 16 | (uint32_t)recv_buf[1] << 8 |
                  (uint32_t)recv_buf[0];

  // clear first byte of buffer so we can listen to see if miso high
  recv_buf[0] = 0;

  // same again for temp, and change to our temp op code
  spi_tx = OP_CONV_D2 | OSR_LEVEL;
  enable_cs();
  HAL_SPI_Transmit(m_hspi, &spi_tx, 1, 10);
  while (recv_buf[0] != 0xFF) {
    HAL_SPI_Receive(m_hspi, recv_buf, 1, 10);
  }
  disable_cs();

  spi_tx = OP_ADC_READ;
  enable_cs();
  HAL_SPI_Transmit(m_hspi, &spi_tx, 1, 10);
  HAL_SPI_Receive(m_hspi, recv_buf, 3, 10);
  disable_cs();

  // swap bit order and push into our struct
  raw->temperature = (uint32_t)recv_buf[0] << 16 | (uint32_t)recv_buf[1] << 8 |
                     (uint32_t)recv_buf[0];

  raw->time_captured = HAL_GetTick();
}

// implemented base on AN520 from MEAS
static uint16_t crc4(MS5607_PROM_DATA *prom) {
  uint16_t n_rem = 0x00;
  // read out crc from struct
  uint16_t crc_read = prom->crc;
  // 0 out crc to avoid including in calculation
  prom->crc = (0xFF00 & crc_read);

  // pointer to struct, to allow traversal
  uint16_t *n_prom = (uint16_t *)prom;

  for (uint8_t cnt = 0; cnt < 16; cnt++) {
    if (cnt % 2 == 1) {
      n_rem ^= (n_prom[cnt >> 1] & 0x00FF);
    } else {
      n_rem ^= (n_prom[cnt >> 1] >> 8);
    }

    for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
      if (n_rem & 0x8000) {
        n_rem = (n_rem << 1) ^ 0x3000;
      } else {
        n_rem = (n_rem << 1);
      }
    }
  }

  n_rem = (0x000F & (n_rem >> 12));
  prom->crc = crc_read;
  return n_rem ^ 0x00;
}

static void enable_cs() { HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET); }

static void disable_cs() { HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET); }