/*
 * BME280_STM32.h
 *
 *  Created on: May 28, 2025
 *      Author: WELCOME
 */

#ifndef INC_BME280_STM32_H_
#define INC_BME280_STM32_H_

#include "stm32f4xx_hal.h"  // Sesuaikan dengan STM32 series Anda (f1xx, f4xx, l4xx, dll)
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// BME280 I2C Addresses
#define BME280_ADDRESS_PRIMARY   0x76
#define BME280_ADDRESS_SECONDARY 0x77

// BME280 Register Addresses
#define BME280_REG_HUM_LSB       0xFE
#define BME280_REG_HUM_MSB       0xFD
#define BME280_REG_TEMP_XLSB     0xFC
#define BME280_REG_TEMP_LSB      0xFB
#define BME280_REG_TEMP_MSB      0xFA
#define BME280_REG_PRESS_XLSB    0xF9
#define BME280_REG_PRESS_LSB     0xF8
#define BME280_REG_PRESS_MSB     0xF7
#define BME280_REG_CONFIG        0xF5
#define BME280_REG_CTRL_MEAS     0xF4
#define BME280_REG_STATUS        0xF3
#define BME280_REG_CTRL_HUM      0xF2
#define BME280_REG_RESET         0xE0
#define BME280_REG_ID            0xD0

// Calibration data registers
#define BME280_REG_CALIB00       0x88
#define BME280_REG_CALIB26       0xE1

// BME280 Chip ID
#define BME280_CHIP_ID           0x60

// Power modes
#define BME280_MODE_SLEEP        0x00
#define BME280_MODE_FORCED       0x01
#define BME280_MODE_NORMAL       0x03

// Oversampling settings
#define BME280_OVERSAMP_SKIP     0x00
#define BME280_OVERSAMP_1X       0x01
#define BME280_OVERSAMP_2X       0x02
#define BME280_OVERSAMP_4X       0x03
#define BME280_OVERSAMP_8X       0x04
#define BME280_OVERSAMP_16X      0x05

// Filter settings
#define BME280_FILTER_OFF        0x00
#define BME280_FILTER_2          0x01
#define BME280_FILTER_4          0x02
#define BME280_FILTER_8          0x03
#define BME280_FILTER_16         0x04

// Standby time settings
#define BME280_STANDBY_0_5       0x00
#define BME280_STANDBY_62_5      0x01
#define BME280_STANDBY_125       0x02
#define BME280_STANDBY_250       0x03
#define BME280_STANDBY_500       0x04
#define BME280_STANDBY_1000      0x05
#define BME280_STANDBY_10        0x06
#define BME280_STANDBY_20        0x07

// I2C timeout
#define BME280_I2C_TIMEOUT       1000

// Calibration data structure
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} BME280_CalibData_t;

// BME280 handle structure
typedef struct {
    I2C_HandleTypeDef *hi2c;     // I2C handle pointer
    uint8_t address;             // I2C device address
    BME280_CalibData_t calibData; // Calibration data
    int32_t t_fine;              // Temperature fine value for compensation
} BME280_HandleTypeDef;

// Function prototypes
bool BME280_Init(BME280_HandleTypeDef *bme, I2C_HandleTypeDef *hi2c, uint8_t address);
bool BME280_IsConnected(BME280_HandleTypeDef *bme);
uint8_t BME280_GetChipID(BME280_HandleTypeDef *bme);
void BME280_Reset(BME280_HandleTypeDef *bme);

// Configuration functions
void BME280_SetMode(BME280_HandleTypeDef *bme, uint8_t mode);
void BME280_SetOversamplingTemperature(BME280_HandleTypeDef *bme, uint8_t oversampling);
void BME280_SetOversamplingPressure(BME280_HandleTypeDef *bme, uint8_t oversampling);
void BME280_SetOversamplingHumidity(BME280_HandleTypeDef *bme, uint8_t oversampling);
void BME280_SetFilter(BME280_HandleTypeDef *bme, uint8_t filter);
void BME280_SetStandbyTime(BME280_HandleTypeDef *bme, uint8_t standby);

// Reading functions
float BME280_ReadTemperature(BME280_HandleTypeDef *bme);
float BME280_ReadPressure(BME280_HandleTypeDef *bme);
float BME280_ReadHumidity(BME280_HandleTypeDef *bme);
float BME280_ReadAltitude(BME280_HandleTypeDef *bme, float seaLevelPressure);

// Utility functions
bool BME280_IsMeasuring(BME280_HandleTypeDef *bme);

// Private function prototypes (internal use)
uint8_t BME280_ReadRegister(BME280_HandleTypeDef *bme, uint8_t reg);
void BME280_WriteRegister(BME280_HandleTypeDef *bme, uint8_t reg, uint8_t value);
void BME280_ReadCalibrationData(BME280_HandleTypeDef *bme);
int32_t BME280_ReadTemperatureRaw(BME280_HandleTypeDef *bme);
int32_t BME280_ReadPressureRaw(BME280_HandleTypeDef *bme);
int32_t BME280_ReadHumidityRaw(BME280_HandleTypeDef *bme);


#endif /* INC_BME280_STM32_H_ */
