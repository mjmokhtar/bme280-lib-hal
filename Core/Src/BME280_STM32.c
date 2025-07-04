/*
 * BME280_STM32.c
 *
 *  Created on: May 28, 2025
 *      Author: WELCOME
 */

#include "BME280_STM32.h"

bool BME280_Init(BME280_HandleTypeDef *bme, I2C_HandleTypeDef *hi2c, uint8_t address) {
    bme->hi2c = hi2c;
    bme->address = address << 1;  // STM32 HAL uses 8-bit address format
    bme->t_fine = 0;

    HAL_Delay(100); // Give time for I2C to stabilize

    // Test I2C communication first
    if (HAL_I2C_IsDeviceReady(bme->hi2c, bme->address, 3, BME280_I2C_TIMEOUT) != HAL_OK) {
        return false;
    }

    // Check chip ID
    uint8_t chipID = BME280_GetChipID(bme);
    if (chipID != BME280_CHIP_ID) {
        return false;
    }

    // Reset sensor
    BME280_Reset(bme);
    HAL_Delay(100); // Wait for reset to complete

    // Verify sensor is responsive after reset
    chipID = BME280_GetChipID(bme);
    if (chipID != BME280_CHIP_ID) {
        return false;
    }

    // Read calibration data
    BME280_ReadCalibrationData(bme);

    // Set default configuration
    BME280_SetOversamplingTemperature(bme, BME280_OVERSAMP_16X);
    BME280_SetOversamplingPressure(bme, BME280_OVERSAMP_16X);
    BME280_SetOversamplingHumidity(bme, BME280_OVERSAMP_16X);
    BME280_SetFilter(bme, BME280_FILTER_16);
    BME280_SetStandbyTime(bme, BME280_STANDBY_0_5);
    BME280_SetMode(bme, BME280_MODE_NORMAL);

    return true;
}

bool BME280_IsConnected(BME280_HandleTypeDef *bme) {
    uint8_t chipID = BME280_GetChipID(bme);
    return (chipID == BME280_CHIP_ID);
}

uint8_t BME280_GetChipID(BME280_HandleTypeDef *bme) {
    return BME280_ReadRegister(bme, BME280_REG_ID);
}

void BME280_Reset(BME280_HandleTypeDef *bme) {
    BME280_WriteRegister(bme, BME280_REG_RESET, 0xB6);
}

uint8_t BME280_ReadRegister(BME280_HandleTypeDef *bme, uint8_t reg) {
    uint8_t data;
    if (HAL_I2C_Mem_Read(bme->hi2c, bme->address, reg, I2C_MEMADD_SIZE_8BIT,
                         &data, 1, BME280_I2C_TIMEOUT) == HAL_OK) {
        return data;
    }
    return 0;
}

void BME280_WriteRegister(BME280_HandleTypeDef *bme, uint8_t reg, uint8_t value) {
    HAL_I2C_Mem_Write(bme->hi2c, bme->address, reg, I2C_MEMADD_SIZE_8BIT,
                      &value, 1, BME280_I2C_TIMEOUT);
    HAL_Delay(1); // Small delay after write
}

void BME280_ReadCalibrationData(BME280_HandleTypeDef *bme) {
    uint8_t calibData[24];
    uint8_t humidData[7];

    // Read temperature and pressure calibration data (0x88-0x9F)
    HAL_I2C_Mem_Read(bme->hi2c, bme->address, BME280_REG_CALIB00, I2C_MEMADD_SIZE_8BIT,
                     calibData, 24, BME280_I2C_TIMEOUT);

    bme->calibData.dig_T1 = (calibData[1] << 8) | calibData[0];
    bme->calibData.dig_T2 = (calibData[3] << 8) | calibData[2];
    bme->calibData.dig_T3 = (calibData[5] << 8) | calibData[4];

    bme->calibData.dig_P1 = (calibData[7] << 8) | calibData[6];
    bme->calibData.dig_P2 = (calibData[9] << 8) | calibData[8];
    bme->calibData.dig_P3 = (calibData[11] << 8) | calibData[10];
    bme->calibData.dig_P4 = (calibData[13] << 8) | calibData[12];
    bme->calibData.dig_P5 = (calibData[15] << 8) | calibData[14];
    bme->calibData.dig_P6 = (calibData[17] << 8) | calibData[16];
    bme->calibData.dig_P7 = (calibData[19] << 8) | calibData[18];
    bme->calibData.dig_P8 = (calibData[21] << 8) | calibData[20];
    bme->calibData.dig_P9 = (calibData[23] << 8) | calibData[22];

    // Read H1 from register 0xA1
    bme->calibData.dig_H1 = BME280_ReadRegister(bme, 0xA1);

    // Read humidity calibration data (0xE1-0xE7)
    HAL_I2C_Mem_Read(bme->hi2c, bme->address, BME280_REG_CALIB26, I2C_MEMADD_SIZE_8BIT,
                     humidData, 7, BME280_I2C_TIMEOUT);

    bme->calibData.dig_H2 = (humidData[1] << 8) | humidData[0];
    bme->calibData.dig_H3 = humidData[2];

    bme->calibData.dig_H4 = (humidData[3] << 4) | (humidData[4] & 0x0F);
    bme->calibData.dig_H5 = (humidData[5] << 4) | (humidData[4] >> 4);
    bme->calibData.dig_H6 = humidData[6];
}

void BME280_SetMode(BME280_HandleTypeDef *bme, uint8_t mode) {
    uint8_t ctrlMeas = BME280_ReadRegister(bme, BME280_REG_CTRL_MEAS);
    ctrlMeas = (ctrlMeas & 0xFC) | (mode & 0x03);
    BME280_WriteRegister(bme, BME280_REG_CTRL_MEAS, ctrlMeas);
}

void BME280_SetOversamplingTemperature(BME280_HandleTypeDef *bme, uint8_t oversampling) {
    uint8_t ctrlMeas = BME280_ReadRegister(bme, BME280_REG_CTRL_MEAS);
    ctrlMeas = (ctrlMeas & 0x1F) | ((oversampling & 0x07) << 5);
    BME280_WriteRegister(bme, BME280_REG_CTRL_MEAS, ctrlMeas);
}

void BME280_SetOversamplingPressure(BME280_HandleTypeDef *bme, uint8_t oversampling) {
    uint8_t ctrlMeas = BME280_ReadRegister(bme, BME280_REG_CTRL_MEAS);
    ctrlMeas = (ctrlMeas & 0xE3) | ((oversampling & 0x07) << 2);
    BME280_WriteRegister(bme, BME280_REG_CTRL_MEAS, ctrlMeas);
}

void BME280_SetOversamplingHumidity(BME280_HandleTypeDef *bme, uint8_t oversampling) {
    BME280_WriteRegister(bme, BME280_REG_CTRL_HUM, oversampling & 0x07);
    // Changes to ctrl_hum only become effective after a write to ctrl_meas
    uint8_t ctrlMeas = BME280_ReadRegister(bme, BME280_REG_CTRL_MEAS);
    BME280_WriteRegister(bme, BME280_REG_CTRL_MEAS, ctrlMeas);
}

void BME280_SetFilter(BME280_HandleTypeDef *bme, uint8_t filter) {
    uint8_t config = BME280_ReadRegister(bme, BME280_REG_CONFIG);
    config = (config & 0xE3) | ((filter & 0x07) << 2);
    BME280_WriteRegister(bme, BME280_REG_CONFIG, config);
}

void BME280_SetStandbyTime(BME280_HandleTypeDef *bme, uint8_t standby) {
    uint8_t config = BME280_ReadRegister(bme, BME280_REG_CONFIG);
    config = (config & 0x1F) | ((standby & 0x07) << 5);
    BME280_WriteRegister(bme, BME280_REG_CONFIG, config);
}

bool BME280_IsMeasuring(BME280_HandleTypeDef *bme) {
    uint8_t status = BME280_ReadRegister(bme, BME280_REG_STATUS);
    return (status & 0x08) != 0;
}

int32_t BME280_ReadTemperatureRaw(BME280_HandleTypeDef *bme) {
    uint8_t msb = BME280_ReadRegister(bme, BME280_REG_TEMP_MSB);
    uint8_t lsb = BME280_ReadRegister(bme, BME280_REG_TEMP_LSB);
    uint8_t xlsb = BME280_ReadRegister(bme, BME280_REG_TEMP_XLSB);

    return ((uint32_t)msb << 12) | ((uint32_t)lsb << 4) | ((xlsb >> 4) & 0x0F);
}

int32_t BME280_ReadPressureRaw(BME280_HandleTypeDef *bme) {
    uint8_t msb = BME280_ReadRegister(bme, BME280_REG_PRESS_MSB);
    uint8_t lsb = BME280_ReadRegister(bme, BME280_REG_PRESS_LSB);
    uint8_t xlsb = BME280_ReadRegister(bme, BME280_REG_PRESS_XLSB);

    return ((uint32_t)msb << 12) | ((uint32_t)lsb << 4) | ((xlsb >> 4) & 0x0F);
}

int32_t BME280_ReadHumidityRaw(BME280_HandleTypeDef *bme) {
    uint8_t msb = BME280_ReadRegister(bme, BME280_REG_HUM_MSB);
    uint8_t lsb = BME280_ReadRegister(bme, BME280_REG_HUM_LSB);

    return ((uint32_t)msb << 8) | lsb;
}

float BME280_ReadTemperature(BME280_HandleTypeDef *bme) {
    int32_t adc_T = BME280_ReadTemperatureRaw(bme);

    // Temperature compensation formula from datasheet
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)bme->calibData.dig_T1 << 1))) *
                    ((int32_t)bme->calibData.dig_T2)) >> 11;

    int32_t var2 = (((((adc_T >> 4) - ((int32_t)bme->calibData.dig_T1)) *
                      ((adc_T >> 4) - ((int32_t)bme->calibData.dig_T1))) >> 12) *
                    ((int32_t)bme->calibData.dig_T3)) >> 14;

    bme->t_fine = var1 + var2;

    float temperature = (bme->t_fine * 5 + 128) >> 8;
    return temperature / 100.0f;
}

float BME280_ReadPressure(BME280_HandleTypeDef *bme) {
    // Read temperature first to get t_fine
    BME280_ReadTemperature(bme);

    int32_t adc_P = BME280_ReadPressureRaw(bme);

    // Pressure compensation formula from datasheet
    int64_t var1 = ((int64_t)bme->t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)bme->calibData.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bme->calibData.dig_P5) << 17);
    var2 = var2 + (((int64_t)bme->calibData.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bme->calibData.dig_P3) >> 8) +
           ((var1 * (int64_t)bme->calibData.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bme->calibData.dig_P1) >> 33;

    if (var1 == 0) {
        return 0; // Avoid division by zero
    }

    int64_t pressure = 1048576 - adc_P;
    pressure = (((pressure << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bme->calibData.dig_P9) * (pressure >> 13) * (pressure >> 13)) >> 25;
    var2 = (((int64_t)bme->calibData.dig_P8) * pressure) >> 19;

    pressure = ((pressure + var1 + var2) >> 8) + (((int64_t)bme->calibData.dig_P7) << 4);

    return (float)pressure / 256.0f;
}

float BME280_ReadHumidity(BME280_HandleTypeDef *bme) {
    // Read temperature first to get t_fine
    BME280_ReadTemperature(bme);

    int32_t adc_H = BME280_ReadHumidityRaw(bme);

    // Humidity compensation formula from datasheet
    int32_t var1 = (bme->t_fine - ((int32_t)76800));

    var1 = (((((adc_H << 14) - (((int32_t)bme->calibData.dig_H4) << 20) -
              (((int32_t)bme->calibData.dig_H5) * var1)) + ((int32_t)16384)) >> 15) *
            (((((((var1 * ((int32_t)bme->calibData.dig_H6)) >> 10) *
                (((var1 * ((int32_t)bme->calibData.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
              ((int32_t)2097152)) * ((int32_t)bme->calibData.dig_H2) + 8192) >> 14));

    var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) *
                     ((int32_t)bme->calibData.dig_H1)) >> 4));

    var1 = (var1 < 0 ? 0 : var1);
    var1 = (var1 > 419430400 ? 419430400 : var1);

    float humidity = (var1 >> 12);
    return humidity / 1024.0f;
}

float BME280_ReadAltitude(BME280_HandleTypeDef *bme, float seaLevelPressure) {
    float pressure = BME280_ReadPressure(bme) / 100.0f; // Convert to hPa
    return 44330.0f * (1.0f - powf(pressure / seaLevelPressure, 0.1903f));
}
