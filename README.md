# BME280 STM32 HAL Library

A lightweight C driver and example project for interfacing the Bosch BME280 environmental sensor with STM32 microcontrollers through the STM32 HAL I2C API.

The included project targets the **STM32F401CCU6** and demonstrates reading temperature, barometric pressure, relative humidity, and estimated altitude. Measurements are available through USART2 and SWV/ITM debug output.

[![Language](https://img.shields.io/badge/language-C-blue.svg)](https://github.com/mjmokhtar/bme280-lib-hal)
[![Platform](https://img.shields.io/badge/platform-STM32-orange.svg)](https://www.st.com/stm32)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

## Features

- STM32 HAL-based I2C communication
- BME280 I2C addresses `0x76` and `0x77`
- Chip-ID verification and software reset
- Factory calibration data loading
- Compensated temperature, pressure, and humidity readings
- Altitude estimation using sea-level pressure
- Configurable oversampling, IIR filter, standby time, and power mode
- Connection and measurement-status checks
- Example output through USART2 and SWV/ITM
- Automatic I2C scan when initialization fails

## Hardware Configuration

| Component | Configuration |
| --- | --- |
| Microcontroller | STM32F401CCU6 |
| Sensor | Bosch BME280 |
| Interface | I2C1 at 100 kHz |
| I2C SCL | PB6 |
| I2C SDA | PB7 |
| Debug output | USART2 at 115200 baud |
| USART2 TX / RX | PA2 / PA3 |
| System clock | 84 MHz |

## Wiring

| BME280 | STM32F401CCU6 | Description |
| --- | --- | --- |
| VIN / VCC | 3.3 V | Sensor power |
| GND | GND | Ground |
| SCL / SCK | PB6 | I2C1 clock |
| SDA / SDI | PB7 | I2C1 data |
| CSB | 3.3 V | Select I2C mode when required |
| SDO | GND or 3.3 V | Select address `0x76` or `0x77` |

> Use 3.3 V logic. Many BME280 modules include I2C pull-up resistors; a bare sensor requires suitable pull-ups on SDA and SCL.

## Repository Structure

```text
.
├── Core
│   ├── Inc
│   │   └── BME280_STM32.h       # Driver API and definitions
│   └── Src
│       ├── BME280_STM32.c       # Driver implementation
│       └── main.c               # Example application
├── Drivers                      # STM32F4 HAL and CMSIS
├── i2c_bme280_test.ioc          # STM32CubeMX configuration
├── STM32F401CCUX_FLASH.ld       # Linker script
└── LICENSE
```

## Getting Started

### 1. Clone the repository

```bash
git clone https://github.com/mjmokhtar/bme280-lib-hal.git
cd bme280-lib-hal
```

### 2. Open the project

Import the repository as an existing project in **STM32CubeIDE**. The original configuration uses STM32CubeMX 6.9.2 and STM32Cube FW_F4 V1.27.1.

If needed, open `i2c_bme280_test.ioc` and regenerate the initialization code for your installed STM32F4 firmware package.

### 3. Connect and configure the sensor

Wire the BME280 to PB6 and PB7. The example uses address `0x76`:

```c
BME280_ADDRESS_PRIMARY
```

Use `BME280_ADDRESS_SECONDARY` for a module configured as `0x77`.

### 4. Build, flash, and monitor

Build and flash the firmware with STM32CubeIDE and an ST-Link programmer. Open a serial terminal at **115200 baud, 8-N-1**. Measurements are updated approximately every two seconds.

Example output:

```text
BME280 initialization successful!
BME280 Chip ID: 0x60

=== Loop #0 ===
Temperature: 27.42 C
Pressure: 100742.31 Pa (1007.42 hPa)
Humidity: 68.15%
Altitude: 49.08 m
```

## Using the Driver in Another Project

Copy these files into your STM32 project:

```text
Core/Inc/BME280_STM32.h
Core/Src/BME280_STM32.c
```

Initialize I2C before calling the driver:

```c
#include "BME280_STM32.h"

BME280_HandleTypeDef bme280;

if (BME280_Init(&bme280, &hi2c1, BME280_ADDRESS_PRIMARY)) {
    float temperature = BME280_ReadTemperature(&bme280);          // °C
    float pressure    = BME280_ReadPressure(&bme280);             // Pa
    float humidity    = BME280_ReadHumidity(&bme280);             // %RH
    float altitude    = BME280_ReadAltitude(&bme280, 1013.25f);   // m
}
```

Pass the normal 7-bit address to `BME280_Init()`. The driver shifts it internally because STM32 HAL expects the address in 8-bit form.

For another STM32 family, replace this include in `BME280_STM32.h`:

```c
#include "stm32f4xx_hal.h"
```

with the correct HAL header, such as `stm32f1xx_hal.h` or `stm32l4xx_hal.h`.

## Default Configuration

`BME280_Init()` applies:

| Setting | Default |
| --- | --- |
| Temperature oversampling | ×16 |
| Pressure oversampling | ×16 |
| Humidity oversampling | ×16 |
| IIR filter | Coefficient 16 |
| Standby time | 0.5 ms |
| Power mode | Normal |

The configuration can be changed after initialization:

```c
BME280_SetMode(&bme280, BME280_MODE_NORMAL);
BME280_SetOversamplingTemperature(&bme280, BME280_OVERSAMP_4X);
BME280_SetOversamplingPressure(&bme280, BME280_OVERSAMP_4X);
BME280_SetOversamplingHumidity(&bme280, BME280_OVERSAMP_2X);
BME280_SetFilter(&bme280, BME280_FILTER_4);
BME280_SetStandbyTime(&bme280, BME280_STANDBY_1000);
```

## Main API

### Initialization and status

```c
bool BME280_Init(BME280_HandleTypeDef *bme, I2C_HandleTypeDef *hi2c, uint8_t address);
bool BME280_IsConnected(BME280_HandleTypeDef *bme);
bool BME280_IsMeasuring(BME280_HandleTypeDef *bme);
uint8_t BME280_GetChipID(BME280_HandleTypeDef *bme);
void BME280_Reset(BME280_HandleTypeDef *bme);
```

### Measurements

```c
float BME280_ReadTemperature(BME280_HandleTypeDef *bme);
float BME280_ReadPressure(BME280_HandleTypeDef *bme);
float BME280_ReadHumidity(BME280_HandleTypeDef *bme);
float BME280_ReadAltitude(BME280_HandleTypeDef *bme, float seaLevelPressure);
```

## Altitude Accuracy

Altitude is estimated from atmospheric pressure. The example uses `1013.25 hPa`, the standard mean sea-level pressure:

```c
BME280_ReadAltitude(&bme280, 1013.25f);
```

For better local accuracy, use the current sea-level pressure reported by a nearby weather station. BME280 altitude is an estimate, not a replacement for GNSS or surveyed elevation.

## Troubleshooting

### Initialization fails

- Confirm the module is powered with compatible 3.3 V logic.
- Check SDA, SCL, and GND connections.
- Try both `0x76` and `0x77`.
- Ensure SDA and SCL have pull-up resistors.
- Verify I2C1 is initialized before `BME280_Init()`.
- Check the serial output; the example scans the bus after a failure.

### Pressure works but humidity is incorrect

Confirm the device is a **BME280**, not a BMP280. The BMP280 has no humidity sensor. The expected BME280 chip ID is `0x60`.

### No UART output

- Use 115200 baud.
- Confirm USART2 TX is available on PA2.
- Ensure the board and serial adapter share a common ground.

## Notes

- The driver uses blocking STM32 HAL I2C calls.
- Pressure and humidity reads refresh the temperature compensation value internally.
- The current implementation supports I2C; SPI is not implemented.
- The header targets STM32F4 HAL but can be adapted to another STM32 HAL family.

## Contributing

Issues and pull requests are welcome. Please include the STM32 target, sensor address, wiring, and reproduction steps when reporting a problem.

## License

Distributed under the [MIT License](LICENSE).

Copyright © 2025 Muhammad Jumi'at Mokhtar.
